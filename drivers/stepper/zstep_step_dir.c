/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT zstep_step_dir

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/clock.h>
#include <zstep/zstep.h>

#include <stepper_ctrl_event_handler.h>

#include "zstep_ramp.h"

LOG_MODULE_REGISTER(zstep_step_dir, CONFIG_STEPPER_LOG_LEVEL);

#define NO_EVENT -1

enum motion {
	MOTION_IDLE,
	MOTION_POSITION,
	MOTION_VELOCITY,
	MOTION_STOPPING,
};

enum phase {
	PHASE_STEP,
	PHASE_PULSE_END,
};

struct zstep_config {
	struct gpio_dt_spec step;
	struct gpio_dt_spec dir;
	const struct device *counter;
	uint32_t step_width_ns;
	uint32_t dir_setup_ns;
	uint8_t channel;
	bool invert_direction;
	bool dual_edge;
};

struct zstep_data {
	struct stepper_ctrl_event_handler_data event_common;
	struct k_spinlock lock;
	struct zstep_profile profile;
	struct zstep_ramp ramp;
	enum motion motion;
	enum phase phase;
	int32_t pos;
	int32_t target;
	int8_t dir;
	int8_t run_dir;
	bool step_level;
	/* Grid time of the current step, the alarm only approximates it. */
	uint32_t t;
	uint32_t due;
	uint32_t freq;
	uint32_t top;
	uint32_t min_delay;
	uint32_t max_delay;
	bool count_up;
};

STEPPER_CONTROLLER_EVENT_COMMON_STRUCT_CHECK(struct zstep_data);

/* Counter drivers are not reentrant, so motors sharing one must not program it in parallel. */
static struct k_spinlock counter_lock;
static struct k_spinlock group_lock;

static void zstep_alarm(const struct device *counter, uint8_t channel, uint32_t ticks,
			void *user_data);

static void emit(const struct device *dev, int event)
{
	if (event != NO_EVENT) {
		stepper_ctrl_event_handler_process_cb(dev, event);
	}
}

static uint32_t counter_now(const struct device *dev)
{
	const struct zstep_config *cfg = dev->config;
	const struct zstep_data *data = dev->data;
	uint32_t ticks = 0;

	(void)counter_get_value(cfg->counter, &ticks);
	return data->count_up ? ticks : data->top - ticks;
}

static int8_t wanted_dir(const struct zstep_data *data)
{
	switch (data->motion) {
	case MOTION_POSITION:
		return (data->target > data->pos) - (data->target < data->pos);
	case MOTION_VELOCITY:
		return data->run_dir;
	default:
		return 0;
	}
}

static uint32_t steps_ahead(const struct zstep_data *data)
{
	if (wanted_dir(data) != data->dir) {
		return 0;
	}

	if (data->motion == MOTION_VELOCITY) {
		return UINT32_MAX;
	}

	return llabs((int64_t)data->target - data->pos);
}

static void set_dir_locked(const struct device *dev, int8_t dir)
{
	const struct zstep_config *cfg = dev->config;
	struct zstep_data *data = dev->data;

	data->dir = dir;
	(void)gpio_pin_set_dt(&cfg->dir, (dir > 0) != cfg->invert_direction);
}

static void set_step_locked(const struct device *dev, bool level)
{
	const struct zstep_config *cfg = dev->config;
	struct zstep_data *data = dev->data;

	data->step_level = level;
	(void)gpio_pin_set_dt(&cfg->step, level);
}

static void cancel_alarm_locked(const struct device *dev)
{
	const struct zstep_config *cfg = dev->config;

	K_SPINLOCK(&counter_lock) {
		(void)counter_cancel_channel_alarm(cfg->counter, cfg->channel);
	}
}

/* Returns -ETIME if the current phase is already due and has to run right away. */
static int arm_locked(const struct device *dev)
{
	const struct zstep_config *cfg = dev->config;
	struct zstep_data *data = dev->data;
	uint32_t at = data->phase == PHASE_STEP ? data->t : data->t + data->min_delay;
	struct counter_alarm_cfg alarm = {
		.callback = zstep_alarm,
		.user_data = (void *)dev,
	};
	k_spinlock_key_t key = k_spin_lock(&counter_lock);
	uint32_t now = counter_now(dev);
	int err;

	/* A late edge keeps the minimum spacing and the grid lets the next ones catch up. */
	alarm.ticks = (at - now) & data->top;
	if (alarm.ticks < data->min_delay || alarm.ticks > data->max_delay) {
		alarm.ticks = data->min_delay;
	}

	err = counter_set_channel_alarm(cfg->counter, cfg->channel, &alarm);
	if (err == 0 && ((counter_now(dev) - now) & data->top) >= alarm.ticks) {
		(void)counter_cancel_channel_alarm(cfg->counter, cfg->channel);
		err = -ETIME;
	}

	if (err == 0) {
		data->due = (now + alarm.ticks) & data->top;
	}

	k_spin_unlock(&counter_lock, key);
	return err;
}

static bool is_due_locked(const struct device *dev)
{
	struct zstep_data *data = dev->data;

	return ((counter_now(dev) + 1 - data->due) & data->top) <= data->max_delay;
}

static int abort_locked(const struct device *dev, int err)
{
	const struct zstep_config *cfg = dev->config;
	struct zstep_data *data = dev->data;

	LOG_ERR("%s: cannot arm counter alarm (%d)", dev->name, err);
	if (!cfg->dual_edge) {
		set_step_locked(dev, false);
	}

	data->motion = MOTION_IDLE;
	return STEPPER_CTRL_EVENT_STOPPED;
}

static int rest_locked(const struct device *dev)
{
	struct zstep_data *data = dev->data;
	int8_t dir = wanted_dir(data);

	if (dir == 0) {
		int event = data->motion == MOTION_POSITION ? STEPPER_CTRL_EVENT_STEPS_COMPLETED
							    : STEPPER_CTRL_EVENT_STOPPED;

		data->motion = MOTION_IDLE;
		return event;
	}

	if (dir != data->dir) {
		set_dir_locked(dev, dir);
	}

	data->t = (data->t + zstep_ramp_pause(&data->ramp.p)) & data->top;
	data->phase = PHASE_STEP;
	return NO_EVENT;
}

static bool plan_locked(struct zstep_data *data)
{
	uint32_t interval = zstep_ramp_next(&data->ramp, steps_ahead(data));

	if (interval == 0) {
		return false;
	}

	data->t = (data->t + interval) & data->top;
	data->phase = PHASE_STEP;
	return true;
}

static int step_locked(const struct device *dev)
{
	const struct zstep_config *cfg = dev->config;
	struct zstep_data *data = dev->data;

	if (zstep_ramp_can_stop(&data->ramp) && wanted_dir(data) != data->dir) {
		return rest_locked(dev);
	}

	data->pos += data->dir;
	set_step_locked(dev, !data->step_level);
	data->phase = PHASE_PULSE_END;

	/* Without a falling edge the next step is planned now, PULSE_END then only holds DIR. */
	if (cfg->dual_edge) {
		(void)plan_locked(data);
	}

	return NO_EVENT;
}

static int pulse_end_locked(const struct device *dev)
{
	const struct zstep_config *cfg = dev->config;
	struct zstep_data *data = dev->data;

	if (cfg->dual_edge) {
		return rest_locked(dev);
	}

	set_step_locked(dev, false);
	return plan_locked(data) ? NO_EVENT : rest_locked(dev);
}

static int run_phase_locked(const struct device *dev)
{
	struct zstep_data *data = dev->data;

	return data->phase == PHASE_STEP ? step_locked(dev) : pulse_end_locked(dev);
}

static int schedule_locked(const struct device *dev)
{
	struct zstep_data *data = dev->data;
	int err;

	while ((err = arm_locked(dev)) == -ETIME) {
		int event = run_phase_locked(dev);

		if (data->motion == MOTION_IDLE) {
			return event;
		}
	}

	return err == 0 ? NO_EVENT : abort_locked(dev, err);
}

static int alarm_locked(const struct device *dev)
{
	struct zstep_data *data = dev->data;

	if (data->motion == MOTION_IDLE) {
		return NO_EVENT;
	}

	/* A stale callback finds the newer alarm still armed, an early one arms it again. */
	if (!is_due_locked(dev) && arm_locked(dev) != -ETIME) {
		return NO_EVENT;
	}

	int event = run_phase_locked(dev);

	return data->motion == MOTION_IDLE ? event : schedule_locked(dev);
}

static void zstep_alarm(const struct device *counter, uint8_t channel, uint32_t ticks,
			void *user_data)
{
	const struct device *dev = user_data;
	struct zstep_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int event = alarm_locked(dev);

	k_spin_unlock(&data->lock, key);
	emit(dev, event);
}

static int start_locked(const struct device *dev, uint32_t at)
{
	struct zstep_data *data = dev->data;

	set_dir_locked(dev, wanted_dir(data));
	data->t = at & data->top;
	data->phase = PHASE_STEP;
	return schedule_locked(dev);
}

static int move_locked(const struct device *dev, enum motion motion, int32_t target, int8_t run_dir,
		       int *event)
{
	struct zstep_data *data = dev->data;
	bool idle = data->motion == MOTION_IDLE;

	if (data->profile.cmin == 0) {
		return -EINVAL;
	}

	data->motion = motion;
	data->target = target;
	data->run_dir = run_dir;
	if (!idle) {
		return 0;
	}

	if (wanted_dir(data) == 0) {
		data->motion = MOTION_IDLE;
		*event = STEPPER_CTRL_EVENT_STEPS_COMPLETED;
		return 0;
	}

	zstep_ramp_reset(&data->ramp, &data->profile);
	*event = start_locked(dev, counter_now(dev) + data->min_delay);
	return 0;
}

static int zstep_move_to(const struct device *dev, const int32_t target)
{
	struct zstep_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int event = NO_EVENT;
	int err = move_locked(dev, MOTION_POSITION, target, 0, &event);

	k_spin_unlock(&data->lock, key);
	emit(dev, event);
	return err;
}

static int zstep_move_by(const struct device *dev, const int32_t micro_steps)
{
	struct zstep_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int event = NO_EVENT;
	int32_t target;
	int err = -ERANGE;

	if (!__builtin_add_overflow(data->pos, micro_steps, &target)) {
		err = move_locked(dev, MOTION_POSITION, target, 0, &event);
	}

	k_spin_unlock(&data->lock, key);
	emit(dev, event);
	return err;
}

static int zstep_run(const struct device *dev, const enum stepper_ctrl_direction direction)
{
	struct zstep_data *data = dev->data;
	int8_t run_dir = direction == STEPPER_CTRL_DIRECTION_POSITIVE ? 1 : -1;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int event = NO_EVENT;
	int err = move_locked(dev, MOTION_VELOCITY, data->target, run_dir, &event);

	k_spin_unlock(&data->lock, key);
	emit(dev, event);
	return err;
}

static int stop_locked(const struct device *dev)
{
	struct zstep_data *data = dev->data;

	if (data->motion == MOTION_IDLE) {
		return STEPPER_CTRL_EVENT_STOPPED;
	}

	data->motion = MOTION_STOPPING;
	if (data->phase == PHASE_PULSE_END || !zstep_ramp_can_stop(&data->ramp)) {
		return NO_EVENT;
	}

	cancel_alarm_locked(dev);
	data->motion = MOTION_IDLE;
	return STEPPER_CTRL_EVENT_STOPPED;
}

static int zstep_stop(const struct device *dev)
{
	struct zstep_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int event = stop_locked(dev);

	k_spin_unlock(&data->lock, key);
	emit(dev, event);
	return 0;
}

static int zstep_is_moving(const struct device *dev, bool *is_moving)
{
	struct zstep_data *data = dev->data;

	K_SPINLOCK(&data->lock) {
		*is_moving = data->motion != MOTION_IDLE;
	}

	return 0;
}

static int zstep_set_reference_position(const struct device *dev, const int32_t value)
{
	struct zstep_data *data = dev->data;

	K_SPINLOCK(&data->lock) {
		data->target = value + ((int64_t)data->target - data->pos);
		data->pos = value;
	}

	return 0;
}

static int zstep_get_actual_position(const struct device *dev, int32_t *value)
{
	struct zstep_data *data = dev->data;

	K_SPINLOCK(&data->lock) {
		*value = data->pos;
	}

	return 0;
}

static int make_profile(const struct device *dev, struct zstep_profile *p, uint64_t cmin,
			uint32_t accel, uint32_t decel)
{
	const struct zstep_config *cfg = dev->config;
	const struct zstep_data *data = dev->data;
	uint64_t shortest = (uint64_t)(cfg->dual_edge ? 1 : 2) * data->min_delay << ZSTEP_RAMP_Q;
	uint64_t longest = (uint64_t)data->max_delay << ZSTEP_RAMP_Q;
	int err = zstep_profile_init(p, data->freq, cmin, accel, decel);

	if (err != 0) {
		return err;
	}

	if (p->cmin < shortest || MAX(MAX(p->c0, p->c0d), p->cmin) > longest) {
		return -EINVAL;
	}

	return 0;
}

static int zstep_set_microstep_interval(const struct device *dev, const uint64_t interval_ns)
{
	struct zstep_data *data = dev->data;
	struct zstep_profile profile;
	uint32_t accel;
	uint32_t decel;
	int err;

	K_SPINLOCK(&data->lock) {
		accel = data->profile.accel;
		decel = data->profile.decel;
	}

	err = make_profile(dev, &profile, zstep_interval_from_ns(data->freq, interval_ns), accel,
			   decel);
	if (err != 0) {
		return err;
	}

	K_SPINLOCK(&data->lock) {
		data->profile = profile;
		data->ramp.p.cmin = profile.cmin;
	}

	return 0;
}

static int zstep_configure_ramp(const struct device *dev, const struct stepper_ctrl_ramp *ramp)
{
	struct zstep_data *data = dev->data;
	struct zstep_profile profile;
	int err;

	err = make_profile(dev, &profile, zstep_interval_from_speed(data->freq, ramp->speed_max),
			   ramp->acceleration_max, ramp->deceleration_max);
	if (err != 0) {
		return err;
	}

	K_SPINLOCK(&data->lock) {
		if (data->motion != MOTION_IDLE) {
			err = -EBUSY;
			K_SPINLOCK_BREAK;
		}

		data->profile = profile;
	}

	return err;
}

static int zstep_set_event_cb(const struct device *dev, stepper_ctrl_event_callback_t callback,
			      void *user_data)
{
	return stepper_ctrl_event_handler_set_event_cb(dev, callback, user_data);
}

static DEVICE_API(stepper_ctrl, zstep_api) = {
	.set_reference_position = zstep_set_reference_position,
	.get_actual_position = zstep_get_actual_position,
	.set_event_cb = zstep_set_event_cb,
	.set_microstep_interval = zstep_set_microstep_interval,
	.configure_ramp = zstep_configure_ramp,
	.move_by = zstep_move_by,
	.move_to = zstep_move_to,
	.run = zstep_run,
	.stop = zstep_stop,
	.is_moving = zstep_is_moving,
};

static int check_group(const struct device *const devs[], const int32_t steps[], size_t count)
{
	if (count == 0 || count > CONFIG_ZSTEP_GROUP_MAX_AXES) {
		return -EINVAL;
	}

	for (size_t i = 0; i < count; i++) {
		const struct zstep_config *cfg = devs[i]->config;
		const struct zstep_config *lead_cfg = devs[0]->config;

		if (devs[i]->api != &zstep_api) {
			return -ENOTSUP;
		}

		if (cfg->counter != lead_cfg->counter || steps[i] == INT32_MIN) {
			return -EINVAL;
		}

		for (size_t j = 0; j < i; j++) {
			if (devs[j] == devs[i]) {
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int start_group_locked(const struct device *const devs[], const int32_t steps[],
			      const struct zstep_profile profiles[], size_t count, int events[])
{
	const struct zstep_config *cfg = devs[0]->config;
	int32_t targets[CONFIG_ZSTEP_GROUP_MAX_AXES];

	for (size_t i = 0; i < count; i++) {
		struct zstep_data *data = devs[i]->data;

		if (data->motion != MOTION_IDLE) {
			return -EBUSY;
		}

		if (__builtin_add_overflow(data->pos, steps[i], &targets[i])) {
			return -ERANGE;
		}
	}

	uint32_t at = counter_now(devs[0]) +
		      counter_us_to_ticks(cfg->counter, CONFIG_ZSTEP_GROUP_START_US);

	for (size_t i = 0; i < count; i++) {
		struct zstep_data *data = devs[i]->data;

		events[i] = NO_EVENT;
		if (steps[i] == 0) {
			events[i] = STEPPER_CTRL_EVENT_STEPS_COMPLETED;
			continue;
		}

		data->motion = MOTION_POSITION;
		data->target = targets[i];
		zstep_ramp_reset(&data->ramp, &profiles[i]);
		events[i] = start_locked(devs[i], at);
	}

	return 0;
}

int zstep_group_move_by(const struct device *const devs[], const int32_t steps[], size_t count,
			const struct stepper_ctrl_ramp *ramp)
{
	struct zstep_profile profiles[CONFIG_ZSTEP_GROUP_MAX_AXES];
	k_spinlock_key_t keys[CONFIG_ZSTEP_GROUP_MAX_AXES];
	int events[CONFIG_ZSTEP_GROUP_MAX_AXES];
	uint32_t lead = 0;
	int err;

	err = check_group(devs, steps, count);
	if (err != 0) {
		return err;
	}

	for (size_t i = 0; i < count; i++) {
		lead = MAX(lead, (uint32_t)abs(steps[i]));
	}

	for (size_t i = 0; i < count; i++) {
		const struct zstep_data *data = devs[i]->data;
		uint64_t cmin = zstep_interval_from_speed(data->freq, ramp->speed_max);

		err = make_profile(devs[i], &profiles[i], cmin, ramp->acceleration_max,
				   ramp->deceleration_max);
		if (err != 0) {
			return err;
		}

		if (steps[i] != 0) {
			zstep_profile_scale(&profiles[i], lead, abs(steps[i]),
					    MIN((uint64_t)data->max_delay << ZSTEP_RAMP_Q,
						ZSTEP_RAMP_INTERVAL_MAX));
		}
	}

	k_spinlock_key_t group_key = k_spin_lock(&group_lock);

	for (size_t i = 0; i < count; i++) {
		keys[i] = k_spin_lock(&((struct zstep_data *)devs[i]->data)->lock);
	}

	err = start_group_locked(devs, steps, profiles, count, events);

	for (size_t i = count; i-- > 0;) {
		k_spin_unlock(&((struct zstep_data *)devs[i]->data)->lock, keys[i]);
	}

	k_spin_unlock(&group_lock, group_key);
	if (err != 0) {
		return err;
	}

	for (size_t i = 0; i < count; i++) {
		emit(devs[i], events[i]);
	}

	return 0;
}

static int zstep_init(const struct device *dev)
{
	const struct zstep_config *cfg = dev->config;
	struct zstep_data *data = dev->data;
	int err;

	if (!gpio_is_ready_dt(&cfg->step) || !gpio_is_ready_dt(&cfg->dir) ||
	    !device_is_ready(cfg->counter)) {
		return -ENODEV;
	}

	if (cfg->channel >= counter_get_num_of_channels(cfg->counter)) {
		LOG_ERR("%s: counter has no channel %u", dev->name, cfg->channel);
		return -EINVAL;
	}

	data->freq = counter_get_frequency(cfg->counter);
	data->top = counter_get_top_value(cfg->counter);
	data->count_up = counter_is_counting_up(cfg->counter);
	if (data->freq == 0 || (data->top & (data->top + 1)) != 0) {
		LOG_ERR("%s: counter needs a clock and a power of two range", dev->name);
		return -ENOTSUP;
	}

	uint64_t hold_ns = MAX(cfg->step_width_ns, cfg->dir_setup_ns);

	/* One extra tick covers the unknown counter phase at the moment an alarm is set. */
	data->min_delay = DIV_ROUND_UP(hold_ns * data->freq, NSEC_PER_SEC) + 1;
	data->max_delay = data->top >> 1;

	err = gpio_pin_configure_dt(&cfg->step, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}

	err = gpio_pin_configure_dt(&cfg->dir, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}

	err = counter_start(cfg->counter);
	if (err != 0 && err != -EALREADY) {
		return err;
	}

	return stepper_ctrl_event_handler_init(dev);
}

#define ZSTEP_TIMING(inst, prop)                                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, stepper_driver),                                   \
		    (DT_PROP_OR(DT_INST_PHANDLE(inst, stepper_driver), prop,                       \
				DT_INST_PROP(inst, prop))),                                        \
		    (DT_INST_PROP(inst, prop)))

#define ZSTEP_DEFINE(inst)                                                                         \
	static const struct zstep_config zstep_config_##inst = {                                   \
		.step = GPIO_DT_SPEC_INST_GET(inst, step_gpios),                                   \
		.dir = GPIO_DT_SPEC_INST_GET(inst, dir_gpios),                                     \
		.counter = DEVICE_DT_GET(DT_INST_PHANDLE(inst, counter)),                          \
		.step_width_ns = ZSTEP_TIMING(inst, step_width_ns),                                \
		.dir_setup_ns = DT_INST_PROP(inst, dir_setup_ns),                                  \
		.channel = DT_INST_PROP(inst, counter_channel),                                    \
		.invert_direction = DT_INST_PROP(inst, invert_direction),                          \
		.dual_edge =                                                                       \
			DT_INST_PROP(inst, dual_edge_step) || ZSTEP_TIMING(inst, dual_edge_step),  \
	};                                                                                         \
	static struct zstep_data zstep_data_##inst = {                                             \
		.event_common = STEPPER_CTRL_EVENT_HANDLER_DT_INST_DATA_INIT(inst),                \
		.dir = 1,                                                                          \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, zstep_init, NULL, &zstep_data_##inst, &zstep_config_##inst,    \
			      POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY, &zstep_api);

DT_INST_FOREACH_STATUS_OKAY(ZSTEP_DEFINE)
