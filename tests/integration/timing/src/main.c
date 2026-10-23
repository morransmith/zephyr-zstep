/* SPDX-License-Identifier: Apache-2.0 */

#include <stdlib.h>

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/ztest.h>
#include <zstep/zstep.h>

#include "gpio_spy.h"
#include "zstep_ramp.h"
#include "zstep_shaper.h"

#define FREQ      1000000U
/* Default 2000 ns step width at 1 MHz rounded up, plus one tick of counter phase. */
#define MIN_DELAY 3
#define MAX_TICKS 8192

struct motor {
	const struct device *dev;
	uint8_t step_pin;
	uint8_t dir_pin;
	bool inverted;
	bool dual_edge;
	struct k_sem done;
	int event;
	int chain;
	int chain_err;
};

#define MOTOR(node)                                                                                \
	{                                                                                          \
		.dev = DEVICE_DT_GET(node),                                                        \
		.step_pin = DT_GPIO_PIN(node, step_gpios),                                         \
		.dir_pin = DT_GPIO_PIN(node, dir_gpios),                                           \
		.inverted = DT_PROP(node, invert_direction),                                       \
		.dual_edge = DT_PROP(node, dual_edge_step),                                        \
	}

static struct motor motors[] = {
	MOTOR(DT_NODELABEL(motor_a)), MOTOR(DT_NODELABEL(motor_b)), MOTOR(DT_NODELABEL(motor_c)),
	MOTOR(DT_NODELABEL(motor_d)), MOTOR(DT_NODELABEL(motor_e)),
};

static struct motor *const a = &motors[0];
static struct motor *const b = &motors[1];
static struct motor *const c = &motors[2];
static struct motor *const d = &motors[3];
static struct motor *const e = &motors[4];

static const struct device *const counter = DEVICE_DT_GET(DT_NODELABEL(counter0));
static const struct device *const spy = DEVICE_DT_GET(DT_NODELABEL(spy));

static const struct stepper_ctrl_ramp slow = {
	.acceleration_max = 10000,
	.speed_max = 2000,
	.deceleration_max = 10000,
};

static const struct stepper_ctrl_ramp quick = {
	.acceleration_max = 50000,
	.speed_max = 5000,
	.deceleration_max = 25000,
};

static const struct stepper_ctrl_ramp fast = {
	.acceleration_max = 100000,
	.speed_max = 20000,
	.deceleration_max = 100000,
};

static uint32_t got[ARRAY_SIZE(motors)][MAX_TICKS];
static uint32_t want[MAX_TICKS];

static void on_event(const struct device *dev, enum stepper_ctrl_event event, void *user_data)
{
	struct motor *m = user_data;

	if (event == STEPPER_CTRL_EVENT_STEPS_COMPLETED && m->chain > 0) {
		m->chain--;
		m->chain_err |= stepper_ctrl_move_by(dev, 10);
		return;
	}

	m->event = event;
	k_sem_give(&m->done);
}

static void wait_for(struct motor *m, enum stepper_ctrl_event event)
{
	zassert_ok(k_sem_take(&m->done, K_SECONDS(10)), "%s timed out", m->dev->name);
	zassert_equal(m->event, event, "%s reported event %d", m->dev->name, m->event);
}

static int32_t position(const struct motor *m)
{
	int32_t pos;

	zassert_ok(stepper_ctrl_get_actual_position(m->dev, &pos));
	return pos;
}

/* Times of the step edges, both edges count for a dual-edge driver. */
static size_t step_ticks(const struct motor *m, uint32_t *out)
{
	const struct gpio_spy_edge *edges;
	size_t count = gpio_spy_edges(&edges);
	size_t n = 0;

	for (size_t i = 0; i < count && n < MAX_TICKS; i++) {
		if (edges[i].pin == m->step_pin && (m->dual_edge || edges[i].level == 1)) {
			out[n++] = edges[i].tick;
		}
	}

	return n;
}

static struct zstep_profile profile_of(const struct stepper_ctrl_ramp *ramp)
{
	struct zstep_profile p;

	zassert_ok(zstep_profile_init(&p, FREQ, zstep_interval_from_speed(FREQ, ramp->speed_max),
				      ramp->acceleration_max, ramp->deceleration_max));
	return p;
}

/* Every step of an n step move must land exactly where the ramp puts it. */
static void check_move(const struct motor *m, const uint32_t *ticks, const struct zstep_profile *p,
		       uint32_t n)
{
	struct zstep_ramp r;

	zstep_ramp_reset(&r, p);
	want[0] = 0;
	for (uint32_t k = 1; k < n; k++) {
		want[k] = want[k - 1] + zstep_ramp_next(&r, n - k);
	}

	for (uint32_t k = 0; k < n; k++) {
		zassert_equal(ticks[k] - ticks[0], want[k], "%s: step %u at %u, expected %u",
			      m->dev->name, k, ticks[k] - ticks[0], want[k]);
	}
}

static size_t steps_up_to(const uint32_t *ticks, size_t n, uint32_t tick)
{
	size_t steps = 0;

	while (steps < n && ticks[steps] <= tick) {
		steps++;
	}

	return steps;
}

ZTEST(zstep_timing, test_edges_follow_ramp)
{
	struct zstep_profile p = profile_of(&slow);
	const struct gpio_spy_edge *edges;
	uint32_t rise = 0;

	zassert_ok(stepper_ctrl_configure_ramp(a->dev, &slow));
	zassert_ok(stepper_ctrl_move_by(a->dev, 500));
	wait_for(a, STEPPER_CTRL_EVENT_STEPS_COMPLETED);

	zassert_equal(position(a), 500);
	zassert_equal(step_ticks(a, got[0]), 500);
	check_move(a, got[0], &p, 500);

	for (size_t i = 0, count = gpio_spy_edges(&edges); i < count; i++) {
		if (edges[i].pin != a->step_pin) {
			continue;
		}

		if (edges[i].level == 1) {
			rise = edges[i].tick;
			continue;
		}

		zassert_equal(edges[i].tick - rise, MIN_DELAY, "pulse width");
	}
}

ZTEST(zstep_timing, test_motors_do_not_disturb_each_other)
{
	static uint32_t alone[MAX_TICKS];
	struct zstep_profile pa = profile_of(&slow);
	struct zstep_profile pc = profile_of(&fast);

	zassert_ok(stepper_ctrl_configure_ramp(b->dev, &quick));
	zassert_ok(stepper_ctrl_move_by(b->dev, 800));
	wait_for(b, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
	zassert_equal(step_ticks(b, alone), 800);

	gpio_spy_reset();
	zassert_ok(stepper_ctrl_configure_ramp(a->dev, &slow));
	zassert_ok(stepper_ctrl_configure_ramp(c->dev, &fast));
	zassert_ok(stepper_ctrl_move_by(a->dev, 600));
	zassert_ok(stepper_ctrl_move_by(c->dev, -3000));
	zassert_ok(stepper_ctrl_move_by(b->dev, 800));
	wait_for(a, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
	wait_for(b, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
	wait_for(c, STEPPER_CTRL_EVENT_STEPS_COMPLETED);

	zassert_equal(position(a), 600);
	zassert_equal(position(b), 1600);
	zassert_equal(position(c), -3000);

	zassert_equal(step_ticks(b, got[1]), 800);
	for (int k = 0; k < 800; k++) {
		zassert_equal(got[1][k] - got[1][0], alone[k] - alone[0], "step %d moved", k);
	}

	zassert_equal(step_ticks(a, got[0]), 600);
	check_move(a, got[0], &pa, 600);
	zassert_equal(step_ticks(c, got[2]), 3000);
	check_move(c, got[2], &pc, 3000);
}

/* Checks DIR setup and hold around every direction change, returns how many there were. */
static int check_dir_timing(const struct motor *m, int dir_level, int32_t net)
{
	const struct gpio_spy_edge *edges;
	uint32_t last_step = 0;
	uint32_t last_dir = 0;
	int reversals = 0;

	for (size_t i = 0, count = gpio_spy_edges(&edges); i < count; i++) {
		if (edges[i].pin == m->dir_pin) {
			zassert_true(edges[i].tick - last_step >= MIN_DELAY, "DIR hold violated");
			dir_level = edges[i].level;
			last_dir = edges[i].tick;
			reversals++;
		}

		if (edges[i].pin == m->step_pin && edges[i].level == 1) {
			zassert_true(reversals == 0 || edges[i].tick - last_dir >= MIN_DELAY,
				     "DIR setup violated");
			net -= dir_level != m->inverted ? 1 : -1;
			last_step = edges[i].tick;
		}
	}

	zassert_equal(net, 0, "%s: steps do not add up to the move", m->dev->name);
	return reversals;
}

ZTEST(zstep_timing, test_reversal_respects_dir_timing)
{
	int dir_level = gpio_pin_get_raw(spy, a->dir_pin);

	zassert_ok(stepper_ctrl_configure_ramp(a->dev, &slow));
	zassert_ok(stepper_ctrl_move_to(a->dev, 300));
	k_msleep(150);
	zassert_ok(stepper_ctrl_move_to(a->dev, -200));
	wait_for(a, STEPPER_CTRL_EVENT_STEPS_COMPLETED);

	zassert_equal(position(a), -200);
	zassert_equal(check_dir_timing(a, dir_level, -200), 1);
}

ZTEST(zstep_timing, test_stop_decelerates)
{
	uint32_t stop_at = 0;

	zassert_ok(stepper_ctrl_configure_ramp(a->dev, &slow));
	zassert_ok(stepper_ctrl_run(a->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(400);

	zassert_ok(counter_get_value(counter, &stop_at));
	zassert_ok(stepper_ctrl_stop(a->dev));
	wait_for(a, STEPPER_CTRL_EVENT_STOPPED);

	size_t n = step_ticks(a, got[0]);
	size_t braking_from = steps_up_to(got[0], n, stop_at);

	zassert_equal(position(a), n);
	zassert_within(n - braking_from, 200, 3, "braked over %zu steps", n - braking_from);
	for (size_t k = braking_from + 2; k < n; k++) {
		zassert_true(got[0][k] - got[0][k - 1] + 1 >= got[0][k - 1] - got[0][k - 2],
			     "sped up while braking at step %zu", k);
	}
}

ZTEST(zstep_timing, test_dual_edge_steps_on_both_edges)
{
	struct zstep_profile p = profile_of(&slow);

	zassert_ok(stepper_ctrl_configure_ramp(c->dev, &slow));
	zassert_ok(stepper_ctrl_move_by(c->dev, 101));
	wait_for(c, STEPPER_CTRL_EVENT_STEPS_COMPLETED);

	zassert_equal(position(c), 101);
	zassert_equal(step_ticks(c, got[2]), 101);
	check_move(c, got[2], &p, 101);
}

ZTEST(zstep_timing, test_group_moves_along_a_line)
{
	const struct device *devs[] = {a->dev, b->dev, c->dev};
	const int32_t steps[] = {3000, -1000, 500};
	struct zstep_profile p = profile_of(&fast);
	struct zstep_profile pb = p;
	struct zstep_profile pc = p;

	zassert_ok(zstep_group_move_by(devs, steps, ARRAY_SIZE(devs), &fast));
	ARRAY_FOR_EACH(devs, i) {
		wait_for(&motors[i], STEPPER_CTRL_EVENT_STEPS_COMPLETED);
		zassert_equal(position(&motors[i]), steps[i]);
		zassert_equal(step_ticks(&motors[i], got[i]), abs(steps[i]));
	}

	zassert_equal(got[0][0], got[1][0], "axes started apart");
	zassert_equal(got[0][0], got[2][0], "axes started apart");

	zstep_profile_scale(&pb, 3000, 1000, ZSTEP_RAMP_INTERVAL_MAX);
	zstep_profile_scale(&pc, 3000, 500, ZSTEP_RAMP_INTERVAL_MAX);
	check_move(a, got[0], &p, 3000);
	check_move(b, got[1], &pb, 1000);
	check_move(c, got[2], &pc, 500);

	for (uint32_t k = 0; k < 3000; k++) {
		double expected_b = k / 3.0 + 1;
		double expected_c = k / 6.0 + 1;

		zassert_within(steps_up_to(got[1], 1000, got[0][k]), expected_b, 3.0);
		zassert_within(steps_up_to(got[2], 500, got[0][k]), expected_c, 3.0);
	}
}

ZTEST(zstep_timing, test_group_rejects_bad_requests)
{
	const struct device *same[] = {a->dev, a->dev};
	const struct device *foreign[] = {a->dev, counter};
	const struct device *pair[] = {a->dev, b->dev};
	const int32_t steps[] = {10, 10};

	zassert_equal(zstep_group_move_by(pair, steps, 0, &slow), -EINVAL);
	zassert_equal(zstep_group_move_by(same, steps, 2, &slow), -EINVAL);
	zassert_equal(zstep_group_move_by(foreign, steps, 2, &slow), -ENOTSUP);

	zassert_ok(stepper_ctrl_configure_ramp(b->dev, &slow));
	zassert_ok(stepper_ctrl_run(b->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	zassert_equal(zstep_group_move_by(pair, steps, 2, &slow), -EBUSY);
	zassert_equal(position(a), 0);
}

ZTEST(zstep_timing, test_moves_chained_from_callback)
{
	const struct stepper_ctrl_ramp constant = {.speed_max = 1000};

	zassert_ok(stepper_ctrl_configure_ramp(a->dev, &constant));
	a->chain = 5;
	zassert_ok(stepper_ctrl_move_by(a->dev, 10));
	wait_for(a, STEPPER_CTRL_EVENT_STEPS_COMPLETED);

	zassert_equal(a->chain_err, 0);
	zassert_equal(position(a), 60);
	zassert_equal(step_ticks(a, got[0]), 60);
}

ZTEST(zstep_timing, test_rejects_bad_limits)
{
	struct stepper_ctrl_ramp bad = slow;

	bad.speed_max = 0;
	zassert_equal(stepper_ctrl_configure_ramp(a->dev, &bad), -EINVAL);

	bad.speed_max = 400000;
	zassert_equal(stepper_ctrl_configure_ramp(a->dev, &bad), -EINVAL, "shorter than pulse");
	zassert_equal(stepper_ctrl_set_microstep_interval(a->dev, 4000), -EINVAL);
	zassert_ok(stepper_ctrl_set_microstep_interval(a->dev, 2 * MIN_DELAY * 1000));

	zassert_ok(stepper_ctrl_configure_ramp(a->dev, &slow));
	zassert_ok(stepper_ctrl_run(a->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	zassert_equal(stepper_ctrl_configure_ramp(a->dev, &quick), -EBUSY);
	zassert_ok(stepper_ctrl_set_microstep_interval(a->dev, 2 * NSEC_PER_MSEC));
}

static struct zstep_shaper shaper_of(enum zstep_shaper_type type)
{
	struct zstep_shaper s;

	zassert_ok(zstep_shaper_init(&s, type, 4000, 100, FREQ));
	return s;
}

/* Physical step times of a shaped n step move, relative to its first commanded step. */
static size_t shaped_model(const struct zstep_profile *p, const struct zstep_shaper *s, uint32_t n,
			   uint32_t *out, uint32_t *command_end)
{
	static struct zstep_echo_step buffer[2048];
	struct zstep_echo echo = {
		.shaper = *s,
		.buf = buffer,
		.size = ARRAY_SIZE(buffer),
		.top = UINT32_MAX,
	};
	struct zstep_ramp r;
	uint32_t made = 0;
	uint32_t t = 0;
	size_t count = 0;

	zstep_ramp_reset(&r, p);
	zstep_echo_reset(&echo, 0);
	for (;;) {
		uint32_t at;
		int8_t dir;

		switch (zstep_echo_next(&echo, &at, &dir)) {
		case ZSTEP_ECHO_STEP:
			out[count++] = at;
			break;
		case ZSTEP_ECHO_NEED_COMMAND:
			if (made == n) {
				echo.closed = true;
				break;
			}
			zassert_ok(zstep_echo_push(&echo, t, 1));
			*command_end = t;
			made++;
			t += zstep_ramp_next(&r, n - made);
			break;
		case ZSTEP_ECHO_DONE:
			return count;
		}
	}
}

static void check_pulses(const struct motor *m)
{
	const struct gpio_spy_edge *edges;
	uint32_t rise = 0;
	bool first = true;

	for (size_t i = 0, count = gpio_spy_edges(&edges); i < count; i++) {
		if (edges[i].pin != m->step_pin) {
			continue;
		}

		if (edges[i].level == 0) {
			zassert_equal(edges[i].tick - rise, MIN_DELAY, "pulse width");
			continue;
		}

		zassert_true(first || edges[i].tick - rise >= 2 * MIN_DELAY, "steps too close");
		rise = edges[i].tick;
		first = false;
	}
}

ZTEST(zstep_timing, test_shaped_motors_follow_model_and_keep_others_intact)
{
	static uint32_t alone[MAX_TICKS];
	static uint32_t model[MAX_TICKS];
	struct motor *shaped[] = {d, e};
	const enum zstep_shaper_type types[] = {ZSTEP_SHAPER_ZVD, ZSTEP_SHAPER_EI};
	struct zstep_profile p = profile_of(&fast);

	zassert_ok(stepper_ctrl_configure_ramp(b->dev, &quick));
	zassert_ok(stepper_ctrl_move_by(b->dev, 800));
	wait_for(b, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
	zassert_equal(step_ticks(b, alone), 800);

	gpio_spy_reset();
	ARRAY_FOR_EACH(shaped, i) {
		zassert_ok(stepper_ctrl_configure_ramp(shaped[i]->dev, &fast));
		zassert_ok(stepper_ctrl_move_by(shaped[i]->dev, 3000));
	}
	zassert_ok(stepper_ctrl_move_by(b->dev, 800));

	wait_for(b, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
	zassert_equal(step_ticks(b, got[1]), 800);
	for (int k = 0; k < 800; k++) {
		zassert_equal(got[1][k] - got[1][0], alone[k] - alone[0], "step %d moved", k);
	}

	ARRAY_FOR_EACH(shaped, i) {
		struct zstep_shaper s = shaper_of(types[i]);
		uint32_t command_end = 0;
		size_t n = shaped_model(&p, &s, 3000, model, &command_end);

		wait_for(shaped[i], STEPPER_CTRL_EVENT_STEPS_COMPLETED);
		zassert_equal(position(shaped[i]), 3000);
		zassert_equal(n, 3000);
		zassert_equal(step_ticks(shaped[i], got[3 + i]), 3000);
		zassert_equal(model[n - 1], command_end + s.delay[s.impulses - 1],
			      "the last step lags the command by the shaper duration");
		for (size_t k = 0; k < n; k++) {
			zassert_equal(got[3 + i][k] - got[3 + i][0], model[k] - model[0],
				      "%s: step %zu at %u, expected %u", shaped[i]->dev->name, k,
				      got[3 + i][k] - got[3 + i][0], model[k] - model[0]);
		}

		check_pulses(shaped[i]);
	}
}

ZTEST(zstep_timing, test_shaped_reversal)
{
	struct motor *shaped[] = {d, e};

	ARRAY_FOR_EACH(shaped, i) {
		struct motor *m = shaped[i];
		int dir_level = gpio_pin_get_raw(spy, m->dir_pin);

		gpio_spy_reset();
		zassert_ok(stepper_ctrl_configure_ramp(m->dev, &fast));
		zassert_ok(stepper_ctrl_move_to(m->dev, 1500));
		k_msleep(60);
		zassert_ok(stepper_ctrl_move_to(m->dev, -500));
		wait_for(m, STEPPER_CTRL_EVENT_STEPS_COMPLETED);

		zassert_equal(position(m), -500);
		zassert_equal(check_dir_timing(m, dir_level, -500), 1, "%s dithers", m->dev->name);
		check_pulses(m);
	}
}

ZTEST(zstep_timing, test_shaped_stop_lets_motion_settle)
{
	struct zstep_shaper s = shaper_of(ZSTEP_SHAPER_EI);
	uint32_t stop_at = 0;

	zassert_ok(stepper_ctrl_configure_ramp(e->dev, &fast));
	zassert_ok(stepper_ctrl_run(e->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(300);
	zassert_ok(counter_get_value(counter, &stop_at));
	zassert_ok(stepper_ctrl_stop(e->dev));
	wait_for(e, STEPPER_CTRL_EVENT_STOPPED);

	size_t n = step_ticks(e, got[4]);

	zassert_equal(position(e), n);
	zassert_true(got[4][n - 1] - stop_at > s.delay[s.impulses - 1],
		     "the shaped motion ended before the shaper settled");
}

ZTEST(zstep_timing, test_shaped_halt_without_ramp)
{
	const struct stepper_ctrl_ramp constant = {.speed_max = 2000};

	zassert_ok(stepper_ctrl_configure_ramp(d->dev, &constant));
	zassert_ok(stepper_ctrl_run(d->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(50);
	zassert_ok(stepper_ctrl_stop(d->dev));
	zassert_ok(k_sem_take(&d->done, K_NO_WAIT), "halt reports STOPPED right away");
	zassert_equal(d->event, STEPPER_CTRL_EVENT_STOPPED);

	bool moving;

	zassert_ok(stepper_ctrl_is_moving(d->dev, &moving));
	zassert_false(moving);
	zassert_equal(position(d), -(int32_t)step_ticks(d, got[3]));
}

ZTEST(zstep_timing, test_set_input_shaper)
{
	struct stepper_ctrl_ramp too_fast = fast;

	zassert_equal(zstep_set_input_shaper(a->dev, ZSTEP_SHAPER_ZVD, 4000, 100), -ENOTSUP,
		      "no shaper buffer");
	zassert_ok(zstep_set_input_shaper(a->dev, ZSTEP_SHAPER_NONE, 0, 0));
	zassert_equal(zstep_set_input_shaper(d->dev, ZSTEP_SHAPER_EI, 0, 100), -EINVAL);

	zassert_ok(stepper_ctrl_configure_ramp(d->dev, &fast));
	zassert_equal(zstep_set_input_shaper(d->dev, ZSTEP_SHAPER_EI, 1000, 100), -EINVAL,
		      "10 Hz EI does not fit the buffer at full speed");
	zassert_ok(zstep_set_input_shaper(d->dev, ZSTEP_SHAPER_EI, 4000, 100));

	too_fast.speed_max = 60000;
	zassert_equal(stepper_ctrl_configure_ramp(d->dev, &too_fast), -EINVAL);

	zassert_ok(stepper_ctrl_run(d->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	zassert_equal(zstep_set_input_shaper(d->dev, ZSTEP_SHAPER_ZVD, 4000, 100), -EBUSY);
	zassert_ok(stepper_ctrl_stop(d->dev));
	wait_for(d, STEPPER_CTRL_EVENT_STOPPED);

	zassert_ok(zstep_set_input_shaper(d->dev, ZSTEP_SHAPER_ZVD, 4000, 100));
}

static void *setup(void)
{
	ARRAY_FOR_EACH(motors, i) {
		zassert_true(device_is_ready(motors[i].dev));
		k_sem_init(&motors[i].done, 0, 1);
		zassert_ok(stepper_ctrl_set_event_cb(motors[i].dev, on_event, &motors[i]));
	}

	return NULL;
}

static void before(void *fixture)
{
	ARRAY_FOR_EACH(motors, i) {
		k_sem_reset(&motors[i].done);
		motors[i].chain = 0;
		motors[i].chain_err = 0;
		zassert_ok(stepper_ctrl_set_reference_position(motors[i].dev, 0));
	}

	gpio_spy_reset();
}

static void after(void *fixture)
{
	ARRAY_FOR_EACH(motors, i) {
		bool moving = true;

		zassert_ok(stepper_ctrl_stop(motors[i].dev));
		for (int tries = 0; moving && tries < 500; tries++) {
			zassert_ok(stepper_ctrl_is_moving(motors[i].dev, &moving));
			k_msleep(10);
		}
		zassert_false(moving, "%s did not stop", motors[i].dev->name);
	}
}

ZTEST_SUITE(zstep_timing, NULL, setup, before, after, NULL);
