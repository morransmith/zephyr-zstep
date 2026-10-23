/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zstep/zstep.h>

static const struct device *const axes[] = {
	DEVICE_DT_GET(DT_NODELABEL(x_axis)),
	DEVICE_DT_GET(DT_NODELABEL(y_axis)),
	DEVICE_DT_GET(DT_NODELABEL(z_axis)),
};

static const struct stepper_ctrl_ramp limits = {
	.acceleration_max = 20000,
	.speed_max = 8000,
	.deceleration_max = 20000,
};

static const int32_t path[][ARRAY_SIZE(axes)] = {
	{8000, 0, 0},        /* +X */
	{0, 8000, 0},        /* +Y */
	{-8000, 0, 0},       /* -X */
	{0, -8000, 1600},    /* -Y, Z up */
	{8000, 8000, -1600}, /* diagonal, Z down */
	{-8000, -8000, 0},   /* diagonal back to the origin */
};

static K_SEM_DEFINE(finished, 0, ARRAY_SIZE(axes));

static void on_event(const struct device *dev, enum stepper_ctrl_event event, void *user_data)
{
	if (event == STEPPER_CTRL_EVENT_STEPS_COMPLETED || event == STEPPER_CTRL_EVENT_STOPPED) {
		k_sem_give(&finished);
	}
}

static int wait_for_axes(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(axes); i++) {
		int err = k_sem_take(&finished, K_SECONDS(30));

		if (err != 0) {
			return err;
		}
	}

	return 0;
}

static void print_position(void)
{
	int32_t pos[ARRAY_SIZE(axes)];

	for (size_t i = 0; i < ARRAY_SIZE(axes); i++) {
		stepper_ctrl_get_actual_position(axes[i], &pos[i]);
	}

	printk("X %6d  Y %6d  Z %6d\n", pos[0], pos[1], pos[2]);
}

int main(void)
{
	int err;

	for (size_t i = 0; i < ARRAY_SIZE(axes); i++) {
		if (!device_is_ready(axes[i])) {
			printk("%s is not ready\n", axes[i]->name);
			return 0;
		}

		stepper_ctrl_set_event_cb(axes[i], on_event, NULL);
		stepper_ctrl_configure_ramp(axes[i], &limits);
	}

	for (size_t i = 0; i < ARRAY_SIZE(path); i++) {
		err = zstep_group_move_by(axes, path[i], ARRAY_SIZE(axes), &limits);
		if (err == 0) {
			err = wait_for_axes();
		}

		if (err != 0) {
			printk("segment %zu failed: %d\n", i, err);
			return 0;
		}

		print_position();
	}

	/* Each axis keeps its own timing, so Z can jog while X and Y move independently. */
	stepper_ctrl_move_by(axes[0], 16000);
	stepper_ctrl_move_by(axes[1], -4000);
	stepper_ctrl_run(axes[2], STEPPER_CTRL_DIRECTION_POSITIVE);
	k_msleep(500);
	stepper_ctrl_stop(axes[2]);

	err = wait_for_axes();
	print_position();
	printk("%s\n", err == 0 ? "done" : "timed out");
	return 0;
}
