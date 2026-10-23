/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @file
 * @brief Synchronized group moves of zstep,step-dir motors.
 */

#ifndef ZSTEP_INCLUDE_ZSTEP_ZSTEP_H_
#define ZSTEP_INCLUDE_ZSTEP_ZSTEP_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Move several axes along a straight line, starting them on the same counter tick.
 *
 * The axis with the longest move follows @p ramp, the others are slowed down in proportion
 * to their share of the move and report STEPPER_CTRL_EVENT_STEPS_COMPLETED on their own.
 *
 * @param devs zstep,step-dir devices sharing one counter.
 * @param steps Relative move of each axis in micro-steps.
 * @param count Number of axes, at most CONFIG_ZSTEP_GROUP_MAX_AXES.
 * @param ramp Motion limits of the lead axis.
 *
 * @retval 0 Success.
 * @retval -EINVAL Invalid arguments or limits, or the axes use different counters.
 * @retval -ENOTSUP A device is not a zstep,step-dir device.
 * @retval -EBUSY An axis is moving.
 * @retval -ERANGE A target position does not fit into int32_t.
 */
int zstep_group_move_by(const struct device *const devs[], const int32_t steps[], size_t count,
			const struct stepper_ctrl_ramp *ramp);

#ifdef __cplusplus
}
#endif

#endif /* ZSTEP_INCLUDE_ZSTEP_ZSTEP_H_ */
