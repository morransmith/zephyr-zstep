/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @file
 * @brief Synchronized group moves and input shaping of zstep,step-dir motors.
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

/** Input shapers, in the order of the input-shaper devicetree property. */
enum zstep_shaper_type {
	/** No shaping. */
	ZSTEP_SHAPER_NONE,
	/** Zero vibration: 2 impulses over half a period. */
	ZSTEP_SHAPER_ZV,
	/** Zero vibration and derivative: 3 impulses over one period. */
	ZSTEP_SHAPER_ZVD,
	/** Extra-insensitive: 3 impulses over one period, damping ratio up to 0.3. */
	ZSTEP_SHAPER_EI,
};

/**
 * @brief Select the input shaper that cancels a resonance of the axis.
 *
 * The physical motion follows the commanded one delayed by the shaper duration, so events
 * and is_moving report the end of the physical motion.
 *
 * @param dev zstep,step-dir device with an input-shaper-buffer-steps property.
 * @param type Shaper type, ZSTEP_SHAPER_NONE turns shaping off.
 * @param freq_centihz Resonance frequency [0.01 Hz].
 * @param damping_permille Damping ratio [0.001], 100 suits most machines, at most 300 for EI.
 *
 * @retval 0 Success.
 * @retval -EINVAL Invalid parameters or the buffer is too short for the configured speed.
 * @retval -ENOTSUP The device has no shaper buffer.
 * @retval -EBUSY The motor is moving.
 */
int zstep_set_input_shaper(const struct device *dev, enum zstep_shaper_type type,
			   uint32_t freq_centihz, uint32_t damping_permille);

#ifdef __cplusplus
}
#endif

#endif /* ZSTEP_INCLUDE_ZSTEP_ZSTEP_H_ */
