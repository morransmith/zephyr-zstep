/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZSTEP_DRIVERS_STEPPER_ZSTEP_RAMP_H_
#define ZSTEP_DRIVERS_STEPPER_ZSTEP_RAMP_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

/** Step intervals are fixed point numbers of counter ticks with this many fraction bits. */
#define ZSTEP_RAMP_Q 8

/** Longest interval the integer math can hold. */
#define ZSTEP_RAMP_INTERVAL_MAX BIT(30)

/** Motion limits converted to step intervals. */
struct zstep_profile {
	/** First interval when starting from rest, 0 disables the ramp. */
	uint32_t c0;
	/** Last interval before coming to rest. */
	uint32_t c0d;
	/** Cruise interval. */
	uint32_t cmin;
	/** Acceleration [steps/s^2], only its ratio to decel is used. */
	uint32_t accel;
	/** Deceleration [steps/s^2]. */
	uint32_t decel;
};

/** Ramp state of one motor. */
struct zstep_ramp {
	struct zstep_profile p;
	/** Current interval, 0 at rest. */
	uint32_t c;
	/** Steps needed to stop, counted at the current rate. */
	uint32_t n;
	/** Remainder of the last recurrence division. */
	uint32_t rest;
	/** Sub-tick part carried over to the next interval. */
	uint32_t frac;
	/** True if n counts in deceleration units. */
	bool braking;
};

/**
 * @brief Build a profile from a cruise interval and acceleration limits.
 *
 * @param p Profile to fill.
 * @param freq Counter frequency [Hz].
 * @param cmin Cruise interval, see zstep_interval_from_speed().
 * @param accel Acceleration [steps/s^2], 0 disables the ramp.
 * @param decel Deceleration [steps/s^2], 0 means equal to @p accel.
 *
 * @retval 0 Success.
 * @retval -EINVAL The limits do not fit the integer math.
 */
int zstep_profile_init(struct zstep_profile *p, uint32_t freq, uint64_t cmin, uint32_t accel,
		       uint32_t decel);

/**
 * @brief Slow a profile down for an axis moving @p steps while the lead axis moves @p lead.
 *
 * Axes scaled this way stay proportional over time and so move along a straight line.
 */
void zstep_profile_scale(struct zstep_profile *p, uint32_t lead, uint32_t steps, uint32_t max);

/** Interval of a step rate [steps/s], 0 if the rate is 0. */
uint64_t zstep_interval_from_speed(uint32_t freq, uint32_t speed);

/** Interval of a step period [ns], UINT64_MAX if it overflows. */
uint64_t zstep_interval_from_ns(uint32_t freq, uint64_t ns);

/** Put the ramp at rest with a new profile. */
void zstep_ramp_reset(struct zstep_ramp *r, const struct zstep_profile *p);

/**
 * @brief Advance the ramp by the step that has just been made.
 *
 * @param r Ramp state.
 * @param togo Steps still allowed in the current direction, 0 to stop as soon as possible.
 *
 * @return Ticks until the next step, or 0 if the motor is at rest now.
 */
uint32_t zstep_ramp_next(struct zstep_ramp *r, uint32_t togo);

/** True if the motor may stop before the next step without violating the ramp. */
static inline bool zstep_ramp_can_stop(const struct zstep_ramp *r)
{
	return r->p.c0 == 0 || r->c == 0;
}

/** Ticks to wait at rest before the first step of a new motion. */
static inline uint32_t zstep_ramp_pause(const struct zstep_profile *p)
{
	return MAX(p->c0, p->cmin) >> ZSTEP_RAMP_Q;
}

#endif /* ZSTEP_DRIVERS_STEPPER_ZSTEP_RAMP_H_ */
