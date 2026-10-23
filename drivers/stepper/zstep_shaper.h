/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZSTEP_DRIVERS_STEPPER_ZSTEP_SHAPER_H_
#define ZSTEP_DRIVERS_STEPPER_ZSTEP_SHAPER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zstep/zstep.h>

#define ZSTEP_SHAPER_IMPULSES_MAX 3

/** Impulse weights are fixed point fractions of one step. */
#define ZSTEP_SHAPER_UNIT (1 << 16)

/** Impulse sequence the commanded motion is convolved with. */
struct zstep_shaper {
	uint8_t impulses;
	/** Weights summing up to ZSTEP_SHAPER_UNIT. */
	uint32_t weight[ZSTEP_SHAPER_IMPULSES_MAX];
	/** Delays in counter ticks, ascending from 0. */
	uint32_t delay[ZSTEP_SHAPER_IMPULSES_MAX];
};

/** Commanded step kept for the delayed impulses. */
struct zstep_echo_step {
	uint32_t t;
	int8_t dir;
};

enum zstep_echo_result {
	/** A physical step is due. */
	ZSTEP_ECHO_STEP,
	/** The next commanded step must be pushed first. */
	ZSTEP_ECHO_NEED_COMMAND,
	/** The shaped motion has ended. */
	ZSTEP_ECHO_DONE,
};

/** Shaped motion of one motor: commanded steps replayed by every impulse. */
struct zstep_echo {
	struct zstep_shaper shaper;
	struct zstep_echo_step *buf;
	uint32_t size;
	uint32_t top;
	uint32_t head;
	uint32_t cursor[ZSTEP_SHAPER_IMPULSES_MAX];
	/** Time of the last merged event, all pending events lie after it. */
	uint32_t ref;
	/** Shaped position in ZSTEP_SHAPER_UNIT steps. */
	int64_t acc;
	/** Position already turned into physical steps. */
	int32_t pos;
	/** True once no more commanded steps follow. */
	bool closed;
};

/**
 * @brief Compute the impulses of an input shaper.
 *
 * @param s Shaper to fill.
 * @param type Shaper type, ZSTEP_SHAPER_NONE gives a single impulse.
 * @param freq_centihz Resonance frequency [0.01 Hz].
 * @param damping_permille Damping ratio [0.001].
 * @param counter_freq Counter frequency [Hz].
 *
 * @retval 0 Success.
 * @retval -EINVAL Unknown type or out of range parameters.
 */
int zstep_shaper_init(struct zstep_shaper *s, enum zstep_shaper_type type, uint32_t freq_centihz,
		      uint32_t damping_permille, uint32_t counter_freq);

/** True if @p size buffered steps cover the shaper at the cruise interval @p cmin. */
bool zstep_shaper_fits(const struct zstep_shaper *s, uint64_t cmin, uint32_t size);

/** Start an empty shaped motion at counter time @p now. */
void zstep_echo_reset(struct zstep_echo *e, uint32_t now);

/** Add a commanded step, -ENOMEM if the buffer is full. */
int zstep_echo_push(struct zstep_echo *e, uint32_t t, int8_t dir);

/** Find the next physical step, see enum zstep_echo_result. */
enum zstep_echo_result zstep_echo_next(struct zstep_echo *e, uint32_t *t, int8_t *dir);

#endif /* ZSTEP_DRIVERS_STEPPER_ZSTEP_SHAPER_H_ */
