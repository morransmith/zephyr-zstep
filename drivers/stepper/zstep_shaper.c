/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Input shapers applied by merging delayed and weighted copies of the commanded step
 * stream.
 */

#include <errno.h>
#include <math.h>

#include "zstep_shaper.h"

#define PI 3.14159265f

/* Residual vibration the extra-insensitive shaper tolerates at its design frequency. */
#define EI_TOLERANCE 0.05f

/* Upper end of the damping range of the EI surface fit. */
#define EI_DAMPING_MAX 0.3f

/* Commanded steps buffered beyond the shaper duration while the first impulse looks ahead. */
#define LOOKAHEAD_STEPS 8

struct impulses {
	uint8_t n;
	/* Amplitudes, normalized later. */
	float a[ZSTEP_SHAPER_IMPULSES_MAX];
	/* Delays in damped periods. */
	float t[ZSTEP_SHAPER_IMPULSES_MAX];
};

/*
 * Damped extra-insensitive shaper, surface fit of eq. 22 in Convolve Inc., "Minimizing
 * structural vibrations with input shaping", NASA CR-189439, 1995.
 */
static struct impulses ei_impulses(float zeta)
{
	const float v = EI_TOLERANCE;

	if (zeta > EI_DAMPING_MAX) {
		return (struct impulses){0};
	}

	float z2 = zeta * zeta;
	float z3 = z2 * zeta;
	float a1 = 0.2497f + 0.2496f * v + 0.8001f * zeta + 1.233f * v * zeta + 0.4960f * z2 +
		   3.173f * v * z2;
	float a3 = 0.2515f + 0.2147f * v - 0.8325f * zeta + 1.415f * v * zeta + 0.8518f * z2 -
		   4.901f * v * z2;
	float t2 = 0.5f + 0.4616f * v * zeta + 4.262f * v * z2 + 1.756f * v * z3 +
		   8.578f * v * v * zeta - 108.6f * v * v * z2 + 337.0f * v * v * z3;

	return (struct impulses){3, {a1, 1.0f - a1 - a3, a3}, {0.0f, t2, 1.0f}};
}

static struct impulses impulses_of(enum zstep_shaper_type type, float zeta)
{
	float df = sqrtf(1.0f - zeta * zeta);
	float k = expf(-zeta * PI / df);

	/*
	 * ZV and ZVD after N. C. Singer, W. P. Seering, "Preshaping command inputs to reduce
	 * system vibration", 1990.
	 */
	switch (type) {
	case ZSTEP_SHAPER_ZV:
		return (struct impulses){2, {1.0f, k}, {0.0f, 0.5f}};
	case ZSTEP_SHAPER_ZVD:
		return (struct impulses){3, {1.0f, 2.0f * k, k * k}, {0.0f, 0.5f, 1.0f}};
	case ZSTEP_SHAPER_EI:
		return ei_impulses(zeta);
	default:
		return (struct impulses){0};
	}
}

int zstep_shaper_init(struct zstep_shaper *s, enum zstep_shaper_type type, uint32_t freq_centihz,
		      uint32_t damping_permille, uint32_t counter_freq)
{
	float sum = 0.0f;
	uint32_t total = 0;

	*s = (struct zstep_shaper){
		.impulses = 1,
		.weight = {ZSTEP_SHAPER_UNIT},
	};
	if (type == ZSTEP_SHAPER_NONE) {
		return 0;
	}

	if (freq_centihz == 0 || damping_permille >= 1000) {
		return -EINVAL;
	}

	float zeta = damping_permille / 1000.0f;
	float period = 100.0f / (freq_centihz * sqrtf(1.0f - zeta * zeta));
	struct impulses imp = impulses_of(type, zeta);

	if (imp.n == 0) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < imp.n; i++) {
		sum += imp.a[i];
	}

	for (uint8_t i = 0; i < imp.n; i++) {
		s->weight[i] = lroundf(imp.a[i] / sum * ZSTEP_SHAPER_UNIT);
		s->delay[i] = lroundf(imp.t[i] * period * counter_freq);
		total += s->weight[i];
	}

	/* Rounding must not leave the shaped motion short of its target. */
	s->weight[0] += ZSTEP_SHAPER_UNIT - total;
	s->impulses = imp.n;
	return 0;
}

bool zstep_shaper_fits(const struct zstep_shaper *s, uint64_t cmin, uint32_t size)
{
	uint64_t span = (uint64_t)s->delay[s->impulses - 1] << 8;

	return span / cmin + LOOKAHEAD_STEPS < size;
}

void zstep_echo_reset(struct zstep_echo *e, uint32_t now)
{
	e->head = 0;
	e->ref = now;
	e->acc = 0;
	e->pos = 0;
	e->closed = false;
	for (uint8_t i = 0; i < ZSTEP_SHAPER_IMPULSES_MAX; i++) {
		e->cursor[i] = 0;
	}
}

static uint32_t next_index(const struct zstep_echo *e, uint32_t i)
{
	return i + 1 == e->size ? 0 : i + 1;
}

int zstep_echo_push(struct zstep_echo *e, uint32_t t, int8_t dir)
{
	uint32_t next = next_index(e, e->head);

	/* The most delayed impulse holds the oldest step still needed. */
	if (next == e->cursor[e->shaper.impulses - 1]) {
		return -ENOMEM;
	}

	e->buf[e->head] = (struct zstep_echo_step){.t = t, .dir = dir};
	e->head = next;
	return 0;
}

/* Returns the impulse whose next event comes first, or -1 if none is pending. */
static int earliest_impulse(const struct zstep_echo *e)
{
	uint32_t best = UINT32_MAX;
	int found = -1;

	for (uint8_t i = 0; i < e->shaper.impulses; i++) {
		if (e->cursor[i] == e->head) {
			continue;
		}

		uint32_t at = e->buf[e->cursor[i]].t + e->shaper.delay[i];
		uint32_t ahead = (at - e->ref) & e->top;

		if (ahead < best) {
			best = ahead;
			found = i;
		}
	}

	return found;
}

enum zstep_echo_result zstep_echo_next(struct zstep_echo *e, uint32_t *t, int8_t *dir)
{
	for (;;) {
		/* The undelayed impulse decides how far the commanded motion is known. */
		if (e->cursor[0] == e->head && !e->closed) {
			return ZSTEP_ECHO_NEED_COMMAND;
		}

		int i = earliest_impulse(e);

		if (i < 0) {
			return ZSTEP_ECHO_DONE;
		}

		const struct zstep_echo_step *step = &e->buf[e->cursor[i]];

		e->ref = (step->t + e->shaper.delay[i]) & e->top;
		e->acc += (int64_t)e->shaper.weight[i] * step->dir;
		e->cursor[i] = next_index(e, e->cursor[i]);

		/* A whole step of hysteresis keeps the output from dithering at reversals. */
		if (e->acc >= (int64_t)(e->pos + 1) * ZSTEP_SHAPER_UNIT) {
			e->pos++;
			*dir = 1;
			*t = e->ref;
			return ZSTEP_ECHO_STEP;
		}

		if (e->acc <= (int64_t)(e->pos - 1) * ZSTEP_SHAPER_UNIT) {
			e->pos--;
			*dir = -1;
			*t = e->ref;
			return ZSTEP_ECHO_STEP;
		}
	}
}
