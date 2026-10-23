/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Real-time trapezoidal ramp after D. Austin, "Generate stepper-motor speed profiles in
 * real time", with a look-ahead stop rule so every move ends at rest on its target step.
 */

#include <errno.h>

#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/clock.h>

#include "zstep_ramp.h"

/* Second interval relative to the first that makes the recurrence match t = sqrt(2n/a). */
#define C1_RATIO_Q16 26581

/* Keeps 4n + 1 and 2c + rest within 32 bits. */
#define N_MAX BIT(29)

static uint64_t isqrt64(uint64_t x)
{
	uint64_t root = 0;
	uint64_t bit = BIT64(62);

	while (bit > x) {
		bit >>= 2;
	}

	while (bit != 0) {
		if (x >= root + bit) {
			x -= root + bit;
			root = (root >> 1) + bit;
		} else {
			root >>= 1;
		}
		bit >>= 2;
	}

	return root;
}

/* Exact time of the first step from rest, f * sqrt(2 / a). */
static uint64_t first_interval(uint32_t freq, uint32_t accel)
{
	return ((uint64_t)freq * isqrt64(BIT64(49) / accel)) >> (24 - ZSTEP_RAMP_Q);
}

int zstep_profile_init(struct zstep_profile *p, uint32_t freq, uint64_t cmin, uint32_t accel,
		       uint32_t decel)
{
	if (cmin < BIT(ZSTEP_RAMP_Q) || cmin > ZSTEP_RAMP_INTERVAL_MAX) {
		return -EINVAL;
	}

	*p = (struct zstep_profile){.cmin = cmin};
	if (accel == 0) {
		return 0;
	}

	decel = decel != 0 ? decel : accel;

	uint64_t speed = ((uint64_t)freq << ZSTEP_RAMP_Q) / cmin;
	uint64_t c0 = first_interval(freq, accel);
	uint64_t c0d = first_interval(freq, decel);

	if (speed * speed / (2ULL * MIN(accel, decel)) >= N_MAX) {
		return -EINVAL;
	}

	if (MAX(c0, c0d) > ZSTEP_RAMP_INTERVAL_MAX) {
		return -EINVAL;
	}

	p->c0 = c0;
	p->c0d = c0d;
	p->accel = accel;
	p->decel = decel;
	return 0;
}

void zstep_profile_scale(struct zstep_profile *p, uint32_t lead, uint32_t steps, uint32_t max)
{
	uint64_t sqrt_ratio = isqrt64(((uint64_t)lead << 32) / steps);

	p->c0 = MIN((p->c0 * sqrt_ratio) >> 16, max);
	p->c0d = MIN((p->c0d * sqrt_ratio) >> 16, max);
	p->cmin = MIN((uint64_t)p->cmin * lead / steps, max);
}

uint64_t zstep_interval_from_speed(uint32_t freq, uint32_t speed)
{
	if (speed == 0) {
		return 0;
	}

	return ((uint64_t)freq << ZSTEP_RAMP_Q) / speed;
}

uint64_t zstep_interval_from_ns(uint32_t freq, uint64_t ns)
{
	uint64_t ticks_ns;

	if (u64_mul_overflow(ns, freq, &ticks_ns)) {
		return UINT64_MAX;
	}

	return ticks_ns / (NSEC_PER_SEC >> ZSTEP_RAMP_Q);
}

void zstep_ramp_reset(struct zstep_ramp *r, const struct zstep_profile *p)
{
	*r = (struct zstep_ramp){.p = *p};
}

static uint32_t emit_ticks(struct zstep_ramp *r)
{
	uint32_t t = r->c + r->frac;

	r->frac = t & BIT_MASK(ZSTEP_RAMP_Q);
	return t >> ZSTEP_RAMP_Q;
}

static void accelerate(struct zstep_ramp *r, uint64_t energy)
{
	if (r->braking) {
		r->n = energy / r->p.accel;
		r->rest = 0;
		r->braking = false;
	}

	r->n++;
	if (r->n == 1) {
		r->c = MAX(((uint64_t)r->c * C1_RATIO_Q16) >> 16, r->p.cmin);
		return;
	}

	uint32_t den = 4 * r->n + 1;
	uint32_t x = 2 * r->c + r->rest;

	r->c = MAX(r->c - x / den, r->p.cmin);
	r->rest = x % den;
}

/* Returns false if the motor is slow enough to stop right away. */
static bool brake(struct zstep_ramp *r, uint64_t energy)
{
	if (!r->braking) {
		r->n = DIV_ROUND_UP(energy, r->p.decel);
		r->rest = 0;
		r->braking = true;
	}

	if (r->n == 0) {
		return false;
	}

	if (r->n == 1) {
		r->n = 0;
		r->c = MAX(r->c, r->p.c0d);
		return true;
	}

	uint32_t den = 4 * r->n - 1;
	uint32_t x = 2 * r->c + r->rest;

	r->c += x / den;
	r->rest = x % den;
	r->n--;
	return true;
}

uint32_t zstep_ramp_next(struct zstep_ramp *r, uint32_t togo)
{
	const struct zstep_profile *p = &r->p;

	if (togo == 0 && zstep_ramp_can_stop(r)) {
		r->c = 0;
		return 0;
	}

	if (p->c0 == 0) {
		r->c = p->cmin;
		return emit_ticks(r);
	}

	if (r->c == 0) {
		r->c = MAX(p->c0, p->cmin);
		return emit_ticks(r);
	}

	/* Both sides are proportional to v^2, so this compares the stopping distance. */
	uint64_t energy = (uint64_t)r->n * (r->braking ? p->decel : p->accel);
	uint64_t budget = togo > 0 ? (uint64_t)(togo - 1) * p->decel : 0;

	if (togo == 0 || energy > budget) {
		if (!brake(r, energy)) {
			r->c = 0;
			return 0;
		}
	} else if (r->c < p->cmin) {
		if (!brake(r, energy) || r->c > p->cmin) {
			r->c = p->cmin;
		}
	} else if (r->c > p->cmin && energy + p->accel <= budget) {
		accelerate(r, energy);
	}

	return emit_ticks(r);
}
