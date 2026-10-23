/* SPDX-License-Identifier: Apache-2.0 */

#include <math.h>
#include <stdlib.h>

#include <zephyr/ztest.h>

#include "zstep_ramp.h"

#define FREQ      1000000U
#define MAX_STEPS 25000

static uint64_t times[MAX_STEPS];
static uint64_t times_minor[MAX_STEPS];

static struct zstep_profile profile(uint32_t speed, uint32_t accel, uint32_t decel)
{
	struct zstep_profile p;

	zassert_ok(
		zstep_profile_init(&p, FREQ, zstep_interval_from_speed(FREQ, speed), accel, decel));
	return p;
}

/* Moves n steps and returns how many were made; out[k] receives the time of step k. */
static uint32_t simulate(const struct zstep_profile *p, uint32_t n, uint64_t *out)
{
	struct zstep_ramp r;
	uint64_t t = 0;
	uint32_t steps = 0;

	zstep_ramp_reset(&r, p);
	while (steps < 2 * n + 10) {
		if (out != NULL && steps < MAX_STEPS) {
			out[steps] = t;
		}

		steps++;
		uint32_t interval = zstep_ramp_next(&r, steps < n ? n - steps : 0);

		if (interval == 0) {
			break;
		}
		t += interval;
	}

	zassert_true(zstep_ramp_can_stop(&r), "ramp not at rest after %u steps", steps);
	return steps;
}

/* Time in seconds at which a trapezoidal move over distance d reaches x. */
static double trapezoid_time(double accel, double decel, double speed, double d, double x)
{
	double xa = speed * speed / (2 * accel);
	double xd = speed * speed / (2 * decel);

	if (xa + xd > d) {
		speed = sqrt(2 * d * accel * decel / (accel + decel));
		xa = speed * speed / (2 * accel);
		xd = d - xa;
	}

	double end = speed / accel + (d - xa - xd) / speed + speed / decel;

	if (x <= xa) {
		return sqrt(2 * x / accel);
	}

	if (x <= d - xd) {
		return speed / accel + (x - xa) / speed;
	}

	return end - sqrt(2 * (d - x) / decel);
}

static void check_timing(uint32_t speed, uint32_t accel, uint32_t decel, uint32_t n)
{
	struct zstep_profile p = profile(speed, accel, decel);
	double tolerance = 0.05 * MAX(p.c0, p.c0d) / BIT(ZSTEP_RAMP_Q) + 4;

	zassert_equal(simulate(&p, n, times), n);
	for (uint32_t k = 0; k < n; k++) {
		double expected = trapezoid_time(accel, decel, speed, n - 1, k) * FREQ;

		zassert_within((double)times[k], expected, tolerance,
			       "v=%u a=%u d=%u n=%u: step %u at %llu, expected %.0f", speed, accel,
			       decel, n, k, times[k], expected);
	}
}

ZTEST(zstep_ramp, test_every_move_ends_on_target)
{
	const uint32_t limits[][3] = {
		{2000, 10000, 10000},    {2000, 20000, 5000},   {2000, 5000, 20000},
		{40000, 200000, 200000}, {500, 100000, 100000},
	};

	ARRAY_FOR_EACH(limits, i) {
		struct zstep_profile p = profile(limits[i][0], limits[i][1], limits[i][2]);

		for (uint32_t n = 1; n <= 2000; n++) {
			zassert_equal(simulate(&p, n, NULL), n, "limits %zu, %u steps", i, n);
		}
	}
}

ZTEST(zstep_ramp, test_follows_trapezoid)
{
	check_timing(2000, 10000, 10000, 2000);
	check_timing(2000, 20000, 5000, 2000);
	check_timing(2000, 5000, 20000, 2000);
	check_timing(40000, 200000, 200000, 20000);
	check_timing(5000, 10000, 10000, 300);
}

ZTEST(zstep_ramp, test_symmetric_profile)
{
	struct zstep_profile p = profile(3000, 15000, 15000);
	uint32_t n = 1000;

	zassert_equal(simulate(&p, n, times), n);
	for (uint32_t k = 1; k < n; k++) {
		int64_t up = times[k] - times[k - 1];
		int64_t down = times[n - k] - times[n - k - 1];

		zassert_true(llabs(up - down) <= 8, "interval %u: %lld vs %lld", k, up, down);
	}
}

ZTEST(zstep_ramp, test_cruise_rate_is_exact)
{
	struct zstep_profile p = profile(3000, 0, 0);
	uint32_t n = 100001;

	zassert_equal(simulate(&p, n, NULL), n);
	zassert_equal(p.c0, 0);

	/* 3000 steps/s on a 1 MHz counter is 333.33 ticks, the fraction must not be lost. */
	struct zstep_ramp r;
	uint64_t t = 0;

	zstep_ramp_reset(&r, &p);
	for (uint32_t k = 1; k < n; k++) {
		t += zstep_ramp_next(&r, n - k);
	}
	zassert_within(t, (uint64_t)(n - 1) * p.cmin >> ZSTEP_RAMP_Q, 1);
	zassert_within(t, 33333333, 33333333 / 100000, "rate off by more than 10 ppm");
}

ZTEST(zstep_ramp, test_stop_distance)
{
	struct zstep_profile p = profile(2000, 10000, 10000);
	struct zstep_ramp r;
	uint32_t braking = 0;

	zstep_ramp_reset(&r, &p);
	for (int k = 0; k < 1000; k++) {
		zstep_ramp_next(&r, UINT32_MAX);
	}

	while (zstep_ramp_next(&r, 0) != 0) {
		braking++;
	}

	/* v^2 / 2d = 200 steps */
	zassert_within(braking, 200, 2, "stopped after %u steps", braking);
}

ZTEST(zstep_ramp, test_speed_change_while_running)
{
	struct zstep_profile p = profile(4000, 20000, 20000);
	uint32_t fast = p.cmin >> ZSTEP_RAMP_Q;
	uint32_t slow = 2 * fast;
	struct zstep_ramp r;
	uint32_t interval = 0;

	zstep_ramp_reset(&r, &p);
	for (int k = 0; k < 2000; k++) {
		interval = zstep_ramp_next(&r, UINT32_MAX);
	}
	zassert_within(interval, fast, 1);

	r.p.cmin = slow << ZSTEP_RAMP_Q;
	for (int k = 0; k < 2000; k++) {
		uint32_t next = zstep_ramp_next(&r, UINT32_MAX);

		zassert_true(next <= interval * 11 / 10 + 1, "abrupt slow down %u -> %u", interval,
			     next);
		interval = next;
	}
	zassert_within(interval, slow, 1);

	r.p.cmin = fast << ZSTEP_RAMP_Q;
	for (int k = 0; k < 2000; k++) {
		interval = zstep_ramp_next(&r, UINT32_MAX);
	}
	zassert_within(interval, fast, 1);
}

ZTEST(zstep_ramp, test_reverse_after_overshoot)
{
	struct zstep_profile p = profile(2000, 10000, 10000);
	struct zstep_ramp r;
	uint32_t past = 0;

	zstep_ramp_reset(&r, &p);
	for (int k = 0; k < 1000; k++) {
		zstep_ramp_next(&r, UINT32_MAX);
	}

	/* The target moved 50 steps ahead, closer than the stopping distance. */
	for (uint32_t togo = 50; zstep_ramp_next(&r, togo) != 0; togo = togo > 0 ? togo - 1 : 0) {
		past++;
	}

	zassert_true(past > 50 && past < 210, "stopped %u steps later", past);
	zassert_true(zstep_ramp_can_stop(&r));
}

ZTEST(zstep_ramp, test_profile_limits)
{
	struct zstep_profile p;

	zassert_equal(zstep_profile_init(&p, FREQ, 0, 1000, 1000), -EINVAL);
	zassert_equal(zstep_profile_init(&p, FREQ, ZSTEP_RAMP_INTERVAL_MAX + 1, 0, 0), -EINVAL);
	zassert_equal(zstep_profile_init(&p, FREQ, zstep_interval_from_speed(FREQ, 400000), 1, 1),
		      -EINVAL, "ramp index overflows");
	zassert_equal(
		zstep_profile_init(&p, 10000000, zstep_interval_from_speed(10000000, 10), 1, 1),
		-EINVAL, "first interval overflows");

	zassert_ok(zstep_profile_init(&p, FREQ, zstep_interval_from_speed(FREQ, 1000), 0, 500));
	zassert_equal(p.c0, 0);

	zassert_ok(zstep_profile_init(&p, FREQ, zstep_interval_from_speed(FREQ, 1000), 500, 0));
	zassert_equal(p.decel, 500);
	zassert_equal(p.c0, p.c0d);

	zassert_equal(zstep_interval_from_speed(FREQ, 0), 0);
	zassert_equal(zstep_interval_from_ns(FREQ, UINT64_MAX), UINT64_MAX);
	zassert_equal(zstep_interval_from_ns(FREQ, 1000), 256);
}

static uint32_t position_at(const uint64_t *t, uint32_t n, uint64_t time)
{
	uint32_t steps = 0;

	while (steps < n && t[steps] <= time) {
		steps++;
	}
	return steps;
}

ZTEST(zstep_ramp, test_group_axes_stay_on_line)
{
	const uint32_t lead = 20000;
	const uint32_t minors[] = {1, 3, 777, 7000, 19999};
	struct zstep_profile p = profile(40000, 200000, 200000);

	zassert_equal(simulate(&p, lead, times), lead);

	ARRAY_FOR_EACH(minors, i) {
		struct zstep_profile q = p;
		uint32_t minor = minors[i];
		double worst = 0;

		zstep_profile_scale(&q, lead, minor, ZSTEP_RAMP_INTERVAL_MAX);
		zassert_equal(simulate(&q, minor, times_minor), minor);

		for (uint32_t k = 0; k < lead; k++) {
			double expected = (double)k * minor / lead + 1;
			double actual = position_at(times_minor, minor, times[k]);

			worst = MAX(worst, fabs(actual - expected));
		}

		zassert_true(worst <= 3, "minor axis %u off the line by %.2f steps", minor, worst);
	}
}

ZTEST_SUITE(zstep_ramp, NULL, NULL, NULL, NULL, NULL);
