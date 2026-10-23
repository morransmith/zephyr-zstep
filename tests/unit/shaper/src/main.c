/* SPDX-License-Identifier: Apache-2.0 */

#include <math.h>

#include <zephyr/ztest.h>

#include "zstep_ramp.h"
#include "zstep_shaper.h"

#define FREQ          1000000U
#define PI            3.14159265358979
#define TUNED_CENTIHZ 4000
#define MAX_STEPS     8192
#define BUFFER        2048

/* A hard printer move: 5000 mm/s^2 and 500 mm/s at 80 steps/mm. */
#define SPEED 40000
#define ACCEL 400000

struct move {
	uint32_t n;
	uint32_t t[MAX_STEPS];
	int8_t dir[MAX_STEPS];
};

static struct move command;
static struct move output;
static struct zstep_echo_step buffer[BUFFER];

static struct zstep_shaper shaper_of(enum zstep_shaper_type type)
{
	struct zstep_shaper s;

	zassert_ok(zstep_shaper_init(&s, type, TUNED_CENTIHZ, 100, FREQ));
	return s;
}

/* Steps of `steps` in direction `dir` from rest, starting at `start`. */
static void add_ramp(struct move *m, uint32_t start, uint32_t steps, int8_t dir)
{
	struct zstep_profile p;
	struct zstep_ramp r;
	uint32_t t = start;

	zassert_ok(
		zstep_profile_init(&p, FREQ, zstep_interval_from_speed(FREQ, SPEED), ACCEL, ACCEL));
	zstep_ramp_reset(&r, &p);
	for (uint32_t k = 1; k <= steps; k++) {
		m->t[m->n] = t;
		m->dir[m->n++] = dir;
		t += zstep_ramp_next(&r, steps - k);
	}
}

static void shape(const struct zstep_shaper *s, const struct move *in, struct move *out)
{
	struct zstep_echo e = {.shaper = *s, .buf = buffer, .size = BUFFER, .top = UINT32_MAX};
	uint32_t pushed = 0;

	out->n = 0;
	zstep_echo_reset(&e, 0);
	for (;;) {
		uint32_t t;
		int8_t dir;

		switch (zstep_echo_next(&e, &t, &dir)) {
		case ZSTEP_ECHO_STEP:
			zassert_true(out->n < MAX_STEPS);
			out->t[out->n] = t;
			out->dir[out->n++] = dir;
			break;
		case ZSTEP_ECHO_NEED_COMMAND:
			if (pushed == in->n) {
				e.closed = true;
				break;
			}
			zassert_ok(zstep_echo_push(&e, in->t[pushed], in->dir[pushed]));
			pushed++;
			break;
		case ZSTEP_ECHO_DONE:
			return;
		}
	}
}

static int32_t displacement(const struct move *m)
{
	int32_t pos = 0;

	for (uint32_t k = 0; k < m->n; k++) {
		pos += m->dir[k];
	}

	return pos;
}

/* Peak deviation from the target of a damped resonance driven by the step positions. */
static double residual(const struct move *m, double freq, double zeta)
{
	const double dt = 4e-6;
	double w = 2 * PI * freq;
	double end = (double)m->t[m->n - 1] / FREQ + 0.02;
	int32_t target = displacement(m);
	double y = 0;
	double v = 0;
	double peak = 0;
	int32_t u = 0;
	uint32_t k = 0;

	for (double t = 0; t < end + 0.25; t += dt) {
		while (k < m->n && m->t[k] <= t * FREQ) {
			u += m->dir[k++];
		}

		v += (w * w * (u - y) - 2 * zeta * w * v) * dt;
		y += v * dt;
		if (t > end) {
			peak = MAX(peak, fabs(y - target));
		}
	}

	return peak;
}

/* Worst residual over several move lengths, a single move depends on its phase. */
static double worst_residual(const struct zstep_shaper *s, double freq, double zeta)
{
	const uint32_t lengths[] = {3000, 4500, 6000};
	double worst = 0;

	ARRAY_FOR_EACH(lengths, i) {
		command.n = 0;
		add_ramp(&command, 0, lengths[i], 1);
		if (s == NULL) {
			worst = MAX(worst, residual(&command, freq, zeta));
			continue;
		}

		shape(s, &command, &output);
		worst = MAX(worst, residual(&output, freq, zeta));
	}

	return worst;
}

static void check_band(enum zstep_shaper_type type, const char *name, double low, double high)
{
	struct zstep_shaper s = shaper_of(type);

	TC_PRINT("%s tuned to 40 Hz, real damping 0.05:\n", name);
	for (double ratio = 0.7; ratio < 1.35; ratio += 0.1) {
		double freq = 40 * ratio;
		double plain = worst_residual(NULL, freq, 0.05);
		double shaped = worst_residual(&s, freq, 0.05);
		bool in_band = ratio > low - 0.01 && ratio < high + 0.01;

		TC_PRINT("  %4.0f Hz: %5.2f -> %4.2f steps (x%4.1f)%s\n", freq, plain, shaped,
			 plain / shaped, in_band ? "" : "  outside the band");
		zassert_true(!in_band || plain / shaped >= 3, "%s at %.0f Hz reduces only x%.1f",
			     name, freq, plain / shaped);
	}
}

ZTEST(zstep_shaper, test_coefficients)
{
	const enum zstep_shaper_type types[] = {ZSTEP_SHAPER_ZVD, ZSTEP_SHAPER_EI};
	double zeta = 0.1;
	double v = 0.05;
	double df = sqrt(1 - zeta * zeta);
	double period = 1 / (40.0 * df);
	double k = exp(-zeta * PI / df);
	/* NASA CR-189439 eq. 22 */
	double a1 = 0.2497 + 0.2496 * v + 0.8001 * zeta + 1.233 * v * zeta + 0.4960 * zeta * zeta +
		    3.173 * v * zeta * zeta;
	double a3 = 0.2515 + 0.2147 * v - 0.8325 * zeta + 1.415 * v * zeta + 0.8518 * zeta * zeta -
		    4.901 * v * zeta * zeta;
	double t2 = 0.5 + 0.4616 * v * zeta + 4.262 * v * pow(zeta, 2) + 1.756 * v * pow(zeta, 3) +
		    8.578 * v * v * zeta - 108.6 * v * v * pow(zeta, 2) +
		    337.0 * v * v * pow(zeta, 3);
	const double expected[][2][3] = {
		{{1, 2 * k, k * k}, {0, 0.5, 1}},
		{{a1, 1 - a1 - a3, a3}, {0, t2, 1}},
	};

	ARRAY_FOR_EACH(types, i) {
		struct zstep_shaper s = shaper_of(types[i]);
		double sum = expected[i][0][0] + expected[i][0][1] + expected[i][0][2];
		uint32_t total = 0;

		zassert_equal(s.impulses, 3);
		for (int j = 0; j < 3; j++) {
			double weight = expected[i][0][j] / sum * ZSTEP_SHAPER_UNIT;
			double delay = expected[i][1][j] * period * FREQ;

			zassert_within(s.weight[j], weight, 2.0, "type %d weight %d", types[i], j);
			zassert_within(s.delay[j], delay, 1.0, "type %d delay %d", types[i], j);
			total += s.weight[j];
		}

		zassert_equal(total, ZSTEP_SHAPER_UNIT);
	}
}

ZTEST(zstep_shaper, test_every_type_is_normalized)
{
	for (int type = ZSTEP_SHAPER_NONE; type <= ZSTEP_SHAPER_EI; type++) {
		struct zstep_shaper s = shaper_of(type);
		uint32_t total = 0;

		for (int j = 0; j < s.impulses; j++) {
			total += s.weight[j];
			zassert_true(j == 0 || s.delay[j] > s.delay[j - 1]);
		}

		zassert_equal(total, ZSTEP_SHAPER_UNIT);
		zassert_equal(s.delay[0], 0);
	}

	struct zstep_shaper s;

	zassert_equal(zstep_shaper_init(&s, ZSTEP_SHAPER_EI, 0, 100, FREQ), -EINVAL);
	zassert_equal(zstep_shaper_init(&s, ZSTEP_SHAPER_EI, 4000, 1000, FREQ), -EINVAL);
	zassert_ok(zstep_shaper_init(&s, ZSTEP_SHAPER_EI, 4000, 300, FREQ));
	zassert_equal(zstep_shaper_init(&s, ZSTEP_SHAPER_EI, 4000, 301, FREQ), -EINVAL,
		      "outside the damping range of the EI fit");
	zassert_ok(zstep_shaper_init(&s, ZSTEP_SHAPER_ZVD, 4000, 500, FREQ));
	zassert_equal(zstep_shaper_init(&s, 42, 4000, 100, FREQ), -EINVAL);
	zassert_equal(s.impulses, 1, "a failed init leaves shaping off");
}

ZTEST(zstep_shaper, test_none_is_identity)
{
	struct zstep_shaper s = shaper_of(ZSTEP_SHAPER_NONE);

	command.n = 0;
	add_ramp(&command, 100, 1000, 1);
	shape(&s, &command, &output);

	zassert_equal(output.n, command.n);
	zassert_mem_equal(output.t, command.t, command.n * sizeof(command.t[0]));
}

ZTEST(zstep_shaper, test_every_move_ends_on_target)
{
	const enum zstep_shaper_type types[] = {ZSTEP_SHAPER_ZVD, ZSTEP_SHAPER_EI};

	ARRAY_FOR_EACH(types, i) {
		struct zstep_shaper s = shaper_of(types[i]);

		for (uint32_t n = 1; n <= 2000; n++) {
			command.n = 0;
			add_ramp(&command, 0, n, 1);
			shape(&s, &command, &output);
			zassert_equal(output.n, n, "type %d, %u steps", types[i], n);
			zassert_equal(displacement(&output), n);
		}
	}
}

/* Commanded position at time t with linear interpolation between steps. */
static double commanded_at(const struct move *m, double t)
{
	if (t < m->t[0]) {
		return 0;
	}

	for (uint32_t k = 0; k + 1 < m->n; k++) {
		if (t < m->t[k + 1]) {
			return k + 1 + (t - m->t[k]) / (m->t[k + 1] - m->t[k]);
		}
	}

	return m->n;
}

ZTEST(zstep_shaper, test_follows_ideal_convolution)
{
	const enum zstep_shaper_type types[] = {ZSTEP_SHAPER_ZVD, ZSTEP_SHAPER_EI};

	command.n = 0;
	add_ramp(&command, 0, 3000, 1);
	ARRAY_FOR_EACH(types, i) {
		struct zstep_shaper s = shaper_of(types[i]);

		shape(&s, &command, &output);
		for (uint32_t k = 0; k < output.n; k += 7) {
			double ideal = 0;

			for (int j = 0; j < s.impulses; j++) {
				ideal += (double)s.weight[j] / ZSTEP_SHAPER_UNIT *
					 commanded_at(&command, (double)output.t[k] - s.delay[j]);
			}

			zassert_within(ideal, k + 1, 1.01, "type %d step %u", types[i], k);
		}
	}
}

ZTEST(zstep_shaper, test_zvd_cancels_resonance)
{
	check_band(ZSTEP_SHAPER_ZVD, "ZVD", 0.8, 1.2);
}

ZTEST(zstep_shaper, test_ei_cancels_resonance)
{
	check_band(ZSTEP_SHAPER_EI, "EI", 0.8, 1.2);
}

ZTEST(zstep_shaper, test_damping_mismatch)
{
	struct zstep_shaper zvd = shaper_of(ZSTEP_SHAPER_ZVD);
	struct zstep_shaper ei = shaper_of(ZSTEP_SHAPER_EI);

	TC_PRINT("40 Hz resonance, shapers assume damping 0.1:\n");
	for (double zeta = 0.02; zeta < 0.16; zeta += 0.0325) {
		double plain = worst_residual(NULL, 40, zeta);
		double by_zvd = worst_residual(&zvd, 40, zeta);
		double by_ei = worst_residual(&ei, 40, zeta);

		TC_PRINT("  damping %.3f: %5.2f -> ZVD %4.2f (x%4.1f), EI %4.2f (x%4.1f)\n", zeta,
			 plain, by_zvd, plain / by_zvd, by_ei, plain / by_ei);
		zassert_true(plain / by_zvd >= 3 && plain / by_ei >= 3, "damping %.3f", zeta);
	}
}

ZTEST(zstep_shaper, test_reversal)
{
	const enum zstep_shaper_type types[] = {ZSTEP_SHAPER_ZVD, ZSTEP_SHAPER_EI};
	struct zstep_profile p;

	zassert_ok(
		zstep_profile_init(&p, FREQ, zstep_interval_from_speed(FREQ, SPEED), ACCEL, ACCEL));
	command.n = 0;
	add_ramp(&command, 0, 3000, 1);
	add_ramp(&command, command.t[command.n - 1] + 5000, 4000, -1);

	ARRAY_FOR_EACH(types, i) {
		struct zstep_shaper s = shaper_of(types[i]);
		uint32_t shortest = UINT32_MAX;
		int reversals = 0;

		shape(&s, &command, &output);
		zassert_equal(displacement(&output), -1000);
		for (uint32_t k = 1; k < output.n; k++) {
			reversals += output.dir[k] != output.dir[k - 1];
			shortest = MIN(shortest, output.t[k] - output.t[k - 1]);
		}

		zassert_equal(reversals, 1, "type %d dithers at the reversal", types[i]);
		zassert_true(shortest >= p.cmin >> ZSTEP_RAMP_Q, "type %d: %u ticks between steps",
			     types[i], shortest);
	}
}

ZTEST(zstep_shaper, test_buffer_limits)
{
	struct zstep_shaper s = shaper_of(ZSTEP_SHAPER_EI);
	struct zstep_echo e = {.shaper = s, .buf = buffer, .size = 4, .top = UINT32_MAX};
	uint64_t cmin = zstep_interval_from_speed(FREQ, SPEED);

	/* EI at 40 Hz lasts 25 ms, which is 1000 steps at full speed. */
	zassert_false(zstep_shaper_fits(&s, cmin, 1000));
	zassert_true(zstep_shaper_fits(&s, cmin, 1100));

	zstep_echo_reset(&e, 0);
	zassert_ok(zstep_echo_push(&e, 0, 1));
	zassert_ok(zstep_echo_push(&e, 10, 1));
	zassert_ok(zstep_echo_push(&e, 20, 1));
	zassert_equal(zstep_echo_push(&e, 30, 1), -ENOMEM);
}

ZTEST_SUITE(zstep_shaper, NULL, NULL, NULL, NULL, NULL);
