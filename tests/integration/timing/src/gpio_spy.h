/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZSTEP_TESTS_INTEGRATION_TIMING_SRC_GPIO_SPY_H_
#define ZSTEP_TESTS_INTEGRATION_TIMING_SRC_GPIO_SPY_H_

#include <stddef.h>
#include <stdint.h>

struct gpio_spy_edge {
	uint32_t tick;
	uint8_t pin;
	uint8_t level;
};

/** Edges recorded since the last reset, in the order they happened. */
size_t gpio_spy_edges(const struct gpio_spy_edge **edges);

void gpio_spy_reset(void);

#endif /* ZSTEP_TESTS_INTEGRATION_TIMING_SRC_GPIO_SPY_H_ */
