/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT zstep_test_gpio_spy

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/spinlock.h>

#include "gpio_spy.h"

#define MAX_EDGES 32768

struct gpio_spy_config {
	struct gpio_driver_config common;
};

struct gpio_spy_data {
	struct gpio_driver_data common;
	gpio_port_value_t value;
};

static const struct device *const counter = DEVICE_DT_GET(DT_NODELABEL(counter0));
static struct gpio_spy_edge edges[MAX_EDGES];
static size_t edge_count;
static struct k_spinlock lock;

static int update(const struct device *dev, gpio_port_value_t value)
{
	struct gpio_spy_data *data = dev->data;

	K_SPINLOCK(&lock) {
		gpio_port_value_t changed = data->value ^ value;
		uint32_t now = 0;

		(void)counter_get_value(counter, &now);
		for (uint8_t pin = 0; changed != 0; pin++, changed >>= 1) {
			if ((changed & 1) == 0 || edge_count == MAX_EDGES) {
				continue;
			}

			edges[edge_count++] = (struct gpio_spy_edge){
				.tick = now,
				.pin = pin,
				.level = (value >> pin) & 1,
			};
		}

		data->value = value;
	}

	return 0;
}

size_t gpio_spy_edges(const struct gpio_spy_edge **out)
{
	*out = edges;
	return edge_count;
}

void gpio_spy_reset(void)
{
	K_SPINLOCK(&lock) {
		edge_count = 0;
	}
}

static int spy_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct gpio_spy_data *data = dev->data;

	if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
		return update(dev, data->value | BIT(pin));
	}

	return update(dev, data->value & ~BIT(pin));
}

static int spy_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	struct gpio_spy_data *data = dev->data;

	*value = data->value;
	return 0;
}

static int spy_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
				   gpio_port_value_t value)
{
	struct gpio_spy_data *data = dev->data;

	return update(dev, (data->value & ~mask) | (value & mask));
}

static int spy_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	struct gpio_spy_data *data = dev->data;

	return update(dev, data->value | pins);
}

static int spy_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	struct gpio_spy_data *data = dev->data;

	return update(dev, data->value & ~pins);
}

static int spy_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	struct gpio_spy_data *data = dev->data;

	return update(dev, data->value ^ pins);
}

static DEVICE_API(gpio, spy_api) = {
	.pin_configure = spy_pin_configure,
	.port_get_raw = spy_port_get_raw,
	.port_set_masked_raw = spy_port_set_masked_raw,
	.port_set_bits_raw = spy_port_set_bits_raw,
	.port_clear_bits_raw = spy_port_clear_bits_raw,
	.port_toggle_bits = spy_port_toggle_bits,
};

#define GPIO_SPY_DEFINE(inst)                                                                      \
	static const struct gpio_spy_config gpio_spy_config_##inst = {                             \
		.common = GPIO_COMMON_CONFIG_FROM_DT_INST(inst),                                   \
	};                                                                                         \
	static struct gpio_spy_data gpio_spy_data_##inst;                                          \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &gpio_spy_data_##inst, &gpio_spy_config_##inst,    \
			      POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, &spy_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_SPY_DEFINE)
