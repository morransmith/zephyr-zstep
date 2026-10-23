<div align="center">

# zstep

**Step/dir stepper motion controller for Zephyr RTOS**

![Release](https://img.shields.io/github/v/tag/morransmith/zephyr-zstep?sort=semver&label=release&style=flat-square&labelColor=2b2f36&color=3b82c4)
![Zephyr v4.4.2](https://img.shields.io/badge/Zephyr-v4.4.2-6f42c1?style=flat-square&labelColor=2b2f36)
![License Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-3b82c4?style=flat-square&labelColor=2b2f36)

[Overview](#overview) · [Getting started](#getting-started) · [Features](#features) · [Samples](#samples) · [Documentation](#documentation) · [License](#license)

</div>

---

## Overview

zstep generates the step and direction signals of stepper motors with trapezoidal acceleration ramps, for CNC machines and 3D printers. It is a Zephyr module that implements the standard `stepper_ctrl` driver API, so it works with the stepper shell, the upstream API tests and the upstream driver-chip drivers (`ti,drv84xx`, `allegro,a4979`, `adi,tmc2209`, ...).

Several motors share one hardware counter, each on its own alarm channel, and keep independent timing. The driver uses only the generic Zephyr counter and GPIO APIs, so the same firmware source builds for chips of different vendors.

## Getting started

### Requirements

- Zephyr v4.4.2 with the Zephyr SDK. Other Zephyr versions are not tested.
- A counter (timer) that runs freely over a power-of-two range (16 or 32 bits), provides one alarm channel per motor and lets an alarm be re-armed from its own callback.
- A counter clock of 1 to 10 MHz. The longest step interval is half the counter range, so prefer a 32-bit timer or set a prescaler on a 16-bit one.

Counter drivers that call the alarm callback synchronously from `counter_set_channel_alarm()` (e.g. `atmel,sam0-tc32`) are not supported.

### Standalone workspace

```sh
west init -m https://github.com/morransmith/zephyr-zstep --mr master zstep-ws
cd zstep-ws
west update
```

The workspace has Zephyr v4.4.2 without its modules, which is enough for the tests and the sample on `native_sim`. A board build needs the HAL of the board, see [Build and test](#build-and-test).

### Adding zstep to a firmware

West manifest of the firmware:

```yaml
manifest:
  remotes:
    - name: zstep
      url-base: https://github.com/morransmith
  projects:
    - name: zephyr-zstep
      remote: zstep
      revision: master
```

zstep needs only Zephyr, so the firmware keeps its own Zephyr and modules and does not import the manifest of zstep.

`master` is the latest release. To fix the version, set `revision` (or `--mr`) to the tag of the needed release. Outside a west workspace, pass `-DZEPHYR_EXTRA_MODULES=/path/to/zephyr-zstep` to the build.

`prj.conf`:

```ini
CONFIG_STEPPER=y
```

The driver is then selected by a `zstep,step-dir` node in the devicetree.

### Devicetree

```dts
&timers5 {
	status = "okay";
	st,prescaler = <95>;		/* 96 MHz / 96 = 1 MHz */
	axis_timer: counter {
		status = "okay";
	};
};

/ {
	x_axis: x-axis {
		compatible = "zstep,step-dir";
		step-gpios = <&gpioa 10 GPIO_ACTIVE_HIGH>;
		dir-gpios = <&gpiob 4 GPIO_ACTIVE_HIGH>;
		counter = <&axis_timer>;
		counter-channel = <0>;
		stepper-driver = <&x_driver>;	/* optional, provides step-width-ns / dual-edge-step */
	};

	y_axis: y-axis {
		compatible = "zstep,step-dir";
		step-gpios = <&gpiob 3 GPIO_ACTIVE_HIGH>;
		dir-gpios = <&gpiob 10 GPIO_ACTIVE_HIGH>;
		counter = <&axis_timer>;
		counter-channel = <1>;
	};
};
```

| Property | Default | Meaning |
|---|---|---|
| `step-gpios`, `dir-gpios` | required | Step and direction outputs |
| `counter`, `counter-channel` | required, `0` | Time base. Motors sharing a counter need different channels |
| `step-width-ns` | `2000` | Minimum step pulse width. Taken from `stepper-driver` if that node has it |
| `dir-setup-ns` | `650` | Minimum time between a DIR change and a step edge |
| `dual-edge-step` | off | The driver steps on both edges. Also taken from `stepper-driver` |
| `invert-direction` | off | Swap the positive and negative directions |

Enable, disable and micro-stepping stay with the driver chip, through the `stepper` API.

### API

```c
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zstep/zstep.h>

static const struct stepper_ctrl_ramp limits = {
	.acceleration_max = 20000,	/* micro-steps/s^2 */
	.speed_max = 8000,		/* micro-steps/s */
	.deceleration_max = 20000,	/* 0 means equal to acceleration */
};

stepper_ctrl_configure_ramp(x, &limits);
stepper_ctrl_move_to(x, 16000);		/* STEPPER_CTRL_EVENT_STEPS_COMPLETED at the end */
stepper_ctrl_run(y, STEPPER_CTRL_DIRECTION_NEGATIVE);
stepper_ctrl_stop(y);			/* decelerates, then STEPPER_CTRL_EVENT_STOPPED */

const struct device *axes[] = {x, y, z};
const int32_t delta[] = {8000, -3000, 400};

zstep_group_move_by(axes, delta, ARRAY_SIZE(axes), &limits);
```

- `acceleration_max = 0`, or `stepper_ctrl_set_microstep_interval()` alone, gives a constant speed without a ramp.
- `stepper_ctrl_set_microstep_interval()` also works while the motor runs: it accelerates or brakes to the new speed.
- `move_to`, `move_by` and `run` may be called while the motor moves. A target behind the motor makes it brake, reverse and come back.
- `stepper_ctrl_configure_ramp()` returns `-EBUSY` while the motor moves.
- Events raised in an ISR are delivered from the system work queue (`CONFIG_STEPPER_CTRL_ISR_SAFE_EVENTS`). Callbacks may call the driver again.

`zstep_group_move_by()` requires all axes to be idle and to share one counter. The axis with the longest move follows `limits`, the others are slowed down in proportion. Every axis reports `STEPPER_CTRL_EVENT_STEPS_COMPLETED` itself.

### Kconfig

| Option | Default | Meaning |
|---|---|---|
| `CONFIG_ZSTEP_STEP_DIR` | `y` with a `zstep,step-dir` node | The driver |
| `CONFIG_ZSTEP_GROUP_MAX_AXES` | `6` | Largest synchronized group |
| `CONFIG_ZSTEP_GROUP_START_US` | `50` | Lead time for arming a group before its first step |

### Build and test

```sh
west twister -T tests -T samples -p native_sim -p native_sim/native/64
```

For a board, build in a workspace that has the HAL of the board, such as a Zephyr workspace or the one of the firmware:

```sh
west build -b nucleo_f446re /path/to/zephyr-zstep/samples/multi_axis -- -DZEPHYR_EXTRA_MODULES=/path/to/zephyr-zstep
west flash
```

## Features

| Feature | Description |
|---|---|
| Acceleration ramps | Separate acceleration, deceleration and speed limits. Every move ends at rest on its target step. Target, direction and speed may change while the motor runs |
| Independent motors | Motors on one counter use their own alarm channels. A motor never waits for another one, and interrupt latency caused by other motors does not accumulate into its timing |
| Synchronized moves | `zstep_group_move_by()` starts several axes on the same counter tick and scales their profiles, so the tool moves along a straight line |
| Run time cost | No busy waiting and no floating point. A step costs one 32-bit division and one counter interrupt, or two interrupts for single-edge drivers |

### Not supported

- Look-ahead planning: a sequence of group moves stops between segments.
- Counter drivers that call the alarm callback synchronously.

### Hardware

| Board | Status |
|---|---|
| `nucleo_f446re` | Built in CI |
| `nrf52840dk/nrf52840` | Built in CI |
| `rpi_pico` | Built in CI |
| `native_sim`, `native_sim/native/64` | Tests and sample in CI |

## Samples

`samples/multi_axis` traces a square with three synchronized axes, then moves them independently. It runs on `native_sim` and has overlays for `nucleo_f446re`, `nrf52840dk/nrf52840` and `rpi_pico`, with the Arduino CNC shield v3 pinout where the board has it.

```sh
west build -b native_sim/native/64 samples/multi_axis
build/zephyr/zephyr.exe -stop_at=20
```

## Documentation

- **API reference:** `doxygen doc/Doxyfile`, output in `doc/build/html`.
- **Design:** [`doc/architecture.md`](doc/architecture.md).

## License

zstep is licensed under the [Apache License 2.0](LICENSE).
