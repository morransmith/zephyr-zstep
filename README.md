<div align="center">

# zstep

**Step/dir stepper motion controller for Zephyr RTOS**

![Release](https://img.shields.io/github/v/tag/morransmith/zephyr-zstep?sort=semver&label=release&style=flat-square&labelColor=2b2f36&color=3b82c4)
![Zephyr v4.4.2](https://img.shields.io/badge/Zephyr-v4.4.2-6f42c1?style=flat-square&labelColor=2b2f36)
![License Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-3b82c4?style=flat-square&labelColor=2b2f36)

[Overview](#overview) · [License](#license)

</div>

---

## Overview

zstep generates the step and direction signals of stepper motors with trapezoidal acceleration ramps, for CNC machines and 3D printers. It is a Zephyr module that implements the standard `stepper_ctrl` driver API, so it works with the stepper shell, the upstream API tests and the upstream driver-chip drivers (`ti,drv84xx`, `allegro,a4979`, `adi,tmc2209`, ...).

## License

zstep is licensed under the [Apache License 2.0](LICENSE).
