# zstep architecture

zstep is a step/dir stepper motion controller for Zephyr, aimed at CNC machines and 3D printers. It implements the `stepper_ctrl` driver API of Zephyr v4.4.2.

## 1. Scope

- Motion of each motor: trapezoidal acceleration ramps with separate acceleration, deceleration and speed limits. Targets, direction and speed may change while the motor runs.
- Several motors on one hardware counter, each on its own alarm channel, with independent timing.
- Synchronized group moves: several axes start on the same counter tick with proportionally scaled profiles, so the tool moves along a straight line.
- Input shaping against the resonance of the frame.

The driver chip (enable, micro-step resolution) stays with the upstream `stepper` drivers, such as `ti,drv84xx`, `allegro,a4979` and `adi,tmc2209`. zstep only generates the step and direction signals.

Excluded: a look-ahead planner (a sequence of group moves stops between segments), kinematics, closed-loop control.

## 2. Packaging and layout

The repository is a Zephyr module and a west T2 manifest repository, following zephyrproject-rtos/example-application. `west.yml` pins Zephyr v4.4.2 without its modules, which is enough for the tests and the sample on native_sim. A firmware adds zstep to its own manifest without importing this one and brings the HALs of its boards. `zephyr/module.yml` sets `build.cmake: .`, `build.kconfig: Kconfig` and `build.settings.dts_root: .` for the `zstep,step-dir` binding.

```
west.yml  zephyr/module.yml  CMakeLists.txt  Kconfig  LICENSE
include/zstep/     zstep.h
drivers/stepper/   zstep_ramp.c zstep_shaper.c zstep_step_dir.c
dts/bindings/      vendor-prefixes.txt stepper/zstep,step-dir.yaml
tests/unit/        ramp/ shaper/
tests/integration/ conformance/ timing/
samples/           multi_axis/
ci/                release.sh
doc/               architecture.md Doxyfile
```

## 3. Conventions

- Positions are in micro-steps, speeds in micro-steps/s, accelerations in micro-steps/s^2, as in the `stepper_ctrl` API.
- Time is counted in ticks of the hardware counter of the motor.
- Devicetree properties carry their unit in the name: `-ns`, `-centihz`, `-permille`.
- The step interrupt uses no floating point, and the driver does not busy-wait.
