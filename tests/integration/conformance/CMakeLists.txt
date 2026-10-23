# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zstep_conformance)

# The upstream stepper_ctrl API suite runs unchanged against this driver.
target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/drivers/stepper/stepper_ctrl/src/main.c)
