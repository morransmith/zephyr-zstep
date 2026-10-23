# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zstep_timing)

target_sources(app PRIVATE src/main.c src/gpio_spy.c src/log_capture.c)
target_include_directories(app PRIVATE ${ZEPHYR_ZSTEP_MODULE_DIR}/drivers/stepper)
