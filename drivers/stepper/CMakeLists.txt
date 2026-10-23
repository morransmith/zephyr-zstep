# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ZSTEP_RAMP)
  zephyr_library()
  zephyr_library_include_directories(.)
  zephyr_library_sources(zstep_ramp.c zstep_shaper.c)
  zephyr_library_sources_ifdef(CONFIG_ZSTEP_STEP_DIR zstep_step_dir.c)
endif()
