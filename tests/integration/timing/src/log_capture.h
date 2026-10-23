/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZSTEP_TESTS_INTEGRATION_TIMING_SRC_LOG_CAPTURE_H_
#define ZSTEP_TESTS_INTEGRATION_TIMING_SRC_LOG_CAPTURE_H_

/** Text of all log messages since the last reset, needs CONFIG_LOG_MODE_IMMEDIATE. */
const char *log_capture_text(void);

void log_capture_reset(void);

#endif /* ZSTEP_TESTS_INTEGRATION_TIMING_SRC_LOG_CAPTURE_H_ */
