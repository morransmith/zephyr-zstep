/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>

#include "log_capture.h"

static char text[1024];
static size_t length;
static uint8_t output_buf[128];

static int append(uint8_t *data, size_t size, void *ctx)
{
	size_t room = sizeof(text) - 1 - length;
	size_t n = MIN(size, room);

	memcpy(&text[length], data, n);
	length += n;
	text[length] = '\0';
	return size;
}

LOG_OUTPUT_DEFINE(capture_output, append, output_buf, sizeof(output_buf));

static void process(const struct log_backend *const backend, union log_msg_generic *msg)
{
	log_output_msg_process(&capture_output, &msg->log, 0);
}

static void panic(const struct log_backend *const backend)
{
}

static const struct log_backend_api capture_api = {
	.process = process,
	.panic = panic,
};

LOG_BACKEND_DEFINE(log_capture, capture_api, true);

const char *log_capture_text(void)
{
	return text;
}

void log_capture_reset(void)
{
	length = 0;
	text[0] = '\0';
}
