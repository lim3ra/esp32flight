#pragma once

#include <stddef.h>

/* A small ring of one-line records in NVS, so the reasons a device dropped
 * off the network or restarted itself are still readable afterwards. The
 * serial log is the better tool when a cable is attached; this exists for
 * when one is not, which is most of the time.
 *
 * Kept deliberately small and written only on notable events - a handful a
 * day - because every append commits to flash. */

void eventlog_init(void);

/* Append one record, printf-style. Prefixed with local time and uptime. */
void eventlog_add(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Oldest first, newline-separated. Returns the length written. */
size_t eventlog_dump(char *dst, size_t n);
