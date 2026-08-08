#pragma once

#include <stdio.h>

// Competition/demo builds default to a concise serial monitor.  Set this to 1
// temporarily when diagnosing audio frames, VAD, raw sensor values or queues.
#ifndef MONITOR_VERBOSE_DIAGNOSTICS
#define MONITOR_VERBOSE_DIAGNOSTICS 0
#endif

#ifndef MONITOR_SHOW_ERRORS
#define MONITOR_SHOW_ERRORS 0
#endif

#if MONITOR_VERBOSE_DIAGNOSTICS
#define MONITOR_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define MONITOR_DEBUG_PRINTF(...) ((void)0)
#endif
