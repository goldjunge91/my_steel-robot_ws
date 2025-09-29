// Minimal stub for pico/time.h used by host tests.
#pragma once

#include <stdint.h>
#include <time.h>

// Provide a minimal us-based time function used in firmware code.
static inline uint64_t time_us_64(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
        return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000ULL);
    }
    return 0;
}

static inline void sleep_ms(unsigned ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

// Provide the 32-bit variant expected by some firmware code.
static inline uint32_t time_us_32(void) { return (uint32_t)(time_us_64() & 0xFFFFFFFFu); }
