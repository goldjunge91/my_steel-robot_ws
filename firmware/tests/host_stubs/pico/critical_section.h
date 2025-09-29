// Minimal stub for pico/critical_section.h used by host tests.
#pragma once

#include <stdint.h>

// A minimal critical_section_t placeholder. The real RP2040 implementation
// provides IRQ-safe locking; for host tests a no-op stub is sufficient.
typedef struct critical_section {
    int _dummy;
} critical_section_t;

static inline void critical_section_init(critical_section_t *cs) { (void)cs; }
static inline void critical_section_enter_blocking(critical_section_t *cs) { (void)cs; }
static inline void critical_section_exit(critical_section_t *cs) { (void)cs; }
