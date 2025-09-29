// Minimal BSP stub for host builds. Real board support headers are not
// available on host; tests should mock hardware interactions.
#pragma once

// Example pin definitions used by firmware; expand as needed.
#define LED_PIN 25

// Placeholder for board initialization function
static inline void BoardPheripheralsInit(void) {}
