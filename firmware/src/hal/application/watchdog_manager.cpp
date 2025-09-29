#include "application/watchdog_manager.h"
#include "hardware/watchdog.h"
#include "shared/system_config.h"

extern "C" {

void watchdog_manager_init(void)
{
  // Enable the watchdog with a timeout of WATCHDOG_MS milliseconds
  watchdog_enable(WATCHDOG_MS, 1);
}

void watchdog_manager_update(void)
{
  watchdog_update();
}

bool watchdog_manager_was_rebooted(void)
{
  return watchdog_caused_reboot();
}

} // extern "C"
