// Lightweight adapter for ICM-20948 to provide a minimal API compatible with existing ImuLib usage
#ifndef ICM20948_ADAPTER_H
#define ICM20948_ADAPTER_H

#include "../../../include/shared/imu_types.h"

#include <cstdint>

class ICM20948Adapter {
  public:
    ICM20948Adapter(int i2c_bus = 0, int address = 0x69);
    ~ICM20948Adapter();

    bool Init();
    imu_queue_t LoopHandler();

  private:
    int _i2c_bus;
    int _address;
};

#endif // ICM20948_ADAPTER_H
