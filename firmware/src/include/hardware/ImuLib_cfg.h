/**
 * @file ImuLib_cfg.h
 * @author Maciej Kurcius
 * @brief
 * @version 0.1
 * @date 2022-02-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ImuLibCfg_H
#define ImuLibCfg_H

#include "imu_types.h"
#include <hardware_cfg.h>

// forward declaration to avoid pulling adapter implementation into this header
class ICM20948Adapter;

class ImuDriver {
  public:
    ImuDriver(int i2c_bus = 0, int address = 0x69);
    ~ImuDriver();
    bool Init();
    imu_queue_t LoopHandler();

  private:
    ICM20948Adapter *_adapter;
};

#endif /* ImuLibCfg_H */
