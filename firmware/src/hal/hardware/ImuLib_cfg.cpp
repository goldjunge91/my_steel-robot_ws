/**
 * @file ImuLib_cfg.cpp
 * @author Maciej Kurcius
 * @brief
 * @version 0.1
 * @date 2022-02-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "hardware/ImuLib_cfg.h"
#include "../../lib/imu-driver/ICM-20948/ICM20948Adapter.h"

#include "hardware/bsp.h"

ImuDriver::ImuDriver(int i2c_bus, int address) { _adapter = new ICM20948Adapter(i2c_bus, address); }

ImuDriver::~ImuDriver() { delete _adapter; }

bool ImuDriver::Init() { return _adapter->Init(); }

imu_queue_t ImuDriver::LoopHandler() { return _adapter->LoopHandler(); }
