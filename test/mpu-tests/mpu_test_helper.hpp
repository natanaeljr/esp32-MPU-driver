// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu_test_helper.hpp
 * MPU Test Helper.
 * Keep I2C/SPI bus configuration, accessed by 'test_mpu.cpp' and 'test_dmp.cpp'.
 */

#pragma once

#include "sdkconfig.h"

namespace test
{
//
#ifdef CONFIG_MPU_I2C
#include "I2Cbus.hpp"
extern I2C_t& i2c;
#elif CONFIG_MPU_SPI
#include "SPIbus.hpp"
#include "driver/spi_master.h"
extern SPI_t& spi;
extern spi_device_handle_t spi_mpu_handle;
#endif

/**
 * Hold state of bus on/off.
 * If a test fail, isBusInit stays true, so the bus is not initialized again.
 * */
extern bool isBusInit;

}  // namespace test
