// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file test_dmp.cpp
 * Test code for DMP interface.
 */

#include "sdkconfig.h"

#if defined CONFIG_MPU_ENABLE_DMP

#define min(a, b) (a < b ? a : b)
#define max(a, b) (a > b ? a : b)

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "unity_config.h"

#include "MPU.hpp"
#include "MPUdmp.hpp"
#include "dmp/image.hpp"
#include "mpu/registers.hpp"
#include "mpu/types.hpp"

#include "mpu_test_helper.hpp"

namespace test
{
/**
 * MPU class modified to initialize the bus automaticaly when
 * instantiated, and close when object is destroyed.
 * Also, resets MPU on construction and destruction.
 * */
class MPU : public mpud::dmp::MPU
{
 public:
    MPU() : mpud::dmp::MPU()
    {
#ifdef CONFIG_MPU_I2C
        if (!isBusInit) {
            i2c.begin((gpio_num_t) CONFIG_MPU_TEST_I2CBUS_SDA_PIN, (gpio_num_t) CONFIG_MPU_TEST_I2CBUS_SCL_PIN,
                      CONFIG_MPU_TEST_I2CBUS_CLOCK_HZ);
        }
        this->setBus(i2c);
        this->setAddr((mpud::mpu_i2caddr_t)(CONFIG_MPU_TEST_I2CBUS_ADDR + mpud::MPU_I2CADDRESS_AD0_LOW));

#elif CONFIG_MPU_SPI
        if (!isBusInit) {
            spi.begin(CONFIG_MPU_TEST_SPIBUS_MOSI_PIN, CONFIG_MPU_TEST_SPIBUS_MISO_PIN,
                      CONFIG_MPU_TEST_SPIBUS_SCLK_PIN);
            spi.addDevice(0, CONFIG_MPU_TEST_SPIBUS_CLOCK_HZ, CONFIG_MPU_TEST_SPIBUS_CS_PIN, &spi_mpu_handle);
        }
        this->setBus(spi);
        this->setAddr(spi_mpu_handle);
#endif

        if (!isBusInit) isBusInit = true;
        this->reset();
    }

    ~MPU()
    {
        this->reset();
#ifdef CONFIG_MPU_I2C
        (void) 0;
#elif CONFIG_MPU_SPI
        spi.removeDevice(this->getAddr());
#endif
        this->bus->close();
        isBusInit = false;
    }
};
using MPUdmp_t = MPU;
}  // namespace test

/******************************************
 * TESTS ----------------------------------
 * ***************************************/

TEST_CASE("DMP firmware loading", "[MPU][DMP]")
{
    test::MPUdmp_t mpu;
    TEST_ESP_OK(mpu.testConnection());
    TEST_ESP_OK(mpu.initialize());
    TEST_ESP_OK(mpu.loadDMP());
    /* Check if the loaded DMP memory matches DMP image */
    uint8_t buffer[kDMPCodeSize];
    uint16_t addr = 0;  // chunk start address
    while (addr < kDMPCodeSize) {
        const uint8_t length = min(kDMPCodeSize - addr, kMemoryChunkSize);
        TEST_ESP_OK(mpu.readMemory(addr, length, &buffer[addr]));
        addr += length;
    }
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kDMPMemory, buffer, kDMPCodeSize);
    printf("> DMP firmware verified!\n");
}

#endif  // defined CONFIG_MPU_ENABLE_DMP