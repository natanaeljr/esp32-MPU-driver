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

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

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
#include "dmp/defines.hpp"
#include "dmp/image.hpp"
#include "dmp/types.hpp"
#include "mpu/math.hpp"
#include "mpu/registers.hpp"
#include "mpu/types.hpp"

#include "mpu_test_helper.hpp"

namespace test
{
/**
 * MPUdmp class modified to initialize the bus automaticaly when
 * instantiated, and close when object is destroyed.
 * Also, resets MPU on construction and destruction.
 * */
class MPUdmp : public mpud::MPUdmp
{
 public:
    MPUdmp() : mpud::MPUdmp()
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

    ~MPUdmp()
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

 public:
    // turn protected methods as public
    esp_err_t writeMemory(uint16_t memAddr, uint8_t length, const uint8_t* data)
    {
        return mpud::MPUdmp::writeMemory(memAddr, length, data);
    }
    esp_err_t readMemory(uint16_t memAddr, uint8_t length, uint8_t* data)
    {
        return mpud::MPUdmp::readMemory(memAddr, length, data);
    }
};
using MPUdmp_t = MPUdmp;
}  // namespace test

/******************************************
 * TESTS ----------------------------------
 * ***************************************/

TEST_CASE("DMP firmware loading", "[MPU][DMP]")
{
    test::MPUdmp_t mpu;
    TEST_ESP_OK(mpu.testConnection());
    TEST_ESP_OK(mpu.initialize());
    TEST_ESP_OK(mpu.loadDMPFirmware());
    /* Check if the loaded DMP memory matches DMP image */
    uint8_t buffer[mpud::kDMPCodeSize];
    uint16_t addr = 0;  // chunk start address
    while (addr < mpud::kDMPCodeSize) {
        const uint8_t length = min(mpud::kDMPCodeSize - addr, mpud::kMemoryChunkSize);
        TEST_ESP_OK(mpu.readMemory(addr, length, &buffer[addr]));
        addr += length;
    }
    TEST_ASSERT_EQUAL_HEX8_ARRAY(mpud::kDMPMemory, buffer, mpud::kDMPCodeSize);
    printf("> DMP firmware verified!\n");
}

//-

TEST_CASE("DMP basic test", "[MPU][DMP]")
{
    test::MPUdmp_t mpu;
    TEST_ESP_OK(mpu.testConnection());
    TEST_ESP_OK(mpu.initialize());
    TEST_ESP_OK(mpu.loadDMPFirmware());
    /* Check DMP activation */
    TEST_ASSERT_FALSE(mpu.getDMPEnabled());
    TEST_ESP_OK(mpu.lastError());
    TEST_ESP_OK(mpu.enableDMP());
    TEST_ASSERT_TRUE(mpu.getDMPEnabled());
    TEST_ESP_OK(mpu.lastError());
    TEST_ESP_OK(mpu.disableDMP());
    TEST_ASSERT_FALSE(mpu.getDMPEnabled());
    TEST_ESP_OK(mpu.lastError());
    TEST_ESP_OK(mpu.enableDMP());
    /* Check Features setter */
    mpud::dmp_feature_t features;
    features = mpud::DMP_FEATURE_ANDROID_ORIENT | mpud::DMP_FEATURE_TAP | mpud::DMP_FEATURE_PEDOMETER;
    TEST_ESP_OK(mpu.setDMPFeatures(features));
    TEST_ASSERT_EQUAL_HEX(features, mpu.getDMPFeatures());
    features = mpud::DMP_FEATURE_LP_3X_QUAT | mpud::DMP_FEATURE_LP_6X_QUAT;
    TEST_ESP_ERR(ESP_ERR_INVALID_ARG, mpu.setDMPFeatures(features));
    features = mpud::DMP_FEATURE_SEND_RAW_GYRO | mpud::DMP_FEATURE_SEND_CAL_GYRO;
    TEST_ESP_ERR(ESP_ERR_INVALID_ARG, mpu.setDMPFeatures(features));
    features = mpud::DMP_FEATURE_SEND_RAW_ACCEL | mpud::DMP_FEATURE_GYRO_CAL | mpud::DMP_FEATURE_SEND_CAL_GYRO |
               mpud::DMP_FEATURE_LP_6X_QUAT | mpud::DMP_FEATURE_PEDOMETER;
    TEST_ESP_OK(mpu.setDMPFeatures(features));
    TEST_ASSERT_EQUAL_HEX(features, mpu.getDMPFeatures());
    TEST_ASSERT_EQUAL_UINT8(28, mpu.getDMPPacketLength());
    /* Test Test section */
}

//-

TEST_CASE("DMP output data rate test", "[MPU][DMP]")
{
    test::MPUdmp_t mpu;
    TEST_ESP_OK(mpu.testConnection());
    TEST_ESP_OK(mpu.initialize());
    TEST_ESP_OK(mpu.loadDMPFirmware());
    /* Invalid rate/state check */
    TEST_ESP_OK(mpu.setDMPOutputRate(0));
    TEST_ESP_OK(mpu.setDMPOutputRate(201));
    TEST_ESP_OK(mpu.setDMPOutputRate(1));
    TEST_ESP_OK(mpu.setDMPOutputRate(200));
    TEST_ESP_OK(mpu.setDMPOutputRate(13));
    TEST_ESP_OK(mpu.setDMPOutputRate(57));
    TEST_ESP_OK(mpu.setDMPOutputRate(-1));
    /** ODR measurement */
    const int8_t orientation[] = {
        1, 0, 0,  //
        0, 1, 0,  // XYZ
        0, 0, 1   //
    };
    TEST_ESP_OK(mpu.setOrientation(mpud::math::orientationMatrixToScalar(orientation)));
    constexpr mpud::dmp_feature_t features =
        (mpud::DMP_FEATURE_SEND_RAW_ACCEL | mpud::DMP_FEATURE_SEND_CAL_GYRO | mpud::DMP_FEATURE_ANDROID_ORIENT |
         mpud::DMP_FEATURE_TAP | mpud::DMP_FEATURE_GYRO_CAL | mpud::DMP_FEATURE_LP_6X_QUAT |
         mpud::DMP_FEATURE_PEDOMETER);
    TEST_ESP_OK(mpu.setDMPFeatures(features));
    TEST_ASSERT_EQUAL_HEX(features, mpu.getDMPFeatures());
    TEST_ESP_OK(mpu.setDMPInterruptMode(mpud::DMP_INT_MODE_CONTINUOUS));
    TEST_ESP_OK(mpu.setInterruptEnabled(mpud::INT_EN_DMP_READY));
    TEST_ESP_OK(mpu.enableDMP());

    constexpr int numOfSamples = 1;
    constexpr uint16_t rates[] = {1, 5, 10, 50, 100, 200};
    for (auto rate : rates) {
        TEST_ESP_OK(mpu.setDMPOutputRate(rate));
        test::mpuMeasureInterruptRate(mpu, rate, numOfSamples);
    }
}

#endif  // defined CONFIG_MPU_ENABLE_DMP
