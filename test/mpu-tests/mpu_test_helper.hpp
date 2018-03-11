// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu_test_helper.hpp
 * MPU Test Helper. Used by the mpu test files.
 */

#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "MPU.hpp"
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

/* FUNCTIONS */

/*! Setup GPIO and ISR for interrupt pin (ISR must declared with IRAM_ATTR). */
esp_err_t mpuConfigInterrupt(void (*isr)(void*), void* arg);
/*! Remove GPIO ISR for interrupt pin (call at end of a interrupt test). */
esp_err_t mpuRemoveInterrupt();
/*! Default Task Notifier, to use as ISR for interrupt signal. */
void IRAM_ATTR mpuTaskNotifier(void* arg);
/*! Interrupt signal Counter, `agr` is the count holder. */
void IRAM_ATTR mpuInterruptCounterISR(void* arg);
/*! Routine to mesasure sample rate through interrupt signal. */
void mpuMeasureInterruptRate(MPU_t& mpu, uint16_t rate, int numOfSamples);

}  // namespace test

// ==========
// Definition
// ==========

namespace test
{
inline void mpuMeasureInterruptRate(MPU_t& mpu, uint16_t rate, int numOfSamples)
{
    const int threshold = 0.05 * rate;  // percentage, 0.05 = 5%
    int count           = 0;
    printf("> Rate to be verified: %d Hz\n", rate);
    printf("> Measuring interrupt rate... wait %d secs\n", numOfSamples);
    TEST_ESP_OK(mpuConfigInterrupt(mpuInterruptCounterISR, (void*) &count));
    vTaskDelay((numOfSamples * 1000) / portTICK_PERIOD_MS);
    TEST_ESP_OK(mpuRemoveInterrupt());
    uint16_t finalRate = round((float) count / numOfSamples);
    printf("> Final measured rate is %d Hz\n", finalRate);
    const int minRate = rate - threshold;
    const int maxRate = rate + threshold;
    TEST_ASSERT((finalRate >= minRate) && (finalRate <= maxRate));
}

}  // namespace test
