// =========================================================================
// Released under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file advanced.cpp
 * This example shows how to:
 *  - Use either SPI or I2C with the same code
 *  - Use the MPU with interrupt signal
 *  - Read sensor data from FIFO
 *  - Perform and check Self-Test
 *  - Calibrate sensor data output through offset registers
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

/* This MACROS are defined in "skdconfig.h" and set through 'menuconfig'.
 * Can use to check which protocol has been selected. */
//# Configurations
#if defined CONFIG_MPU_I2C

#include "I2Cbus.hpp"

static I2C_t&               i2c            = i2c0;
static constexpr gpio_num_t kI2C_SDA       = GPIO_NUM_22;
static constexpr gpio_num_t kI2C_SCL       = GPIO_NUM_23;
static constexpr uint32_t   kI2CClockSpeed = 400000;
#elif defined               CONFIG_MPU_SPI
#include "SPIbus.hpp"
static SPI_t&             spi            = hspi;
static constexpr int      kSPI_MOSI      = 22;
static constexpr int      kSPI_MISO      = 21;
static constexpr int      kSPI_SCLK      = 23;
static constexpr int      kSPI_CS        = 16;
static constexpr uint32_t kSPIClockSpeed = 1000000;
#endif

//# Variables
static MPU_t       MPU;
static const char* TAG = "example";

//# Functions
static void mpuISR(void*);
static void mpuTask(void*);

//# Main
extern "C" void app_main() {
    printf("$ MPU Driver Example: Advanced\n");
    fflush(stdout);
    // Initialize bus through either the Library API or esp-idf API
#if defined CONFIG_MPU_I2C
    i2c.begin(kI2C_SDA, kI2C_SCL, kI2CClockSpeed);
#elif defined CONFIG_MPU_SPI
    spi.begin(kSPI_MOSI, kSPI_MISO, kSPI_SCLK);
#endif
    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 1024, NULL, 5, NULL);
}

//# MPU configuration
static constexpr uint16_t           kSampleRate   = 10;  // Hz
static constexpr mpud::accel_fs_t   kAccelFS      = mpud::ACCEL_FS_4G;
static constexpr mpud::gyro_fs_t    kGyroFS       = mpud::GYRO_FS_500DPS;
static constexpr mpud::dlpf_t       kDLPF         = mpud::DLPF_42HZ;
static constexpr int                kInterruptPin = 17;  // GPIO_NUM_17
static constexpr mpud::int_config_t kInterruptConfig{.level = mpud::INT_LVL_ACTIVE_HIGH,
                                                     .drive = mpud::INT_DRV_PUSHPULL,
                                                     .mode  = mpud::INT_MODE_PULSE50US,
                                                     .clear = mpud::INT_CLEAR_STATUS_REG};

//# MPU Task
static void mpuTask(void*) {
// Let MPU know which bus and adress/handle to use
#if defined CONFIG_MPU_I2C
    MPU.setBus(i2c);
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
#elif defined CONFIG_MPU_SPI
    spi_device_handle_t mpu_spi_handle;
    spi.addDevice(0, kSPIClockSpeed, kSPI_CS, &mpu_spi_handle);
    MPU.setBus(spi);
    MPU.setAddr(mpu_spi_handle);
#endif

    // Verify connection
    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize());

    // Self-Test
    mpud::selftest_t retSelfTest;
    while (esp_err_t err = MPU.selfTest(&retSelfTest)) {
        ESP_LOGE(TAG, "Failed to perform MPU Self-Test, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU Self-Test Result: gyro=%s accel=%s", (retSelfTest & mpud::SELF_TEST_GYRO_FAIL ? "FAIL" : "OK"),
             (retSelfTest & mpud::SELF_TEST_ACCEL_FAIL ? "FAIL" : "OK"));

    // Configure
    ESP_ERROR_CHECK(MPU.setAccelFullScale(kAccelFS));
    ESP_ERROR_CHECK(MPU.setGyroFullScale(kGyroFS));
    ESP_ERROR_CHECK(MPU.setSampleRate(kSampleRate));
    ESP_ERROR_CHECK(MPU.setDigitalLowPassFilter(kDLPF));

    // Calibrate
    mpud::raw_axes_t accelBias, gyroBias;
    ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
    ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));

    // Setup FIFO
    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));
    constexpr uint16_t kFIFOPacketSize = 12;

    // Setup Interrupt
    constexpr gpio_config_t kGPIOConfig{.pin_bit_mask = (uint64_t)0x1 << kInterruptPin,
                                        .mode         = GPIO_MODE_INPUT,
                                        .pull_up_en   = GPIO_PULLUP_DISABLE,
                                        .pull_down_en = GPIO_PULLDOWN_ENABLE,
                                        .intr_type    = GPIO_INTR_POSEDGE};
    gpio_config(&kGPIOConfig);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t)kInterruptPin, mpuISR, xTaskGetCurrentTaskHandle());
    ESP_ERROR_CHECK(MPU.setInterruptConfig(kInterruptConfig));
    ESP_ERROR_CHECK(MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY));

    ESP_ERROR_CHECK(MPU.resetFIFO());  // start clean

    // Reading Loop
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Check FIFO count
        uint16_t fifocount = MPU.getFIFOCount();
        if (fifocount > kFIFOPacketSize) {
            if (!(fifocount % kFIFOPacketSize)) {
                ESP_LOGE(TAG, "Sample Rate too high!, not keeping up the pace!, count: %d", fifocount);
            } else {
                ESP_LOGE(TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", kFIFOPacketSize, fifocount);
            }
            fflush(stdout);
            vTaskDelay(1000 / portTICK_PERIOD_MS);  // delay not really needed, it's just to show the error message
            MPU.resetFIFO();
            continue;
        }
        // Burst read data from FIFO
        uint8_t buffer[kFIFOPacketSize];
        if (esp_err_t err = MPU.readFIFO(kFIFOPacketSize, buffer)) {
            ESP_LOGE(TAG, "Error reading sensor data, %#X", err);
            fflush(stdout);
            vTaskDelay(1000 / portTICK_PERIOD_MS);  // delay not really needed, it's just to show the error message
            MPU.resetFIFO();
            continue;
        }
        // Format
        mpud::raw_axes_t rawAccel, rawGyro;
        rawAccel.x = buffer[0] << 8 | buffer[1];
        rawAccel.y = buffer[2] << 8 | buffer[3];
        rawAccel.z = buffer[4] << 8 | buffer[5];
        rawGyro.x  = buffer[6] << 8 | buffer[7];
        rawGyro.y  = buffer[8] << 8 | buffer[9];
        rawGyro.z  = buffer[10] << 8 | buffer[11];
        // Convert
        mpud::float_axes_t accelG  = mpud::math::accelGravity(rawAccel, kAccelFS);
        mpud::float_axes_t gyroDPS = mpud::math::gyroDegPerSec(rawGyro, kGyroFS);
        // Print
        printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.x, accelG.y, accelG.z);
        printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyroDPS[0], gyroDPS[1], gyroDPS[2]);
    }
    vTaskDelete(NULL);
}

//# MPU Interrupt Service Routine
static void mpuISR(TaskHandle_t taskHandle) {
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}
