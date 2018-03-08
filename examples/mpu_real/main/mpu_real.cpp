// =========================================================================
// Released under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu_real.cpp
 * A more 'elaborated' example. Shows how to:
 *  - Use either SPI or I2C in the same code
 *  - Use the MPU with interrupt signal
 *  - Read sensor data from FIFO
 *  - Perform Self-Test check
 *  - Calibrate sensor data output using offset registers
 *  - Calculate Tilt Angles
 * 
 * @note
 * To try this example: \n
 * Set the I2C/SPI pins in 'Bus configuration' and the interrupt pin in 'MPU configuration'.
 *
 * @todo Document the steps
 */

#include <math.h>
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

/* Bus configuration */

// This MACROS are defined in "skdconfig.h" and set through 'menuconfig'.
// Can use to check which protocol has been selected.
#if defined CONFIG_MPU_I2C
#include "I2Cbus.hpp"
static I2C_t& i2c                     = i2c0;  // i2c0 or i2c1
static constexpr gpio_num_t SDA       = GPIO_NUM_14;
static constexpr gpio_num_t SCL       = GPIO_NUM_26;
static constexpr uint32_t CLOCK_SPEED = 400000;  // 400 KHz
#elif defined CONFIG_MPU_SPI
#include "SPIbus.hpp"
static SPI_t& spi                     = hspi;  // hspi or vspi
static constexpr int MOSI             = 22;
static constexpr int MISO             = 21;
static constexpr int SCLK             = 23;
static constexpr int CS               = 16;
static constexpr uint32_t CLOCK_SPEED = 1000000;  // 1MHz
#endif

/* MPU configuration */

static constexpr int kInterruptPin         = 17;  // GPIO_NUM
static constexpr uint16_t kSampleRate      = 250;  // Hz
static constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
static constexpr mpud::gyro_fs_t kGyroFS   = mpud::GYRO_FS_500DPS;
static constexpr mpud::dlpf_t kDLPF        = mpud::DLPF_98HZ;
static constexpr mpud::int_config_t kInterruptConfig{
    .level = mpud::INT_LVL_ACTIVE_HIGH,
    .drive = mpud::INT_DRV_PUSHPULL,
    .mode  = mpud::INT_MODE_PULSE50US,
    .clear = mpud::INT_CLEAR_STATUS_REG  //
};

/*-*/

static const char* TAG = "example";

static void mpuISR(void*);
static void mpuTask(void*);
static void printTask(void*);

// Main
extern "C" void app_main()
{
    printf("$ MPU Driver Example: Advanced\n");
    fflush(stdout);
    // Initialize bus through either the Library API or esp-idf API
#if defined CONFIG_MPU_I2C
    i2c.begin(SDA, SCL, CLOCK_SPEED);
#elif defined CONFIG_MPU_SPI
    spi.begin(MOSI, MISO, SCLK);
#endif
    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 1024, nullptr, 6, nullptr);
    // Create a task to print angles
    xTaskCreate(printTask, "printTask", 2 * 1024, nullptr, 5, nullptr);
}

/* Tasks */

static MPU_t MPU;
float roll{0}, pitch{0}, yaw{0};

static void mpuTask(void*)
{
// Let MPU know which bus and address to use
#if defined CONFIG_MPU_I2C
    MPU.setBus(i2c);
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
#elif defined CONFIG_MPU_SPI
    MPU.setBus(spi);
    spi_device_handle_t mpu_spi_handle;
    spi.addDevice(0, CLOCK_SPEED, CS, &mpu_spi_handle);
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
    ESP_LOGI(TAG, "MPU Self-Test result: Gyro=%s Accel=%s",  //
             (retSelfTest & mpud::SELF_TEST_GYRO_FAIL ? "FAIL" : "OK"),
             (retSelfTest & mpud::SELF_TEST_ACCEL_FAIL ? "FAIL" : "OK"));

    // Calibrate
    mpud::raw_axes_t accelBias, gyroBias;
    ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
    ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));

    // Configure
    ESP_ERROR_CHECK(MPU.setAccelFullScale(kAccelFS));
    ESP_ERROR_CHECK(MPU.setGyroFullScale(kGyroFS));
    ESP_ERROR_CHECK(MPU.setSampleRate(kSampleRate));
    ESP_ERROR_CHECK(MPU.setDigitalLowPassFilter(kDLPF));

    // Setup FIFO
    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));
    constexpr uint16_t kFIFOPacketSize = 12;

    // Setup Interrupt
    constexpr gpio_config_t kGPIOConfig{
        .pin_bit_mask = (uint64_t) 0x1 << kInterruptPin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE  //
    };
    gpio_config(&kGPIOConfig);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t) kInterruptPin, mpuISR, xTaskGetCurrentTaskHandle());
    ESP_ERROR_CHECK(MPU.setInterruptConfig(kInterruptConfig));
    ESP_ERROR_CHECK(MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY));

    // Ready to start reading
    ESP_ERROR_CHECK(MPU.resetFIFO());  // start clean

    // Reading Loop
    while (true) {
        // Wait for notification from mpuISR
        uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notificationValue > 1) {
            ESP_LOGW(TAG, "Task Notification higher than 1, value: %d", notificationValue);
            MPU.resetFIFO();
            continue;
        }
        // Check FIFO count
        uint16_t fifocount = MPU.getFIFOCount();
        if (esp_err_t err = MPU.lastError()) {
            ESP_LOGE(TAG, "Error reading fifo count, %#X", err);
            MPU.resetFIFO();
            continue;
        }
        if (fifocount > kFIFOPacketSize * 2) {
            if (!(fifocount % kFIFOPacketSize)) {
                ESP_LOGE(TAG, "Sample Rate too high!, not keeping up the pace!, count: %d", fifocount);
            }
            else {
                ESP_LOGE(TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", kFIFOPacketSize, fifocount);
            }
            MPU.resetFIFO();
            continue;
        }
        // Burst read data from FIFO
        uint8_t buffer[kFIFOPacketSize];
        if (esp_err_t err = MPU.readFIFO(kFIFOPacketSize, buffer)) {
            ESP_LOGE(TAG, "Error reading sensor data, %#X", err);
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
        // Calculate tilt angle
        // range: (roll[-180,180]  pitch[-90,90]  yaw[-180,180])
        constexpr double kRadToDeg = 57.2957795131;
        constexpr float kDeltaTime = 1.f / kSampleRate;
        float gyroRoll             = roll + mpud::math::gyroDegPerSec(rawGyro.x, kGyroFS) * kDeltaTime;
        float gyroPitch            = pitch + mpud::math::gyroDegPerSec(rawGyro.y, kGyroFS) * kDeltaTime;
        float gyroYaw              = yaw + mpud::math::gyroDegPerSec(rawGyro.z, kGyroFS) * kDeltaTime;
        float accelRoll            = atan2(-rawAccel.x, rawAccel.z) * kRadToDeg;
        float accelPitch = atan2(rawAccel.y, sqrt(rawAccel.x * rawAccel.x + rawAccel.z * rawAccel.z)) * kRadToDeg;
        // Fusion
        roll  = gyroRoll * 0.95f + accelRoll * 0.05f;
        pitch = gyroPitch * 0.95f + accelPitch * 0.05f;
        yaw   = gyroYaw;
        // correct yaw
        if (yaw > 180.f)
            yaw -= 360.f;
        else if (yaw < -180.f)
            yaw += 360.f;
    }
    vTaskDelete(nullptr);
}

static void printTask(void*)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    while (true) {
        printf("Pitch: %+6.1f \t Roll: %+6.1f \t Yaw: %+6.1f \n", pitch, roll, yaw);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}
