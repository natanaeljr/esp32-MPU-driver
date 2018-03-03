// =========================================================================
// Released under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu_spi.cpp
 * Example on how to setup MPU through SPI for basic usage.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "MPU.hpp"
#include "SPIbus.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

static const char* TAG = "example";

static constexpr int MOSI = 22;
static constexpr int MISO = 21;
static constexpr int SCLK = 23;
static constexpr int CS = 16;
static constexpr uint32_t CLOCK_SPEED = 1000000;  // up to 1MHz for all registers, and 20MHz for sensor data registers only

extern "C" void app_main() {
    printf("$ MPU Driver Example: MPU-SPI\n");
    fflush(stdout);

    spi_device_handle_t mpu_spi_handle;
    // Initialize SPI on HSPI host through SPIbus interface:
    hspi.begin(MOSI, MISO, SCLK);
    hspi.addDevice(0, CLOCK_SPEED, CS, &mpu_spi_handle);

    // Or directly with esp-idf API:
    /*
    spi_bus_config_t spi_config;
    spi_config.mosi_io_num = MOSI;
    spi_config.miso_io_num = MISO;
    spi_config.sclk_io_num = SCLK;
    spi_config.quadwp_io_num = -1;
    spi_config.quadhd_io_num = -1;
    spi_config.max_transfer_sz = SPI_MAX_DMA_LEN;
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_config, 0));
    spi_device_interface_config_t dev_config;
    dev_config.command_bits = 0;
    dev_config.address_bits = 8;
    dev_config.dummy_bits = 0;
    dev_config.mode = 0;
    dev_config.duty_cycle_pos = 128;
    dev_config.cs_ena_pretrans = 0; 
    dev_config.cs_ena_posttrans = 0;
    dev_config.clock_speed_hz = CLOCK_SPEED;
    dev_config.spics_io_num = CS;
    dev_config.flags = 0;
    dev_config.queue_size = 1;
    dev_config.pre_cb = NULL;
    dev_config.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &mpu_spi_handle));
    */

    MPU_t MPU;  // create a default MPU object
    MPU.setBus(hspi);  // set bus port, not really needed here since default is HSPI
    MPU.setAddr(mpu_spi_handle);  // set spi_device_handle, always needed!

    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize());  // initialize the chip and set initial configurations
    // Setup with your configurations
    // ESP_ERROR_CHECK(MPU.setSampleRate(50));  // set sample rate to 50 Hz
    // ESP_ERROR_CHECK(MPU.setGyroFullScale(mpud::GYRO_FS_500DPS));
    // ESP_ERROR_CHECK(MPU.setAccelFullScale(mpud::ACCEL_FS_4G));

    // Reading sensor data
    printf("Reading sensor data:\n");
    mpud::raw_axes_t accelRaw;   // x, y, z axes as int16
    mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
    mpud::float_axes_t accelG;   // accel axes in (g) gravity format
    mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    while (true) {
        // Read
        MPU.acceleration(&accelRaw);  // fetch raw data from the registers
        MPU.rotation(&gyroRaw);       // fetch raw data from the registers
        // MPU.motion(&accelRaw, &gyroRaw);  // read both in one shot
        // Convert
        accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
        gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
        // Debug
        printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.x, accelG.y, accelG.z);
        printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyroDPS[0], gyroDPS[1], gyroDPS[2]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
