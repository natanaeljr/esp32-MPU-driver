/* =========================================================================
This library is placed under the MIT License
Copyright 2017 Natanael Josue Rabello. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
 ========================================================================= */

#ifndef _MPU_UTILS_HPP_
#define _MPU_UTILS_HPP_

#include <stdint.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"


/* ^^^^^^^^^^^^^^^^^^^^^^
 * Embedded Motion Driver
 * ^^^^^^^^^^^^^^^^^^^^^^ */
namespace emd {

/* ^^^^^^^^^^^^^^^^^^^^^
 * Motion Processor Unit
 * ^^^^^^^^^^^^^^^^^^^^^ */
namespace mpu {

inline namespace utils {
/**
 * @brief: Compute Accelerometer and Gyroscope offsets.
 * 
 * This algorithm takes about ~400ms to compute offsets.
 * When calculating the biases the MPU must remain as horizontal as possible (0 degrees).
 * It is better to call computeOffsets() before any configuration is done (right afeter initialize()).
 * 
 * Note: Gyro offset output are LSB in 1000DPS format.
 * Note: Accel offset output are LSB in 16G format.
 * 
 * */
inline esp_err_t computeOffsets(MPU_t &mpu, raw_axes_t *accel, raw_axes_t *gyro) {
    // backup previous configuration
    const uint16_t prevSampleRate = mpu.getSampleRate();
    if (mpu.lastError()) return mpu.lastError();
    const dlpf_t prevDLPF = mpu.getDigitalLowPassFilter();
    if (mpu.lastError()) return mpu.lastError();
    const accel_fs_t prevAccelFS = mpu.getAccelFullScale();
    if (mpu.lastError()) return mpu.lastError();
    const gyro_fs_t prevGyroFS = mpu.getGyroFullScale();
    if (mpu.lastError()) return mpu.lastError();
    const fifo_config_t prevFIFOConfig = mpu.getFIFOConfig();
    if (mpu.lastError()) return mpu.lastError();
    const bool prevFIFOState = mpu.getFIFOEnabled();
    if (mpu.lastError()) return mpu.lastError();
    // configurations to compute offsets
    constexpr uint16_t kSampleRate = 1000;
    constexpr dlpf_t kDLPF = DLPF_188HZ;
    constexpr accel_fs_t kAccelFS = ACCEL_FS_2G;  // most sensitive
    constexpr gyro_fs_t kGyroFS = GYRO_FS_250DPS;  // most sensitive
    constexpr fifo_config_t kFIFOConfig = FIFO_CFG_ACCEL | FIFO_CFG_GYRO;
    constexpr size_t kPacketSize = 12;
    // setup
    if (mpu.setSampleRate(kSampleRate)) return mpu.lastError();
    if (mpu.setDigitalLowPassFilter(kDLPF)) return mpu.lastError();
    if (mpu.setAccelFullScale(kAccelFS)) return mpu.lastError();
    if (mpu.setGyroFullScale(kGyroFS)) return mpu.lastError();
    if (mpu.setFIFOConfig(kFIFOConfig)) return mpu.lastError();
    if (mpu.setFIFOEnabled(true)) return mpu.lastError();
    // wait for 200ms for sensors to stabilize
    vTaskDelay(200 / portTICK_PERIOD_MS);
    // fill FIFO for 100ms
    if (mpu.resetFIFO()) return mpu.lastError();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (mpu.setFIFOConfig(FIFO_CFG_NONE)) return mpu.lastError();
    // get FIFO count
    const uint16_t fifoCount = mpu.getFIFOCount();
    if (mpu.lastError()) return mpu.lastError();
    const int packetCount = fifoCount / kPacketSize;
    printf("fifocount: %d, packetcount: %d\n", fifoCount, packetCount);
    // read overrun bytes, if any
    const int overrunCount = fifoCount - (packetCount * kPacketSize);
    uint8_t buffer[kPacketSize] = {0};
    if (overrunCount > 0)
        if (mpu.readFIFO(overrunCount, buffer)) return mpu.lastError();
    // fetch data and add up
    axes_t<int> accelAvg, gyroAvg;
    for (int i = 0; i < packetCount; i++) {
        if (mpu.readFIFO(kPacketSize, buffer)) return mpu.lastError();
        // retrieve data
        raw_axes_t accelCur, gyroCur;
        accelCur.x = (buffer[0] << 8) | buffer[1];
        accelCur.y = (buffer[2] << 8) | buffer[3];
        accelCur.z = (buffer[4] << 8) | buffer[5];
        gyroCur.x  = (buffer[6] << 8) | buffer[7];
        gyroCur.y  = (buffer[8] << 8) | buffer[9];
        gyroCur.z  = (buffer[10] << 8) | buffer[11];
        // add up
        accelAvg.x += accelCur.x;
        accelAvg.y += accelCur.y;
        accelAvg.z += accelCur.z;
        gyroAvg.x += gyroCur.x;
        gyroAvg.y += gyroCur.y;
        gyroAvg.z += gyroCur.z;
    }
    // calculate average
    accelAvg.x /= packetCount;
    accelAvg.y /= packetCount;
    accelAvg.z /= packetCount;
    gyroAvg.x /= packetCount;
    gyroAvg.y /= packetCount;
    gyroAvg.z /= packetCount;
    // remove gravity from Accel Z axis
    constexpr uint16_t gravityLSB = INT16_MAX >> (kAccelFS + 1);
    accelAvg.z -= gravityLSB;
    // save offsets (inverted) in 16G and 1000DPS format
    for (int i = 0; i < 3; i++) {
        (*accel)[i] = - (accelAvg[i] >> (types::ACCEL_FS_16G - kAccelFS));
        (*gyro)[i] = - (gyroAvg[i] >> (types::GYRO_FS_1000DPS - kGyroFS));
    }
    // set back previous configs
    if (mpu.setSampleRate(prevSampleRate)) return mpu.lastError();
    if (mpu.setDigitalLowPassFilter(prevDLPF)) return mpu.lastError();
    if (mpu.setAccelFullScale(prevAccelFS)) return mpu.lastError();
    if (mpu.setGyroFullScale(prevGyroFS)) return mpu.lastError();
    if (mpu.setFIFOConfig(prevFIFOConfig)) return mpu.lastError();
    if (mpu.setFIFOEnabled(prevFIFOState)) return mpu.lastError();
    return ESP_OK;
}
}  // namespace utils

}  // namespace mpu

}  // namespace emd



#endif  /* end of include guard: _MPU_UTILS_HPP_ */
