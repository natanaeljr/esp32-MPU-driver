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

#ifndef _MPU_MATH_HPP_
#define _MPU_MATH_HPP_

#include <stdint.h>
#include "sdkconfig.h"
#include "mpu/types.hpp"


/* ^^^^^^^^^^^^^^^^^^^^^^
 * Embedded Motion Driver
 * ^^^^^^^^^^^^^^^^^^^^^^ */
namespace emd {

/* ^^^^^^^^^^^^^^^^^^^^^
 * Motion Processor Unit
 * ^^^^^^^^^^^^^^^^^^^^^ */
namespace mpu {

inline namespace math {
constexpr uint8_t accelFSRvalue(const accel_fsr_t fsr) {
    return 2 << fsr;
}

constexpr uint8_t gyroFSRvalue(const gyro_fsr_t fsr) {
    return 250 << fsr;
}

constexpr uint16_t accelSensitivity(const accel_fsr_t fsr) {
    return 16384 >> fsr;
}

constexpr float gyroSensitivity(const gyro_fsr_t fsr) {
    return 131.f / (1 << fsr);
}

constexpr float accelResolution(const accel_fsr_t fsr) {
    return static_cast<float>(accelFSRvalue(fsr) / INT16_MAX);
}

constexpr float gyroResolution(const gyro_fsr_t fsr) {
    return static_cast<float>(gyroFSRvalue(fsr) / INT16_MAX);
}

inline float accelGravity(const int16_t axis, const accel_fsr_t fsr) {
    return axis * accelResolution(fsr);
}

inline float gyroDegPerSec(const int16_t axis, const gyro_fsr_t fsr) {
    return axis * gyroResolution(fsr);
}


#if defined CONFIG_MPU6500 || defined CONFIG_MPU9250
constexpr int16_t kRoomTempOffset = 0;       // LSB
constexpr float kCelsiusOffset = 21.f;       // ºC
constexpr float kTempSensitivity = 333.87f;  // LSB/ºC

#elif defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
constexpr int16_t kRoomTempOffset = -521;    // LSB
constexpr float kCelsiusOffset = 35.f;       // ºC
constexpr float kTempSensitivity = 340.f;    // LSB/ºC
#endif

constexpr float kTempResolution = 98.67f / INT16_MAX;
constexpr float kFahrenheitOffset = kCelsiusOffset * 1.8f + 32;  // ºF


inline float tempCelsius(const int16_t temp) {
    // TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + DegreesCelsius_Offset
    return (temp - kRoomTempOffset) * kTempResolution + kCelsiusOffset;
}

inline float tempFahrenheit(const int16_t temp) {
    return (temp - kRoomTempOffset) * kTempResolution * 1.8f + kFahrenheitOffset;
}

#if defined CONFIG_MPU_AK89xx
inline int16_t magAdjust(const int16_t axis, const uint8_t adjValue) {
    // Hadj = H * ((((ASA - 128) * 0.5) / 128) + 1)
    // return axis * ((((adjValue - 128) * 0.5f) / 128) + 1);
    constexpr float factor = 0.5f / 128;
    return axis * ((adjValue - 128) * factor + 1);
}
#endif
}  // namespace math

}  // namespace mpu

}  // namespace emd



#endif  /* end of include guard: _MPU_MATH_HPP_ */
