// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu/math.hpp
 * @brief MPU Math helper file
 */

#ifndef _MPU_MATH_HPP_
#define _MPU_MATH_HPP_

#include <math.h>
#include <stdint.h>
#include "mpu/types.hpp"
#include "sdkconfig.h"

/*! MPU Driver namespace */
namespace mpud
{
/*! Math namespace */
inline namespace math
{
//
inline uint8_t accelFSRvalue(const accel_fs_t fs)
{
    return 2 << fs;
}

inline uint16_t gyroFSRvalue(const gyro_fs_t fs)
{
    return 250 << fs;
}

inline uint16_t accelSensitivity(const accel_fs_t fs)
{
    return 16384 >> fs;
}

inline float gyroSensitivity(const gyro_fs_t fs)
{
    return 131.f / (1 << fs);
}

inline float accelResolution(const accel_fs_t fs)
{
    return static_cast<float>(accelFSRvalue(fs)) / INT16_MAX;
}

inline float gyroResolution(const gyro_fs_t fs)
{
    return static_cast<float>(gyroFSRvalue(fs)) / INT16_MAX;
}

inline float accelGravity(const int16_t axis, const accel_fs_t fs)
{
    return axis * accelResolution(fs);
}

inline float_axes_t accelGravity(const raw_axes_t& raw_axes, const accel_fs_t fs)
{
    float_axes_t axes;
    axes.x = raw_axes.x * accelResolution(fs);
    axes.y = raw_axes.y * accelResolution(fs);
    axes.z = raw_axes.z * accelResolution(fs);
    return axes;
}

inline float gyroDegPerSec(const int16_t axis, const gyro_fs_t fs)
{
    return axis * gyroResolution(fs);
}

inline float_axes_t gyroDegPerSec(const raw_axes_t& raw_axes, const gyro_fs_t fs)
{
    float_axes_t axes;
    axes.x = raw_axes.x * gyroResolution(fs);
    axes.y = raw_axes.y * gyroResolution(fs);
    axes.z = raw_axes.z * gyroResolution(fs);
    return axes;
}

inline float gyroRadPerSec(const int16_t axis, const gyro_fs_t fs)
{
    return (M_PI / 180) * gyroDegPerSec(axis, fs);
}

inline float_axes_t gyroRadPerSec(const raw_axes_t& raw_axes, const gyro_fs_t fs)
{
    float_axes_t axes;
    axes.x = (M_PI / 180) * gyroDegPerSec(raw_axes.x, fs);
    axes.y = (M_PI / 180) * gyroDegPerSec(raw_axes.y, fs);
    axes.z = (M_PI / 180) * gyroDegPerSec(raw_axes.z, fs);
    return axes;
}

#if defined CONFIG_MPU6500 || defined CONFIG_MPU9250
constexpr int16_t kRoomTempOffset = 0;        // LSB
constexpr float kCelsiusOffset    = 21.f;     // ºC
constexpr float kTempSensitivity  = 333.87f;  // LSB/ºC
#elif defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
constexpr int16_t kRoomTempOffset = -521;   // LSB
constexpr float kCelsiusOffset    = 35.f;   // ºC
constexpr float kTempSensitivity  = 340.f;  // LSB/ºC
#endif

constexpr float kTempResolution   = 98.67f / INT16_MAX;
constexpr float kFahrenheitOffset = kCelsiusOffset * 1.8f + 32;  // ºF

inline float tempCelsius(const int16_t temp)
{
    // TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + DegreesCelsius_Offset
    return (temp - kRoomTempOffset) * kTempResolution + kCelsiusOffset;
}

inline float tempFahrenheit(const int16_t temp)
{
    return (temp - kRoomTempOffset) * kTempResolution * 1.8f + kFahrenheitOffset;
}

#if defined CONFIG_MPU_AK89xx
inline int16_t magAdjust(const int16_t axis, const uint8_t adjValue)
{
    // Hadj = H * ((((ASA - 128) * 0.5) / 128) + 1)
    // return axis * ((((adjValue - 128) * 0.5f) / 128) + 1);
    constexpr float factor = 0.5f / 128;
    return axis * ((adjValue - 128) * factor + 1);
}
#endif

}  // namespace math

}  // namespace mpud

#endif /* end of include guard: _MPU_MATH_HPP_ */
