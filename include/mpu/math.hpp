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
#include "dmp/types.hpp"
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
static constexpr int16_t kRoomTempOffset = 0;        // LSB
static constexpr float kCelsiusOffset    = 21.f;     // ºC
static constexpr float kTempSensitivity  = 333.87f;  // LSB/ºC
#elif defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
static constexpr int16_t kRoomTempOffset = -521;   // LSB
static constexpr float kCelsiusOffset    = 35.f;   // ºC
static constexpr float kTempSensitivity  = 340.f;  // LSB/ºC
#endif

static constexpr float kTempResolution   = 98.67f / INT16_MAX;
static constexpr float kFahrenheitOffset = kCelsiusOffset * 1.8f + 32;  // ºF

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

/**
 * @brief Convert Orientation Matrix to Scalar.
 */
uint16_t orientationMatrixToScalar(const int8_t* matrix);

inline float q30_to_float(int32_t q30)
{
    return (float) q30 / ((float) (1 << 30));
}

inline double q30_to_double(int32_t q30)
{
    return (double) q30 / ((double) (1 << 30));
}

inline float q16_to_float(int32_t q16)
{
    return (float) q16 / (1 << 16);
}

inline double q16_to_double(int32_t q16)
{
    return (double) q16 / (1 << 16);
}

/**
 * Performs a multiply and shift by 29.
 */
inline int32_t q29_multiply(int32_t a, int32_t b)
{
#if true
    int32_t result;
    result = (int32_t)((float) a * b / (1L << 29));
    return result;
#else
    int64_t temp;
    int32_t result;
    temp   = (int64_t) a * b;
    result = (int32_t)(temp >> 29);
    return result;
#endif
}

inline quat_t q30_to_float(const quat_q30_t& quatQ30)
{
    quat_t quatFloat;
    quatFloat.w = q30_to_float(quatQ30.w);
    quatFloat.x = q30_to_float(quatQ30.x);
    quatFloat.y = q30_to_float(quatQ30.y);
    quatFloat.z = q30_to_float(quatQ30.z);
    return quatFloat;
}

inline float_axes_t q16_to_float(const axes_q16_t& eulerQ16)
{
    float_axes_t eulerFloat;
    eulerFloat.x = q16_to_float(eulerQ16.x);
    eulerFloat.y = q16_to_float(eulerQ16.y);
    eulerFloat.z = q16_to_float(eulerQ16.z);
    return eulerFloat;
}

/**
 *  @brief Body-to-world frame euler angles.
 *  The euler angles are output with the following convention:
 *  Pitch: -180 to 180
 *  Roll: -90 to 90
 *  Yaw: -180 to 180
 */
axes_q16_t quaternionToEuler(const quat_q30_t& quat);

}  // namespace math

}  // namespace mpud

#endif /* end of include guard: _MPU_MATH_HPP_ */
