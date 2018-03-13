// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file dmp/types.hpp
 * Declare Types and Definitions for MPUdmp.
 */

#ifndef _DMP_TYPES_HPP_
#define _DMP_TYPES_HPP_

#include <stdint.h>
#include "sdkconfig.h"

/*! MPU Driver namespace */
namespace mpud
{
/*! Types namespace */
inline namespace types
{
/*! DMP Interrupt mode */
typedef enum {
    DMP_INT_MODE_CONTINUOUS = 0,  //! Signals every FIFO period, normal mode.
    DMP_INT_MODE_GESTURE    = 1   //! Signals when a Tap event has been detected.
} dmp_int_mode_t;

/**
 * @brief Enable DMP features
 * Combine features with _OR operator_ `|` and send them for enabling.
 * @note
 *  - DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually exclusive.
 *  - DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also mutually exclusive.
 *  - DMP_FEATURE_PEDOMETER is always enabled.
 */
typedef uint16_t dmp_feature_t;
static constexpr dmp_feature_t DMP_FEATURE_NONE           = {0x000};  //!< Disable all Features
static constexpr dmp_feature_t DMP_FEATURE_TAP            = {0x001};  //!< Tap Gesture Recognition
static constexpr dmp_feature_t DMP_FEATURE_ANDROID_ORIENT = {0x002};  //!< Orientation Gesture Recognition
static constexpr dmp_feature_t DMP_FEATURE_PEDOMETER      = {0x008};  //!< Pedometer Gesture Recognition. Always Enabled
static constexpr dmp_feature_t DMP_FEATURE_LP_3X_QUAT     = {0x004};  //!< Low-Power 3-axis (Gyro only) Quaternions
static constexpr dmp_feature_t DMP_FEATURE_LP_6X_QUAT     = {0x010};  //!< Low-Power 6 axis (Gyro and Accel) Quaternions
static constexpr dmp_feature_t DMP_FEATURE_GYRO_CAL       = {0x020};  //!< Constant Gyroscope Calibration
static constexpr dmp_feature_t DMP_FEATURE_SEND_RAW_ACCEL = {0x040};  //!< Raw Accelerometer data
static constexpr dmp_feature_t DMP_FEATURE_SEND_RAW_GYRO  = {0x080};  //!< Raw Gyroscope data
static constexpr dmp_feature_t DMP_FEATURE_SEND_CAL_GYRO  = {0x100};  //!< Calibrated Gyroscope data

/*! DMP Tap Axes */
typedef uint8_t dmp_tap_axis_t;
static constexpr dmp_tap_axis_t DMP_TAP_X   = {0x30};
static constexpr dmp_tap_axis_t DMP_TAP_Y   = {0x0C};
static constexpr dmp_tap_axis_t DMP_TAP_Z   = {0x03};
static constexpr dmp_tap_axis_t DMP_TAP_XYZ = {0x3F};

/*! DMP Tap Configuration */
typedef struct
{
    uint16_t threshold_X;     //!< Tap threshold for accel X axis in mg/ms, (max: 1600).
    uint16_t threshold_Y;     //!< Tap threshold for accel Y axis in mg/ms, (max: 1600).
    uint16_t threshold_Z;     //!< Tap threshold for accel Z axis in mg/ms, (max: 1600).
    uint8_t count;            //!< Minimum number of consecutive taps needed for an interrupt (1-4).
    uint16_t time;            //!< Length between valid taps, in milliseconds.
    uint16_t time_multi_tap;  //!< Maximum time between taps to register as a multi-tap.
    struct
    {
        uint16_t threshold; /*!< Shake rejection threshold in DPS.
                             * If the DMP detects a gyro sample larger than `threshold`, taps are rejected. */
        uint16_t time;      /*!< Shake rejection time in milliseconds.
                             * Sets the length of time that the gyro must be outside of the threshold
                             * before taps are rejected. A mandatory 60 ms is added to this parameter. */
        uint16_t timeout;   /*!< Shake rejection timeout in milliseconds.
                             * Sets the length of time after a shake rejection that the gyro must stay
                             * inside of the `threshold` before taps can be detected again.
                             * A mandatory 60 ms is added to this parameter. */
    } shake_reject;
} dmp_tap_config_t;

/*! Quaternion struct */
template <class T>
struct Quaternion
{
    union
    {
        T wxyz[4];
        struct
        {
            T w;
            T x;
            T y;
            T z;
        };
    };
    Quaternion() : w{1}, x{0}, y{0}, z{0} {}
    Quaternion(T w, T x, T y, T z) : w{w}, x{x}, y{y}, z{z} {}
    T& operator[](int i) { return wxyz[i]; }
    const T& operator[](int i) const { return wxyz[i]; }
};
/* Readymade Quaternion types */
typedef Quaternion<int32_t> quat_q30_t;  //!< Quaternion in q30 fixed point.
typedef Quaternion<float> quat_t;        //!< Quaternion in floating point (float) format.

}  // namespace types

}  // namespace mpud

#endif /* end of include guard: _DMP_TYPES_HPP_ */
