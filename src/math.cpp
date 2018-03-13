// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/*
 $ Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 */

/**
 * @file math.cpp
 * Implement functions declared in math.hpp.
 */

#include "mpu/math.hpp"
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
uint16_t orientationMatrixToScalar(const int8_t* matrix)
{
    auto rowToScale = [](const int8_t* row) {
        uint16_t b;
        if (row[0] > 0)
            b = 0;
        else if (row[0] < 0)
            b = 4;
        else if (row[1] > 0)
            b = 1;
        else if (row[1] < 0)
            b = 5;
        else if (row[2] > 0)
            b = 2;
        else if (row[2] < 0)
            b = 6;
        else
            b = 7;  // error
        return b;
    };
    uint16_t scalar = 0;
    scalar |= rowToScale(matrix);
    scalar |= rowToScale(matrix + 3) << 3;
    scalar |= rowToScale(matrix + 6) << 6;
    return scalar;
}

axes_q16_t quaternionToEuler(const quat_q30_t& quat)
{
    int32_t t1, t2, t3;
    int32_t q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
    float values[3];

    q00 = q29_multiply(quat[0], quat[0]);
    q01 = q29_multiply(quat[0], quat[1]);
    q02 = q29_multiply(quat[0], quat[2]);
    q03 = q29_multiply(quat[0], quat[3]);
    q11 = q29_multiply(quat[1], quat[1]);
    q12 = q29_multiply(quat[1], quat[2]);
    q13 = q29_multiply(quat[1], quat[3]);
    q22 = q29_multiply(quat[2], quat[2]);
    q23 = q29_multiply(quat[2], quat[3]);
    q33 = q29_multiply(quat[3], quat[3]);

    /* X component of the Ybody axis in World frame */
    t1 = q12 - q03;

    /* Y component of the Ybody axis in World frame */
    t2        = q22 + q00 - (1L << 30);
    values[2] = -atan2f((float) t1, (float) t2) * 180.f / (float) M_PI;

    /* Z component of the Ybody axis in World frame */
    t3        = q23 + q01;
    values[0] = atan2f((float) t3, sqrtf((float) t1 * t1 + (float) t2 * t2)) * 180.f / (float) M_PI;
    /* Z component of the Zbody axis in World frame */
    t2 = q33 + q00 - (1L << 30);
    if (t2 < 0) {
        if (values[0] >= 0)
            values[0] = 180.f - values[0];
        else
            values[0] = -180.f - values[0];
    }

    /* X component of the Xbody axis in World frame */
    t1 = q11 + q00 - (1L << 30);
    /* Y component of the Xbody axis in World frame */
    t2 = q12 + q03;
    /* Z component of the Xbody axis in World frame */
    t3 = q13 - q02;

    values[1] = (atan2f((float) (q33 + q00 - (1L << 30)), (float) (q13 - q02)) * 180.f / (float) M_PI - 90);
    if (values[1] >= 90) values[1] = 180 - values[1];

    if (values[1] < -90) values[1] = -180 - values[1];

    axes_q16_t euler;
    euler[0] = (int32_t)(values[0] * 65536.f);
    euler[1] = (int32_t)(values[1] * 65536.f);
    euler[2] = (int32_t)(values[2] * 65536.f);
    return euler;
}

}  // namespace math

}  // namespace mpud
