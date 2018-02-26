// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu/utils.hpp
 * @brief MPU Utilities
 */

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

/*! MPU Driver namespace */
namespace mpud {

/*! Utilities namespace */
inline namespace utils {
}  // namespace utils

}  // namespace mpud

#endif  /* end of include guard: _MPU_UTILS_HPP_ */
