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

#ifndef _MPU_LOG_HPP_
#define _MPU_LOG_HPP_

/********************************************************************
 * This header is intended to be used ONLY inside the library itself
 * Do not include this file in your application.
 ********************************************************************/

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

// Note: declare TAG before include this header
// Note: include only in .cpp files from this library

#define MPU_LOGE(format, ...) if (CONFIG_MPU_LOG_LEVEL >= ESP_LOG_ERROR)   { ESP_LOGE(TAG, format, ##__VA_ARGS__); }
#define MPU_LOGW(format, ...) if (CONFIG_MPU_LOG_LEVEL >= ESP_LOG_WARN)    { ESP_LOGW(TAG, format, ##__VA_ARGS__); }
#define MPU_LOGI(format, ...) if (CONFIG_MPU_LOG_LEVEL >= ESP_LOG_INFO)    { ESP_LOGI(TAG, format, ##__VA_ARGS__); }
#define MPU_LOGD(format, ...) if (CONFIG_MPU_LOG_LEVEL >= ESP_LOG_DEBUG)   { ESP_LOGD(TAG, format, ##__VA_ARGS__); }
#define MPU_LOGV(format, ...) if (CONFIG_MPU_LOG_LEVEL >= ESP_LOG_VERBOSE) { ESP_LOGV(TAG, format, ##__VA_ARGS__); }

#define MPU_LOGEMSG(msg, format, ...) MPU_LOGE("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define MPU_LOGWMSG(msg, format, ...) MPU_LOGW("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define MPU_LOGIMSG(msg, format, ...) MPU_LOGI("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define MPU_LOGDMSG(msg, format, ...) MPU_LOGD("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define MPU_LOGVMSG(msg, format, ...) MPU_LOGV("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)

#ifdef MPU_LOG_ERROR_TRACES
#define MPU_ERR_CHECK(x) emd::mpu::log::errorCheckLogger(x, __ASSERT_FUNC, __LINE__, #x)
#else
#define MPU_ERR_CHECK(x) (x)
#endif

/* ^^^^^^^^^^^^^^^^^^^^^^
 * Embedded Motion Driver
 * ^^^^^^^^^^^^^^^^^^^^^^ */
namespace emd {

/* ^^^^^^^^^^^^^^^^^^^^^
 * Motion Processor Unit
 * ^^^^^^^^^^^^^^^^^^^^^ */
namespace mpu {

inline namespace log {
namespace msgs {

static const char INVALID_ARG[]           = "Invalid Argument";
static const char INVALID_STATE[]         = "Invalid State";
static const char INVALID_LENGTH[]        = "Invalid length";
static const char INVALID_FIFO_RATE[]     = "Invalid FIFO rate";
static const char INVALID_SAMPLE_RATE[]   = "Invalid Sample rate";
static const char INVALID_TAP_THRESH[]    = "Invalid Tap threshold";
static const char DMP_LOAD_FAIL[]         = "Failed to load DMP firmware";
static const char DMP_NOT_LOADED[]        = "DMP firmware has not been loaded";
static const char UNKNOWN_DMP_CFG_STATE[] = "Unknown DMP config state";
static const char NO_AXIS_PASSED[]        = "No Axis passed";
static const char BANK_BOUNDARIES[]       = "Bank boundaries overpass";
static const char FIFO_CORRUPTION[]       = "FIFO Corruption. Quaternion data outside of the acceptable threshold";
static const char AUX_I2C_DISABLED[]      = "Auxiliary I2C is disabled";
static const char AUX_I2C_SLAVE_NACK[]    = "Auxiliary I2C Slave NACK";
static const char AUX_I2C_LOST_ARB[]      = "Auxiliary I2C Master loose abitraion of the bus";
static const char COMPASS_DISABLED[]      = "Compass is disabled";
static const char NOT_SUPPORTED[]         = "Not supported";
static const char TIMEOUT[]               = "Timeout";
static const char EMPTY[]                 = "";

}  // namespace msgs

static inline esp_err_t errorCheckLogger(esp_err_t x, const char* func, const int line, const char* expr) {
    if (x)
        MPU_LOGE("func:%s @ line:%d, expr:\"%s\", error:0x%X ", func, line, expr, x);
    return x;
}
}  // namespace log

}  // namespace mpu

}  // namespace emd



#endif  /* end of include guard: _MPU_LOG_HPP_ */
