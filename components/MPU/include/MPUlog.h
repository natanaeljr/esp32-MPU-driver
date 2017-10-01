#ifndef _MPU_LOG_H_
#define _MPU_LOG_H_

#include "esp_log.h"
#include "esp_err.h"

/********************************************************************
 * This header is intended to be used ONLY inside the library itself
 * Do not include this file in your application.
 ********************************************************************/

// declare MPU_TAG before include header
// include only in .cpp files from this library

#ifdef CONFIG_MPU_LOG_ERRORS // TODO add log selection in Kconfig
#define MPU_LOGE(format, ... )  ESP_LOGE(MPU_TAG, format, ##__VA_ARGS__)
#define MPU_LOGW(format, ... )  ESP_LOGW(MPU_TAG, format, ##__VA_ARGS__)
#define MPU_LOGI(format, ... )  ESP_LOGI(MPU_TAG, format, ##__VA_ARGS__)
#define MPU_LOGD(format, ... )  ESP_LOGD(MPU_TAG, format, ##__VA_ARGS__)
#define MPU_LOGV(format, ... )  ESP_LOGV(MPU_TAG, format, ##__VA_ARGS__)
#else
#define MPU_LOGE(format, ... )  (void)0
#define MPU_LOGW(format, ... )  (void)0
#define MPU_LOGI(format, ... )  (void)0
#define MPU_LOGD(format, ... )  (void)0
#define MPU_LOGV(format, ... )  (void)0
#endif

#ifdef CONFIG_MPU_LOG_ERRORS
#define MPU_LOGEMSG(msg)    MPU_LOGE("%s() -> " msg, __FUNCTION__) 
#endif

#ifdef CONFIG_MPU_LOG_ERRORS
static inline esp_err_t mpuLogErrorCheck(esp_err_t x, const char* func, const int line, const char* expr) {
    if(x)  
        MPU_LOGE("func:%s @ line:%d, expr:\"%s\", error:0x%X ", func, line, expr, x);
    return x;
}
#define MPU_ERR_CHECK(x)  mpuLogErrorCheck(x, __ASSERT_FUNC, __LINE__, #x)
#else
#define MPU_ERR_CHECK(x)  x
#endif





#endif /* end of include guard: _MPU_LOG_H_ */