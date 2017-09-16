#include "MPU.h"
#include "MPUconfig.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"


static const char* TAG = "MPU_t";

#if defined MPU_ERROR_LOGGER
#define MPU_CHECK(x)                                                                        \
    do {                                                                                    \
        err = x; /* need to create this esp_err_t before */                                 \
        if(!err) {                                                                          \
            ESP_LOGE(TAG,"%s:%d (%s): error=%#x", __FILE__, __LINE__, __FUNCTION__, err);   \
            return err;                                                                     \
        }                                                                                   \
    }while(0)
#else
#define MPU_CHECK(x) x
#endif


esp_err_t MPU_t::init(mpu_address_t addr) {
    esp_err_t err = ESP_OK;
    MPU_CHECK(reset());
    vTaskDelay(100 / portTICK_PERIOD_MS);
    MPU_CHECK(sleep(false));
    config.accel_half = false;
    #ifdef _MPU6500_
    /* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
     * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
     */
    MPU_CHECK(i2cbus->write_bits(addr, MPU6500_REG_ACCEL_CONFIG2,
                                       MPU6500_ACONFIG2_FIFO_SIZE_BIT,
                                       MPU6500_ACONFIG2_FIFO_SIZE_LENGTH,
                                       MPU6500_FIFO_SIZE_1K));
    #endif
    MPU_CHECK(setGyroFSR(MPU_GYRO_FSR_2000DPS));
    MPU_CHECK(setAccelFSR(MPU_ACCEL_FSR_2G));
    MPU_CHECK(setLowPassFilter(MPU_DLPF_42HZ));
    MPU_CHECK(setSampleRate(50));
    MPU_CHECK(setFIFOSensors(0x0));
    #ifdef AK89xx_SECONDARY
    MPU_CHECK(compassInit());
    MPU_CHECK(setCompassSampleRate(10));
    #endif
    MPU_CHECK(setI2CBypass(false));
    MPU_CHECK(setSensors(0x0));
    return err;
}

esp_err_t MPU_t::reset() {
    return i2cbus->write_bit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_DEVICE_RESET_BIT, 1);
}

esp_err_t MPU_t::sleep(bool enable) {
    return i2cbus->write_bit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_SLEEP_BIT, enable);
}

esp_err_t MPU_t::setGyroFSR(mpu_gyro_fsr_t fsr) {
    return i2cbus->write_bits(addr, MPU_REG_GYRO_CONFIG, MPU_GCONFIG_FS_SEL_BIT, MPU_GCONFIG_FS_SEL_LENGTH, fsr);
}

esp_err_t MPU_t::setAccelFSR(mpu_accel_fsr_t fsr) {
    return i2cbus->write_bits(addr, MPU_REG_ACCEL_CONFIG, MPU_ACONFIG_FS_SEL_BIT, MPU_ACONFIG_FS_SEL_LENGTH, fsr);
}
















#define END_MPU
