#ifndef _MPU_H_
#define _MPU_H_

#include "MPUconfig.h"
#include "MPUregistermap.h"
#include "MPUtypes.h"
#include <stdint.h>
#include "i2cbus.h"
#include "esp_err.h"


const mpu_address_t MPU_DEFAULT_ADDRESS = MPU_ADDRESS_AD0_LOW;

/*******************************************************************************
* MPU_t class
******************************************************************************/

class MPU_t {
private:
    i2cbus_t* i2cbus;
    mpu_address_t addr;
    mpu_config_t config;

public:
    // define addr in constructor // check for i2cbus null poiter
    MPU_t(i2cbus_t* _i2cbus) : i2cbus(_i2cbus) {}
    esp_err_t init(mpu_address_t addr = MPU_DEFAULT_ADDRESS);
    esp_err_t reset();
    esp_err_t sleep(bool enable);
    esp_err_t setSensors(uint8_t mask);
    esp_err_t setGyroFSR(mpu_gyro_fsr_t fsr);
    esp_err_t setAccelFSR(mpu_accel_fsr_t fsr);
    esp_err_t setLowPassFilter(mpu_dlpf_t dlpf);
    esp_err_t setSampleRate(uint16_t rate);
    esp_err_t setFIFOSensors(uint8_t mask);
    esp_err_t setFIFOGyroXYZ(bool enable);
    esp_err_t setFIFOAccelXYZ(bool enable);
    esp_err_t setI2CBypass(bool enable);

private:

#ifdef AK89xx_SECONDARY
    esp_err_t compassInit();
    esp_err_t setCompassSampleRate(uint16_t rate);
#endif
};






#endif /* end of include guard: _MPU_H_ */
