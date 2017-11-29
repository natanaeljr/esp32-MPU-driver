#ifndef _MPU_HPP_
#define _MPU_HPP_


#include <stdint.h>
#include "sdkconfig.h"

// Select the communication protocol in 'menuconfig', default = I2C
#ifdef CONFIG_MPU_I2C
 /* #if !defined I2CBUS_COMPONENT_NAME
  #error ""MPU component requires I2Cbus library. \
  Make sure the I2Cbus library is included in your components directory. \
  See MPUs README.md for more information.""
 #endif */
 #include "I2Cbus.hpp"
#elif CONFIG_MPU_SPI
 #if !defined SPIBUS_COMPONENT_NAME
  #error ""MPU component requires SPIbus library. \
  Make sure the SPIbus library is included in your components directory. \
  See MPUs README.md for more information.""
 #endif
 #include "SPIbus.hpp"
#else
 #error ""MPU communication protocol not specified""
#endif

#include "mpu_types.hpp"
#include "esp_err.h"


/* Some #defines regarding chip model are
 * declared (passed to preprocessor) in 'component.mk'
 *
 * MPU9250 is the same as MPU6500 + AK8963
 * MPU9150 is the same as MPU6050 + AK8975
 * */

const mpu_i2caddr_t MPU_DEFAULT_ADDRESS = MPU_ADDRESS_AD0_LOW;

/******************************************************************************
* MPU class
******************************************************************************/
class MPU_t {

private:
    mpu_bus_t& BUS;
    mpu_addr_handle_t addrHandle;
    esp_err_t err = ESP_OK;
    uint8_t buffer[16] = {0};
    class DMP_t; // definition is in 'dmp.hpp'
    DMP_t* DMP = nullptr;
    
public:
    MPU_t(mpu_bus_t& BUS = MPU_DEFAULT_BUS_HOST);
    ~MPU_t();
    void setBus(mpu_bus_t& BUS);
    mpu_bus_t& getBus();
    void setAddressHandle(mpu_addr_handle_t addrHandle);
    mpu_addr_handle_t getAddressHandle();
    esp_err_t getLastError();
    DMP_t& dmp() {return *DMP;}

    // SETUP
    esp_err_t initialize();
    esp_err_t reset();
    esp_err_t sleep(bool enable);
    bool getSleepStatus();
    bool testConnection();
    uint8_t getDeviceID();

    // CONFIGS
    esp_err_t setClockSource(mpu_clock_src_t clockSrc);
    mpu_clock_src_t getClockSource();
    esp_err_t setLowPassFilter(mpu_lpf_t lpf);
    mpu_lpf_t getLowPassFilter();
    esp_err_t setSampleRate(uint16_t rate);
    uint16_t getSampleRate();
    esp_err_t setLowPowerAccelMode(bool enable);
    bool getLowPowerAccelMode();
    esp_err_t setLowPowerAccelRate(mpu_lp_accel_rate_t rate);
    mpu_lp_accel_rate_t getLowPowerAccelRate();
    #ifdef CONFIG_MPU6050
    esp_err_t setAuxVDDIOLevel(mpu_auxvddio_lvl_t level);
    mpu_auxvddio_lvl_t getAuxVDDIOLevel();
    #endif

    // I2C
    esp_err_t setI2CBypass(bool enable);
    bool getI2CBypass();
    esp_err_t setAuxI2CMaster(bool enable);
    bool getAuxI2CMaster();
    
    // GYROSCOPE
    esp_err_t setGyroFullScale(mpu_gyro_fsr_t fs);
    mpu_gyro_fsr_t getGyroFullScale();
    mpu_axes_t getRotation();
    esp_err_t getRotation(int16_t *x, int16_t *y, int16_t *z);
    
    // ACCELEROMETER
    esp_err_t setAccelFullScale(mpu_accel_fsr_t fs);
    mpu_accel_fsr_t getAccelFullScale();
    mpu_axes_t getAcceleration();
    esp_err_t getAcceleration(int16_t *x, int16_t *y, int16_t *z);

    // MOTION
    esp_err_t getMotion6(mpu_axes_t *accel, mpu_axes_t *gyro);
    esp_err_t getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

    // TEMPERATURE
    esp_err_t setTemperatureEnabled(bool enable);
    bool getTemperatureEnabled();
    int16_t getTemperature();
    int32_t getTemperatureC();
    // int32_t getTemperatureF();
    
    // FIFO
    esp_err_t setFIFOEnabled(bool enable);
    bool getFIFOEnabled();
    esp_err_t setFIFOSensorsEnabled(mpu_fifo_sensors_t sensors);
    mpu_fifo_sensors_t getFIFOSensorsEnabled();
    esp_err_t writeFIFOByte(uint8_t data);
    uint8_t readFIFOByte();
    esp_err_t readFIFO(uint8_t *data, size_t length);
    uint16_t getFIFOCount();
    esp_err_t resetFIFO(); // TODO: change to clear FIFO
    
    // INTERRUPT
    esp_err_t setIntConfig(mpu_int_config_t intConfig);
    mpu_int_config_t getIntConfig();
    esp_err_t setIntEnabled(mpu_int_t mask);
    mpu_int_t getIntEnabled();
    mpu_int_t getIntStatus();
    bool getIntFIFOOverflowStatus();
    bool getIntDataReadyStatus();

    // COMPASS


    // MEMORY
    esp_err_t writeMemory(uint16_t memAddr, uint16_t length, const uint8_t *data);
    esp_err_t readMemory(uint16_t memAddr, uint16_t length, uint8_t *data);
    esp_err_t setProgramStartAddress(uint16_t prgmAddr);
    uint16_t getProgramStartAddress();

    // OTHERS
    esp_err_t registerDump();
    esp_err_t memoryDump(int_fast8_t bank = -1);

    

private:
    #ifdef AK89xx_SECONDARY
    esp_err_t compassInit();
    esp_err_t setCompassSampleRate(uint16_t rate);
    #endif
    
    // REGISTERS
    esp_err_t readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    esp_err_t readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
    esp_err_t readByte(uint8_t regAddr, uint8_t *data);
    esp_err_t readBytes(uint8_t regAddr, size_t length, uint8_t *data);
    esp_err_t writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
    esp_err_t writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    esp_err_t writeByte(uint8_t regAddr, uint8_t data);
    esp_err_t writeBytes(uint8_t regAddr, size_t length, const uint8_t *data);

    
}; /* end of class MPU_t */








/*********************************
 * Inline methods implementation
 *********************************/

inline esp_err_t MPU_t::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    err = BUS.readBit(addrHandle, regAddr, bitNum, data);
    return err;
}
inline esp_err_t MPU_t::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    err = BUS.readBits(addrHandle, regAddr, bitStart, length, data);
    return err;
}
inline esp_err_t MPU_t::readByte(uint8_t regAddr, uint8_t *data) {
    err = BUS.readByte(addrHandle, regAddr, data);
    return err;
}
inline esp_err_t MPU_t::readBytes(uint8_t regAddr, size_t length, uint8_t *data) {
    err = BUS.readBytes(addrHandle, regAddr, length, data);
    return err;
}
inline esp_err_t MPU_t::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    err = BUS.writeBit(addrHandle, regAddr, bitNum, data);
    return err;
}
inline esp_err_t MPU_t::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    err = BUS.writeBits(addrHandle, regAddr, bitStart, length, data);
    return err;
}
inline esp_err_t MPU_t::writeByte(uint8_t regAddr, uint8_t data) {
    err = BUS.writeByte(addrHandle, regAddr, data);
    return err;
}
inline esp_err_t MPU_t::writeBytes(uint8_t regAddr, size_t length, const uint8_t *data) {
    err = BUS.writeBytes(addrHandle, regAddr, length, data);
    return err;
}





#endif /* end of include guard: _MPU_HPP_ */
