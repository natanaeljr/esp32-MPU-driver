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

#ifndef _MPU_HPP_
#define _MPU_HPP_

#include <stdint.h>
#include "sdkconfig.h"
#include "esp_err.h"

/**
 * MPU library requires I2Cbus or SPIbus library
 * Select the communication protocol in 'menuconfig'
 * and include the corresponding library to project components
 * */
#ifdef CONFIG_MPU_I2C
#if !defined I2CBUS_COMPONENT_TRUE
#error ''MPU component requires I2Cbus library. \
Make sure the I2Cbus library is included in your components directory. \
See MPUs README.md for more information.''
#endif
#include "I2Cbus.hpp"
#elif CONFIG_MPU_SPI
#if !defined SPIBUS_COMPONENT_TRUE
#error ''MPU component requires SPIbus library. \
Make sure the SPIbus library is included in your components directory. \
See MPUs README.md for more information.''
#endif
#include "SPIbus.hpp"
#else
#error ''MPU communication protocol not specified''
#endif

#include "mpu/types.hpp"


/**
 * Important!
 * MPU9250 is the same as MPU6500 + AK8963
 * MPU9150 is the same as MPU6050 + AK8975
 * MPU6000 code is the same as MPU6050
 * MPU6555 equals MPU6500, MPU9255 equals MPU9250
 * */


/* ^^^^^^^^^^^^^^^^^^^^^^
 * Embedded Motion Driver
 * ^^^^^^^^^^^^^^^^^^^^^^ */
namespace emd {

/* ^^^^^^^^^^^^^^^^^^^^^
 * Motion Processor Unit
 * ^^^^^^^^^^^^^^^^^^^^^ */
namespace mpu {

/* ^^^^^^^^^
 * class MPU
 * ^^^^^^^^^ */
class MPU {
 public:
    MPU();
    explicit MPU(mpu_bus_t& bus);
    MPU(mpu_bus_t& bus, mpu_addr_handle_t addr);
    ~MPU();

    MPU& setBus(mpu_bus_t& bus);
    MPU& setAddr(mpu_addr_handle_t addr);
    mpu_bus_t& getBus();
    mpu_addr_handle_t getAddr();
    esp_err_t lastError();

    esp_err_t initialize();
    esp_err_t reset();
    esp_err_t setSleep(bool enable);
    esp_err_t testConnection();
    uint8_t whoAmI();
    bool getSleep();

    esp_err_t setSampleRate(uint16_t rate);
    esp_err_t setClockSource(clock_src_t clockSrc);
    esp_err_t setDigitalLowPassFilter(dlpf_t dlpf);
    uint16_t getSampleRate();
    clock_src_t getClockSource();
    dlpf_t getDigitalLowPassFilter();

    esp_err_t setLowPowerAccelMode(bool enable);
    esp_err_t setLowPowerAccelRate(lp_accel_rate_t rate);
    lp_accel_rate_t getLowPowerAccelRate();
    bool getLowPowerAccelMode();

    esp_err_t setStandbyMode(stby_en_t mask);
    stby_en_t getStandbyMode();
    esp_err_t resetSignalPath();

    esp_err_t setGyroFullScale(gyro_fs_t fsr);
    esp_err_t setAccelFullScale(accel_fs_t fsr);
    gyro_fs_t getGyroFullScale();
    accel_fs_t getAccelFullScale();

    esp_err_t setGyroOffset(raw_axes_t bias);
    esp_err_t setAccelOffset(raw_axes_t bias);
    raw_axes_t getGyroOffset();
    raw_axes_t getAccelOffset();

    esp_err_t setInterruptConfig(int_config_t config);
    esp_err_t setInterruptEnabled(int_en_t mask);
    int_stat_t getInterruptStatus();
    int_config_t getInterruptConfig();
    int_en_t getInterruptEnabled();

    esp_err_t setFIFOMode(fifo_mode_t mode);
    esp_err_t setFIFOConfig(fifo_config_t config);
    esp_err_t setFIFOEnabled(bool enable);
    esp_err_t resetFIFO();
    uint16_t getFIFOCount();
    esp_err_t readFIFO(size_t length, uint8_t *data);
    esp_err_t writeFIFO(size_t length, const uint8_t *data);
    fifo_mode_t getFIFOMode();
    fifo_config_t getFIFOConfig();
    bool getFIFOEnabled();

    esp_err_t setAuxI2CConfig(const auxi2c_config_t &config);
    esp_err_t setAuxI2CEnabled(bool enable);
    esp_err_t setAuxI2CSlaveConfig(const auxi2c_slv_config_t &config);
    esp_err_t setAuxI2CSlaveEnabled(auxi2c_slv_t slave, bool enable);
    esp_err_t setAuxI2CBypass(bool enable);
    esp_err_t readAuxI2CRxData(size_t length, uint8_t *data, size_t skip = 0);
    esp_err_t restartAuxI2C();
    auxi2c_stat_t getAuxI2CStatus();
    auxi2c_config_t getAuxI2CConfig();
    auxi2c_slv_config_t getAuxI2CSlaveConfig(auxi2c_slv_t slave);
    bool getAuxI2CEnabled();
    bool getAuxI2CSlaveEnabled(auxi2c_slv_t slave);
    bool getAuxI2CBypass();
    esp_err_t auxI2CWriteByte(uint8_t devAddr, uint8_t regAddr, const uint8_t data);
    esp_err_t auxI2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);

    esp_err_t setFsyncConfig(int_lvl_t level);
    esp_err_t setFsyncEnabled(bool enable);
    int_lvl_t getFsyncConfig();
    bool getFsyncEnabled();

    #if defined CONFIG_MPU6500 || defined CONFIG_MPU9250
    esp_err_t setFchoice(fchoice_t fchoice);
    fchoice_t getFchoice();
    #endif

    #if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
    esp_err_t setAuxVDDIOLevel(auxvddio_lvl_t level);
    auxvddio_lvl_t getAuxVDDIOLevel();
    #endif

    esp_err_t setMotionDetectConfig(mot_config_t& config);
    mot_config_t getMotionDetectConfig();
    esp_err_t setMotionFeatureEnabled(bool enable);
    bool getMotionFeatureEnabled();

    #if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    esp_err_t setZeroMotionConfig(zrmot_config_t& config);
    zrmot_config_t getZeroMotionConfig();
    esp_err_t setFreeFallConfig(ff_config_t& config);
    ff_config_t getFreeFallConfig();
    mot_stat_t getMotionDetectStatus();
    #endif

    esp_err_t registerDump(uint8_t start = 0x0, uint8_t end = 0x7F);

    #if defined CONFIG_MPU_AK89xx
    esp_err_t compassInit();
    esp_err_t compassTestConnection();
    esp_err_t compassSetMode(mag_mode_t mode);
    esp_err_t compassGetAdjustment(uint8_t* x, uint8_t* y, uint8_t* z);
    mag_mode_t compassGetMode();
    uint8_t compassWhoAmI();
    uint8_t compassGetInfo();
    esp_err_t compassReadByte(uint8_t regAddr, uint8_t *data);
    esp_err_t compassWriteByte(uint8_t regAddr, const uint8_t data);
    bool compassSelfTest(raw_axes_t *result = nullptr);
    #endif

    #if defined CONFIG_MPU_AK8963
    esp_err_t compassReset();
    esp_err_t compassSetSensitivity(mag_sensy_t sensy);
    mag_sensy_t compassGetSensitivity();
    #endif

    esp_err_t readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    esp_err_t readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
    esp_err_t readByte(uint8_t regAddr, uint8_t *data);
    esp_err_t readBytes(uint8_t regAddr, size_t length, uint8_t *data);
    esp_err_t writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
    esp_err_t writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    esp_err_t writeByte(uint8_t regAddr, uint8_t data);
    esp_err_t writeBytes(uint8_t regAddr, size_t length, const uint8_t *data);

    esp_err_t acceleration(raw_axes_t* accel);
    esp_err_t acceleration(int16_t* x, int16_t* y, int16_t* z);
    esp_err_t rotation(raw_axes_t* gyro);
    esp_err_t rotation(int16_t* x, int16_t* y, int16_t* z);
    esp_err_t temperature(int16_t* temp);
    esp_err_t motion(raw_axes_t* accel, raw_axes_t* gyro);
    #if defined CONFIG_MPU_AK89xx
    esp_err_t heading(raw_axes_t* mag);
    esp_err_t heading(int16_t* x, int16_t* y, int16_t* z);
    esp_err_t motion(raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);
    #endif
    esp_err_t sensors(raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
    esp_err_t sensors(sensors_t* sensors, size_t extsens_len = 0);

 protected:
    mpu_bus_t* bus;
    mpu_addr_handle_t addr;
    uint8_t buffer[16];
    esp_err_t err;
};

}  // namespace mpu


using MPU_t = mpu::MPU;







/****************
 * Inline methods
 ****************/
namespace mpu {

inline MPU::MPU() : MPU(MPU_DEFAULT_BUS) {
}
inline MPU::MPU(mpu_bus_t& bus) : MPU(bus, MPU_DEFAULT_ADDR_HANDLE) {
}
inline MPU::MPU(mpu_bus_t& bus, mpu_addr_handle_t addr) :
    bus{&bus},
    addr{addr},
    buffer{0},
    err{ESP_OK} {
}
inline MPU::~MPU() {
}

inline MPU& MPU::setBus(mpu_bus_t& bus) {
    this->bus = &bus;
    return *this;
}
inline mpu_bus_t& MPU::getBus() {
    return *bus;
}
inline MPU& MPU::setAddr(mpu_addr_handle_t addr) {
    this->addr = addr;
    return *this;
}
inline mpu_addr_handle_t MPU::getAddr() {
    return addr;
}
inline esp_err_t MPU::lastError() {
    return err;
}

inline esp_err_t MPU::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    return err = bus->readBit(addr, regAddr, bitNum, data);
}
inline esp_err_t MPU::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    return err = bus->readBits(addr, regAddr, bitStart, length, data);
}
inline esp_err_t MPU::readByte(uint8_t regAddr, uint8_t *data) {
    return err = bus->readByte(addr, regAddr, data);
}
inline esp_err_t MPU::readBytes(uint8_t regAddr, size_t length, uint8_t *data) {
    return err = bus->readBytes(addr, regAddr, length, data);
}
inline esp_err_t MPU::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    return err = bus->writeBit(addr, regAddr, bitNum, data);
}
inline esp_err_t MPU::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    return err = bus->writeBits(addr, regAddr, bitStart, length, data);
}
inline esp_err_t MPU::writeByte(uint8_t regAddr, uint8_t data) {
    return err = bus->writeByte(addr, regAddr, data);
}
inline esp_err_t MPU::writeBytes(uint8_t regAddr, size_t length, const uint8_t *data) {
    return err = bus->writeBytes(addr, regAddr, length, data);
}

}  // namespace mpu

}  // namespace emd



#endif /* end of include guard: _MPU_HPP_ */
