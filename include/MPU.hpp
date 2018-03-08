// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file MPU.hpp
 * MPU library main file. Declare MPU class.
 *
 * @attention
 *  MPU library requires I2Cbus or SPIbus library.
 *  Select the communication protocol in `menuconfig`
 *  and include the corresponding library to project components.
 *
 * @note
 *  The following is taken in the code:
 *  - MPU9250 is the same as MPU6500 + AK8963
 *  - MPU9150 is the same as MPU6050 + AK8975
 *  - MPU6000 code equals MPU6050
 *  - MPU6555 code equals MPU6500
 *  - MPU9255 code equals MPU9250
 * */

#ifndef _MPU_HPP_
#define _MPU_HPP_

#include <stdint.h>
#include "esp_err.h"
#include "sdkconfig.h"

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

/*! MPU Driver namespace */
namespace mpud
{
class MPU;
}

/*! Easy alias for MPU class */
typedef mpud::MPU MPU_t;

namespace mpud
{
/*! Motion Processing Unit */
class MPU
{
 public:
    //! \name Constructors / Destructor
    //! \{
    MPU();
    explicit MPU(mpu_bus_t& bus);
    MPU(mpu_bus_t& bus, mpu_addr_handle_t addr);
    ~MPU();
    //! \}
    //! \name Basic
    //! \{
    MPU& setBus(mpu_bus_t& bus);
    MPU& setAddr(mpu_addr_handle_t addr);
    mpu_bus_t& getBus();
    mpu_addr_handle_t getAddr();
    esp_err_t lastError();
    //! \}
    //! \name Setup
    //! \{
    esp_err_t initialize();
    esp_err_t reset();
    esp_err_t setSleep(bool enable);
    esp_err_t testConnection();
    esp_err_t selfTest(selftest_t* result);
    esp_err_t resetSignalPath();
    uint8_t whoAmI();
    bool getSleep();
    //! \}
    //! \name Main configurations
    //! \{
    esp_err_t setSampleRate(uint16_t rate);
    esp_err_t setClockSource(clock_src_t clockSrc);
    esp_err_t setDigitalLowPassFilter(dlpf_t dlpf);
    uint16_t getSampleRate();
    clock_src_t getClockSource();
    dlpf_t getDigitalLowPassFilter();
    //! \}
    //! \name Power management
    //! \{
    esp_err_t setLowPowerAccelMode(bool enable);
    esp_err_t setLowPowerAccelRate(lp_accel_rate_t rate);
    lp_accel_rate_t getLowPowerAccelRate();
    bool getLowPowerAccelMode();
    esp_err_t setStandbyMode(stby_en_t mask);
    stby_en_t getStandbyMode();
    //! \}
    //! \name Full-Scale Range
    //! \{
    esp_err_t setGyroFullScale(gyro_fs_t fsr);
    esp_err_t setAccelFullScale(accel_fs_t fsr);
    gyro_fs_t getGyroFullScale();
    accel_fs_t getAccelFullScale();
    //! \}
    //! \name Offset / Bias
    //! \{
    esp_err_t setGyroOffset(raw_axes_t bias);
    esp_err_t setAccelOffset(raw_axes_t bias);
    raw_axes_t getGyroOffset();
    raw_axes_t getAccelOffset();
    esp_err_t computeOffsets(raw_axes_t* accel, raw_axes_t* gyro);
    //! \}
    //! \name Interrupt
    //! \{
    esp_err_t setInterruptConfig(int_config_t config);
    esp_err_t setInterruptEnabled(int_en_t mask);
    int_stat_t getInterruptStatus();
    int_config_t getInterruptConfig();
    int_en_t getInterruptEnabled();
    //! \}
    //! \name FIFO
    //! \{
    esp_err_t setFIFOMode(fifo_mode_t mode);
    esp_err_t setFIFOConfig(fifo_config_t config);
    esp_err_t setFIFOEnabled(bool enable);
    esp_err_t resetFIFO();
    uint16_t getFIFOCount();
    esp_err_t readFIFO(size_t length, uint8_t* data);
    esp_err_t writeFIFO(size_t length, const uint8_t* data);
    fifo_mode_t getFIFOMode();
    fifo_config_t getFIFOConfig();
    bool getFIFOEnabled();
    //! \}
    //! \name Auxiliary I2C Master
    //! \{
    esp_err_t setAuxI2CConfig(const auxi2c_config_t& config);
    esp_err_t setAuxI2CEnabled(bool enable);
    esp_err_t setAuxI2CSlaveConfig(const auxi2c_slv_config_t& config);
    esp_err_t setAuxI2CSlaveEnabled(auxi2c_slv_t slave, bool enable);
    esp_err_t setAuxI2CBypass(bool enable);
    esp_err_t readAuxI2CRxData(size_t length, uint8_t* data, size_t skip = 0);
    esp_err_t restartAuxI2C();
    auxi2c_stat_t getAuxI2CStatus();
    auxi2c_config_t getAuxI2CConfig();
    auxi2c_slv_config_t getAuxI2CSlaveConfig(auxi2c_slv_t slave);
    bool getAuxI2CEnabled();
    bool getAuxI2CSlaveEnabled(auxi2c_slv_t slave);
    bool getAuxI2CBypass();
    esp_err_t auxI2CWriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    esp_err_t auxI2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data);
    //! \}
    //! \name Motion Detection Interrupt
    //! \{
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
    //! \}
    //! \name Compass | Magnetometer
    //! \{
#if defined CONFIG_MPU_AK89xx
    esp_err_t compassInit();
    esp_err_t compassTestConnection();
    esp_err_t compassSetMode(mag_mode_t mode);
    esp_err_t compassGetAdjustment(uint8_t* x, uint8_t* y, uint8_t* z);
    mag_mode_t compassGetMode();
    uint8_t compassWhoAmI();
    uint8_t compassGetInfo();
    esp_err_t compassReadByte(uint8_t regAddr, uint8_t* data);
    esp_err_t compassWriteByte(uint8_t regAddr, uint8_t data);
    bool compassSelfTest(raw_axes_t* result = nullptr);
#endif
#if defined CONFIG_MPU_AK8963
    esp_err_t compassReset();
    esp_err_t compassSetSensitivity(mag_sensy_t sensy);
    mag_sensy_t compassGetSensitivity();
#endif
    //! \}
    //! \name Miscellaneous
    //! \{
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
    //! \}
    //! \name Read / Write
    //! Functions to perform direct read or write operation(s) to registers.
    //! \{
    esp_err_t readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data);
    esp_err_t readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
    esp_err_t readByte(uint8_t regAddr, uint8_t* data);
    esp_err_t readBytes(uint8_t regAddr, size_t length, uint8_t* data);
    esp_err_t writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
    esp_err_t writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    esp_err_t writeByte(uint8_t regAddr, uint8_t data);
    esp_err_t writeBytes(uint8_t regAddr, size_t length, const uint8_t* data);
    esp_err_t registerDump(uint8_t start = 0x0, uint8_t end = 0x7F);
    //! \}
    //! \name Sensor readings
    //! \{
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
    //! \}

 protected:
    esp_err_t accelSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result);
    esp_err_t gyroSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result);
    esp_err_t getBiases(accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                        bool selftest);

    mpu_bus_t* bus;         /*!< Communication bus pointer, I2C / SPI */
    mpu_addr_handle_t addr; /*!< I2C address / SPI device handle */
    uint8_t buffer[16];     /*!< Commom buffer for temporary data */
    esp_err_t err;          /*!< Holds last error code */
};

}  // namespace mpud

// ==============
// Inline methods
// ==============
namespace mpud
{
/*! Default Constructor. */
inline MPU::MPU() : MPU(MPU_DEFAULT_BUS){};
/**
 * @brief Contruct a MPU in the given communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
inline MPU::MPU(mpu_bus_t& bus) : MPU(bus, MPU_DEFAULT_ADDR_HANDLE) {}
/**
 * @brief Construct a MPU in the given communication bus and address.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
inline MPU::MPU(mpu_bus_t& bus, mpu_addr_handle_t addr) : bus{&bus}, addr{addr}, buffer{0}, err{ESP_OK} {}
/** Default Destructor, does nothing. */
inline MPU::~MPU() = default;
/**
 * @brief Set communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
inline MPU& MPU::setBus(mpu_bus_t& bus)
{
    this->bus = &bus;
    return *this;
}
/**
 * @brief Return communication bus object.
 */
inline mpu_bus_t& MPU::getBus()
{
    return *bus;
}
/**
 * @brief Set I2C address or SPI device handle.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
inline MPU& MPU::setAddr(mpu_addr_handle_t addr)
{
    this->addr = addr;
    return *this;
}
/**
 * @brief Return I2C address or SPI device handle.
 */
inline mpu_addr_handle_t MPU::getAddr()
{
    return addr;
}
/*! Return last error code. */
inline esp_err_t MPU::lastError()
{
    return err;
}
/*! Read a single bit from a register*/
inline esp_err_t MPU::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data)
{
    return err = bus->readBit(addr, regAddr, bitNum, data);
}
/*! Read a range of bits from a register */
inline esp_err_t MPU::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data)
{
    return err = bus->readBits(addr, regAddr, bitStart, length, data);
}
/*! Read a single register */
inline esp_err_t MPU::readByte(uint8_t regAddr, uint8_t* data)
{
    return err = bus->readByte(addr, regAddr, data);
}
/*! Read data from sequence of registers */
inline esp_err_t MPU::readBytes(uint8_t regAddr, size_t length, uint8_t* data)
{
    return err = bus->readBytes(addr, regAddr, length, data);
}
/*! Write a single bit to a register */
inline esp_err_t MPU::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    return err = bus->writeBit(addr, regAddr, bitNum, data);
}
/*! Write a range of bits to a register */
inline esp_err_t MPU::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    return err = bus->writeBits(addr, regAddr, bitStart, length, data);
}
/*! Write a value to a register */
inline esp_err_t MPU::writeByte(uint8_t regAddr, uint8_t data)
{
    return err = bus->writeByte(addr, regAddr, data);
}
/*! Write a sequence to data to a sequence of registers */
inline esp_err_t MPU::writeBytes(uint8_t regAddr, size_t length, const uint8_t* data)
{
    return err = bus->writeBytes(addr, regAddr, length, data);
}

}  // namespace mpud

#endif /* end of include guard: _MPU_HPP_ */
