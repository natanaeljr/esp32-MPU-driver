// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file MPUdmp.hpp
 * Declare MPU class with DMP interface.
 */

#ifndef _MPU_DMP_HPP_
#define _MPU_DMP_HPP_

#include "MPU.hpp"
#include "dmp/types.hpp"
#include "mpu/types.hpp"

#if !defined CONFIG_MPU_ENABLE_DMP
#warning ''You must enable the option DMP in \
menuconfig -> components -> MPU driver, to compile the DMP source code''
#endif

/*! MPU Driver namespace */
namespace mpud
{
class MPUdmp;
}

/*! Easy alias for MPU class with DMP interface */
typedef mpud::MPUdmp MPUdmp_t;

namespace mpud
{
/*! MPU with Digital Motion Processing interface */
class MPUdmp : public MPU
{
 public:
    //! \name Constructors / Destructor
    //! \{
    MPUdmp();
    explicit MPUdmp(mpu_bus_t& bus);
    MPUdmp(mpu_bus_t& bus, mpu_addr_handle_t addr);
    ~MPUdmp();
    //! \}
    //! \name Setup
    //! \{
    esp_err_t loadDMPFirmware();
    esp_err_t enableDMP();
    esp_err_t disableDMP();
    esp_err_t resetDMP();
    bool getDMPEnabled();
    //! \}
    //! \name Configuration
    //! \{
    esp_err_t setDMPFeatures(dmp_feature_t features);
    esp_err_t setDMPOutputRate(uint16_t rate);
    esp_err_t setDMPInterruptMode(dmp_int_mode_t mode);
    esp_err_t setOrientation(uint16_t orient);
    esp_err_t setTapConfig(const dmp_tap_config_t& config);
    esp_err_t setTapAxisEnabled(dmp_tap_axis_t axes);
    dmp_feature_t getDMPFeatures();
    //! \}
    //! \name Data
    //! \{
    uint8_t getDMPPacketLength(dmp_feature_t otherFeatures = 0);
    esp_err_t getDMPQuaternion(const uint8_t* fifoPacket, quat_q30_t* quat);
    esp_err_t getDMPAccel(const uint8_t* fifoPacket, raw_axes_t* accel);
    esp_err_t getDMPGyro(const uint8_t* fifoPacket, raw_axes_t* gyro);
    esp_err_t readDMPPacket(quat_q30_t* quat, raw_axes_t* gyro, raw_axes_t* accel);
    //! \}

 protected:
    esp_err_t setGyroCalFeature(bool enable);
    esp_err_t setLP3xQuatFeature(bool enable);
    esp_err_t setLP6xQuatFeature(bool enable);

    esp_err_t writeMemory(uint16_t memAddr, uint8_t length, const uint8_t* data);
    esp_err_t readMemory(uint16_t memAddr, uint8_t length, uint8_t* data);
    int8_t getPacketIndex(dmp_feature_t feature);

    uint8_t packetLength;          /*!< DMP Packet length, depends on enabled features */
    dmp_feature_t enabledFeatures; /*!< Current enabled Features */
};

}  // namespace mpud

// ==============
// Inline methods
// ==============
namespace mpud
{
/*! Default Constructor. */
inline MPUdmp::MPUdmp() : MPUdmp(MPU_DEFAULT_BUS){};
/**
 * @brief Contruct a MPUdmp in the given communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
inline MPUdmp::MPUdmp(mpu_bus_t& bus) : MPUdmp(bus, MPU_DEFAULT_ADDR_HANDLE) {}
/**
 * @brief Construct a MPUdmp in the given communication bus and address.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
inline MPUdmp::MPUdmp(mpu_bus_t& bus, mpu_addr_handle_t addr)
  : MPU(bus, addr), packetLength{0}, enabledFeatures{DMP_FEATURE_NONE}
{
}
/*! Default Destructor, does nothing. */
inline MPUdmp::~MPUdmp() = default;

/*! Return current enabled DMP Features */
inline dmp_feature_t MPUdmp::getDMPFeatures()
{
    return this->enabledFeatures;
}

}  // namespace mpud

#endif /* end of include guard: _MPU_DMP_HPP_ */
