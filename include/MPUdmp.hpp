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
class MPUdmp : public mpud::MPU
{
 public:
    //! \name Setup
    //! \{
    esp_err_t loadDMPFirware();
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
    dmp_feature_t getDMPFeatures();
    //! \}
    //! \name Packet
    //! \{
    uint8_t getDMPPacketLength();
    //! \}

 protected:
    esp_err_t setGyroCalFeature(bool enable);
    esp_err_t setLP3xQuatFeature(bool enable);
    esp_err_t setLP6xQuatFeature(bool enable);

    esp_err_t writeMemory(uint16_t memAddr, uint8_t length, const uint8_t* data);
    esp_err_t readMemory(uint16_t memAddr, uint8_t length, uint8_t* data);

    void updatePacketLength();

    uint8_t packetLength;          /*! DMP Packet length, depends on enabled features */
    dmp_feature_t enabledFeatures; /*! Current enabled Features */
};

}  // namespace mpud

// ==============
// Inline methods
// ==============
namespace mpud
{
/*! Return current enabled DMP Features */
inline dmp_feature_t MPUdmp::getDMPFeatures()
{
    return this->enabledFeatures;
}
/*! Return DMP Packet Length */
inline uint8_t MPUdmp::getDMPPacketLength()
{
    return this->packetLength;
}

}  // namespace mpud

#endif /* end of include guard: _MPU_DMP_HPP_ */
