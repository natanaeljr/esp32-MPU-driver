// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file MPUdmp.cpp
 * Implement MPU class with DMP interface.
 */

#include "MPUdmp.hpp"
#include <stdint.h>
#include <string.h>
#include "MPU.hpp"
#include "dmp/defines.hpp"
#include "dmp/image.hpp"
#include "dmp/keys.hpp"
#include "dmp/map.hpp"
#include "dmp/types.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu/registers.hpp"
#include "mpu/types.hpp"
#include "sdkconfig.h"

static const char* TAG = CONFIG_MPU_CHIP_MODEL;

#include "mpu/log.hpp"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

/*! MPU Driver namespace */
namespace mpud
{
/* CONSTANTS */
static constexpr size_t kGyroScaleFactor                 = (46850825LL * 200 / kDMPSampleRate);
static constexpr dmp_feature_t DMP_FEATURE_SEND_ANY_GYRO = (DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_SEND_CAL_GYRO);
static constexpr dmp_feature_t DMP_FEATURE_ANY_LP_QUAT   = (DMP_FEATURE_LP_3X_QUAT | DMP_FEATURE_LP_6X_QUAT);

/**
 * @brief Activate the DMP.
 * @attention
 *   The DMP image must already have been pushed to memory with loadDMPFirware().
 * @warning
 *   This function sets the Sample Rate to 200 Hz which is the default DMP operation rate.
 *   Cannot be changed while DMP enabled. Use setDMPOutputRate() to change DMP Output Rate to FIFO.
 */
esp_err_t MPUdmp::enableDMP()
{
    if (MPU_ERR_CHECK(setSampleRate(kDMPSampleRate))) return err;
    return MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_DMP_EN_BIT, 0x1));
}

/**
 * @brief Deactivate the DMP.
 * @note
 *  DMP features are disabled after the current processing round has completed.
 */
esp_err_t MPUdmp::disableDMP()
{
    return MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_DMP_EN_BIT, 0x0));
}

/**
 * @brief Return DMP enabled/disabled.
 */
bool MPUdmp::getDMPEnabled()
{
    MPU_ERR_CHECK(readBit(regs::USER_CTRL, regs::USERCTRL_DMP_EN_BIT, buffer));
    return (bool) buffer[0];
}

/**
 * @brief Reset the DMP.
 * @warning This functions delays 50ms.
 * @note
 *  This bit resets the DMP when set to 1 while DMP_EN equals 0.
 *  This bit automatically clears to 0 after the reset has been triggered.
 */
esp_err_t MPUdmp::resetDMP()
{
    if (MPU_ERR_CHECK(readByte(regs::USER_CTRL, buffer))) return err;
    const bool prevState = buffer[0] & regs::USERCTRL_DMP_EN_BIT;
    buffer[0] &= ~(regs::USERCTRL_DMP_EN_BIT);  // zero DMP_EN_BIT
    buffer[0] |= regs::USERCTRL_DMP_RESET_BIT;  // set DMP_RESET_BIT
    if (MPU_ERR_CHECK(writeByte(regs::USER_CTRL, buffer[0]))) return err;
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if (prevState) {
        buffer[0] |= (1 << regs::USERCTRL_DMP_EN_BIT);  // set DMP_EN_BIT
        if (MPU_ERR_CHECK(writeByte(regs::USER_CTRL, buffer[0]))) return err;
    }
    return err = ESP_OK;
}

/**
 * @brief Load DMP firmware into DMP Memory.
 * @note DMP Memory is vollatile, so it has to be reload every power-up.
 */
esp_err_t MPUdmp::loadDMPFirware()
{
    uint16_t addr = 0;  // chunk start address
    while (addr < kDMPCodeSize) {
        // watch out for the last chunk, might be smaller
        const uint16_t codeLeft = kDMPCodeSize - addr;
        const uint8_t length    = min(codeLeft, kMemoryChunkSize);
        // write this chunk
        if (MPU_ERR_CHECK(writeMemory(addr, length, &kDMPMemory[addr]))) return err;
        // read chunk back to verify
        buffer[kMemoryChunkSize] = {0};
        if (MPU_ERR_CHECK(readMemory(addr, length, buffer))) return err;
        // compare read chunk against this DMP code chunk
        if (memcmp(&kDMPMemory[addr], buffer, length) != 0) {
            MPU_LOGEMSG(msgs::DMP_LOAD_FAIL, " @ bank: %d, address: 0x%X, length: %d",  //
                        (addr >> 8), (addr & 0xFF), length);
            return err = ESP_FAIL;
        }
        MPU_LOGVMSG("done", " chunk: %d, length: %d, left: %d", addr, length, codeLeft - length);
        addr += length;
    }
    buffer[0] = kProgramStartAddress >> 8;
    buffer[1] = kProgramStartAddress & 0xFF;
    if (MPU_ERR_CHECK(writeBytes(regs::PRGM_START_H, 2, buffer))) return err;
    MPU_LOGI("DMP firmware loaded");
    return err = ESP_OK;
}

/**
 * @brief Write to the DMP memory.
 * This function prevents I2C writes past the bank boundaries. \n
 * The DMP memory is only accessible when the chip is awake.
 *
 * @param memAddr Memory location (bank << 8 | start_address)
 * @param length  Number of bytes to write.
 * @param data    Bytes to write to memory.
 */
esp_err_t MPUdmp::writeMemory(uint16_t memAddr, uint8_t length, const uint8_t* data)
{
    buffer[0] = memAddr >> 8;
    buffer[1] = memAddr & 0xFF;
    // check bank boundaries
    if (int endAddr = (buffer[1] + length) > kMemoryBankSize) {
        MPU_LOGEMSG(msgs::BANK_BOUNDARIES, "mem_addr: 0x%X, length: %d, excess: %d",  //
                    memAddr, length, endAddr - kMemoryBankSize);
        return err = ESP_ERR_INVALID_SIZE;
    }
    // set memory bank & start address
    if (MPU_ERR_CHECK(writeBytes(regs::BANK_SEL, 2, buffer))) return err;
    // write data to memory
    return MPU_ERR_CHECK(writeBytes(regs::MEM_R_W, length, data));
}

/**
 * @brief Read from the DMP memory.
 * This function prevents I2C writes past the bank boundaries. \n
 * The DMP memory is only accessible when the chip is awake.
 *
 * @param memAddr Memory location (bank << 8 | start_address)
 * @param length  Number of bytes to read.
 * @param data    Bytes to read from memory.
 */
esp_err_t MPUdmp::readMemory(uint16_t memAddr, uint8_t length, uint8_t* data)
{
    buffer[0] = memAddr >> 8;
    buffer[1] = memAddr & 0xFF;
    // check bank boundaries
    if (int endAddr = (buffer[1] + length) > kMemoryBankSize) {
        MPU_LOGEMSG(msgs::BANK_BOUNDARIES, "mem_addr: 0x%X, length: %d, excess: %d",  //
                    memAddr, length, endAddr - kMemoryBankSize);
        return err = ESP_ERR_INVALID_SIZE;
    }
    // set memory bank & start address
    if (MPU_ERR_CHECK(writeBytes(regs::BANK_SEL, 2, buffer))) return err;
    // read data from memory
    return MPU_ERR_CHECK(readBytes(regs::MEM_R_W, length, data));
}

/**
 * @brief Enable DMP Features
 * @note
 *  - DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually exclusive.
 *  - DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also mutually exclusive.
 *  - DMP_FEATURE_PEDOMETER is always enabled.
 * @param features Combined (`ORed`) features to enable.
 * @todo Implement rest of the features
 * @return
 *  - `ESP_ERR_INVALID_ARG`: Invalid set of features, see note.
 *  - `I2C/SPI` default read/write errors.
 */
esp_err_t MPUdmp::setDMPFeatures(dmp_feature_t features)
{
    /* Check for invalid options */
    if ((features & DMP_FEATURE_LP_3X_QUAT) && (features & DMP_FEATURE_LP_6X_QUAT)) {
        MPU_LOGEMSG("LP_3X_QUAT and LP_6X_QUAT are mutually exclusive", "");
        return err = ESP_ERR_INVALID_ARG;
    }
    if ((features & DMP_FEATURE_SEND_RAW_GYRO) && (features & DMP_FEATURE_SEND_CAL_GYRO)) {
        MPU_LOGEMSG("SEND_RAW_GYRO and SEND_CAL_GYRO are mutually exclusive", "");
        return err = ESP_ERR_INVALID_ARG;
    }

    /* Temporary */
    // TODO: Implement and remove
    if (features & DMP_FEATURE_TAP) {
        MPU_LOGWMSG("TAP not yet supported", "");
        features &= ~(dmp_feature_t)(DMP_FEATURE_TAP);
    }
    if (features & DMP_FEATURE_ANDROID_ORIENT) {
        MPU_LOGWMSG("ANDROID_ORIENT not yet supported", "");
        features &= ~(dmp_feature_t)(DMP_FEATURE_ANDROID_ORIENT);
    }

    /* Set integration scale factor */
    buffer[0] = (kGyroScaleFactor >> 24) & 0xFF;
    buffer[1] = (kGyroScaleFactor >> 16) & 0xFF;
    buffer[2] = (kGyroScaleFactor >> 8) & 0xFF;
    buffer[3] = (kGyroScaleFactor) &0xFF;
    if (MPU_ERR_CHECK(writeMemory(D_0_104, 4, buffer))) return err;

    /* Send sensor data to the FIFO. */
    memset(buffer, 0xA3, 10);
    if (features & DMP_FEATURE_SEND_RAW_ACCEL) {
        buffer[1] = 0xC0;
        buffer[2] = 0xC8;
        buffer[3] = 0xC2;
    }
    if (features & DMP_FEATURE_SEND_ANY_GYRO) {
        buffer[4] = 0xC4;
        buffer[5] = 0xCC;
        buffer[6] = 0xC6;
    }
    if (MPU_ERR_CHECK(writeMemory(CFG_15, 10, buffer))) return err;

    /* Send gesture data to the FIFO. */
    const int enableGesture = features & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT);
    buffer[0]               = enableGesture ? DINA20 : 0xD8;
    if (MPU_ERR_CHECK(writeMemory(CFG_27, 1, buffer))) return err;

    /* Enable Gyro calibration feature */
    if (MPU_ERR_CHECK(setGyroCalFeature(features & DMP_FEATURE_GYRO_CAL))) return err;

    /* Enable Gyro data features */
    if (features & DMP_FEATURE_SEND_ANY_GYRO) {
        if (features & DMP_FEATURE_SEND_CAL_GYRO) {
            buffer[0] = 0xB2;
            buffer[1] = 0x8B;
            buffer[2] = 0xB6;
            buffer[3] = 0x9B;
        }
        else {
            buffer[0] = DINAC0;
            buffer[1] = DINA80;
            buffer[2] = DINAC2;
            buffer[3] = DINA90;
        }
        if (MPU_ERR_CHECK(writeMemory(CFG_GYRO_RAW_DATA, 4, buffer))) return err;
    }

    /* Enable Tap feature */
    if (features & DMP_FEATURE_TAP) {
        (void) 0;  // TODO: implement
    }
    else {
        buffer[0] = 0xD8;
        if (MPU_ERR_CHECK(writeMemory(CFG_20, 1, buffer))) return err;
    }

    /* Enable Android Orientation feature */
    const int enableOrient = (features & DMP_FEATURE_ANDROID_ORIENT);
    buffer[0]              = enableOrient ? 0xD9 : 0xD8;
    if (MPU_ERR_CHECK(writeMemory(CFG_ANDROID_ORIENT_INT, 1, buffer))) return err;

    /* Enable LP 3-axis Quaternions */
    const int enableLP3XQuat = (features & DMP_FEATURE_LP_3X_QUAT);
    if (MPU_ERR_CHECK(setLP3xQuatFeature(enableLP3XQuat))) return err;

    /* Enable LP 6-axis Quaternions */
    const int enableLP6XQuat = (features & DMP_FEATURE_LP_6X_QUAT);
    if (MPU_ERR_CHECK(setLP6xQuatFeature(enableLP6XQuat))) return err;

    /* Pedometer is always enabled. */
    this->enabledFeatures = features | DMP_FEATURE_PEDOMETER;

    // Clean FIFO buffer
    if (MPU_ERR_CHECK(resetFIFO())) return err;

    // Update FIFO packet length
    updatePacketLength();

    return err = ESP_OK;
}

/**
 * @brief Enable Calibrate the gyro data in the DMP.
 * After eight seconds of no motion, the DMP will compute gyro biases and
 * subtract them from the quaternion output. If DMP_FEATURE_SEND_CAL_GYRO is enabled,
 * the biases will also be subtracted from the gyro output.
 */
esp_err_t MPUdmp::setGyroCalFeature(bool enable)
{
    if (enable) {
        uint8_t buffer[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
        return MPU_ERR_CHECK(writeMemory(CFG_MOTION_BIAS, 9, buffer));
    }
    else {
        uint8_t buffer[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
        return MPU_ERR_CHECK(writeMemory(CFG_MOTION_BIAS, 9, buffer));
    }
}

/**
 * @brief Enable get 3-axis quaternions from the DMP.
 * In this driver, the 3-axis and 6-axis DMP quaternion features are mutually exclusive.
 */
esp_err_t MPUdmp::setLP3xQuatFeature(bool enable)
{
    uint8_t buffer[4] = {0x8B};  // 0x8B -> disable
    if (enable) {
        buffer[0] = DINBC0;
        buffer[1] = DINBC2;
        buffer[2] = DINBC4;
        buffer[3] = DINBC6;
    }
    return MPU_ERR_CHECK(writeMemory(CFG_LP_QUAT, 4, buffer));
}

/**
 * @brief Enable get 6-axis quaternions from the DMP.
 * In this driver, the 3-axis and 6-axis DMP quaternion features are mutually exclusive.
 */
esp_err_t MPUdmp::setLP6xQuatFeature(bool enable)
{
    uint8_t buffer[4] = {0xA3};  // 0xA3 -> disable
    if (enable) {
        buffer[0] = DINA20;
        buffer[1] = DINA28;
        buffer[2] = DINA30;
        buffer[3] = DINA38;
    }
    return MPU_ERR_CHECK(writeMemory(CFG_8, 4, buffer));
}

/**
 * @brief Set DMP Output Data Rate.
 * DMP Output rate: 1 ~ 200 Hz. Should be a perfect divider of 200. \n
 * Only used when DMP is on.
 * @param rate Desired FIFO rate in Hz.
 */
esp_err_t MPUdmp::setDMPOutputRate(uint16_t rate)
{
    const uint8_t kRegsEnd[12] = {DINAFE, DINAF2, DINAAB, 0xc4, DINAAA, DINAF1,
                                  DINADF, DINADF, 0xBB,   0xAF, DINADF, DINADF};

    if (rate > kDMPSampleRate) {
        MPU_LOGWMSG(msgs::INVALID_DMP_RATE, " %d, maximum rate is %d", rate, kDMPSampleRate);
        rate = kDMPSampleRate;
    }
    else if (rate < 1) {
        MPU_LOGWMSG(msgs::INVALID_DMP_RATE, " %d, minimum rate is %d", rate, 1);
        rate = 1;
    }
    // calculate and write rate divider to DMP
    uint16_t divider = kDMPSampleRate / rate - 1;
    buffer[0]        = divider >> 8;
    buffer[1]        = divider & 0xFF;
    if (MPU_ERR_CHECK(writeMemory(D_0_22, 2, buffer))) return err;
    if (MPU_ERR_CHECK(writeMemory(CFG_6, 12, kRegsEnd))) return err;

    // calculate true rate
    uint16_t finalRate = kDMPSampleRate / (1 + divider);
    if (finalRate != rate) {
        MPU_LOGW("DMP Output rate constrained to %d Hz", finalRate);
    }
    else {
        MPU_LOGI("DMP Output rate set to %d Hz", finalRate);
    }
    return err = ESP_OK;
}

/**
 * @brief Specify when a DMP interrupt should occur.
 * A DMP interrupt can be configured to trigger on either of the two conditions below:
 *  \n a. One FIFO period has elapsed (set by setDMPOutputRate()).
 *  \n b. A tap event has been detected.
 * @note Call setInterruptEnabled() make the DMP interrupt propagate to INT pin.
 * @param mode `DMP_INT_GESTURE` or `DMP_INT_CONTINUOUS`.
 */
esp_err_t MPUdmp::setDMPInterruptMode(dmp_int_mode_t mode)
{
    const uint8_t kRegsContinuous[11] = {0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9};
    const uint8_t kRegsGesture[11]    = {0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda};
    switch (mode) {
        case DMP_INT_MODE_CONTINUOUS:
            return MPU_ERR_CHECK(writeMemory(CFG_FIFO_ON_EVENT, 11, kRegsContinuous));
        case DMP_INT_MODE_GESTURE:
            return MPU_ERR_CHECK(writeMemory(CFG_FIFO_ON_EVENT, 11, kRegsGesture));
        default:
            MPU_LOGEMSG(msgs::INVALID_ARG, "");
            return err = ESP_ERR_INVALID_ARG;
    }
}

/**
 * @brief Calculate and Set the Packet Length field
 */
void MPUdmp::updatePacketLength()
{
    this->packetLength = 0;
    if (enabledFeatures & DMP_FEATURE_ANY_LP_QUAT) packetLength += 16;
    if (enabledFeatures & DMP_FEATURE_SEND_RAW_ACCEL) packetLength += 6;
    if (enabledFeatures & DMP_FEATURE_SEND_ANY_GYRO) packetLength += 6;
    if (enabledFeatures & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) packetLength += 4;
}

}  // namespace mpud
