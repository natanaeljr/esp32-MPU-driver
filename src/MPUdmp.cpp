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
#include "mpu/math.hpp"
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
static constexpr size_t kGyroScaleFactor                 = (46850825 * (200 / kDMPSampleRate));
static constexpr dmp_feature_t DMP_FEATURE_SEND_ANY_GYRO = (DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_SEND_CAL_GYRO);
static constexpr dmp_feature_t DMP_FEATURE_ANY_LP_QUAT   = (DMP_FEATURE_LP_3X_QUAT | DMP_FEATURE_LP_6X_QUAT);

/**
 * @brief Activate the DMP.
 * The DMP image must already have been pushed to memory with loadDMPFirmware().
 * @note
 *  - This function sets the Sample Rate to 200 Hz which is the default DMP operation rate.
 *    Cannot be changed while DMP enabled. Use setDMPOutputRate() to change DMP Output Rate to FIFO.
 *  - This function enables the FIFO, and sets Accel FSR to 2G, Gyro FSR to 2000 DPS.
 *    These configs cannot be changed while DMP enabled.
 */
esp_err_t MPUdmp::enableDMP()
{
    if (MPU_ERR_CHECK(setSampleRate(kDMPSampleRate))) return err;
    if (MPU_ERR_CHECK(setAccelFullScale(ACCEL_FS_2G))) return err;
    if (MPU_ERR_CHECK(setGyroFullScale(GYRO_FS_2000DPS))) return err;
    if (MPU_ERR_CHECK(setFIFOEnabled(true))) return err;
    if (MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_DMP_EN_BIT, 0x1))) return err;
    MPU_LOGI("DMP activated");
    return err;
}

/**
 * @brief Deactivate the DMP.
 * @note
 *  DMP features are disabled after the current processing round has completed.
 */
esp_err_t MPUdmp::disableDMP()
{
    if (MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_DMP_EN_BIT, 0x0))) return err;
    MPU_LOGI("DMP deactivated");
    return err;
}

/**
 * @brief Return DMP enabled/disabled.
 */
bool MPUdmp::getDMPEnabled()
{
    uint8_t data;
    MPU_ERR_CHECK(readBit(regs::USER_CTRL, regs::USERCTRL_DMP_EN_BIT, &data));
    return (bool) data;
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
    uint8_t data;
    if (MPU_ERR_CHECK(readByte(regs::USER_CTRL, &data))) return err;
    const bool prevState = data & regs::USERCTRL_DMP_EN_BIT;
    data &= ~(regs::USERCTRL_DMP_EN_BIT);  // zero DMP_EN_BIT
    data |= regs::USERCTRL_DMP_RESET_BIT;  // set DMP_RESET_BIT
    if (MPU_ERR_CHECK(writeByte(regs::USER_CTRL, data))) return err;
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if (prevState) {
        data |= (1 << regs::USERCTRL_DMP_EN_BIT);  // set DMP_EN_BIT
        if (MPU_ERR_CHECK(writeByte(regs::USER_CTRL, data))) return err;
    }
    return err = ESP_OK;
}

/**
 * @brief Load DMP firmware into DMP Memory.
 * @note DMP Memory is vollatile, so the firmware has to be reloaded every power-up.
 */
esp_err_t MPUdmp::loadDMPFirmware()
{
    uint16_t addr = 0;  // chunk start address
    while (addr < kDMPCodeSize) {
        // watch out for the last chunk, might be smaller
        const uint16_t codeLeft = kDMPCodeSize - addr;
        const uint8_t length    = min(codeLeft, kMemoryChunkSize);
        // write this chunk
        if (MPU_ERR_CHECK(writeMemory(addr, length, &kDMPMemory[addr]))) return err;
        // read chunk back to verify
        uint8_t buffer[kMemoryChunkSize] = {0};
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
    uint8_t buffer[2];
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
    uint8_t buffer[2];
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
    uint8_t buffer[2];
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
 *  - `DMP_FEATURE_LP_QUAT` and `DMP_FEATURE_6X_LP_QUAT` are mutually exclusive.
 *  - `DMP_FEATURE_SEND_RAW_GYRO` and `DMP_FEATURE_SEND_CAL_GYRO` are also mutually exclusive.
 *  - `DMP_FEATURE_PEDOMETER` is always enabled.
 * @param features Combined _(ORed)_ features to enable.
 * @todo Implement rest of the features
 * @return
 *  - `ESP_ERR_INVALID_ARG`: Invalid set of features, see note.
 *  - I2C/SPI default read/write errors.
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
        MPU_LOGWMSG("TAP not yet fully supported", "");
    }
    if (features & DMP_FEATURE_ANDROID_ORIENT) {
        MPU_LOGWMSG("ANDROID_ORIENT not yet fully supported", "");
    }

    uint8_t buffer[10];

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
    const int enableGyroCal = (features & DMP_FEATURE_GYRO_CAL);
    if (MPU_ERR_CHECK(setGyroCalFeature(enableGyroCal))) return err;

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
        buffer[0] = 0xF8;  // enable
        if (MPU_ERR_CHECK(writeMemory(CFG_20, 1, buffer))) return err;
        const dmp_tap_config_t tapConf{
            .threshold_X    = 250,
            .threshold_Y    = 250,
            .threshold_Z    = 250,
            .count          = 1,
            .time           = 100,
            .time_multi_tap = 500,
            {
                // shake reject:
                .threshold = 200,
                .time      = 40,
                .timeout   = 10  //
            }                    //
        };
        if (MPU_ERR_CHECK(setTapConfig(tapConf))) return err;
        if (MPU_ERR_CHECK(setTapAxisEnabled(DMP_TAP_XYZ))) return err;
    }
    else {
        buffer[0] = 0xD8;  // disable
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

    // Clean FIFO an INT_STATUS
    if (MPU_ERR_CHECK(resetFIFO())) return err;
    this->getInterruptStatus();
    if (MPU_ERR_CHECK(lastError())) return err;

    // Update FIFO packet length
    this->packetLength = getDMPPacketLength(enabledFeatures);

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
    uint8_t buffer[4];
    if (enable) {
        buffer[0] = DINBC0;
        buffer[1] = DINBC2;
        buffer[2] = DINBC4;
        buffer[3] = DINBC6;
    }
    else {
        memset(buffer, 0x8B, 4);  // 0x8B -> disable
    }
    return MPU_ERR_CHECK(writeMemory(CFG_LP_QUAT, 4, buffer));
}

/**
 * @brief Enable get 6-axis quaternions from the DMP.
 * In this driver, the 3-axis and 6-axis DMP quaternion features are mutually exclusive.
 */
esp_err_t MPUdmp::setLP6xQuatFeature(bool enable)
{
    uint8_t buffer[4];
    if (enable) {
        buffer[0] = DINA20;
        buffer[1] = DINA28;
        buffer[2] = DINA30;
        buffer[3] = DINA38;
    }
    else {
        memset(buffer, 0xA3, 4);  // 0xA3 -> disable
    }
    return MPU_ERR_CHECK(writeMemory(CFG_8, 4, buffer));
}

/**
 * @brief Configure Tap Gesture Recognition feature.
 * @todo Document it.
 */
esp_err_t MPUdmp::setTapConfig(const dmp_tap_config_t& config)
{
    /* Check for invalid input */
    if (config.threshold_X > 1600 || config.threshold_Y > 1600 || config.threshold_Z > 1600) {
        MPU_LOGEMSG("Invalid Tap Threshold", ", maximum is 1600 mg/ms");
        return err = ESP_ERR_INVALID_ARG;
    }
    if (config.count < 1 || config.count > 4) {
        MPU_LOGEMSG("Invalid Tap count", ", min 1, max 4");
        return err = ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[4];
    const accel_fs_t kAccelFS = getAccelFullScale();
    if (MPU_ERR_CHECK(lastError())) return err;
    const uint16_t kAccelSensitivity = math::accelSensitivity(kAccelFS);

    /* Set Tap threshold */
    const uint16_t threshold[3] = {
        config.threshold_X,  //
        config.threshold_Y,  //
        config.threshold_Z   //
    };
    for (int i = 0; i < 3; i++) {
        const float scaledThresh = threshold[i] / kDMPSampleRate;
        uint16_t dmpThresh_1     = scaledThresh * kAccelSensitivity;
        uint16_t dmpThresh_2     = scaledThresh * kAccelSensitivity * 0.75f;
        buffer[0]                = dmpThresh_1 >> 8;
        buffer[1]                = dmpThresh_1 & 0xFF;
        buffer[2]                = dmpThresh_2 >> 8;
        buffer[3]                = dmpThresh_2 & 0xFF;
        if (MPU_ERR_CHECK(writeMemory((DMP_TAP_THX + (i * 4)), 2, &buffer[0]))) return err;
        if (MPU_ERR_CHECK(writeMemory((D_1_36 + (i * 4)), 2, &buffer[2]))) return err;
    }

    /* Set Tap count */
    buffer[0] = config.count - 1;
    if (MPU_ERR_CHECK(writeMemory(D_1_79, 1, buffer))) return err;

    /* Set Tap time */
    const uint16_t dmpTime = config.time / (1000 / kDMPSampleRate);
    buffer[0]              = (dmpTime >> 8);
    buffer[1]              = (dmpTime & 0xFF);
    if (MPU_ERR_CHECK(writeMemory(DMP_TAPW_MIN, 2, buffer))) return err;

    /* Set Multi-tap time */
    const uint16_t dmpTimeMultiTap = config.time / (1000 / kDMPSampleRate);
    buffer[0]                      = (dmpTimeMultiTap >> 8);
    buffer[1]                      = (dmpTimeMultiTap & 0xFF);
    if (MPU_ERR_CHECK(writeMemory(D_1_218, 2, buffer))) return err;

    /* Set Shake reject threshold */
    int32_t scaledThresh = kGyroScaleFactor / 1000 * config.shake_reject.threshold;
    buffer[0]            = (scaledThresh >> 24) & 0xFF;
    buffer[1]            = (scaledThresh >> 16) & 0xFF;
    buffer[2]            = (scaledThresh >> 8) & 0xFF;
    buffer[3]            = (scaledThresh & 0xFF);
    if (MPU_ERR_CHECK(writeMemory(D_1_92, 4, buffer))) return err;

    /* Set Shake reject time */
    const uint16_t dmpShakeRejectTime = config.shake_reject.time / (1000 / kDMPSampleRate);
    buffer[0]                         = (dmpShakeRejectTime >> 8);
    buffer[1]                         = (dmpShakeRejectTime & 0xFF);
    if (MPU_ERR_CHECK(writeMemory(D_1_90, 2, buffer))) return err;

    /* Set Shake reject timeout */
    const uint16_t dmpShakeRejectTimeout = config.shake_reject.timeout / (1000 / kDMPSampleRate);
    buffer[0]                            = (dmpShakeRejectTimeout >> 8);
    buffer[1]                            = (dmpShakeRejectTimeout & 0xFF);
    if (MPU_ERR_CHECK(writeMemory(D_1_88, 2, buffer))) return err;

    return err = ESP_OK;
}

/**
 * @brief Enable Tap Detection on the given axes.
 * @todo Document it.
 */
esp_err_t MPUdmp::setTapAxisEnabled(dmp_tap_axis_t axes)
{
    return MPU_ERR_CHECK(writeMemory(D_1_72, 1, &axes));
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
    const uint16_t divider = kDMPSampleRate / rate - 1;
    uint8_t buffer[2];
    buffer[0] = (divider >> 8) & 0xFF;
    buffer[1] = (divider & 0xFF);
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
 * @brief Push gyro and accel orientation to the DMP.
 * @param orient Gyro and accel orientation in body frame.
 * @todo Document it.
 */
esp_err_t MPUdmp::setOrientation(uint16_t orient)
{
    constexpr uint8_t kGyroAxes[3]  = {DINA4C, DINACD, DINA6C};
    constexpr uint8_t kAccelAxes[3] = {DINA0C, DINAC9, DINA2C};
    constexpr uint8_t kGyroSign[3]  = {DINA36, DINA56, DINA76};
    constexpr uint8_t kAccelSign[3] = {DINA26, DINA46, DINA66};
    uint8_t gyroRegs[3], accelRegs[3];

    /* Chip-to-body, axes only. */
    gyroRegs[0]  = kGyroAxes[orient & 3];
    gyroRegs[1]  = kGyroAxes[(orient >> 3) & 3];
    gyroRegs[2]  = kGyroAxes[(orient >> 6) & 3];
    accelRegs[0] = kAccelAxes[orient & 3];
    accelRegs[1] = kAccelAxes[(orient >> 3) & 3];
    accelRegs[2] = kAccelAxes[(orient >> 6) & 3];
    if (MPU_ERR_CHECK(writeMemory(FCFG_1, 3, gyroRegs))) return err;
    if (MPU_ERR_CHECK(writeMemory(FCFG_2, 3, accelRegs))) return err;

    /* Chip-to-body, sign only. */
    memcpy(gyroRegs, kGyroSign, 3);
    memcpy(accelRegs, kAccelSign, 3);
    if (orient & 4) {
        gyroRegs[0] |= 1;
        accelRegs[0] |= 1;
    }
    if (orient & 0x20) {
        gyroRegs[1] |= 1;
        accelRegs[1] |= 1;
    }
    if (orient & 0x100) {
        gyroRegs[2] |= 1;
        accelRegs[2] |= 1;
    }
    if (MPU_ERR_CHECK(writeMemory(FCFG_3, 3, gyroRegs))) return err;
    if (MPU_ERR_CHECK(writeMemory(FCFG_7, 3, accelRegs))) return err;

    return err = ESP_OK;
}

/**
 * @brief Return the DMP Packet Length based on DMP active features.
 * If `otherFeatures` parameter is passed, it will compute and return the packet length
 * for this `otherFeatures`, instead of current active features in the DMP. *
 * @param otherFeature Optional paramater. Other features to compute the packet length.
 */
uint8_t MPUdmp::getDMPPacketLength(dmp_feature_t otherFeatures)
{
    if (!otherFeatures) return this->packetLength;

    uint8_t otherPacketLength = 0;
    if (otherFeatures & DMP_FEATURE_ANY_LP_QUAT) otherPacketLength += 16;
    if (otherFeatures & DMP_FEATURE_SEND_RAW_ACCEL) otherPacketLength += 6;
    if (otherFeatures & DMP_FEATURE_SEND_ANY_GYRO) otherPacketLength += 6;
    if (otherFeatures & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) otherPacketLength += 4;
    return otherPacketLength;
}

/**
 * @brief Return the Packet Index of a certain feature.
 * @param feature Single Feature to get index of. Multiple ORed features not allowed.
 * @return
 *  - `int8_t > 0`: If feature enabled and it does sends data to FIFO.
 *  - `int8_t = -1`: If feature not enabled and/or it does not send data to FIFO.
 */
int8_t MPUdmp::getPacketIndex(dmp_feature_t feature)
{
    int8_t index = 0;
    if (enabledFeatures & DMP_FEATURE_ANY_LP_QUAT) {
        if (feature & DMP_FEATURE_ANY_LP_QUAT) return index;
        index += 16;
    }
    if (enabledFeatures & DMP_FEATURE_SEND_RAW_ACCEL) {
        if (feature & DMP_FEATURE_SEND_RAW_ACCEL) return index;
        index += 6;
    }
    if (enabledFeatures & DMP_FEATURE_SEND_ANY_GYRO) {
        if (feature & DMP_FEATURE_SEND_ANY_GYRO) return index;
        index += 6;
    }
    if (enabledFeatures & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) {
        if (feature & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) return index;
        index += 4;
    }
    return -1;
}

/**
 * @brief Return the Quaternion data from a DMP Packet.
 * Features `3X_LP_QUAT` or `6X_LP_QUAT` must be enabled.
 * @param fifoPacket DMP Packet got from FIFO.
 * @param quat Quaternion data.
 * @return esp_err_t
 */
esp_err_t MPUdmp::getDMPQuaternion(const uint8_t* fifoPacket, quat_q30_t* quat)
{
    const int8_t index = getPacketIndex(enabledFeatures & DMP_FEATURE_ANY_LP_QUAT);
    if (index < 0) {
        MPU_LOGEMSG(msgs::DMP_FEATURE_NOT_EN, "");
        return err = ESP_FAIL;
    }
    const uint8_t* data = fifoPacket + index;

    quat->w = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
    quat->x = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | (data[7]);
    quat->y = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | (data[11]);
    quat->z = (data[12] << 24) | (data[13] << 16) | (data[14] << 8) | (data[15]);
#if defined CONFIG_MPU_FIFO_CORRUPTION_CHECK
    // TODO
#endif
    return err = ESP_OK;
}

/**
 * @brief Return the Accelerometer raw data from a DMP Packet.
 * Feature `SEND_RAW_ACCEL` must be enabled.
 * @param fifoPacket DMP Packet got from FIFO.
 * @param accel Accelerometer raw data.
 */
esp_err_t MPUdmp::getDMPAccel(const uint8_t* fifoPacket, raw_axes_t* accel)
{
    const int8_t index = getPacketIndex(DMP_FEATURE_SEND_RAW_ACCEL);
    if (index < 0) {
        MPU_LOGEMSG(msgs::DMP_FEATURE_NOT_EN, "");
        return err = ESP_FAIL;
    }
    const uint8_t* data = fifoPacket + index;

    accel->x = (data[0] << 8) | (data[1]);
    accel->y = (data[2] << 8) | (data[3]);
    accel->z = (data[4] << 8) | (data[5]);

    return err = ESP_OK;
}

/**
 * @brief Return the Gyroscope raw data from a DMP Packet.
 * Features `SEND_RAW_GYRO` or `SEND_CAL_GYRO` must be enabled.
 * @param fifoPacket DMP Packet got from FIFO
 * @param accel Gyroscope raw or calibrated data, depending on enabled feature.
 */
esp_err_t MPUdmp::getDMPGyro(const uint8_t* fifoPacket, raw_axes_t* gyro)
{
    const int8_t index = getPacketIndex(DMP_FEATURE_SEND_ANY_GYRO);
    if (index < 0) {
        MPU_LOGEMSG(msgs::DMP_FEATURE_NOT_EN, "");
        return err = ESP_FAIL;
    }
    const uint8_t* data = fifoPacket + index;

    gyro->x = (data[0] << 8) | (data[1]);
    gyro->y = (data[2] << 8) | (data[3]);
    gyro->z = (data[4] << 8) | (data[5]);

    return err = ESP_OK;
}

/**
 * @brief Read DMP output data (single packet) from FIFO directly.
 */
esp_err_t MPUdmp::readDMPPacket(quat_q30_t* quat, raw_axes_t* gyro, raw_axes_t* accel)
{
    uint8_t* data = (uint8_t*) alloca(kDMPMaxPacketLength);
    if (MPU_ERR_CHECK(readFIFO(packetLength, data))) return err;

    if (enabledFeatures & DMP_FEATURE_ANY_LP_QUAT) {
        quat->w = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
        quat->x = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | (data[7]);
        quat->y = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | (data[11]);
        quat->z = (data[12] << 24) | (data[13] << 16) | (data[14] << 8) | (data[15]);
        data += 16;
#if defined CONFIG_MPU_FIFO_CORRUPTION_CHECK
        // TODO
#endif
    }

    if (enabledFeatures & DMP_FEATURE_SEND_RAW_ACCEL) {
        accel->x = (data[0] << 8) | (data[1]);
        accel->y = (data[2] << 8) | (data[3]);
        accel->z = (data[4] << 8) | (data[5]);
        data += 6;
    }

    if (enabledFeatures & DMP_FEATURE_SEND_ANY_GYRO) {
        gyro->x = (data[0] << 8) | (data[1]);
        gyro->y = (data[2] << 8) | (data[3]);
        gyro->z = (data[4] << 8) | (data[5]);
        data += 6;
    }

    /* Gesture data is at the end of the DMP packet. Parse it and call
     * the gesture callbacks (if registered).
     */
    // if (dmp.feature_mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) decode_gesture(fifo_data + ii);

    return err = ESP_OK;
}

}  // namespace mpud
