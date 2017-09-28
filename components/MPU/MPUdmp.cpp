#include "MPU.h"
#include "MPUdmp.h"
#include "MPUdefine.h"
#include "MPUtypes.h"
#include "MPUregistermap.h"
#include "MPUdmpmap.h"
#include "MPUdmpkey.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char* TAG = {"MPUdmp"};

#if defined MPU_ERROR_LOGGER

#define MPU_CHECK_RET(x)                                                                            \
    do {                                                                                            \
        MPU.err = x;                                                                                \
        if(MPU.err) {                                                                               \
            ESP_LOGE(TAG, "%s:%d (%s): error=%#x", __FILE__, __LINE__, __FUNCTION__, MPU.err);      \
            return MPU.err;                                                                         \
        }                                                                                           \
    }while(0)
#define MPU_CHECK_NORET(x)                                                                          \
    do {                                                                                            \
        MPU.err = x;                                                                                \
        if(MPU.err) {                                                                               \
            ESP_LOGE(TAG, "%s:%d (%s): error=%#x", __FILE__, __LINE__, __FUNCTION__, MPU.err);      \
        }                                                                                           \
    }while(0)                

#else /* ! MPU_ERROR_LOGGER */

#define MPU_CHECK_RET(x)                                                                            \
    do {                                                                                            \
        MPU.err = x;                                                                                \
        if(MPU.err) return MPU.err;                                                                 \
    }while(0)
#define MPU_CHECK_NORET(x)                                                                          \
    do {                                                                                            \
        MPU.err = x;                                                                                \
    }while(0)   

#endif /* end of MPU_ERROR_LOGGER */


#if defined FIFO_CORRUPTION_CHECK
#define QUAT_ERROR_THRESH       (1L<<24)
#define QUAT_MAG_SQ_NORMALIZED  (1L<<28)
#define QUAT_MAG_SQ_MIN         (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX         (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)
#endif

#define GYRO_SCALE_FACTOR   (46850825LL * 200 / DMP_SAMPLE_RATE)
#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_SEND_CAL_GYRO)




/* DMP_t class is declared 
 * within MPU_t class as private */

MPU_t::DMP_t::DMP_t(MPU_t& MPU) : MPU(MPU) {
}


bool MPU_t::DMP_t::isLoaded() {
    return loaded;
}


esp_err_t MPU_t::DMP_t::load() {
    // check if dmp is already loaded
    if(loaded) return ESP_OK;
    
    uint8_t buffer[DMP_CHUNK_SIZE];
    uint8_t length;
    uint16_t i = 0;

    // load the firmware to memory
    while(i < DMP_CODE_SIZE) {
        // watch out for the last chunk, it might be smaller
        length = (DMP_CHUNK_SIZE < (DMP_CODE_SIZE - i)) ? (DMP_CHUNK_SIZE) : (DMP_CODE_SIZE - i);
        // write chunk of dmp code
        MPU_CHECK_RET(MPU.writeMemory(i, length, ((uint8_t*) DMP_FIRMWARE + i)));
        // read chunk back
        MPU_CHECK_RET(MPU.readMemory(i, length, buffer));
        // compare data
        if(memcmp(DMP_FIRMWARE + i, buffer, length)) {
            #ifdef MPU_ERROR_LOGGER
                ESP_LOGE(TAG, "failed to load DMP, bank %d, chunk %d, length %d", (i >> 8), i, length);
            #endif
            MPU.err = ESP_FAIL;
            return MPU.err;
        }
        i += length;
    }

    // set DMP program start address
    MPU_CHECK_RET(MPU.setProgramStartAddress(DMP_START_ADDRESS));

    loaded = true;
    MPU.err = ESP_OK;
    return MPU.err;
}


esp_err_t MPU_t::DMP_t::initialize() {
    MPU_CHECK_RET(load());
    MPU.err = ESP_OK;
    return MPU.err;
}


esp_err_t MPU_t::DMP_t::setEnabled(bool enable) {
    if(dmpOn) {
        MPU.err = ESP_OK;
        return MPU.err;
    }
    // check for firmware loaded
    MPU_CHECK_RET((!loaded) ? ESP_FAIL : ESP_OK);
    MPU_CHECK_RET(MPU.I2C.writeBit(MPU.addr, MPU_REG_USER_CTRL, MPU_USERCTRL_DMP_EN_BIT, enable));
    if(enable) {
        MPU_CHECK_RET(reset());
    }
    dmpOn = enable;
    MPU.err = ESP_OK;
    return MPU.err;
}


bool MPU_t::DMP_t::getEnabled() {
    MPU_CHECK_NORET(MPU.I2C.readBit(MPU.addr, MPU_REG_USER_CTRL, MPU_USERCTRL_DMP_EN_BIT, MPU.buffer));
    return MPU.buffer[0];
}


esp_err_t MPU_t::DMP_t::reset() {
    return MPU.I2C.writeBit(MPU.addr, MPU_REG_USER_CTRL, MPU_USERCTRL_DMP_RESET_BIT, true);
}


esp_err_t MPU_t::DMP_t::setChipOrientation(uint16_t scalar) {
    const uint8_t gyro_axes[3] = {DINA4C, DINACD, DINA6C};
    const uint8_t accel_axes[3] = {DINA0C, DINAC9, DINA2C};
    const uint8_t gyro_sign[3] = {DINA36, DINA56, DINA76};
    const uint8_t accel_sign[3] = {DINA26, DINA46, DINA66};

    uint8_t gyro_regs[3], accel_regs[3];

    gyro_regs[0] = gyro_axes[scalar & 3];
    gyro_regs[1] = gyro_axes[(scalar >> 3) & 3];
    gyro_regs[2] = gyro_axes[(scalar >> 6) & 3];
    accel_regs[0] = accel_axes[scalar & 3];
    accel_regs[1] = accel_axes[(scalar >> 3) & 3];
    accel_regs[2] = accel_axes[(scalar >> 6) & 3];

    /* Chip-to-body, axes only. */
    MPU_CHECK_RET(MPU.writeMemory(DMP_FCFG_1, 3, gyro_regs));
    MPU_CHECK_RET(MPU.writeMemory(DMP_FCFG_2, 3, accel_regs));

    memcpy(gyro_regs, gyro_sign, 3);
    memcpy(accel_regs, accel_sign, 3);
    if (scalar & 0x4) {
        gyro_regs[0] |= 1;
        accel_regs[0] |= 1;
    }
    if (scalar & 0x20) {
        gyro_regs[1] |= 1;
        accel_regs[1] |= 1;
    }
    if (scalar & 0x100) {
        gyro_regs[2] |= 1;
        accel_regs[2] |= 1;
    }

    /* Chip-to-body, sign only. */
    MPU_CHECK_RET(MPU.writeMemory(DMP_FCFG_3, 3, gyro_regs));
    MPU_CHECK_RET(MPU.writeMemory(DMP_FCFG_7, 3, accel_regs));
    MPU.err = ESP_OK;
    return MPU.err;
}


uint16_t MPU_t::DMP_t::getChipOrientation() {
    uint16_t scalar = 0;
    // just read gyro config, don't need to read accel cfg 
    // because it's the same scalar as gyro
    // @see setOrientation() for understanding
    const uint8_t gyro_axes[3] = {DINA4C, DINACD, DINA6C};
    const uint8_t gyro_sign[3] = {DINA36, DINA56, DINA76};

    uint8_t gyro_regs[3];
    
    MPU_CHECK_RET(MPU.readMemory(DMP_FCFG_1, 3, gyro_regs));

    /* axes only */
    uint8_t axis = 0;
    for(int8_t i = 0; i < 3 && axis < 3; i++) { // should loop between 3 to 9 times
        if(gyro_regs[0 + axis] == gyro_axes[i]) {
            scalar |= i << (3 * axis);
            axis++;
            i = -1;
        }
    }
    MPU_CHECK_RET(axis < 2);

    MPU_CHECK_RET(MPU.readMemory(DMP_FCFG_3, 3, gyro_regs));

    /* sign only */
    int8_t i;
    for(i = 0; i < 3; i++) {
        if(gyro_regs[i] == (gyro_sign[i] | 1))
            scalar |= 1 << (i * 3 + 2);
    }

    return scalar;
}


uint16_t MPU_t::DMP_t::getScalarFromMatrix(const int8_t *matrix) {
    /* Scalar:
     * XYZ  010_001_000 Identity Matrix
     * XZY  001_010_000
     * YXZ  010_000_001
     * YZX  000_010_001
     * ZXY  001_000_010
     * ZYX  000_001_010
     *
     * X = 00
     * Y = 01
     * Z = 10
     *
     * bit2 = sign
     * -X = 100  -Y = 101  -Z = 110
     */
    uint16_t scalar = 0;
    uint16_t b;

    for(size_t i = 0; i < 7; i+=3) {
        if(matrix[0 + i] > 0)
            b = 0;
        else if(matrix[0 + i] < 0)
            b = 4;
        else if(matrix[1 + i] > 0)
            b = 1;
        else if(matrix[1 + i] < 0)
            b = 5;
        else if(matrix[2 + i] > 0)
            b = 2;
        else if(matrix[2 + i] < 0)
            b = 6;
        else
            b = 7;  // error
        scalar |= b << i; 
    }

    return scalar;
}


void MPU_t::DMP_t::getMatrixFromScalar(int8_t *matrix, uint16_t scalar) {
    // zero matrix
    for(size_t i = 0; i < 9; i++)
        matrix[i] = 0;

    uint16_t b;
    for(size_t i = 0; i < 7; i+=3) {
        b = (scalar >> i) & 7;
        if(b == 0)
            matrix[0 + i] = 1;
        else if(b == 4)
            matrix[0 + i] = -1;
        else if(b == 1)
            matrix[1 + i] = 1;
        else if(b == 5)
            matrix[1 + i] = -1;
        else if(b == 2)
            matrix[2 + i] = 1;
        else if(b == 6)
            matrix[2 + i] = -1;
        else{   
            matrix[0 + i] = b; // error
            matrix[1 + i] = b;
            matrix[2 + i] = b;
        }
    }
}


static const uint8_t sendRawAccelRegsEnabled[3] = {0xC0, 0xC8, 0xC2};
static const uint8_t sendAnyGyroRegsEnabled[3] = {0xC4, 0xCC, 0xC6};
static const uint8_t sendRawDataRegsDisabled[3] = {0xA3, 0xA3, 0xA3};
static const uint8_t sendCalGyroRegsEnabled[4] = {0xB2, 0x8B, 0xB6, 0x9B};
static const uint8_t sendCalGyroRegsDisabled[4] = {DINAC0, DINA80, DINAC2, DINA90};

esp_err_t MPU_t::DMP_t::setFeaturesEnabled(dmp_features_t features) {
    /* Set integration scale factor. */
    MPU.buffer[0] = (uint8_t)((GYRO_SCALE_FACTOR >> 24) & 0xFF);
    MPU.buffer[1] = (uint8_t)((GYRO_SCALE_FACTOR >> 16) & 0xFF);
    MPU.buffer[2] = (uint8_t)((GYRO_SCALE_FACTOR >> 8) & 0xFF);
    MPU.buffer[3] = (uint8_t)(GYRO_SCALE_FACTOR & 0xFF);
    MPU_CHECK_RET(MPU.writeMemory(DMP_D_0_104, 4, MPU.buffer));

    /* Send sensor data to the FIFO. */
    MPU.buffer[0] = 0xA3;
    if (features & DMP_FEATURE_SEND_RAW_ACCEL) {
        memcpy(MPU.buffer+1, sendRawAccelRegsEnabled, 3);
    } else {
        memcpy(MPU.buffer+1, sendRawDataRegsDisabled, 3);
    }
    if (features & DMP_FEATURE_SEND_ANY_GYRO) {
        memcpy(MPU.buffer+4, sendAnyGyroRegsEnabled, 3);
    } else {
        memcpy(MPU.buffer+4, sendRawDataRegsDisabled, 3);
    }
    MPU.buffer[7] = 0xA3;
    MPU.buffer[8] = 0xA3;
    MPU.buffer[9] = 0xA3;
    MPU_CHECK_RET(MPU.writeMemory(DMP_CFG_15, 10, MPU.buffer));

    if (features & DMP_FEATURE_SEND_ANY_GYRO) {
        if (features & DMP_FEATURE_SEND_CAL_GYRO) {
            memcpy(MPU.buffer, sendCalGyroRegsEnabled, 4);
        } else {
            memcpy(MPU.buffer, sendCalGyroRegsDisabled, 4);
        }
        MPU_CHECK_RET(MPU.writeMemory(DMP_CFG_GYRO_RAW_DATA, 4, MPU.buffer));
    }
    
    /* Send gesture data to the FIFO. */
    if (features & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        MPU.buffer[0] = DINA20;
    else
        MPU.buffer[0] = 0xD8;
    MPU_CHECK_RET(MPU.writeMemory(DMP_CFG_27, 1, MPU.buffer));

    /* Enable Gyro Calibration */
    setGyroAutoCalibrationEnabled(features & DMP_FEATURE_GYRO_CAL);

    /* Enable Tap. */
    if (features & DMP_FEATURE_TAP) {
        MPU.buffer[0] = 0xF8;
        MPU_CHECK_RET(MPU.writeMemory(DMP_CFG_20, 1, MPU.buffer));

        MPU_CHECK_RET(setTapThreshold(DMP_TAP_XYZ, 250));
        MPU_CHECK_RET(setTapAxesEnabled(DMP_TAP_XYZ));
        MPU_CHECK_RET(setTapCount(1));
        MPU_CHECK_RET(setTapTime(100));
        MPU_CHECK_RET(setTapTimeMulti(500));

        MPU_CHECK_RET(setShakeRejectThreshold(200));
        MPU_CHECK_RET(setShakeRejectTime(40));
        MPU_CHECK_RET(setShakeRejectTimeout(10));
    } else {
        MPU.buffer[0] = 0xD8;
        MPU_CHECK_RET(MPU.writeMemory(DMP_CFG_20, 1, MPU.buffer));
    }

    /* Enable Android Orientation */
    if (features & DMP_FEATURE_ANDROID_ORIENT)
        MPU.buffer[0] = 0xD9;
    else
        MPU.buffer[0] = 0xD8;
    MPU_CHECK_RET(MPU.writeMemory(DMP_CFG_ANDROID_ORIENT_INT, 1, MPU.buffer));

    /* Enable gyro-only Quaternion */
    setLPQuaternionEnabled(features & DMP_FEATURE_LP_QUAT);

    /* Enable gyro-accel Quaternion */
    setLPQuaternion6XEnabled(features & DMP_FEATURE_6X_LP_QUAT);

    /* Pedometer is always enabled */

    // Clean FIFO
    MPU_CHECK_RET(MPU.resetFIFO());

    // Calculate packet size
    packetSize = 0;
    if (features & DMP_FEATURE_SEND_RAW_ACCEL)
        packetSize += DMP_ACCEL_PACKET_SIZE;
    if (features & DMP_FEATURE_SEND_ANY_GYRO)
        packetSize += DMP_GYRO_PACKET_SIZE;
    if (features & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
        packetSize += DMP_QUATERNION_PACKET_SIZE;
    if (features & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        packetSize += DMP_GESTURE_PACKET_SIZE;

    this->features = features;
    MPU.err = ESP_OK;
    return MPU.err;
}


dmp_features_t MPU_t::DMP_t::getFeaturesEnabled() {
    dmp_features_t features = 0;
    MPU.err = ESP_OK;
    /* check SEND_RAW_ACCEL and SEND_ANY_GYRO regs*/
    MPU_CHECK_NORET(MPU.readMemory(DMP_CFG_15, 10, MPU.buffer));
    if(!memcmp(MPU.buffer+1, sendRawAccelRegsEnabled, 3))
        features |= DMP_FEATURE_SEND_RAW_ACCEL;
    else if(memcmp(MPU.buffer+1, sendRawDataRegsDisabled, 3)) {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGW(TAG, "%s -> Unknown state for SEND_RAW_ACCEL regs. No match.", __FUNCTION__);
        #endif
        MPU.err = ESP_ERR_INVALID_STATE;
    }
    if(!memcmp(MPU.buffer+4, sendAnyGyroRegsEnabled, 3)) {
        MPU_CHECK_NORET(MPU.readMemory(DMP_CFG_GYRO_RAW_DATA, 4, MPU.buffer+10));
        if(!memcmp(MPU.buffer+10, sendCalGyroRegsEnabled, 4))
            features |= DMP_FEATURE_SEND_CAL_GYRO;
        else if(memcmp(MPU.buffer+10, sendCalGyroRegsDisabled, 4)) {
            #ifdef MPU_ERROR_LOGGER
                ESP_LOGW(TAG, "%s -> Unknown state for SEND_CAL_GYRO regs. No match.", __FUNCTION__);
            #endif
            MPU.err = ESP_ERR_INVALID_STATE;
        }
        else
            features |= DMP_FEATURE_SEND_RAW_GYRO;
    }
    else if(memcmp(MPU.buffer+4, sendRawDataRegsDisabled, 3)) {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGW(TAG, "%s -> Unknown state for SEND_ANY_GYRO regs. No match.", __FUNCTION__);
        #endif
        MPU.err = ESP_ERR_INVALID_STATE;        
    }

    /* check GESTURES regs */
    MPU_CHECK_NORET(MPU.readMemory(DMP_CFG_27, 1, MPU.buffer));
    if(MPU.buffer[0] == DINA20) {
        /* check TAP reg */
        MPU_CHECK_NORET(MPU.readMemory(DMP_CFG_20, 1, MPU.buffer+1));
        if(MPU.buffer[1] == 0xF8)
            features |= DMP_FEATURE_TAP;
        else if(MPU.buffer[1] != 0xD8) {
            #ifdef MPU_ERROR_LOGGER
                ESP_LOGW(TAG, "%s -> Unknown state for TAP reg. No match.", __FUNCTION__);
            #endif
            MPU.err = ESP_ERR_INVALID_STATE;            
        }
        /* check ANDROID_ORIENT reg */
        MPU_CHECK_NORET(MPU.readMemory(DMP_CFG_ANDROID_ORIENT_INT, 1, MPU.buffer+1));
        if(MPU.buffer[1] == 0xD9)
            features |= DMP_FEATURE_ANDROID_ORIENT;
        else if(MPU.buffer[1] != 0xD8) {
            #ifdef MPU_ERROR_LOGGER
                ESP_LOGW(TAG, "%s -> Unknown state for ANDROID_ORIENT reg. No match.", __FUNCTION__);
            #endif
            MPU.err = ESP_ERR_INVALID_STATE;  
        }
    }
    else if(MPU.buffer[0] != 0xD8) {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGW(TAG, "%s -> Unknown state for GESTURE reg. No match.", __FUNCTION__);
        #endif
        MPU.err = ESP_ERR_INVALID_STATE;
    }

    if(getGyroAutoCalibrationEnabled())
        features |= DMP_FEATURE_GYRO_CAL;

    if(getLPQuaternionEnabled())
        features |= DMP_FEATURE_LP_QUAT;

    if(getLPQuaternion6XEnabled())
        features |= DMP_FEATURE_6X_LP_QUAT;
    
    /* Pedometer is always enabled */
    features |= DMP_FEATURE_PEDOMETER;

    return features;
}


static const uint8_t gyroCalibRegsEnable[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
static const uint8_t gyroCalibRegsDisable[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};

esp_err_t MPU_t::DMP_t::setGyroAutoCalibrationEnabled(bool enable) {
    MPU_CHECK_NORET(MPU.writeMemory(DMP_CFG_MOTION_BIAS, 9, 
        (uint8_t*)(enable ? gyroCalibRegsEnable : gyroCalibRegsDisable)));
    return MPU.err;
}


bool MPU_t::DMP_t::getGyroAutoCalibrationEnabled() {
    uint8_t regs[9];
    MPU_CHECK_NORET(MPU.readMemory(DMP_CFG_MOTION_BIAS, 9, regs));
    if(!memcmp(regs, gyroCalibRegsEnable, 9))
        return true;
    else if(!memcmp(regs, gyroCalibRegsDisable, 9))
        return false;
    else {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGE(TAG, "%s -> Unknown gyro calibration state. No match.", __FUNCTION__);
        #endif
        MPU.err = ESP_ERR_INVALID_STATE;
    }
    return false;
}

static const uint8_t lpQuaternionRegsEnable[4] = {DINBC0, DINBC2, DINBC4, DINBC6};
static const uint8_t lpQuaternionRegsDisable[4] = {0x8B, 0x8B, 0x8B, 0x8B};

esp_err_t MPU_t::DMP_t::setLPQuaternionEnabled(bool enable) {
    MPU_CHECK_RET(MPU.writeMemory(DMP_CFG_LP_QUAT, 4, (uint8_t*)(enable ? lpQuaternionRegsEnable : lpQuaternionRegsDisable)));
    MPU_CHECK_NORET(MPU.resetFIFO());
    return MPU.err;
}


bool MPU_t::DMP_t::getLPQuaternionEnabled() {
    uint8_t regs[4];
    MPU_CHECK_NORET(MPU.readMemory(DMP_CFG_LP_QUAT, 4, regs));

    if(!memcmp(regs, lpQuaternionRegsEnable, 4))
        return true;
    else if(!memcmp(regs, lpQuaternionRegsDisable, 4))
        return false;
    else {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGE(TAG, "%s -> Unknown LP Quaternion State. No match.", __FUNCTION__);
        #endif
        MPU.err = ESP_ERR_INVALID_STATE;
    }
    return false;
}


static const uint8_t lpQuaternion6XRegsEnable[4] = {DINA20, DINA28, DINA30, DINA38};
static const uint8_t lpQuaternion6XRegsDisable[4] = {0xA3, 0xA3, 0xA3, 0xA3};

esp_err_t MPU_t::DMP_t::setLPQuaternion6XEnabled(bool enable) {
    MPU_CHECK_RET(MPU.writeMemory(DMP_CFG_8, 4, (uint8_t*)(enable ? lpQuaternion6XRegsEnable : lpQuaternion6XRegsDisable)));
    MPU_CHECK_NORET(MPU.resetFIFO());
    return MPU.err;
}


bool MPU_t::DMP_t::getLPQuaternion6XEnabled() {
    uint8_t regs[4];
    MPU_CHECK_NORET(MPU.readMemory(DMP_CFG_8, 4, regs));

    if(!memcmp(regs, lpQuaternion6XRegsEnable, 4))
        return true;
    else if(!memcmp(regs, lpQuaternion6XRegsDisable, 4))
        return false;
    else {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGE(TAG, "%s -> Unknown LP Quaternion 6X State. No match.", __FUNCTION__);
        #endif
        MPU.err = ESP_ERR_INVALID_STATE;
    }
    return false;
}


static const uint8_t intModeRegsPacket[11] = {0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9};
static const uint8_t intModeRegsGesture[11] = {0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda};

esp_err_t MPU_t::DMP_t::setDMPIntMode(dmp_int_mode_t mode) {
    if(mode == DMP_INT_MODE_PACKET)
        MPU_CHECK_NORET(MPU.writeMemory(DMP_CFG_FIFO_ON_EVENT, 11, (uint8_t*)intModeRegsPacket));
    else // == DMP_INT_MODE_GESTURE
        MPU_CHECK_NORET(MPU.writeMemory(DMP_CFG_FIFO_ON_EVENT, 11, (uint8_t*)intModeRegsGesture));
    return MPU.err;
}


dmp_int_mode_t MPU_t::DMP_t::getDMPIntMode() {
    uint8_t regs[11];
    MPU_CHECK_NORET(MPU.readMemory(DMP_CFG_FIFO_ON_EVENT, 11, regs));
    if(!memcmp(regs, intModeRegsPacket, 11))
        return DMP_INT_MODE_PACKET;
    else if(!memcmp(regs, intModeRegsGesture, 11))
        return DMP_INT_MODE_GESTURE;
    else {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGE(TAG, "%s -> Unknown DMP Interrupt Mode. No match.", __FUNCTION__);
        #endif
        MPU.err = ESP_FAIL;
    }
    return (dmp_int_mode_t) -1;
}


static const uint8_t fifoRateRegsEnd[12] = {DINAFE, DINAF2, DINAAB, 0xc4, DINAAA, DINAF1, DINADF, DINADF, 0xBB, 0xAF, DINADF, DINADF};

esp_err_t MPU_t::DMP_t::setFIFORate(uint16_t rate) {
    if(rate > DMP_SAMPLE_RATE) {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGE(TAG, "%s -> Invalid FIFO rate (%d). It must be <= %d", __FUNCTION__, rate, DMP_SAMPLE_RATE);
        #endif
        MPU.err = ESP_ERR_INVALID_ARG;
        return MPU.err;
    }

    uint16_t div = DMP_SAMPLE_RATE / rate - 1;
    MPU.buffer[0] = (uint8_t)((div >> 8) & 0xFF);
    MPU.buffer[1] = (uint8_t)(div & 0xFF);

    MPU_CHECK_RET(MPU.writeMemory(DMP_D_0_22, 2, MPU.buffer));
    MPU_CHECK_NORET(MPU.writeMemory(DMP_CFG_6, 12, (uint8_t*)fifoRateRegsEnd));

    return MPU.err;
}


uint16_t MPU_t::DMP_t::getFIFORate() {
    MPU_CHECK_RET(MPU.readMemory(DMP_D_0_22, 2, MPU.buffer));
    uint16_t div = (MPU.buffer[0] << 8) | MPU.buffer[1];
    return DMP_SAMPLE_RATE / (div + 1);
}


uint8_t MPU_t::DMP_t::getFIFOPacketSize() {
    return packetSize;
}


bool MPU_t::DMP_t::packetAvailable() {
    return MPU.getFIFOCount() >= packetSize;
}


esp_err_t MPU_t::DMP_t::getFIFOPacket(uint8_t *packet) {
    MPU_CHECK_NORET(MPU.readFIFO(packet, packetSize));
    return MPU.err;
}


esp_err_t MPU_t::DMP_t::setTapThreshold(dmp_tap_axis_t axis, uint16_t thresh) {
    if(!(axis & DMP_TAP_XYZ) || thresh > 1600) {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGE(TAG, "%s -> Invalid argument. No axis passed or threshold > 1600.", __FUNCTION__);
        #endif
        MPU.err = ESP_ERR_INVALID_ARG;
        return MPU.err;
    }

    
    /* Accel full-scale-range to scale-factor */
    uint16_t accelScaleFactor = pow(2, (MPU.getAccelFullScale() + 1)) * 1024;
    if(MPU.err) return MPU.err;
    
    float scaledThresh = (float)thresh / DMP_SAMPLE_RATE;

    uint16_t dmpThresh[2];
    dmpThresh[0] = (uint16_t)(scaledThresh * accelScaleFactor);
    dmpThresh[1] = (uint16_t)(scaledThresh * (accelScaleFactor * 0.75f));

    MPU.buffer[0] = (uint8_t)(dmpThresh[0] >> 8);
    MPU.buffer[1] = (uint8_t)(dmpThresh[0] & 0xFF);
    MPU.buffer[2] = (uint8_t)(dmpThresh[1] >> 8);
    MPU.buffer[3] = (uint8_t)(dmpThresh[1] & 0xFF);

    if (axis & DMP_TAP_X) {
        MPU_CHECK_RET(MPU.writeMemory(DMP_TAP_THX, 2, MPU.buffer));
        MPU_CHECK_RET(MPU.writeMemory(DMP_D_1_36, 2, MPU.buffer+2));
    }
    if (axis & DMP_TAP_Y) {
        MPU_CHECK_RET(MPU.writeMemory(DMP_TAP_THY, 2, MPU.buffer));
        MPU_CHECK_RET(MPU.writeMemory(DMP_D_1_40, 2, MPU.buffer+2));
    }
    if (axis & DMP_TAP_Z) {
        MPU_CHECK_RET(MPU.writeMemory(DMP_TAP_THZ, 2, MPU.buffer));
        MPU_CHECK_RET(MPU.writeMemory(DMP_D_1_44, 2, MPU.buffer+2));
    }

    MPU.err = ESP_OK;
    return MPU.err;
}


uint16_t MPU_t::DMP_t::getTapThreshold(dmp_tap_axis_t axis) {
    if (axis & DMP_TAP_X) {
        MPU_CHECK_RET(MPU.readMemory(DMP_TAP_THX, 2, MPU.buffer));
    }
    else if (axis & DMP_TAP_Y) {
        MPU_CHECK_RET(MPU.readMemory(DMP_TAP_THY, 2, MPU.buffer));
    }
    else if (axis & DMP_TAP_Z) {
        MPU_CHECK_RET(MPU.readMemory(DMP_TAP_THZ, 2, MPU.buffer));
    }

    uint16_t dmpThresh;
    dmpThresh = (MPU.buffer[0] << 8) | MPU.buffer[1];

    /* Accel full-scale-range to scale-factor */
    uint16_t accelScaleFactor = pow(2, (MPU.getAccelFullScale() + 1)) * 1024;
    if(MPU.err) return MPU.err;

    uint16_t thresh = (uint16_t)(((float)dmpThresh / accelScaleFactor) * DMP_SAMPLE_RATE);

    return thresh;
}


esp_err_t MPU_t::DMP_t::setTapAxesEnabled(dmp_tap_axis_t axis) {
    if(!(axis & DMP_TAP_XYZ)) {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGE(TAG, "%s -> Invalid argument. No axis passed.", __FUNCTION__);
        #endif
        MPU.err = ESP_ERR_INVALID_ARG;
        return MPU.err;
    }

    axis &= DMP_TAP_XYZ;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_D_1_72, 1, &axis));
    return MPU.err;
}


dmp_tap_axis_t MPU_t::DMP_t::getTapAxesEnabled() {
    dmp_tap_axis_t axis;
    MPU_CHECK_NORET(MPU.readMemory(DMP_D_1_72, 1, &axis));
    return (dmp_tap_axis_t) (axis & DMP_TAP_XYZ);
}


esp_err_t MPU_t::DMP_t::setTapCount(uint8_t count) {    
    if (count < 1) {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGW(TAG, "%s -> Count (%d) constrained to %d, range(1~4)", __FUNCTION__, count, 1);
        #endif
        count = 1;
    }
    else if (count > 4) {
        #ifdef MPU_ERROR_LOGGER
            ESP_LOGW(TAG, "%s -> Count (%d) constrained to %d, range(1~4)", __FUNCTION__, count, 4);
        #endif
        count = 4;
    }

    MPU.buffer[0] = count - 1;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_D_1_79, 1, MPU.buffer));
    return MPU.err;
}


uint8_t MPU_t::DMP_t::getTapCount() {
    uint8_t count;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_D_1_79, 1, &count));
    count++;

    #ifdef MPU_ERROR_LOGGER
        if(count < 1 || count > 4) {
            ESP_LOGW(TAG, "%s -> read count (%d) does not match constrains.", __FUNCTION__, count);
        }
    #endif

    return count;
}


esp_err_t MPU_t::DMP_t::setTapTime(uint16_t time) {
    uint16_t dmpTime = time / (1000 / DMP_SAMPLE_RATE);
    MPU.buffer[0] = dmpTime >> 8;
    MPU.buffer[1] = dmpTime & 0xFF;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_TAPW_MIN, 2, MPU.buffer));
    return MPU.err;
}


uint16_t MPU_t::DMP_t::getTapTime() {
    MPU_CHECK_NORET(MPU.readMemory(DMP_TAPW_MIN, 2, MPU.buffer));
    uint16_t dmpTime = (MPU.buffer[0] << 8) | MPU.buffer[1];
    return dmpTime * (1000 / DMP_SAMPLE_RATE);
}


esp_err_t MPU_t::DMP_t::setTapTimeMulti(uint16_t time) {
    uint16_t dmpTime = time / (1000 / DMP_SAMPLE_RATE);
    MPU.buffer[0] = dmpTime >> 8;
    MPU.buffer[1] = dmpTime & 0xFF;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_D_1_218, 2, MPU.buffer));
    return MPU.err;
}


uint16_t MPU_t::DMP_t::getTapTimeMulti() {
    MPU_CHECK_NORET(MPU.readMemory(DMP_D_1_218, 2, MPU.buffer));
    uint16_t dmpTime = (MPU.buffer[0] << 8) | MPU.buffer[1];
    return dmpTime * (1000 / DMP_SAMPLE_RATE);
}


esp_err_t MPU_t::DMP_t::setShakeRejectThreshold(uint16_t thresh) {
    uint32_t scaledThresh = GYRO_SCALE_FACTOR / 1000 * thresh;
    MPU.buffer[0] = (scaledThresh >> 24) & 0xFF;
    MPU.buffer[1] = (scaledThresh >> 16) & 0xFF;
    MPU.buffer[2] = (scaledThresh >> 8) & 0xFF;
    MPU.buffer[3] = scaledThresh & 0xFF;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_D_1_92, 4, MPU.buffer));
    return MPU.err;
}


uint16_t MPU_t::DMP_t::getShakeRejectThreshold() {
    MPU_CHECK_NORET(MPU.readMemory(DMP_D_1_92, 4, MPU.buffer));
    uint32_t scaledThresh = (MPU.buffer[0] << 24) | (MPU.buffer[1] << 16) | (MPU.buffer[2] << 8) | MPU.buffer[3];
    return scaledThresh / (GYRO_SCALE_FACTOR / 1000);
}


esp_err_t MPU_t::DMP_t::setShakeRejectTime(uint16_t time) {
    uint16_t dmpTime = time / (1000 / DMP_SAMPLE_RATE);
    MPU.buffer[0] = dmpTime >> 8;
    MPU.buffer[1] = dmpTime & 0xFF;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_D_1_90, 2, MPU.buffer));
    return MPU.err;
}


uint16_t MPU_t::DMP_t::getShakeRejectTime() {
    MPU_CHECK_NORET(MPU.readMemory(DMP_D_1_90, 2, MPU.buffer));
    uint16_t dmpTime = (MPU.buffer[0] << 8) | MPU.buffer[1];
    return dmpTime * (1000 / DMP_SAMPLE_RATE);
}


esp_err_t MPU_t::DMP_t::setShakeRejectTimeout(uint16_t timeout) {
    uint16_t dmpTimeout = timeout / (1000 / DMP_SAMPLE_RATE);
    MPU.buffer[0] = dmpTimeout >> 8;
    MPU.buffer[1] = dmpTimeout & 0xFF;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_D_1_88, 2, MPU.buffer));
    return MPU.err;
}


uint16_t MPU_t::DMP_t::getShakeRejectTimeout() {
    MPU_CHECK_NORET(MPU.readMemory(DMP_D_1_88, 2, MPU.buffer));
    uint16_t dmpTimeout = (MPU.buffer[0] << 8) | MPU.buffer[1];
    return dmpTimeout * (1000 / DMP_SAMPLE_RATE);
}


esp_err_t MPU_t::DMP_t::setPedometerStepCount(uint32_t count) {
    MPU.buffer[0] = (count >> 24) & 0xFF;
    MPU.buffer[1] = (count >> 16) & 0xFF;
    MPU.buffer[2] = (count >> 8) & 0xFF;
    MPU.buffer[3] = count & 0xFF;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_D_PEDSTD_STEPCTR, 4, MPU.buffer));
    return MPU.err;
}


uint32_t MPU_t::DMP_t::getPedometerStepCount() {
    MPU_CHECK_NORET(MPU.readMemory(DMP_D_PEDSTD_STEPCTR, 4, MPU.buffer));
    uint32_t count = (MPU.buffer[0] << 24) | (MPU.buffer[1] << 16) | (MPU.buffer[2] << 8) | MPU.buffer[3];
    return count;
}


esp_err_t MPU_t::DMP_t::setPedometerWalkTime(uint32_t time) {
    uint32_t dmpTime = time / 20;
    MPU.buffer[0] = (dmpTime >> 24) & 0xFF;
    MPU.buffer[1] = (dmpTime >> 16) & 0xFF;
    MPU.buffer[2] = (dmpTime >> 8) & 0xFF;
    MPU.buffer[3] = dmpTime & 0xFF;
    MPU_CHECK_NORET(MPU.writeMemory(DMP_D_PEDSTD_TIMECTR, 4, MPU.buffer));
    return MPU.err;
}


uint32_t MPU_t::DMP_t::getPedometerWalkTime() {
    MPU_CHECK_NORET(MPU.readMemory(DMP_D_PEDSTD_TIMECTR, 4, MPU.buffer));
    uint32_t dmpTime = (MPU.buffer[0] << 24) | (MPU.buffer[1] << 16) | (MPU.buffer[2] << 8) | MPU.buffer[3];
    return dmpTime * 20;
}


esp_err_t MPU_t::DMP_t::getPacketIndex(dmp_features_t feature, uint8_t *index) {
    // returns ESP_FAIL if feature is no enabled
    if(!(features & feature))
        return ESP_ERR_NOT_FOUND;

    const dmp_features_t featuresSequence[4] = {
        DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_LP_QUAT,
        DMP_FEATURE_SEND_RAW_ACCEL,
        DMP_FEATURE_SEND_ANY_GYRO,
        DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT
    };

    const uint8_t featuresPacketSize[4] = {
        DMP_QUATERNION_PACKET_SIZE,
        DMP_ACCEL_PACKET_SIZE,
        DMP_GYRO_PACKET_SIZE,
        DMP_GESTURE_PACKET_SIZE
    };

    int i;
    for(i = 0; i < 4; i++) {
        // when got the feature, stops incrementing index
        if(feature & featuresSequence[i])
            break;
        // only increment index if previous feature is enabled
        if(features & featuresSequence[i])
            *index += featuresPacketSize[i];
    }

    if(i > 3)
        MPU.err = ESP_FAIL;
    else
        MPU.err = ESP_OK;

    return MPU.err;
}


esp_err_t MPU_t::DMP_t::getQuaternion(int32_t *quat, uint8_t *packet) {
    uint8_t i;
    MPU_CHECK_RET(getPacketIndex(DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT, &i));
    quat[0] = (packet[i] << 24) | (packet[i+1] << 16) | (packet[i+2] << 8) | packet[i+3];
    quat[1] = (packet[i+4] << 24) | (packet[i+5] << 16) | (packet[i+6] << 8) | packet[i+7];
    quat[2] = (packet[i+8] << 24) | (packet[i+9] << 16) | (packet[i+10] << 8) | packet[i+11];
    quat[3] = (packet[i+12] << 24) | (packet[i+13] << 16) | (packet[i+14] << 8) | packet[i+15];

    #ifdef FIFO_CORRUPTION_CHECK
        /* We can detect a corrupted FIFO by monitoring the quaternion data and
         * ensuring that the magnitude is always normalized to one. This
         * shouldn't happen in normal operation, but if an I2C error occurs,
         * the FIFO reads might become misaligned.
         *
         * Scaling down the quaternion data to avoid long long math.
         */
        int32_t quatQ14[4], quatMagSq;

        quatQ14[0] = quat[0] >> 16;
        quatQ14[1] = quat[1] >> 16;
        quatQ14[2] = quat[2] >> 16;
        quatQ14[3] = quat[3] >> 16;

        quatMagSq = quatQ14[0] * quatQ14[0] + quatQ14[1] * quatQ14[1] +
            quatQ14[2] * quatQ14[2] + quatQ14[3] * quatQ14[3];

        if ((quatMagSq < QUAT_MAG_SQ_MIN) || (quatMagSq > QUAT_MAG_SQ_MAX)) {
            /* Quaternion is outside of the acceptable threshold. */
            MPU.resetFIFO();
            #ifdef MPU_ERROR_LOGGER
                ESP_LOGW(TAG, "%s -> FIFO Corruption. Quaternion is outside of the acceptable threshold.", __FUNCTION__);
            #endif
            MPU.err = ESP_ERR_INVALID_STATE;
            return MPU.err;
        }

    #endif /* FIFO_CORRUPTION_CHECK */

    MPU.err = ESP_OK;
    return MPU.err;
}


esp_err_t MPU_t::DMP_t::getAccel(mpu_axis_t *axes, uint8_t *packet) {
    uint8_t i;
    MPU_CHECK_RET(getPacketIndex(DMP_FEATURE_SEND_RAW_ACCEL, &i));

    axes->x = (packet[i+0] << 8) | packet[i+1];
    axes->y = (packet[i+2] << 8) | packet[i+3];
    axes->z = (packet[i+4] << 8) | packet[i+5];

    MPU.err = ESP_OK;
    return MPU.err;
}


esp_err_t MPU_t::DMP_t::getGyro(mpu_axis_t *axes, uint8_t *packet) {
    uint8_t i;
    MPU_CHECK_RET(getPacketIndex(DMP_FEATURE_SEND_ANY_GYRO, &i));

    axes->x = (packet[i+0] << 8) | packet[i+1];
    axes->y = (packet[i+2] << 8) | packet[i+3];
    axes->z = (packet[i+4] << 8) | packet[i+5];

    MPU.err = ESP_OK;
    return MPU.err;
}


#define TAP_INT_BIT (0x01)
esp_err_t MPU_t::DMP_t::getTap(uint8_t *direction, uint8_t *count, uint8_t *packet) {
    uint8_t i;
    MPU_CHECK_RET(getPacketIndex(DMP_FEATURE_TAP, &i));

    if(packet[i+1] & TAP_INT_BIT) {
        uint8_t tap = packet[i+3] & DMP_TAP_XYZ;
        *direction = tap >> 3;
        *count = (tap % 8) + 1;
    }

    MPU.err = ESP_OK;
    return MPU.err;
}


#define ANDROID_ORIENT_INT_BIT (0x8)
esp_err_t MPU_t::DMP_t::getAndroidOrientation(uint8_t *orient, uint8_t *packet) {
    uint8_t i;
    MPU_CHECK_RET(getPacketIndex(DMP_FEATURE_ANDROID_ORIENT, &i));

    if(packet[i+1] & ANDROID_ORIENT_INT_BIT) {
        *orient = (packet[i+3] & 0xC0) >> 6;
    }

    MPU.err = ESP_OK;
    return MPU.err;
}

















