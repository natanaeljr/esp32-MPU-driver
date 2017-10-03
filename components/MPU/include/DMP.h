#ifndef _DMP_H_
#define _DMP_H_

#include "MPU.h"
#include <stdint.h>
#include "esp_err.h"


class DMP_t {
private: 
    MPU_t& MPU;
    uint8_t* buffer = MPU.buffer;
    esp_err_t& err = MPU.err;
    bool dmpOn = false;
    bool loaded = false;
    dmp_features_t features;
    uint8_t packetSize = 0;

public:
    DMP_t(MPU_t& MPU);
    bool isLoaded();
    
    // SETUP
    esp_err_t initialize();
    esp_err_t setEnabled(bool enable);
    bool getEnabled();
    esp_err_t reset();
    
    // CONFIGS
    esp_err_t setChipOrientation(uint16_t scalar);
    uint16_t getChipOrientation();
    uint16_t getScalarFromMatrix(const int8_t *matrix);
    void getMatrixFromScalar(int8_t *matrix, uint16_t scalar);

    // FEATURES
    esp_err_t setFeaturesEnabled(dmp_features_t features);
    dmp_features_t getFeaturesEnabled(); // TODO: finish
    esp_err_t setGyroAutoCalibrationEnabled(bool enable);
    bool getGyroAutoCalibrationEnabled();
    esp_err_t setLPQuaternionEnabled(bool enable);
    bool getLPQuaternionEnabled();
    esp_err_t setLPQuaternion6XEnabled(bool enable);
    bool getLPQuaternion6XEnabled();
    
    // INTERRUPT
    esp_err_t setDMPIntMode(dmp_int_mode_t mode);
    dmp_int_mode_t getDMPIntMode();
    
    // PACKETS
    esp_err_t setFIFORate(uint16_t rate);
    uint16_t getFIFORate();
    uint8_t getFIFOPacketSize();
    bool packetAvailable();
    esp_err_t getFIFOPacket(uint8_t *packet);

    // TAP config
    esp_err_t setTapThreshold(dmp_tap_axis_t axis, uint16_t thresh);
    esp_err_t setTapAxesEnabled(dmp_tap_axis_t axis);
    esp_err_t setTapCount(uint8_t count);
    esp_err_t setTapTime(uint16_t time);
    esp_err_t setTapTimeMulti(uint16_t time);
    esp_err_t setShakeRejectThreshold(uint16_t thresh);
    esp_err_t setShakeRejectTime(uint16_t time);
    esp_err_t setShakeRejectTimeout(uint16_t timeout);
    uint16_t getTapThreshold(dmp_tap_axis_t axis);
    dmp_tap_axis_t getTapAxesEnabled();
    uint8_t getTapCount();
    uint16_t getTapTime();
    uint16_t getTapTimeMulti();
    uint16_t getShakeRejectThreshold();
    uint16_t getShakeRejectTime();
    uint16_t getShakeRejectTimeout();

    // ANDROID ORIENTATION

    // PEDOMETER
    esp_err_t setPedometerStepCount(uint32_t count);
    uint32_t getPedometerStepCount();
    esp_err_t setPedometerWalkTime(uint32_t time);
    uint32_t getPedometerWalkTime();
    
    // DATA
    esp_err_t getQuaternion(int32_t *quat, uint8_t *packet);
    esp_err_t getAccel(mpu_axis_t *axes, uint8_t *packet);
    esp_err_t getGyro(mpu_axis_t *axes, uint8_t *packet);
    esp_err_t getTap(uint8_t *direction, uint8_t *count, uint8_t *packet);
    esp_err_t getAndroidOrientation(uint8_t *orient, uint8_t *packet);
    
    
private:
    /** This pushes the DMP firmware into the MPU memory. */
    esp_err_t load();
    esp_err_t getPacketIndex(dmp_features_t feature, uint8_t *index);
    
}; /* end of DMP_t */

























#endif /* end of include guard: _DMP_H_ */