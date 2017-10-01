#ifndef _MPU_H_
#define _MPU_H_

#include "MPUdefine.h"
#include "MPUregisters.h"
#include "MPUtypes.h"
#include <stdint.h>
#include "I2Cbus.h"
#include "esp_err.h"


const mpu_addr_t MPU_DEFAULT_ADDRESS = MPU_ADDRESS_AD0_LOW;

/*******************************************************************************
* MPU class
******************************************************************************/
// TODO: rewrite mask methods with structs
// TODO: change some set.. methods to enable..
// TODO: override operator[] for axes structs and other suitable types and fits
class MPU_t {

private:
    I2Cbus& I2C;
    mpu_addr_t addr = MPU_DEFAULT_ADDRESS;
    esp_err_t err = ESP_OK;
    uint8_t buffer[14];
    
public:
    MPU_t(I2Cbus& I2C = I2Cbus0);
    void setI2Cbus(I2Cbus& I2C);
    I2Cbus& getI2Cbus();
    void setAddress(mpu_addr_t addr);
    mpu_addr_t getAddress();
    esp_err_t getLastError();
    uint8_t readRegister(uint8_t reg);

    // SETUP
    esp_err_t initialize();
    esp_err_t reset();
    esp_err_t sleep(bool enable);
    bool getSleepStatus();
    bool testConnection();
    uint8_t getDeviceID(); // 0x34

    // CONFIGS
    esp_err_t setClockSource(mpu_clock_src_t clockSrc);
    esp_err_t setLowPassFilter(mpu_dlpf_t dlpf);
    esp_err_t setSampleRate(uint16_t rate);
    esp_err_t setLowPowerAccelMode(bool enable);
    esp_err_t setLowPowerAccelRate(mpu_lp_accel_rate_t rate);
    mpu_clock_src_t getClockSource();
    mpu_dlpf_t getLowPassFilter();
    uint16_t getSampleRate();
    bool getLowPowerAccelMode();
    mpu_lp_accel_rate_t getLowPowerAccelRate();

    // I2C
    esp_err_t setI2CBypass(bool enable);
    bool getI2CBypass();
    
    // GYROSCOPE
    esp_err_t setGyroFullScale(mpu_gyro_fsr_t fs);
    mpu_gyro_fsr_t getGyroFullScale();
    mpu_axis_t getGyro();
    esp_err_t getGyro(int16_t *x, int16_t *y, int16_t *z);
    
    // ACCELEROMETER
    esp_err_t setAccelFullScale(mpu_accel_fsr_t fs);
    mpu_accel_fsr_t getAccelFullScale();
    mpu_axis_t getAccel();
    esp_err_t getAccel(int16_t *x, int16_t *y, int16_t *z);

    // MOTION
    esp_err_t getMotion6(mpu_axis_t *accel, mpu_axis_t *gyro);
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
    esp_err_t readFIFO(uint8_t *data, uint16_t length);
    uint16_t getFIFOCount();
    esp_err_t resetFIFO(); // TODO: change to clear FIFO
    
    // INTERRUPT
    esp_err_t setIntLevel(mpu_int_lvl_t level);
    esp_err_t setIntMode(mpu_int_mode_t mode);
    esp_err_t setIntDrive(mpu_int_drive_t drive);
    esp_err_t setIntClear(mpu_int_clear_t clear);
    esp_err_t setIntEnabled(mpu_int_t mask);
    mpu_int_lvl_t getIntLevel();
    mpu_int_mode_t getIntMode();
    mpu_int_drive_t getIntDrive();
    mpu_int_clear_t getIntClear();
    mpu_int_t getIntEnabled();
    uint8_t getIntStatus();
    bool getIntFIFOOverflowStatus();
    bool getIntDataReadyStatus();

    // COMPASS


private:
    // private: MEMORY
    esp_err_t setMemoryAddress(uint16_t memAddr);
    uint16_t getMemoryAddress();
    esp_err_t writeMemory(uint16_t memAddr, uint16_t length, uint8_t *data);
    esp_err_t readMemory(uint16_t memAddr, uint16_t length, uint8_t *data);
    esp_err_t setProgramStartAddress(uint16_t prgmAddr);
    uint16_t getProgramStartAddress();

    // private: COMPASS
#ifdef AK89xx_SECONDARY
    esp_err_t compassInit();
    esp_err_t setCompassSampleRate(uint16_t rate);
#endif


    /*************** DMP (Digital Motion Processor) interface *******************/
    class DMP_t {
    private: 
        MPU_t& MPU;
        uint8_t* buffer = MPU.buffer;
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
    /* class DMP_t is defined in 'MPUdmp.cpp' */
    
    
public:
    DMP_t dmp = DMP_t(*this);
    
}; /* end of MPU_t */






#endif /* end of include guard: _MPU_H_ */
