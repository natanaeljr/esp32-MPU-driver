#include "MPU.h"
#include "MPUdefine.h"
#include "MPUregisters.h"
#include "MPUtypes.h"
#include "DMPdefine.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_err.h"


static const char* MPU_TAG = "MPUcpp";

#include "MPUlog.h"




MPU_t::MPU_t(I2Cbus& I2C) : I2C(I2C) {
}


void MPU_t::setI2Cbus(I2Cbus& I2C) {
    this->I2C = I2C;
}


I2Cbus& MPU_t::getI2Cbus() {
    return I2C;
}


void MPU_t::setAddress(mpu_addr_t addr) {
    this->addr = addr;
}


mpu_addr_t MPU_t::getAddress() {
    return addr;
}


esp_err_t MPU_t::getLastError() {
    return err;
}


esp_err_t MPU_t::initialize() {
    // reset device and wait a little to clear all registers
    if(MPU_ERR_CHECK(reset()))
        return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // wake-up the device (boot-up default state is asleep)
    if(MPU_ERR_CHECK(sleep(false)))
        return err;
    // set clock source to gyro PLL which is better than internal clock
    if(MPU_ERR_CHECK(setClockSource(MPU_CLOCK_PLL)))
        return err;

    #ifdef CONFIG_MPU6500
    /* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
     * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
     */
    if(MPU_ERR_CHECK(writeBits(MPU6500_REG_ACCEL_CONFIG2, MPU6500_ACONFIG2_FIFO_SIZE_BIT, MPU6500_ACONFIG2_FIFO_SIZE_LENGTH, MPU6500_FIFO_SIZE_1K)))
        return err;
    #endif
    // set Full Scale range to most sensitive
    if(MPU_ERR_CHECK(setGyroFullScale(MPU_GYRO_FS_250DPS)))
        return err;
    if(MPU_ERR_CHECK(setAccelFullScale(MPU_ACCEL_FS_2G)))
        return err;
    // set Digital Low Pass Filter to get smoother data
    if(MPU_ERR_CHECK(setLowPassFilter(MPU_DLPF_42HZ)))
        return err;
    
    #ifdef AK89xx_SECONDARY
    if(MPU_ERR_CHECK(compassInit()))
        return err;
    #endif

    return ESP_OK;
}


esp_err_t MPU_t::reset() {
    return MPU_ERR_CHECK(writeBit(MPU_REG_PWR_MGMT1, MPU_PWR1_DEVICE_RESET_BIT, 1));
}


esp_err_t MPU_t::sleep(bool enable) {
    return MPU_ERR_CHECK(writeBit(MPU_REG_PWR_MGMT1, MPU_PWR1_SLEEP_BIT, enable));
}


bool MPU_t::getSleepStatus() {
    MPU_ERR_CHECK(readBit(MPU_REG_PWR_MGMT1, MPU_PWR1_SLEEP_BIT, buffer));
    return buffer[0];
}


bool MPU_t::testConnection() {
    return getDeviceID() == 0x34;
}


uint8_t MPU_t::getDeviceID() {
    MPU_ERR_CHECK(readBits(MPU_REG_WHO_AM_I, MPU_WHO_AM_I_BIT, MPU_WHO_AM_I_LENGTH, buffer));
    return buffer[0];
}


esp_err_t MPU_t::setLowPowerAccelMode(bool enable) {
    // first time the frequency will be set to minimum
    // turn on accel axis if stby
    // check if fifo is enabled
    if(MPU_ERR_CHECK(setTemperatureEnabled(enable)))
        return err;
    // enable accel cycle mode
    if(MPU_ERR_CHECK(writeBit(MPU_REG_PWR_MGMT1, MPU_PWR1_CYCLE_BIT, enable)))
        return err;
    // set gyro standby bits (accel keeps on)
    buffer[0] = enable ? (MPU_PWR2_STBY_XG_BIT | MPU_PWR2_STBY_YG_BIT | MPU_PWR2_STBY_ZG_BIT) : 0;
    return MPU_ERR_CHECK(writeBits(MPU_REG_PWR_MGMT2, MPU_PWR2_STBY_XA_BIT, 6, buffer[0]));
}


bool MPU_t::getLowPowerAccelMode() {
    MPU_ERR_CHECK(readBit(MPU_REG_PWR_MGMT1, MPU_PWR1_CYCLE_BIT, buffer));
    return buffer[0];
}


esp_err_t MPU_t::setLowPowerAccelRate(mpu_lp_accel_rate_t rate) {
    //@note: DLPF will be off (LPA bandwith: 1.1KHz)
    // check if LPAccel is off, error
    #ifdef CONFIG_MPU6050
    return MPU_ERR_CHECK(writeBits(MPU_REG_PWR_MGMT2, MPU_PWR2_LP_WAKE_CTRL_BIT, MPU_PWR2_LP_WAKE_CTRL_LENGTH, rate));
    #else // CONFIG_MPU6500
    if(MPU_ERR_CHECK(writeBits(MPU6500_REG_LP_ACCEL_ODR, MPU6500_LPA_ODR_CLKSEL_BIT, MPU6500_LPA_ODR_CLKSEL_LENGTH, rate)))
        return err;
    return MPU_ERR_CHECK(writeBit(MPU6500_REG_ACCEL_CONFIG2, MPU6500_ACONFIG2_ACCEL_FCHOICE_B_BIT, 1));
    #endif
}


mpu_lp_accel_rate_t MPU_t::getLowPowerAccelRate() {
    #ifdef CONFIG_MPU6050
    MPU_ERR_CHECK(readBits(MPU_REG_PWR_MGMT2, MPU_PWR2_LP_WAKE_CTRL_BIT, MPU_PWR2_LP_WAKE_CTRL_LENGTH, buffer));
    #elif CONFIG_MPU6500
    MPU_ERR_CHECK(readBits(MPU6500_REG_LP_ACCEL_ODR, MPU6500_LPA_ODR_CLKSEL_BIT, MPU6500_LPA_ODR_CLKSEL_LENGTH, buffer));
    #endif
    return (mpu_lp_accel_rate_t) buffer[0];
}


esp_err_t MPU_t::setClockSource(mpu_clock_src_t clockSrc) {
    return MPU_ERR_CHECK(writeBits(MPU_REG_PWR_MGMT1, MPU_PWR1_CLKSEL_BIT, MPU_PWR1_CLKSEL_LENGTH, clockSrc));
}


mpu_clock_src_t MPU_t::getClockSource() {
    MPU_ERR_CHECK(readBits(MPU_REG_PWR_MGMT1, MPU_PWR1_CLKSEL_BIT, MPU_PWR1_CLKSEL_LENGTH, buffer));
    return (mpu_clock_src_t) buffer[0];
}


esp_err_t MPU_t::setGyroFullScale(mpu_gyro_fsr_t fs) {
    return MPU_ERR_CHECK(writeBits(MPU_REG_GYRO_CONFIG, MPU_GCONFIG_FS_SEL_BIT, MPU_GCONFIG_FS_SEL_LENGTH, fs));
}


mpu_gyro_fsr_t MPU_t::getGyroFullScale() {
    MPU_ERR_CHECK(readBits(MPU_REG_GYRO_CONFIG, MPU_GCONFIG_FS_SEL_BIT, MPU_GCONFIG_FS_SEL_LENGTH, buffer));
    return (mpu_gyro_fsr_t) buffer[0];
}


esp_err_t MPU_t::setAccelFullScale(mpu_accel_fsr_t fs) {
    return MPU_ERR_CHECK(writeBits(MPU_REG_ACCEL_CONFIG, MPU_ACONFIG_FS_SEL_BIT, MPU_ACONFIG_FS_SEL_LENGTH, fs));
}


mpu_accel_fsr_t MPU_t::getAccelFullScale() {
    MPU_ERR_CHECK(readBits(MPU_REG_ACCEL_CONFIG, MPU_ACONFIG_FS_SEL_BIT, MPU_ACONFIG_FS_SEL_LENGTH, buffer));
    return (mpu_accel_fsr_t) buffer[0];
}


esp_err_t MPU_t::setLowPassFilter(mpu_dlpf_t dlpf) {
    // if all sensor are disabled, throw error
    if(MPU_ERR_CHECK(writeBits(MPU_REG_CONFIG, MPU_CONFIG_DLPF_CFG_BIT, MPU_CONFIG_DLPF_CFG_LENGTH, dlpf)))
        return err;
    
    #ifdef CONFIG_MPU6500
    //MPU6500 accel/gyro dlpf separately
    MPU_ERR_CHECK(writeBits(MPU6500_REG_ACCEL_CONFIG2, MPU6500_ACONFIG2_A_DLPF_CFG_BIT, MPU6500_ACONFIG2_A_DLPF_CFG_LENGTH, dlpf));
    #endif
    
    return err;
}


mpu_dlpf_t MPU_t::getLowPassFilter() {
    MPU_ERR_CHECK(readBits(MPU_REG_CONFIG, MPU_CONFIG_DLPF_CFG_BIT, MPU_CONFIG_DLPF_CFG_LENGTH, buffer));
    return (mpu_dlpf_t) buffer[0];
}


esp_err_t MPU_t::setSampleRate(uint16_t rate) {
    // if sensor are off, error
    // if dmp is on, error
    // adjust to get proper gyro output rate
    
    // rate: 4Hz ~ 1KHz
    if(rate < 4) rate = 4;
    else if(rate > 1000) rate = 1000;
    buffer[0] = 1000 / rate - 1;
    return MPU_ERR_CHECK(writeByte(MPU_REG_SMPLRT_DIV, buffer[0]));
}


uint16_t MPU_t::getSampleRate() {
    // adjust to get proper gyro output rate
    MPU_ERR_CHECK(readByte(MPU_REG_SMPLRT_DIV, buffer));
    uint16_t rate = 1000 / (1 + buffer[0]);
    return rate;
}


esp_err_t MPU_t::setFIFOEnabled(bool enable) {
    return MPU_ERR_CHECK(writeBit(MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_EN_BIT, 1));
}


bool MPU_t::getFIFOEnabled() {
    MPU_ERR_CHECK(readBit(MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_EN_BIT, buffer));
    return buffer[0];
}


esp_err_t MPU_t::setFIFOSensorsEnabled(mpu_fifo_sensors_t sensors) {
    // if dmp is on, error
    // if sensors asleep, error
    // if some sensor in mask is asleep, error
    // if FIFO already on, reset fifo
    if(sensors) {
        if(MPU_ERR_CHECK(writeByte(MPU_REG_FIFO_EN, (uint8_t)sensors)))
            return err;
        if (sensors & MPU_FIFO_SLAVE3)
            MPU_ERR_CHECK(writeBit(MPU_REG_I2C_MST_CTRL, MPU_I2C_MST_SLV_3_FIFO_EN_BIT, 1));
        return err;
    }
    else {
        if(MPU_ERR_CHECK(writeByte(MPU_REG_FIFO_EN, 0)))
            return err;
        return MPU_ERR_CHECK(writeBit(MPU_REG_I2C_MST_CTRL, MPU_I2C_MST_SLV_3_FIFO_EN_BIT, 0));
    }
}


mpu_fifo_sensors_t MPU_t::getFIFOSensorsEnabled() {
    MPU_ERR_CHECK(readBytes(MPU_REG_FIFO_EN, 2, buffer));
    mpu_fifo_sensors_t fifo_sen = ((buffer[1] << 8) & MPU_FIFO_SLAVE3) | buffer[0];
    return fifo_sen;
}


uint16_t MPU_t::getFIFOCount() {
    uint16_t count;
    MPU_ERR_CHECK(readBytes(MPU_REG_FIFO_COUNT_H, 2, buffer));
    count = (buffer[0] << 8) | buffer[1];
    return count;
}


esp_err_t MPU_t::resetFIFO() {
    return MPU_ERR_CHECK(writeBit(MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_RESET_BIT, true));
}


uint8_t MPU_t::readFIFOByte() {
    MPU_ERR_CHECK(readByte(MPU_REG_FIFO_R_W, buffer));
    return buffer[0];
}


esp_err_t MPU_t::readFIFO(uint8_t *data, size_t length) {
    if(length < 1) {
        err = ESP_ERR_INVALID_SIZE;
        MPU_LOGEMSG(MPU_MSG_INVALID_LENGTH);
        return err;
    }
    return MPU_ERR_CHECK(readBytes(MPU_REG_FIFO_R_W, length, data));
}


esp_err_t MPU_t::writeFIFOByte(uint8_t data) {
    return MPU_ERR_CHECK(writeByte(MPU_REG_FIFO_R_W, data));
}


esp_err_t MPU_t::setI2CBypass(bool enable) {
    return MPU_ERR_CHECK(writeBit(MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_EN_BIT, enable));
}


bool MPU_t::getI2CBypass() {
    MPU_ERR_CHECK(readBit(MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_EN_BIT, buffer));
    return buffer[0];
}


esp_err_t MPU_t::setIntConfig(mpu_int_config_t intConfig) {
    if(MPU_ERR_CHECK(readByte(MPU_REG_INT_PIN_CFG, buffer)))
        return err;
    buffer[0] &= 0x3; // clear the interesting bits, keep the others
    buffer[0] |= ((intConfig.level & 1) << MPU_INT_LEVEL_BIT)
               | ((intConfig.mode  & 1) << MPU_INT_LATCH_EN_BIT)
               | ((intConfig.drive & 1) << MPU_INT_OPEN_BIT)
               | ((intConfig.clear & 1) << MPU_INT_ANYRD_2CLEAR_BIT)
               | ((intConfig.fsyncLevel & 1) << MPU_INT_FSYNC_LEVEL_BIT);
    return MPU_ERR_CHECK(writeByte(MPU_REG_INT_PIN_CFG, buffer[0]));
}


mpu_int_config_t MPU_t::getIntConfig() {
    MPU_ERR_CHECK(readByte(MPU_REG_INT_PIN_CFG, buffer));
    mpu_int_config_t intConfig;
    intConfig.level = (mpu_int_lvl_t) (buffer[0] & (1 << MPU_INT_LEVEL_BIT));
    intConfig.mode = (mpu_int_mode_t) (buffer[0] & (1 << MPU_INT_LATCH_EN_BIT));
    intConfig.drive = (mpu_int_drive_t) (buffer[0] & (1 << MPU_INT_OPEN_BIT));
    intConfig.clear = (mpu_int_clear_t) (buffer[0] & (1 << MPU_INT_ANYRD_2CLEAR_BIT));
    intConfig.fsyncLevel = (mpu_int_lvl_t) (buffer[0] & (1 << MPU_INT_FSYNC_LEVEL_BIT));
    return intConfig;
}


esp_err_t MPU_t::setIntEnabled(mpu_int_t mask) {
    return MPU_ERR_CHECK(writeByte(MPU_REG_INT_ENABLE, mask));
}


mpu_int_t MPU_t::getIntEnabled() {
    MPU_ERR_CHECK(readByte(MPU_REG_INT_ENABLE, buffer));
    return (mpu_int_t) buffer[0];
}


mpu_int_t MPU_t::getIntStatus() {
    MPU_ERR_CHECK(readByte(MPU_REG_INT_STATUS, buffer));
    return (mpu_int_t) buffer[0];
}


bool MPU_t::getIntFIFOOverflowStatus() {
    MPU_ERR_CHECK(readBit(MPU_REG_INT_STATUS, MPU_INT_STATUS_FIFO_OFLOW_BIT, buffer));
    return buffer[0];
}


bool MPU_t::getIntDataReadyStatus() {
    MPU_ERR_CHECK(readBit(MPU_REG_INT_STATUS, MPU_INT_STATUS_RAW_DATA_RDY_BIT, buffer));
    return buffer[0];
}


mpu_axes_t MPU_t::getRotation() {
    mpu_axes_t gyro;
    MPU_ERR_CHECK(getRotation(&(gyro.x), &(gyro.y), &(gyro.z)));
    return gyro;
}


esp_err_t MPU_t::getRotation(int16_t *x, int16_t *y, int16_t *z) {
    MPU_ERR_CHECK(readBytes(MPU_REG_GYRO_XOUT_H, 6, buffer));
    *x = (buffer[0] << 8) | buffer[1];
    *y = (buffer[2] << 8) | buffer[3];
    *z = (buffer[4] << 8) | buffer[5];
    return err;
}


mpu_axes_t MPU_t::getAcceleration() {
    mpu_axes_t accel;
    MPU_ERR_CHECK(getAcceleration(&(accel.x), &(accel.y), &(accel.z)));
    return accel;
}


esp_err_t MPU_t::getAcceleration(int16_t *x, int16_t *y, int16_t *z) {
    MPU_ERR_CHECK(readBytes(MPU_REG_ACCEL_XOUT_H, 6, buffer));
    *x = (buffer[0] << 8) | buffer[1];
    *y = (buffer[2] << 8) | buffer[3];
    *z = (buffer[4] << 8) | buffer[5];
    return err;
}


esp_err_t MPU_t::getMotion6(mpu_axes_t *accel, mpu_axes_t *gyro) {
    return MPU_ERR_CHECK(getMotion6(&(accel->x), &(accel->y), &(accel->z), &(gyro->x), &(gyro->y), &(gyro->z)));
}


esp_err_t MPU_t::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
    MPU_ERR_CHECK(readBytes(MPU_REG_ACCEL_XOUT_H, 14, buffer));
    *ax = (buffer[0] << 8) | buffer[1];
    *ay = (buffer[2] << 8) | buffer[3];
    *az = (buffer[4] << 8) | buffer[5];
    *gx = (buffer[8] << 8) | buffer[9];
    *gy = (buffer[10] << 8) | buffer[11];
    *gz = (buffer[12] << 8) | buffer[13];
    return err;
}


esp_err_t MPU_t::setTemperatureEnabled(bool enable) {
    return MPU_ERR_CHECK(writeBit(MPU_REG_PWR_MGMT1, MPU_PWR1_TEMP_DIS_BIT, enable));
}


bool MPU_t::getTemperatureEnabled() {
    MPU_ERR_CHECK(readBit(MPU_REG_PWR_MGMT1, MPU_PWR1_TEMP_DIS_BIT, buffer));
    return buffer[0];
}


int16_t MPU_t::getTemperature() {
    int16_t temp;
    MPU_ERR_CHECK(readBytes(MPU_REG_TEMP_OUT_H, 2, buffer));
    temp = (buffer[0] << 8) | buffer[1];
    return temp;
}


int32_t MPU_t::getTemperatureC() {
    int32_t temp = getTemperature();
    #ifdef CONFIG_MPU6050
    int16_t temp_offset = -521;
    int16_t temp_sens = 340;
    #else //  CONFIG_MPU6500
    int16_t temp_offset = 0;
    int16_t temp_sens = 321;
    #endif    
    temp = (35 + ((temp - (float)temp_offset) / temp_sens)) * 65536L;
    return temp;
}


esp_err_t MPU_t::writeMemory(uint16_t memAddr, uint16_t length, const uint8_t *data) {
    // check bank boundaries
    if(((memAddr & 0xFF) + length) > DMP_BANK_SIZE) {
        err = ESP_ERR_INVALID_ARG;
        MPU_LOGEMSG(MPU_MSG_BANK_BOUNDARIES ", memAddr: 0x%x, length: %d, exceed: %d", memAddr, length, ((memAddr & 0xFF) + length - DMP_BANK_SIZE));
        return err;
    }
    buffer[0] = (memAddr >> 8); // bank index
    buffer[1] = memAddr & 0xFF; // memory start address
    if(MPU_ERR_CHECK(writeBytes(MPU_REG_BANK_SEL, 2, buffer)))
        return err;
    return MPU_ERR_CHECK(writeBytes(MPU_REG_MEM_R_W, length, data));
}


esp_err_t MPU_t::readMemory(uint16_t memAddr, uint16_t length, uint8_t *data) {
    // check bank boundaries
    if(((memAddr & 0xFF) + length) > DMP_BANK_SIZE) {
        err = ESP_ERR_INVALID_ARG;
        MPU_LOGEMSG(MPU_MSG_BANK_BOUNDARIES ", memAddr: 0x%x, length: %d, exceed: %d", memAddr, length, ((memAddr & 0xFF) + length - DMP_BANK_SIZE));
        return err;
    }
    buffer[0] = (memAddr >> 8); // bank index
    buffer[1] = memAddr & 0xFF; // memory start address
    if(MPU_ERR_CHECK(writeBytes(MPU_REG_BANK_SEL, 2, buffer)))
        return err;
    return MPU_ERR_CHECK(readBytes(MPU_REG_MEM_R_W, length, data));
}


esp_err_t MPU_t::setProgramStartAddress(uint16_t prgmAddr) {
    buffer[0] = (prgmAddr >> 8);
    buffer[1] = prgmAddr & 0xFF;
    return MPU_ERR_CHECK(writeBytes(MPU_REG_PRGM_START_H, 2, buffer));
}


uint16_t MPU_t::getProgramStartAddress() {
    MPU_ERR_CHECK(readBytes(MPU_REG_PRGM_START_H, 2, buffer));
    uint16_t prgmAddr = (buffer[0] << 8) | buffer[1];
    return prgmAddr;
}










