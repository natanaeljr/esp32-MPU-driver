#include "MPU.h"
#include "MPUconfig.h"
#include "MPUtypes.h"
#include "MPUregistermap.h"
#include "MPUdmp.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"



#if defined MPU_ERROR_LOGGER

#define MPU_CHECK_RET(x)                                                                        \
    do {                                                                                        \
        err = x;                                                                                \
        if(err) {                                                                               \
            ESP_LOGE("MPUclass","%s:%d (%s): error=%#x", __FILE__, __LINE__, __FUNCTION__, err);\
            return err;                                                                         \
        }                                                                                       \
    }while(0)
#define MPU_CHECK_NORET(x)                                                                      \
    do {                                                                                        \
        err = x;                                                                                \
        if(err) {                                                                               \
            ESP_LOGE("MPUclass","%s:%d (%s): error=%#x", __FILE__, __LINE__, __FUNCTION__, err);\
        }                                                                                       \
    }while(0)                

#else /* ! MPU_ERROR_LOGGER */

#define MPU_CHECK_RET(x)                                                                        \
    do {                                                                                        \
        err = x;                                                                                \
        if(err) return err;                                                                     \
    }while(0)
#define MPU_CHECK_NORET(x)                                                                      \
    do {                                                                                        \
        err = x;                                                                                \
    }while(0)   

#endif /* end of MPU_ERROR_LOGGER */





MPU_t::MPU_t(I2Cbus& I2C) : I2C(I2C) {
}


void MPU_t::setI2Cbus(I2Cbus& I2C) {
    this->I2C = I2C;
}


I2Cbus& MPU_t::getI2Cbus() {
    return I2C;
}


void MPU_t::setAddress(mpu_address_t addr) {
    this->addr = addr;
}


mpu_address_t MPU_t::getAddress() {
    return addr;
}


esp_err_t MPU_t::getLastError() {
    return err;
}


uint8_t MPU_t::readRegister(uint8_t reg) {
    MPU_CHECK_NORET(I2C.readByte(addr, reg, buffer));
    return buffer[0];
}


esp_err_t MPU_t::initialize(mpu_address_t addr) {
    // reset device and wait a little to clear all registers
    MPU_CHECK_RET(reset());
    vTaskDelay(50 / portTICK_PERIOD_MS);
    // wake-up the device (boot-up default state is asleep)
    MPU_CHECK_RET(sleep(false));
    // set clock source to gyro PLL which is better than internal clock
    MPU_CHECK_RET(setClockSource(MPU_CLOCK_PLL));

    #ifdef _MPU6500_
    /* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
     * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
     */
    MPU_CHECK_RET(I2C.writeBit(addr, MPU6500_REG_ACCEL_CONFIG2,
                                   MPU6500_ACONFIG2_FIFO_SIZE_BIT,
                                   MPU6500_ACONFIG2_FIFO_SIZE_LENGTH,
                                   MPU6500_FIFO_SIZE_1K));
    #endif
    // set Full Scale range to most sensitive
    MPU_CHECK_RET(setGyroFullScale(MPU_GYRO_FS_250DPS));
    MPU_CHECK_RET(setAccelFullScale(MPU_ACCEL_FS_2G));
    // set Digital Low Pass Filter to get smoother data
    MPU_CHECK_RET(setLowPassFilter(MPU_DLPF_42HZ));
    
    #ifdef AK89xx_SECONDARY
    MPU_CHECK_RET(compassInit());
    #endif

    return ESP_OK;
}


esp_err_t MPU_t::reset() {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_DEVICE_RESET_BIT, 1));
    return err;
}


esp_err_t MPU_t::sleep(bool enable) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_SLEEP_BIT, enable));
    return err;
}


bool MPU_t::getSleepStatus() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_SLEEP_BIT, buffer));
    return buffer[0];
}


bool MPU_t::testConnection() {
    return getDeviceID() == 0x34;
}


uint8_t MPU_t::getDeviceID() {
    MPU_CHECK_NORET(I2C.readBits(addr, MPU_REG_WHO_AM_I, MPU_WHO_AM_I_BIT, MPU_WHO_AM_I_LENGTH, buffer));
    return buffer[0];
}


esp_err_t MPU_t::setLowPowerAccelMode(bool enable) {
    // first time the frequency will be set to minimum
    // turn on accel axis if stby
    // check if fifo is enabled
    MPU_CHECK_RET(setTemperatureEnabled(enable));
    // enable accel cycle mode
    MPU_CHECK_RET(I2C.writeBit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_CYCLE_BIT, enable));
    // set gyro standby bits (accel keeps on)
    buffer[0] = enable ? (MPU_PWR2_STBY_XG_BIT | MPU_PWR2_STBY_YG_BIT | MPU_PWR2_STBY_ZG_BIT) : 0;
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_PWR_MGMT2, MPU_PWR2_STBY_XA_BIT, 6, buffer[0]));
    return err;
}


bool MPU_t::getLowPowerAccelMode() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_CYCLE_BIT, buffer));
    return buffer[0];
}


esp_err_t MPU_t::setLowPowerAccelRate(mpu_lp_accel_rate_t rate) {
    //@note: DLPF will be off (LPA bandwith: 1.1KHz)
    // check if LPAccel is off, error
    #ifdef _MPU6050_
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_PWR_MGMT2, MPU_PWR2_LP_WAKE_CTRL_BIT, MPU_PWR2_LP_WAKE_CTRL_LENGTH, rate));
    #else // _MPU6500_
    MPU_CHECK_RET(I2C.writeBit(addr, MPU6500_REG_LP_ACCEL_ODR, MPU6500_LPA_ODR_CLKSEL_BIT, MPU6500_LPA_ODR_CLKSEL_LENGTH, rate));
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU6500_REG_ACCEL_CONFIG2, MPU6500_ACONFIG2_ACCEL_FCHOICE_B_BIT, 1));
    #endif
    return err;
}


mpu_lp_accel_rate_t MPU_t::getLowPowerAccelRate() {
    #ifdef _MPU6050_
    MPU_CHECK_NORET(I2C.readBits(addr, MPU_REG_PWR_MGMT2, MPU_PWR2_LP_WAKE_CTRL_BIT, MPU_PWR2_LP_WAKE_CTRL_LENGTH, buffer));
    #else // _MPU6500_
    MPU_CHECK_NORET(I2C.readBits(addr, MPU6500_REG_LP_ACCEL_ODR, MPU6500_LPA_ODR_CLKSEL_BIT, MPU6500_LPA_ODR_CLKSEL_LENGTH, buffer));
    #endif
    return (mpu_lp_accel_rate_t) buffer[0];
}


esp_err_t MPU_t::setClockSource(mpu_clock_src_t clockSrc) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_CLKSEL_BIT, MPU_PWR1_CLKSEL_LENGTH, clockSrc));
    return err;
}


mpu_clock_src_t MPU_t::getClockSource() {
    MPU_CHECK_NORET(I2C.readBits(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_CLKSEL_BIT, MPU_PWR1_CLKSEL_LENGTH, buffer));
    return (mpu_clock_src_t) buffer[0];
}


esp_err_t MPU_t::setGyroFullScale(mpu_gyro_fsr_t fs) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_GYRO_CONFIG, MPU_GCONFIG_FS_SEL_BIT, MPU_GCONFIG_FS_SEL_LENGTH, fs));
    return err;
}


mpu_gyro_fsr_t MPU_t::getGyroFullScale() {
    MPU_CHECK_NORET(I2C.readBits(addr, MPU_REG_GYRO_CONFIG, MPU_GCONFIG_FS_SEL_BIT, MPU_GCONFIG_FS_SEL_LENGTH, buffer));
    return (mpu_gyro_fsr_t) buffer[0];
}


esp_err_t MPU_t::setAccelFullScale(mpu_accel_fsr_t fs) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_ACCEL_CONFIG, MPU_ACONFIG_FS_SEL_BIT, MPU_ACONFIG_FS_SEL_LENGTH, fs));
    return err;
}


mpu_accel_fsr_t MPU_t::getAccelFullScale() {
    MPU_CHECK_NORET(I2C.readBits(addr, MPU_REG_ACCEL_CONFIG, MPU_ACONFIG_FS_SEL_BIT, MPU_ACONFIG_FS_SEL_LENGTH, buffer));
    return (mpu_accel_fsr_t) buffer[0];
}


esp_err_t MPU_t::setLowPassFilter(mpu_dlpf_t dlpf) {
    // if all sensor are disabled, throw error
    MPU_CHECK_RET(I2C.writeBit(addr, MPU_REG_CONFIG, MPU_CONFIG_DLPF_CFG_BIT, MPU_CONFIG_DLPF_CFG_LENGTH, dlpf));
    
    #ifdef _MPU6500_
    //MPU6500 accel/gyro dlpf separately
    MPU_CHECK_RET(I2C.writeBit(addr, MPU6500_REG_ACCEL_CONFIG2, MPU6500_ACONFIG2_A_DLPF_CFG_BIT, MPU6500_ACONFIG2_A_DLPF_CFG_LENGTH, dlpf));
    #endif
    
    return ESP_OK;
}


mpu_dlpf_t MPU_t::getLowPassFilter() {
    MPU_CHECK_NORET(I2C.readBits(addr, MPU_REG_CONFIG, MPU_CONFIG_DLPF_CFG_BIT, MPU_CONFIG_DLPF_CFG_LENGTH, buffer));
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
    MPU_CHECK_NORET(I2C.writeByte(addr, MPU_REG_SMPLRT_DIV, buffer[0]));
    return err;
}


uint16_t MPU_t::getSampleRate() {
    // adjust to get proper gyro output rate
    MPU_CHECK_NORET(I2C.readByte(addr, MPU_REG_SMPLRT_DIV, buffer));
    uint16_t rate = 1000 / (1 + buffer[0]);
    return rate;
}


esp_err_t MPU_t::setFIFOEnabled(bool enable) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_EN_BIT, 1));
    return err;
}


bool MPU_t::getFIFOEnabled() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_EN_BIT, buffer));
    return buffer[0];
}


esp_err_t MPU_t::setFIFOSensorsEnabled(mpu_fifo_sensors_t sensors) {
    // if dmp is on, error
    // if sensors asleep, error
    // if some sensor in mask is asleep, error
    // if FIFO already on, reset fifo
    if(!sensors) {
        MPU_CHECK_RET(I2C.writeByte(addr, MPU_REG_FIFO_EN, (uint8_t)sensors));
        if (sensors & MPU_FIFO_SLAVE3)
            MPU_CHECK_RET(I2C.writeBit(addr, MPU_REG_I2C_MST_CTRL, MPU_I2C_MST_SLV_3_FIFO_EN_BIT, 1));
    }
    else {
        MPU_CHECK_RET(I2C.writeByte(addr, MPU_REG_FIFO_EN, 0));
        MPU_CHECK_RET(I2C.writeBit(addr, MPU_REG_I2C_MST_CTRL, MPU_I2C_MST_SLV_3_FIFO_EN_BIT, 0));
    }
    return ESP_OK;
}


mpu_fifo_sensors_t MPU_t::getFIFOSensorsEnabled() {
    MPU_CHECK_NORET(I2C.readBytes(addr, MPU_REG_FIFO_EN, 2, buffer));
    mpu_fifo_sensors_t fifo_sen = ((buffer[1] << 8) & MPU_FIFO_SLAVE3) | buffer[0];
    return fifo_sen;
}


uint16_t MPU_t::getFIFOCount() {
    uint16_t count;
    MPU_CHECK_NORET(I2C.readBytes(addr, MPU_REG_FIFO_COUNT_H, 2, buffer));
    count = (buffer[0] << 8) | buffer[1];
    return count;
}


esp_err_t MPU_t::resetFIFO() {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_RESET_BIT, true));
    return err;
}


uint8_t MPU_t::readFIFOByte() {
    MPU_CHECK_NORET(I2C.readByte(addr, MPU_REG_FIFO_R_W, buffer));
    return buffer[0];
}


esp_err_t MPU_t::readFIFO(uint8_t *data, uint16_t length) {
    *data = 0;
    if(length > 0)
        MPU_CHECK_RET(I2C.readBytes(addr, MPU_REG_FIFO_R_W, length, data));
    return ESP_OK;
}


esp_err_t MPU_t::writeFIFOByte(uint8_t data) {
    MPU_CHECK_NORET(I2C.writeByte(addr, MPU_REG_FIFO_R_W, data));
    return err;
}


esp_err_t MPU_t::setI2CBypass(bool enable) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_EN_BIT, enable));
    return err;
}


bool MPU_t::getI2CBypass() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_EN_BIT, buffer));
    return buffer[0];
}


esp_err_t MPU_t::setIntLevel(mpu_int_lvl_t level) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_INT_PIN_CFG, MPU_INT_LEVEL_BIT, level));
    return err;
}


mpu_int_lvl_t MPU_t::getIntLevel() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_INT_PIN_CFG, MPU_INT_LEVEL_BIT, buffer));
    return (mpu_int_lvl_t) buffer[0];
}


esp_err_t MPU_t::setIntMode(mpu_int_mode_t mode) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_INT_PIN_CFG, MPU_INT_LATCH_EN_BIT, mode));
    return err;
}


mpu_int_mode_t MPU_t::getIntMode() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_INT_PIN_CFG, MPU_INT_LATCH_EN_BIT, buffer));
    return (mpu_int_mode_t) buffer[0];
}


esp_err_t MPU_t::setIntDrive(mpu_int_drive_t drive) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_INT_PIN_CFG, MPU_INT_OPEN_BIT, drive));
    return err;
}


mpu_int_drive_t MPU_t::getIntDrive() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_INT_PIN_CFG, MPU_INT_OPEN_BIT, buffer));
    return (mpu_int_drive_t) buffer[0];
}


esp_err_t MPU_t::setIntClear(mpu_int_clear_t clear) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_INT_PIN_CFG, MPU_INT_ANYRD_2CLEAR_BIT, clear));
    return err;
}


mpu_int_clear_t MPU_t::getIntClear() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_INT_PIN_CFG, MPU_INT_ANYRD_2CLEAR_BIT, buffer));
    return (mpu_int_clear_t) buffer[0];
}


esp_err_t MPU_t::setIntEnabled(mpu_int_t mask) {
    MPU_CHECK_NORET(I2C.writeByte(addr, MPU_REG_INT_ENABLE, mask));
    return err;
}


mpu_int_t MPU_t::getIntEnabled() {
    MPU_CHECK_NORET(I2C.readByte(addr, MPU_REG_INT_ENABLE, buffer));
    return (mpu_int_t) buffer[0];
}


uint8_t MPU_t::getIntStatus() {
    MPU_CHECK_NORET(I2C.readByte(addr, MPU_REG_INT_STATUS, buffer));
    return buffer[0];
}


bool MPU_t::getIntFIFOOverflowStatus() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_INT_STATUS, MPU_INT_STATUS_FIFO_OFLOW_BIT, buffer));
    return buffer[0];
}


bool MPU_t::getIntDataReadyStatus() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_INT_STATUS, MPU_INT_STATUS_RAW_DATA_RDY_BIT, buffer));
    return buffer[0];
}


mpu_axis_t MPU_t::getGyro() {
    mpu_axis_t gyro;
    getGyro(&(gyro.x), &(gyro.y), &(gyro.z));
    return gyro;
}


void MPU_t::getGyro(int16_t *x, int16_t *y, int16_t *z) {
    MPU_CHECK_NORET(I2C.readBytes(addr, MPU_REG_GYRO_XOUT_H, 6, buffer));
    *x = (buffer[0] << 8) | buffer[1];
    *y = (buffer[2] << 8) | buffer[3];
    *z = (buffer[4] << 8) | buffer[5];
}


mpu_axis_t MPU_t::getAccel() {
    mpu_axis_t accel;
    getAccel(&(accel.x), &(accel.y), &(accel.z));
    return accel;
}


esp_err_t MPU_t::getAccel(int16_t *x, int16_t *y, int16_t *z) {
    MPU_CHECK_RET(I2C.readBytes(addr, MPU_REG_ACCEL_XOUT_H, 6, buffer));
    *x = (buffer[0] << 8) | buffer[1];
    *y = (buffer[2] << 8) | buffer[3];
    *z = (buffer[4] << 8) | buffer[5];
    return ESP_OK;
}


esp_err_t MPU_t::getMotion6(mpu_axis_t *accel, mpu_axis_t *gyro) {
    *accel = getAccel();
    if(!err)
        *gyro = getGyro();
    return err;
}


esp_err_t MPU_t::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
    getAccel(ax, ay, az);
    if(!err)
        getGyro(gx, gy, gz);
    return err;
}


esp_err_t MPU_t::setTemperatureEnabled(bool enable) {
    MPU_CHECK_NORET(I2C.writeBit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_TEMP_DIS_BIT, enable));
    return err;
}


bool MPU_t::getTemperatureEnabled() {
    MPU_CHECK_NORET(I2C.readBit(addr, MPU_REG_PWR_MGMT1, MPU_PWR1_TEMP_DIS_BIT, buffer));
    return buffer[0];
}


int16_t MPU_t::getTemperature() {
    int16_t temp;
    MPU_CHECK_NORET(I2C.readBytes(addr, MPU_REG_TEMP_OUT_H, 2, buffer));
    temp = (buffer[0] << 8) | buffer[1];
    return temp;
}


int32_t MPU_t::getTemperatureCelsius() {
    int32_t temp = getTemperature();
    #ifdef _MPU6050_
    int16_t temp_offset = -521;
    int16_t temp_sens = 340;
    #else //  _MPU6500_
    int16_t temp_offset = 0;
    int16_t temp_sens = 321;
    #endif    
    temp = (35 + ((temp - (float)temp_offset) / temp_sens)) * 65536L;
    return temp;
}


esp_err_t MPU_t::setMemoryAddress(uint16_t memAddr) {
    buffer[0] = (memAddr >> 8) & 0xFF; // bank index
    buffer[1] = memAddr & 0xFF; // memory start address
    MPU_CHECK_NORET(I2C.writeBytes(addr, MPU_REG_BANK_SEL, 2, buffer));
    return err;
}


uint16_t MPU_t::getMemoryAddress() {
    MPU_CHECK_NORET(I2C.readBytes(addr, MPU_REG_BANK_SEL, 2, buffer));
    uint16_t memAddr = (buffer[0] << 8) | buffer[1];
    return memAddr;
}


esp_err_t MPU_t::writeMemory(uint16_t memAddr, uint16_t length, uint8_t *data) {
    // check bank boundaries
    // TODO: log proper error
    MPU_CHECK_RET(((memAddr & 0xFF) + length > DMP_BANK_SIZE) ? ESP_ERR_INVALID_SIZE : ESP_OK);
    setMemoryAddress(memAddr);
    MPU_CHECK_NORET(I2C.writeBytes(addr, MPU_REG_MEM_R_W, length, data));
    return err;
}


esp_err_t MPU_t::readMemory(uint16_t memAddr, uint16_t length, uint8_t *data) {
    MPU_CHECK_RET(((memAddr & 0xFF) + length > DMP_BANK_SIZE) ? ESP_ERR_INVALID_SIZE : ESP_OK);
    setMemoryAddress(memAddr);
    MPU_CHECK_NORET(I2C.readBytes(addr, MPU_REG_MEM_R_W, length, data));
    return err;
}


esp_err_t MPU_t::setProgramStartAddress(uint16_t prgmAddr) {
    buffer[0] = (prgmAddr >> 8) & 0xFF;
    buffer[1] = prgmAddr & 0xFF;
    MPU_CHECK_NORET(I2C.writeBytes(addr, MPU_REG_PRGM_START_H, 2, buffer));
    return err;
}


uint16_t MPU_t::getProgramStartAddress() {
    MPU_CHECK_NORET(I2C.readBytes(addr, MPU_REG_PRGM_START_H, 2, buffer));
    uint16_t prgmAddr = (buffer[0] << 8) | buffer[1];
    return prgmAddr;
}










