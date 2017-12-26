/* =========================================================================
This library is placed under the MIT License
Copyright 2017 Natanael Josue Rabello. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
 ========================================================================= */

#include "MPU.hpp"
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "mpu/registers.hpp"
#include "mpu/types.hpp"
#include "mpu/math.hpp"

static const char* TAG = CONFIG_MPU_CHIP_MODEL;
#include "mpu/log.hpp"


/* ^^^^^^^^^^^^^^^^^^^^^^
 * Embedded Motion Driver
 * ^^^^^^^^^^^^^^^^^^^^^^ */
namespace emd {

/* ^^^^^^^^^^^^^^^^^^^^^
 * Motion Processor Unit
 * ^^^^^^^^^^^^^^^^^^^^^ */
namespace mpu {

/** mpu::MPU class implementation */


/**
 * Initialization of MPU device. Initial configuration:
 *  - Gyro FSR: 500DPS
 *  - Accel FSR: 4G
 *  - Clock source: gyro PLL
 *  - DLPF: 42Hz
 *  - Sample rate: 100Hz
 *  - INT pin: disabled
 *  - FIFO: disabled
 *  for MPU9150 and MPU9250:
 *  - Aux I2C master: enabled at 400KHz
 *  - Compass: enabled (using Slave 0 and Slave 1)
 * 
 * Note: a soft reset is performed which takes 100-200ms
 * Note: when using SPI, the primary I2C slave module is disabled right away.
 * */
esp_err_t MPU::initialize() {
    // reset device (wait a little to clear all registers)
    if (MPU_ERR_CHECK(reset()))
        return err;
    // wake-up the device (power on-reset state is asleep for some models)
    if (MPU_ERR_CHECK(setSleep(false)))
        return err;
    // disable MPU's I2C slave module when using SPI
    #ifdef CONFIG_MPU_SPI
    if (MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_I2C_IF_DIS_BIT, 1)))
        return err;
    #endif
    // set clock source to gyro PLL which is better than internal clock
    if (MPU_ERR_CHECK(setClockSource(CLOCK_PLL)))
        return err;

    #ifdef CONFIG_MPU6500
    /* MPU6500 / MPU9250 share 4kB of memory between the DMP and the FIFO. Since the
     * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
     */
    if (MPU_ERR_CHECK(writeBits(regs::ACCEL_CONFIG2, regs::ACONFIG2_FIFO_SIZE_BIT, regs::ACONFIG2_FIFO_SIZE_LENGTH, FIFO_SIZE_1K)))
        return err;
    #endif

    // set Full Scale range
    if (MPU_ERR_CHECK(setGyroFullScale(GYRO_FS_500DPS)))
        return err;
    if (MPU_ERR_CHECK(setAccelFullScale(ACCEL_FS_4G)))
        return err;
    // set Digital Low Pass Filter to get smoother data
    if (MPU_ERR_CHECK(setDigitalLowPassFilter(DLPF_42HZ)))
        return err;

    // setup magnetometer
    #ifdef CONFIG_MPU_AK89xx
    if (MPU_ERR_CHECK(compassInit()))
        return err;
    #ifdef CONFIG_MPU_AK8963
    if (MPU_ERR_CHECK(compassSetSensitivity(MAG_SENSITIVITY_0_15_uT)))
        return err;
    #endif
    #endif

    // set sample rate to 100Hz
    if (MPU_ERR_CHECK(setSampleRate(100)))
        return err;
    MPU_LOGI("Initialization complete");
    return err;
}


/**
 * Reset the internal registers and restores the default settings.
 * Note: this function delays 100ms when using I2C
 * Note: this function delays 200ms when using SPI
 * */
esp_err_t MPU::reset() {
    if (MPU_ERR_CHECK(writeBit(regs::PWR_MGMT1, regs::PWR1_DEVICE_RESET_BIT, 1)))
        return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    #ifdef CONFIG_MPU_SPI
    if (MPU_ERR_CHECK(resetSignalPath()))
        return err;
    #endif
    MPU_LOGI("Reset!");
    return err;
}


/**
 * Enable / disable Sleep
 * */
esp_err_t MPU::setSleep(bool enable) {
    return MPU_ERR_CHECK(writeBit(regs::PWR_MGMT1, regs::PWR1_SLEEP_BIT, enable));
}

bool MPU::getSleep() {
    MPU_ERR_CHECK(readBit(regs::PWR_MGMT1, regs::PWR1_SLEEP_BIT, buffer));
    return buffer[0];
}


/**
 * Test connection with MPU by reading WHO_AM_IM register and 
 * check its value according to the chip model.
 * */
esp_err_t MPU::testConnection() {
    const uint8_t wai = whoAmI();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    #if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    return (wai == 0x68) ? ESP_OK : ESP_ERR_NOT_FOUND;
    #elif defined CONFIG_MPU9255
    return (wai == 0x73) ? ESP_OK : ESP_ERR_NOT_FOUND;
    #elif defined CONFIG_MPU9250
    return (wai == 0x71) ? ESP_OK : ESP_ERR_NOT_FOUND;
    #elif defined CONFIG_MPU6555
    return (wai == 0x7C) ? ESP_OK : ESP_ERR_NOT_FOUND;
    #elif defined CONFIG_MPU6500
    return (wai == 0x70) ? ESP_OK : ESP_ERR_NOT_FOUND;
    #endif
}

uint8_t MPU::whoAmI() {
    MPU_ERR_CHECK(readByte(regs::WHO_AM_I, buffer));
    return buffer[0];
}

/**
 * Sample Rate = Internal Output Rate / (1 + SMPLRT_DIV)
 * @param   rate: 4Hz ~ 1KHz
 * For sample rate of 8KHz: set digital low pass filter to DLPF_256HZ_NOLPF
 * For sample rate of 32KHZ [MPU6500 / MPU9250 only]: set Fchoice to FCHOICE_0
 * 
 * Note: for MPU9150 & MPU9250: when using compass,
 *  this function alters Aux I2C Master sample_delay property
 *  to adjust the compass sample rate. (also, wait_for_es property to adjust interrupt).
 *  If sample rate lesser than 100 Hz, data-ready interrupt will wait for compass data.
 *  If sample rate greater than 100 Hz, data-ready interrupt will not be delayed by the compass.
 * */
esp_err_t MPU::setSampleRate(uint16_t rate) {
    // Check value range
    if (rate < 4) {
        MPU_LOGWMSG(msgs::INVALID_SAMPLE_RATE, " %d, minimum rate is 4", rate);
        rate = 4;
    } else if (rate > 1000) {
        MPU_LOGWMSG(msgs::INVALID_SAMPLE_RATE, " %d, maximum rate is 1000", rate);
        rate = 1000;
    }

    #if CONFIG_MPU_LOG_LEVEL >= ESP_LOG_WARN
    // Check selected Fchoice [MPU6500 and MPU9250 only]
    #ifdef CONFIG_MPU6500
    fchoice_t fchoice = getFchoice();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    if (fchoice != FCHOICE_3) {
        MPU_LOGWMSG(msgs::INVALID_STATE, ", sample rate divider is not effective when Fchoice != 3");
    }
    #endif
    // Check dlpf configuration
    dlpf_t dlpf = getDigitalLowPassFilter();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    if (dlpf == 0 || dlpf == 7)
        MPU_LOGWMSG(msgs::INVALID_STATE, ", sample rate divider is not effective when DLPF is (0 or 7)");
    #endif

    constexpr uint16_t internalSampleRate = 1000;
    uint16_t divider = internalSampleRate / rate - 1;
    // Check for rate match
    uint16_t finalRate = (internalSampleRate / (1 + divider));
    if (finalRate != rate) {
        MPU_LOGW("Sample rate constrained to %d Hz", finalRate);
    } else {
        MPU_LOGI("Sample rate set to %d Hz", finalRate);
    }
    // Write divider to register
    if (MPU_ERR_CHECK(writeByte(regs::SMPLRT_DIV, (uint8_t)divider)))
        return err;

    // check and set compass sample rate
    #ifdef CONFIG_MPU_AK89xx
    const auxi2c_slv_config_t magSlaveChgModeConf = getAuxI2CSlaveConfig(MAG_SLAVE_CHG_MODE);
    if (MPU_ERR_CHECK(lastError()))
        return err;
    const bool magSlaveChgModeEnabled = getAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE);
    if (magSlaveChgModeEnabled && magSlaveChgModeConf.addr == COMPASS_I2CADDRESS &&
            (magSlaveChgModeConf.txdata & 0xF) == MAG_MODE_SINGLE_MEASURE) {
        auxi2c_config_t auxi2cConf = getAuxI2CConfig();
        if (MPU_ERR_CHECK(lastError()))
            return err;
        if (rate <= COMPASS_SAMPLE_RATE_MAX) {
            auxi2cConf.wait_for_es = 1;
            auxi2cConf.sample_delay = 0;
        } else {
            auxi2cConf.wait_for_es = 0;
            auxi2cConf.sample_delay = ceil(static_cast<float>(finalRate) / COMPASS_SAMPLE_RATE_MAX) - 1;
            const uint8_t compassRate = finalRate / (auxi2cConf.sample_delay + 1);
            MPU_LOGW("Compass sample rate constrained to %d, magnetometer's maximum is %d Hz",
                compassRate, COMPASS_SAMPLE_RATE_MAX);
        }
        if (MPU_ERR_CHECK(setAuxI2CConfig(auxi2cConf)))
            return err;
    }
    #endif

    return err;
}


uint16_t MPU::getSampleRate() {
    #ifdef CONFIG_MPU6500
    fchoice_t fchoice = getFchoice();
    MPU_ERR_CHECK(lastError());
    if (fchoice != FCHOICE_3)
        return SAMPLE_RATE_MAX;
    #endif

    constexpr uint16_t sampleRateMax_nolpf = 8000;
    dlpf_t dlpf = getDigitalLowPassFilter();
    MPU_ERR_CHECK(lastError());
    if (dlpf == 0 || dlpf == 7)
        return sampleRateMax_nolpf;

    constexpr uint16_t internalSampleRate = 1000;
    MPU_ERR_CHECK(readByte(regs::SMPLRT_DIV, buffer));
    uint16_t rate = internalSampleRate / (1 + buffer[0]);
    return rate;
}


esp_err_t MPU::setClockSource(clock_src_t clockSrc) {
    return MPU_ERR_CHECK(writeBits(regs::PWR_MGMT1, regs::PWR1_CLKSEL_BIT, regs::PWR1_CLKSEL_LENGTH, clockSrc));
}


clock_src_t MPU::getClockSource() {
    MPU_ERR_CHECK(readBits(regs::PWR_MGMT1, regs::PWR1_CLKSEL_BIT, regs::PWR1_CLKSEL_LENGTH, buffer));
    return (clock_src_t) buffer[0];
}


esp_err_t MPU::setDigitalLowPassFilter(dlpf_t dlpf) {
    if (MPU_ERR_CHECK(writeBits(regs::CONFIG, regs::CONFIG_DLPF_CFG_BIT, regs::CONFIG_DLPF_CFG_LENGTH, dlpf)))
        return err;
    #ifdef CONFIG_MPU6500
    MPU_ERR_CHECK(writeBits(regs::ACCEL_CONFIG2, regs::ACONFIG2_A_DLPF_CFG_BIT, regs::ACONFIG2_A_DLPF_CFG_LENGTH, dlpf));
    #endif
    return err;
}

dlpf_t MPU::getDigitalLowPassFilter() {
    MPU_ERR_CHECK(readBits(regs::CONFIG, regs::CONFIG_DLPF_CFG_BIT, regs::CONFIG_DLPF_CFG_LENGTH, buffer));
    return (dlpf_t) buffer[0];
}


/**
 * Reset all gyro digital signal path, accel digital signal path, and temp
 * digital signal path. This also clears all the sensor registers.
 * Note: this function delays 100 ms, needed for reset to complete
 * */
esp_err_t MPU::resetSignalPath() {
    if (MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_SIG_COND_RESET_BIT, 1)))
        return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return err;
}


/**
 * @brief      Enter low-power accel-only mode.
 * In low-power accel mode, the chip goes to sleep and only wakes up to sample
 * the accelerometer at a certain frequency:
 * @see setLowPowerAccelRate() to set the frequency
 * 
 * Note: this function does the following to enable
 *  Set CYCLE bit to 1
 *  Set SLEEP bit to 0
 *  Set TEMP_DIS bit to 1
 *  Set STBY_XG, STBY_YG, STBY_ZG bits to 1
 *  Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *  Set FCHOICE to 0 (ACCEL_FCHOICE_B bit to 1) [MPU6500 / MPU9250 only]
 *  Disable Auxiliary I2C Master I/F
 * 
 * Note: this function does the following to disable
 *  Set CYCLE bit to 0
 *  Set TEMP_DIS bit to 0
 *  Set STBY_XG, STBY_YG, STBY_ZG bits to 0
 *  Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *  Set FCHOICE to 3 (ACCEL_FCHOICE_B bit to 0) [MPU6500 / MPU9250 only]
 *  Enable Auxiliary I2C Master I/F
 * */
esp_err_t MPU::setLowPowerAccelMode(bool enable) {
    // set FCHOICE
    #ifdef CONFIG_MPU6500
    fchoice_t fchoice = enable ? FCHOICE_0 : FCHOICE_3;
    if (MPU_ERR_CHECK(setFchoice(fchoice)))
            return err;
    MPU_LOGVMSG(msgs::EMPTY, "Fchoice set to %d", fchoice);
    #endif
    // read PWR_MGMT1 and PWR_MGMT2 at once
    if (MPU_ERR_CHECK(readBytes(regs::PWR_MGMT1, 2, buffer)))
        return err;
    if (enable) {
        // set CYCLE bit to 1 and SLEEP bit to 0 and TEMP_DIS bit to 1
        buffer[0] |= 1 << regs::PWR1_CYCLE_BIT;
        buffer[0] &= ~(1 << regs::PWR1_SLEEP_BIT);
        buffer[0] |= 1 << regs::PWR1_TEMP_DIS_BIT;
        // set STBY_XG, STBY_YG, STBY_ZG bits to 1
        buffer[1] |= regs::PWR2_STBY_XYZG_BITS;
    } else {  // disable
        // set CYCLE bit to 0 and TEMP_DIS bit to 0
        buffer[0] &= ~(1 << regs::PWR1_CYCLE_BIT);
        buffer[0] &= ~(1 << regs::PWR1_TEMP_DIS_BIT);
        // set STBY_XG, STBY_YG, STBY_ZG bits to 0
        buffer[1] &= ~(regs::PWR2_STBY_XYZG_BITS);
    }
    // set STBY_XA, STBY_YA, STBY_ZA bits to 0
    buffer[1] &= ~(regs::PWR2_STBY_XYZA_BITS);
    // write back PWR_MGMT1 and PWR_MGMT2 at once
    if (MPU_ERR_CHECK(writeBytes(regs::PWR_MGMT1, 2, buffer)))
        return err;
    // disable Auxiliary I2C Master I/F in case it was active
    if (MPU_ERR_CHECK(setAuxI2CEnabled(!enable)))
        return err;
    return err;
}

/**
 * Return Low Power Accelerometer state
 * Note: condition to return true:
 *  CYCLE bit is 1
 *  SLEEP bit is 0
 *  TEMP_DIS bit is 1
 *  STBY_XG, STBY_YG, STBY_ZG bits are 1
 *  STBY_XA, STBY_YA, STBY_ZA bits are 0
 *  FCHOICE is 0 (ACCEL_FCHOICE_B bit is 1) [MPU6500 / MPU9250 only]
 * */
bool MPU::getLowPowerAccelMode() {
    // check FCHOICE
    #ifdef CONFIG_MPU6500
    fchoice_t fchoice = getFchoice();
    MPU_ERR_CHECK(lastError());
    if (fchoice != FCHOICE_0)
        return false;
    #endif
    // read PWR_MGMT1 and PWR_MGMT2 at once
    MPU_ERR_CHECK(readBytes(regs::PWR_MGMT1, 2, buffer));
    // define configuration bits
    constexpr uint8_t LPACCEL_CONFIG_BITMASK[2] = {
        (1 << regs::PWR1_SLEEP_BIT) | (1 << regs::PWR1_CYCLE_BIT) | (1 << regs::PWR1_TEMP_DIS_BIT),
        regs::PWR2_STBY_XYZA_BITS | regs::PWR2_STBY_XYZG_BITS
    };
    constexpr uint8_t LPACCEL_ENABLED_VALUE[2] = {
        (1 << regs::PWR1_CYCLE_BIT) | (1 << regs::PWR1_TEMP_DIS_BIT),
        regs::PWR2_STBY_XYZG_BITS
    };
    // get just the configuration bits
    buffer[0] &= LPACCEL_CONFIG_BITMASK[0];
    buffer[1] &= LPACCEL_CONFIG_BITMASK[1];
    // check pattern
    if (buffer[0] == LPACCEL_ENABLED_VALUE[0] && buffer[1] == LPACCEL_ENABLED_VALUE[1])
        return true;
    return false;
}


/**
 * Set Low Power Accelerometer frequency of wake-up
 * [MPU6000 / MPU6050 / MPU9150]: 1.25Hz, 5Hz, 20Hz, 40Hz
 * [MPU6500 / MPU9250]: 0.24Hz, 0.49Hz, 0.98Hz, 1.95Hz, 3.91Hz, 7.81Hz, 15.63Hz, 31.25Hz, 62.5Hz, 125Hz, 250Hz, 500Hz
 * Note: ODR = Output Data Rate
 * */
esp_err_t MPU::setLowPowerAccelRate(lp_accel_rate_t rate) {
    #if defined CONFIG_MPU6050
    return MPU_ERR_CHECK(writeBits(regs::PWR_MGMT2, regs::PWR2_LP_WAKE_CTRL_BIT, regs::PWR2_LP_WAKE_CTRL_LENGTH, rate));
    #elif defined CONFIG_MPU6500
    return MPU_ERR_CHECK(writeBits(regs::LP_ACCEL_ODR, regs::LPA_ODR_CLKSEL_BIT, regs::LPA_ODR_CLKSEL_LENGTH, rate));
    #endif
}

lp_accel_rate_t MPU::getLowPowerAccelRate() {
    #if defined CONFIG_MPU6050
    MPU_ERR_CHECK(readBits(regs::PWR_MGMT2, regs::PWR2_LP_WAKE_CTRL_BIT, regs::PWR2_LP_WAKE_CTRL_LENGTH, buffer));
    #elif defined CONFIG_MPU6500
    MPU_ERR_CHECK(readBits(regs::LP_ACCEL_ODR, regs::LPA_ODR_CLKSEL_BIT, regs::LPA_ODR_CLKSEL_LENGTH, buffer));
    #endif
    return (lp_accel_rate_t) buffer[0];
}


/**
 * Enable/disable Wake-on-Motion mode
 * Important: The configurations must've already been set with setWakeOnMotionConfig() before enabling WOM mode!
 * Note: call getMotionDetectStatus() to find out which axis generated motion interrupt. [MPU6000, MPU6050, MPU9150]
 * Note: It's recommended to make the WOM interrupt to propagate to the INT pin.
 *  You may configure using setInterruptEnabled().
 * 
 * Note: On enable, this function modifies the DLPF, puts gyro and temperature
 *  sensors in standby mode, and disable I2C master I/F.
 * Note: On disable, this function sets back DLPF to 42Hz and disable
 *  standby mode for all sensors, enable I2C master I/F.
 * */
esp_err_t MPU::setWakeOnMotionMode(bool enable) {
    stby_en_t stbyMask;
    if (enable) {
        // make sure accel is running and set other sensors to stanby
        stbyMask = STBY_EN_GYRO | STBY_EN_TEMP;
    } else {
        stbyMask = STBY_EN_NONE;
    }
    if (MPU_ERR_CHECK(setStandbyMode(stbyMask)))
        return err;
    if (enable) {
        #if defined CONFIG_MPU6050
        if (MPU_ERR_CHECK(writeBits(regs::ACCEL_CONFIG, regs::ACONFIG_HPF_BIT, regs::ACONFIG_HPF_LENGTH, ACCEL_DHPF_RESET)))
            return err;
        constexpr dlpf_t kDLPF = DLPF_256HZ_NOLPF;
        #elif defined CONFIG_MPU6500
        if (MPU_ERR_CHECK(setFchoice(FCHOICE_3)))
            return err;
        constexpr dlpf_t kDLPF = DLPF_188HZ;
        #endif
        if (MPU_ERR_CHECK(setDigitalLowPassFilter(kDLPF)))
            return err;
        #if defined CONFIG_MPU6050
        // give a time for accumulation of samples
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (MPU_ERR_CHECK(writeBits(regs::ACCEL_CONFIG, regs::ACONFIG_HPF_BIT, regs::ACONFIG_HPF_LENGTH, ACCEL_DHPF_HOLD)))
            return err;
        #elif defined CONFIG_MPU6500
        if (MPU_ERR_CHECK(writeByte(regs::ACCEL_INTEL_CTRL, (1 << regs::ACCEL_INTEL_EN_BIT) | (1 << regs::ACCEL_INTEL_MODE_BIT))))
            return err;
        #endif
    } else {  /* disable */
        #if defined CONFIG_MPU6500
        if (MPU_ERR_CHECK(writeByte(regs::ACCEL_INTEL_CTRL, 0x0)))
            return err;
        #endif
        constexpr dlpf_t kDLPF = DLPF_42HZ;
        if (MPU_ERR_CHECK(setDigitalLowPassFilter(kDLPF)))
            return err;
    }
    // disable Auxiliary I2C Master I/F in case it was active
    if (MPU_ERR_CHECK(setAuxI2CEnabled(!enable)))
        return err;
    // enable cycling through sleep/wake
    return MPU_ERR_CHECK(writeBit(regs::PWR_MGMT1, regs::PWR1_CYCLE_BIT, enable));
}

bool MPU::getWakeOnMotionMode() {
    stby_en_t stbyMask = getStandbyMode();
    MPU_ERR_CHECK(lastError());
    if (((stbyMask & STBY_EN_ACCEL) != 0) &&
        ((stbyMask & (STBY_EN_GYRO | STBY_EN_TEMP)) != (STBY_EN_GYRO | STBY_EN_TEMP))) {
        return false;
    }
    uint8_t data;
    MPU_ERR_CHECK(readBit(regs::PWR_MGMT1, regs::PWR1_CYCLE_BIT, &data));
    if (!data)
        return false;
    #if defined CONFIG_MPU6500
    MPU_ERR_CHECK(readByte(regs::ACCEL_INTEL_CTRL, &data));
    if (data != ((1 << regs::ACCEL_INTEL_EN_BIT) | (1 << regs::ACCEL_INTEL_MODE_BIT)))
        return false;
    #endif
    return true;
}


/**
 * Configure Wake-on-Motion feature
 * 
 * The behaviour of this feature is very different between the MPU6050 (MPU9150) and the
 * MPU6500 (MPU9250). Each chip's version of this feature is explained below.
 * 
 * [MPU6050, MPU6000, MPU9150]:
 * Accelerometer measurements are passed through a configurable digital high pass filter (DHPF)
 * in order to eliminate bias due to gravity. A qualifying motion sample is one where the high passed sample
 * from any axis has an absolute value exceeding a user-programmable threshold.
 * A counter increments for each qualifying sample, and decrements for each non-qualifying sample.
 * Once the counter reaches a user-programmable counter threshold, a motion interrupt is triggered.
 * The axis and polarity which caused the interrupt to be triggered is flagged in the MOT_DETECT_STATUS register.
 * 
 * [MPU6500, MPU9250]:
 * Unlike the MPU6050 version, the hardware does not "lock in" a reference sample.
 * The hardware monitors the accel data and detects any large change over a short period of time.
 * A qualifying motion sample is one where the high passed sample from any axis has
 * an absolute value exceeding the threshold.
 * The hardware motion threshold can be between 4mg and 1020mg in 4mg increments.
 * */
esp_err_t MPU::setWakeOnMotionConfig(wom_config_t& config) {
    #if defined CONFIG_MPU6050
    if (MPU_ERR_CHECK(writeByte(regs::MOTION_DUR, config.time)))
        return err;
    if (MPU_ERR_CHECK(writeBits(regs::MOTION_DETECT_CTRL, regs::MOTCTRL_ACCEL_ON_DELAY_BIT,
        regs::MOTCTRL_ACCEL_ON_DELAY_LENGTH, config.accel_on_delay)))
        return err;
    if (MPU_ERR_CHECK(writeBits(regs::MOTION_DETECT_CTRL, regs::MOTCTRL_MOT_COUNT_BIT,
        regs::MOTCTRL_MOT_COUNT_LENGTH, config.counter)))
        return err;
    #endif
    if (MPU_ERR_CHECK(writeByte(regs::MOTION_THR, config.threshold)))
        return err;
    return MPU_ERR_CHECK(setLowPowerAccelRate(config.rate));
}

wom_config_t MPU::getWakeOnMotionConfig() {
    wom_config_t config;
    #if defined CONFIG_MPU6050
    MPU_ERR_CHECK(readByte(regs::MOTION_DUR, &config.time));
    MPU_ERR_CHECK(readByte(regs::MOTION_DETECT_CTRL, buffer));
    config.accel_on_delay = 0x3 & (buffer[0] >> (regs::MOTCTRL_ACCEL_ON_DELAY_BIT - regs::MOTCTRL_ACCEL_ON_DELAY_LENGTH + 1));
    config.counter = (mot_counter_t) (0x3 & (buffer[0] >> (regs::MOTCTRL_MOT_COUNT_BIT - regs::MOTCTRL_MOT_COUNT_LENGTH + 1)));
    #endif
    MPU_ERR_CHECK(readByte(regs::MOTION_THR, &config.threshold));
    config.rate = getLowPowerAccelRate();
    MPU_ERR_CHECK(lastError());
    return config;
}


#if defined CONFIG_MPU6050
/**
 * Return Motion Detection Status
 * Note: Reading this register clears all motion detection status bits.
 * */
mot_stat_t MPU::getMotionDetectStatus() {
    MPU_ERR_CHECK(readByte(regs::MOTION_DETECT_STATUS, buffer));
    return (mot_stat_t) buffer[0];
}
#endif


/**
 * Standby mode
 * */
esp_err_t MPU::setStandbyMode(stby_en_t mask) {
    const uint8_t kPwr1StbyBits = mask >> 6;
    if (MPU_ERR_CHECK(writeBits(regs::PWR_MGMT1, regs::PWR1_GYRO_STANDBY_BIT, 2, kPwr1StbyBits)))
        return err;
    return MPU_ERR_CHECK(writeBits(regs::PWR_MGMT2, regs::PWR2_STBY_XA_BIT, 6, mask));
}

stby_en_t MPU::getStandbyMode() {
    MPU_ERR_CHECK(readBytes(regs::PWR_MGMT1, 2, buffer));
    constexpr uint8_t kStbyTempAndGyroPLLBits = STBY_EN_TEMP | STBY_EN_LOWPWR_GYRO_PLL_ON;
    stby_en_t mask = buffer[0] << 3 & kStbyTempAndGyroPLLBits;
    constexpr uint8_t kStbyAccelAndGyroBits = STBY_EN_ACCEL | STBY_EN_GYRO;
    mask |= buffer[1] & kStbyAccelAndGyroBits;
    return mask;
}


#ifdef CONFIG_MPU6500
/**
 * Note that FCHOICE is the inverted value of
 * FCHOICE_B (e.g. FCHOICE=2b’00 is same as FCHOICE_B=2b’11).
 * Reset value is FCHOICE_3
 * */
esp_err_t MPU::setFchoice(fchoice_t fchoice) {
    buffer[0] = (~(fchoice) & 0x3);  // invert to fchoice_b
    if (MPU_ERR_CHECK(writeBits(regs::GYRO_CONFIG, regs::GCONFIG_FCHOICE_B, regs::GCONFIG_FCHOICE_B_LENGTH, buffer[0])))
        return err;
    return MPU_ERR_CHECK(writeBit(regs::ACCEL_CONFIG2, regs::ACONFIG2_ACCEL_FCHOICE_B_BIT, (buffer[0] == 0) ? 0 : 1));
}

fchoice_t MPU::getFchoice() {
    MPU_ERR_CHECK(readBits(regs::GYRO_CONFIG, regs::GCONFIG_FCHOICE_B, regs::GCONFIG_FCHOICE_B_LENGTH, buffer));
    return (fchoice_t) (~(buffer[0]) & 0x3);
}
#endif


/**
 * Gyroscope Full-scale range
 * */
esp_err_t MPU::setGyroFullScale(gyro_fs_t fsr) {
    return MPU_ERR_CHECK(writeBits(regs::GYRO_CONFIG, regs::GCONFIG_FS_SEL_BIT, regs::GCONFIG_FS_SEL_LENGTH, fsr));
}

gyro_fs_t MPU::getGyroFullScale() {
    MPU_ERR_CHECK(readBits(regs::GYRO_CONFIG, regs::GCONFIG_FS_SEL_BIT, regs::GCONFIG_FS_SEL_LENGTH, buffer));
    return (gyro_fs_t) buffer[0];
}


/**
 * Accelerometer Full-scale range
 * */
esp_err_t MPU::setAccelFullScale(accel_fs_t fsr) {
    return MPU_ERR_CHECK(writeBits(regs::ACCEL_CONFIG, regs::ACONFIG_FS_SEL_BIT, regs::ACONFIG_FS_SEL_LENGTH, fsr));
}

accel_fs_t MPU::getAccelFullScale() {
    MPU_ERR_CHECK(readBits(regs::ACCEL_CONFIG, regs::ACONFIG_FS_SEL_BIT, regs::ACONFIG_FS_SEL_LENGTH, buffer));
    return (accel_fs_t) buffer[0];
}


/**
 * Push biases to the gyro offset registers.
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 * 
 * Note: Bias inputs are LSB in +-1000dps format.
 * */
esp_err_t MPU::setGyroOffset(raw_axes_t bias) {
    buffer[0] = bias.x >> 8;
    buffer[1] = bias.x;
    buffer[2] = bias.y >> 8;
    buffer[3] = bias.y;
    buffer[4] = bias.z >> 8;
    buffer[5] = bias.z;
    return MPU_ERR_CHECK(writeBytes(regs::XG_OFFSET_H, 6, buffer));
}

/**
 * Note: Bias output are LSB in +-1000dps format.
 * */
raw_axes_t MPU::getGyroOffset() {
    MPU_ERR_CHECK(readBytes(regs::XG_OFFSET_H, 6, buffer));
    raw_axes_t bias;
    bias.x = (buffer[0] << 8) | buffer[1];
    bias.y = (buffer[2] << 8) | buffer[3];
    bias.z = (buffer[4] << 8) | buffer[5];
    return bias;
}


/**
 * Push biases to the accel offset registers.
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 * 
 * Note: Bias inputs are LSB in +-16G format.
 * */
esp_err_t MPU::setAccelOffset(raw_axes_t bias) {
    raw_axes_t facBias;
    // first, read OTP values of Accel factory trim

    #if defined CONFIG_MPU6050
    if (MPU_ERR_CHECK(readBytes(regs::XA_OFFSET_H, 6, buffer)))
        return err;
    facBias.x = (buffer[0] << 8) | buffer[1];
    facBias.y = (buffer[2] << 8) | buffer[3];
    facBias.z = (buffer[4] << 8) | buffer[5];

    #elif defined CONFIG_MPU6500
    if (MPU_ERR_CHECK(readBytes(regs::XA_OFFSET_H, 8, buffer)))
        return err;
    // note: buffer[2] and buffer[5], stay the same,
    //  they are read just to keep the burst reading
    facBias.x = (buffer[0] << 8) | buffer[1];
    facBias.y = (buffer[3] << 8) | buffer[4];
    facBias.z = (buffer[6] << 8) | buffer[7];
    #endif

    // note: preserve bit 0 of factory value (for temperature compensation)
    facBias.x += (bias.x & ~1);
    facBias.y += (bias.y & ~1);
    facBias.z += (bias.z & ~1);

    #if defined CONFIG_MPU6050
    buffer[0] = facBias.x >> 8;
    buffer[1] = facBias.x;
    buffer[2] = facBias.y >> 8;
    buffer[3] = facBias.y;
    buffer[4] = facBias.z >> 8;
    buffer[5] = facBias.z;
    if (MPU_ERR_CHECK(writeBytes(regs::XA_OFFSET_H, 6, buffer)))
        return err;

    #elif defined CONFIG_MPU6500
    buffer[0] = facBias.x >> 8;
    buffer[1] = facBias.x;
    buffer[3] = facBias.y >> 8;
    buffer[4] = facBias.y;
    buffer[6] = facBias.z >> 8;
    buffer[7] = facBias.z;
    return MPU_ERR_CHECK(writeBytes(regs::XA_OFFSET_H, 8, buffer));
    #endif

    return err;
}

/**
 * Note: This returns the biases with OTP values from factory trim added,
 * so returned values will be different than that ones set with setAccelOffset().
 * 
 * Note: Bias output are LSB in +-16G format.
 * */
raw_axes_t MPU::getAccelOffset() {
    raw_axes_t bias;

    #if defined CONFIG_MPU6050
    MPU_ERR_CHECK(readBytes(regs::XA_OFFSET_H, 6, buffer));
    bias.x = (buffer[0] << 8) | buffer[1];
    bias.y = (buffer[2] << 8) | buffer[3];
    bias.z = (buffer[4] << 8) | buffer[5];

    #elif defined CONFIG_MPU6500
    MPU_ERR_CHECK(readBytes(regs::XA_OFFSET_H, 8, buffer));
    bias.x = (buffer[0] << 8) | buffer[1];
    bias.y = (buffer[3] << 8) | buffer[4];
    bias.z = (buffer[6] << 8) | buffer[7];
    #endif

    return bias;
}


/**
 * Read accelerometer data
 * */
esp_err_t MPU::acceleration(raw_axes_t* accel) {
    if (MPU_ERR_CHECK(readBytes(regs::ACCEL_XOUT_H, 6, buffer)))
        return err;
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    return err;
}

esp_err_t MPU::acceleration(int16_t* x, int16_t* y, int16_t* z) {
    if (MPU_ERR_CHECK(readBytes(regs::ACCEL_XOUT_H, 6, buffer)))
        return err;
    *x = buffer[0] << 8 | buffer[1];
    *y = buffer[2] << 8 | buffer[3];
    *z = buffer[4] << 8 | buffer[5];
    return err;
}


/**
 * Read gyroscope data
 * */
esp_err_t MPU::rotation(raw_axes_t* gyro) {
    if (MPU_ERR_CHECK(readBytes(regs::GYRO_XOUT_H, 6, buffer)))
        return err;
    gyro->x = buffer[0] << 8 | buffer[1];
    gyro->y = buffer[2] << 8 | buffer[3];
    gyro->z = buffer[4] << 8 | buffer[5];
    return err;
}

esp_err_t MPU::rotation(int16_t* x, int16_t* y, int16_t* z) {
    if (MPU_ERR_CHECK(readBytes(regs::GYRO_XOUT_H, 6, buffer)))
        return err;
    *x = buffer[0] << 8 | buffer[1];
    *y = buffer[2] << 8 | buffer[3];
    *z = buffer[4] << 8 | buffer[5];
    return err;
}


/**
 * Read temperature data
 * */
esp_err_t MPU::temperature(int16_t* temp) {
    if (MPU_ERR_CHECK(readBytes(regs::TEMP_OUT_H, 2, buffer)))
        return err;
    *temp = buffer[0] << 8 | buffer[1];
    return err;
}


/**
 * Read accelerometer and gyroscope data at once
 * */
esp_err_t MPU::motion(raw_axes_t* accel, raw_axes_t* gyro) {
    if (MPU_ERR_CHECK(readBytes(regs::ACCEL_XOUT_H, 14, buffer)))
        return err;
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    gyro->x  = buffer[8] << 8 | buffer[9];
    gyro->y  = buffer[10] << 8 | buffer[11];
    gyro->z  = buffer[12] << 8 | buffer[13];
    return err;
}


#if defined CONFIG_MPU_AK89xx
/**
 * Read compass data
 * */
esp_err_t MPU::heading(raw_axes_t* mag) {
    if (MPU_ERR_CHECK(readBytes(regs::EXT_SENS_DATA_01, 6, buffer)))
        return err;
    mag->x = buffer[1] << 8 | buffer[0];
    mag->y = buffer[3] << 8 | buffer[2];
    mag->z = buffer[5] << 8 | buffer[4];
    return err;
}

esp_err_t MPU::heading(int16_t* x, int16_t* y, int16_t* z) {
    if (MPU_ERR_CHECK(readBytes(regs::EXT_SENS_DATA_01, 6, buffer)))
        return err;
    *x = buffer[1] << 8 | buffer[0];
    *y = buffer[3] << 8 | buffer[2];
    *z = buffer[5] << 8 | buffer[4];
    return err;
}


/**
 * Read accelerometer, gyroscope, compass
 * */
esp_err_t MPU::motion(raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag) {
    uint8_t buffer[22];
    if (MPU_ERR_CHECK(readBytes(regs::ACCEL_XOUT_H, 22, buffer)))
        return err;
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    gyro->x  = buffer[8] << 8 | buffer[9];
    gyro->y  = buffer[10] << 8 | buffer[11];
    gyro->z  = buffer[12] << 8 | buffer[13];
    mag->x   = buffer[16] << 8 | buffer[15];
    mag->x   = buffer[18] << 8 | buffer[17];
    mag->x   = buffer[20] << 8 | buffer[19];
    return err;
}
#endif  // AK89xx


/**
 * Read data from all internal sensors
 * */
esp_err_t MPU::sensors(raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp) {
    if (MPU_ERR_CHECK(readBytes(regs::ACCEL_XOUT_H, 14, buffer)))
        return err;
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    *temp    = buffer[6] << 8 | buffer[7];
    gyro->x  = buffer[8] << 8 | buffer[9];
    gyro->y  = buffer[10] << 8 | buffer[11];
    gyro->z  = buffer[12] << 8 | buffer[13];
    return err;
}


/**
 * Read data from all sensors, including external
 * */
esp_err_t MPU::sensors(sensors_t* sensors, size_t extsens_len) {
    constexpr size_t kIntSensLenMax = 14;  // internal sensors data length max
    constexpr size_t kExtSensLenMax = 24;  // external sensors data length max
    uint8_t buffer[kIntSensLenMax + kExtSensLenMax];
    #if defined CONFIG_MPU_AK89xx
    constexpr size_t kMagLen = 8;  // magnetometer data length
    const size_t length = kIntSensLenMax + extsens_len  + kMagLen;
    #else
    const size_t length = kIntSensLenMax + extsens_len;
    #endif
    if (MPU_ERR_CHECK(readBytes(regs::ACCEL_XOUT_H, length, buffer)))
        return err;
    sensors->accel.x = buffer[0] << 8 | buffer[1];
    sensors->accel.y = buffer[2] << 8 | buffer[3];
    sensors->accel.z = buffer[4] << 8 | buffer[5];
    sensors->temp    = buffer[6] << 8 | buffer[7];
    sensors->gyro.x  = buffer[8] << 8 | buffer[9];
    sensors->gyro.y  = buffer[10] << 8 | buffer[11];
    sensors->gyro.z  = buffer[12] << 8 | buffer[13];
    #if CONFIG_MPU_AK89xx
    sensors->mag.x   = buffer[16] << 8 | buffer[15];
    sensors->mag.y   = buffer[18] << 8 | buffer[17];
    sensors->mag.z   = buffer[20] << 8 | buffer[19];
    #endif
    memcpy(sensors->extsens, buffer + (length - extsens_len), extsens_len);
    return err;
}


#if defined CONFIG_MPU6050 && !defined CONFIG_MPU6000
/**
 * The MPU-6050’s I/O logic levels are set to be either VDD or VLOGIC
 * 
 * VLOGIC may be set to be equal to VDD or to another voltage. However, VLOGIC must be ≤ VDD at all
 * times. When AUX_VDDIO is set to 0 (its power-on-reset value), VLOGIC is the power supply voltage for
 * both the microprocessor system bus and the auxiliary I C bus.
 * When AUX_VDDIO is set to 1, VLOGIC is the power supply voltage for 
 * the microprocessor system bus and VDD is the supply for the auxiliary I2C bus
 * */
esp_err_t MPU::setAuxVDDIOLevel(auxvddio_lvl_t level) {
    return MPU_ERR_CHECK(writeBit(regs::YG_OTP_OFFSET_TC, regs::TC_PWR_MODE_BIT, level));
}

auxvddio_lvl_t MPU::getAuxVDDIOLevel() {
    MPU_ERR_CHECK(readBit(regs::YG_OTP_OFFSET_TC, regs::TC_PWR_MODE_BIT, buffer));
    return (auxvddio_lvl_t) buffer[0];
}
#endif


esp_err_t MPU::setInterruptConfig(int_config_t config) {
    if (MPU_ERR_CHECK(readByte(regs::INT_PIN_CONFIG, buffer)))
        return err;
    // zero the bits we're setting, but keep the others we're not setting as they are;
    constexpr uint8_t INT_PIN_CONFIG_BITMASK =
        (1 << regs::INT_CFG_LEVEL_BIT) | (1 << regs::INT_CFG_OPEN_BIT) |
        (1 << regs::INT_CFG_LATCH_EN_BIT) | (1 << regs::INT_CFG_ANYRD_2CLEAR_BIT);
    buffer[0] &= ~INT_PIN_CONFIG_BITMASK;
    // set the configurations
    buffer[0] |= config.level << regs::INT_CFG_LEVEL_BIT;
    buffer[0] |= config.drive << regs::INT_CFG_OPEN_BIT;
    buffer[0] |= config.mode << regs::INT_CFG_LATCH_EN_BIT;
    buffer[0] |= config.clear << regs::INT_CFG_ANYRD_2CLEAR_BIT;
    return MPU_ERR_CHECK(writeByte(regs::INT_PIN_CONFIG, buffer[0]));
}

int_config_t MPU::getInterruptConfig() {
    MPU_ERR_CHECK(readByte(regs::INT_PIN_CONFIG, buffer));
    int_config_t config;
    config.level = (int_lvl_t)   ((buffer[0] >> regs::INT_CFG_LEVEL_BIT) & 0x1);
    config.drive = (int_drive_t) ((buffer[0] >> regs::INT_CFG_OPEN_BIT) & 0x1);
    config.mode  = (int_mode_t)  ((buffer[0] >> regs::INT_CFG_LATCH_EN_BIT) & 0x1);
    config.clear = (int_clear_t) ((buffer[0] >> regs::INT_CFG_ANYRD_2CLEAR_BIT) & 0x1);
    return config;
}


esp_err_t MPU::setInterruptEnabled(int_en_t mask) {
    return MPU_ERR_CHECK(writeByte(regs::INT_ENABLE, mask));
}

int_en_t MPU::getInterruptEnabled() {
    MPU_ERR_CHECK(readByte(regs::INT_ENABLE, buffer));
    return (int_en_t) buffer[0];
}


esp_err_t MPU::setInterruptDisableAll() {
    return MPU_ERR_CHECK(writeByte(regs::INT_ENABLE, 0x0));
}


int_en_t MPU::getInterruptStatus() {
    MPU_ERR_CHECK(readByte(regs::INT_STATUS, buffer));
    return (int_en_t) buffer[0];
}


/**
 * Change FIFO mode
 * Options:
 * FIFO_MODE_OVERWRITE  // when the fifo is full, additional writes will be written to the fifo, replacing the oldest data.
 * FIFO_MODE_STOP_FULL  // when the fifo is full, additional writes will not be written to fifo.
 * */
esp_err_t MPU::setFIFOMode(fifo_mode_t mode) {
    return MPU_ERR_CHECK(writeBit(regs::CONFIG, regs::CONFIG_FIFO_MODE_BIT, mode));
}

fifo_mode_t MPU::getFIFOMode() {
    MPU_ERR_CHECK(readBit(regs::CONFIG, regs::CONFIG_FIFO_MODE_BIT, buffer));
    return (fifo_mode_t) buffer[0];
}


/**
 * Configure the sensors that will be written to the FIFO
 * */
esp_err_t MPU::setFIFOConfig(fifo_config_t config) {
    if (MPU_ERR_CHECK(writeByte(regs::FIFO_EN, (uint8_t) config)))
        return err;
    return MPU_ERR_CHECK(writeBit(regs::I2C_MST_CTRL, regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT, config >> 8));
}

fifo_config_t MPU::getFIFOConfig() {
    MPU_ERR_CHECK(readBytes(regs::FIFO_EN, 2, buffer));
    fifo_config_t config = buffer[0];
    config |= (buffer[1] & (1 << mpu::regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT)) << 3;
    return config;
}


/**
 * Enabled / disable FIFO module
 * */
esp_err_t MPU::setFIFOEnabled(bool enable) {
    return MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_FIFO_EN_BIT, enable));
}

bool MPU::getFIFOEnabled() {
    MPU_ERR_CHECK(readBit(regs::USER_CTRL, regs::USERCTRL_FIFO_EN_BIT, buffer));
    return buffer[0];
}


/**
 * Reset FIFO module (zero FIFO count), reset is asynchronous.
 * The bit auto clears after one clock cycle.
 * */
esp_err_t MPU::resetFIFO() {
    return MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_FIFO_RESET_BIT, 1));
}


/**
 * Return number of written bytes in the FIFO
 * Note: FIFO overflow generates an interrupt which can be check with getInterruptStatus()
 * */
uint16_t MPU::getFIFOCount() {
    MPU_ERR_CHECK(readBytes(regs::FIFO_COUNT_H, 2, buffer));
    uint16_t count = buffer[0] << 8 | buffer[1];
    return count;
}


/**
 * Read data from FIFO buffer
 * */
esp_err_t MPU::readFIFO(size_t length, uint8_t *data) {
    return MPU_ERR_CHECK(readBytes(regs::FIFO_R_W, length, data));
}

/**
 * Write data to FIFO buffer
 * */
esp_err_t MPU::writeFIFO(size_t length, const uint8_t *data) {
    return MPU_ERR_CHECK(writeBytes(regs::FIFO_R_W, length, data));
}


/**
 * Configures the Auxiliar I2C Master
 * */
esp_err_t MPU::setAuxI2CConfig(const auxi2c_config_t &config) {
    // TODO: check compass enabled, to constrain sample_delay which defines the compass read sample rate
    if (MPU_ERR_CHECK(readBit(regs::I2C_MST_CTRL, regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT, buffer)))
        return err;
    buffer[0] <<= regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT;
    buffer[0] |= config.multi_master_en << regs::I2CMST_CTRL_MULT_EN_BIT;
    buffer[0] |= config.wait_for_es << regs::I2CMST_CTRL_WAIT_FOR_ES_BIT;
    buffer[0] |= config.transition << regs::I2CMST_CTRL_P_NSR_BIT;
    buffer[0] |= config.clock;
    if (MPU_ERR_CHECK(writeByte(regs::I2C_MST_CTRL, buffer[0])))
        return err;
    if (MPU_ERR_CHECK(writeBits(regs::I2C_SLV4_CTRL, regs::I2C_SLV4_MST_DELAY_BIT,
            regs::I2C_SLV4_MST_DELAY_LENGTH, config.sample_delay))) {
        return err;
    }
    if (MPU_ERR_CHECK(writeBit(regs::I2C_MST_DELAY_CRTL, regs::I2CMST_DLY_ES_SHADOW_BIT, config.shadow_delay_en)))
        return err;
    MPU_LOGVMSG(msgs::EMPTY, "Master:: multi_master_en: %d, wait_for_es: %d,"
            "transition: %d, clock: %d, sample_delay: %d, shadow_delay_en: %d",
            config.multi_master_en, config.wait_for_es, config.transition, config.clock,
            config.sample_delay, config.shadow_delay_en);
    return err;
}

auxi2c_config_t MPU::getAuxI2CConfig() {
    MPU_ERR_CHECK(readByte(regs::I2C_MST_CTRL, buffer));
    auxi2c_config_t config;
    config.multi_master_en = buffer[0] >> regs::I2CMST_CTRL_MULT_EN_BIT;
    config.wait_for_es = (buffer[0] >> regs::I2CMST_CTRL_WAIT_FOR_ES_BIT) & 0x1;
    config.transition = (auxi2c_trans_t) ((buffer[0] >> regs::I2CMST_CTRL_P_NSR_BIT) & 0x1);
    config.clock = (auxi2c_clock_t) (buffer[0] & ((1 << regs::I2CMST_CTRL_CLOCK_LENGTH) - 1));
    MPU_ERR_CHECK(readBits(regs::I2C_SLV4_CTRL, regs::I2C_SLV4_MST_DELAY_BIT, regs::I2C_SLV4_MST_DELAY_LENGTH, buffer+1));
    config.sample_delay = buffer[1];
    MPU_ERR_CHECK(readBit(regs::I2C_MST_DELAY_CRTL, regs::I2CMST_DLY_ES_SHADOW_BIT, buffer+2));
    config.shadow_delay_en = buffer[2];
    return config;
}


/**
 * Enable / disable Auxiliary I2C Master module
 * */
esp_err_t MPU::setAuxI2CEnabled(bool enable) {
    if (MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_EN_BIT, enable)))
        return err;
    if (enable) {
        return MPU_ERR_CHECK(writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_I2C_BYPASS_EN_BIT, 0));
    }
    return err;
}

bool MPU::getAuxI2CEnabled() {
    MPU_ERR_CHECK(readBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_EN_BIT, buffer));
    MPU_ERR_CHECK(readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_I2C_BYPASS_EN_BIT, buffer+1));
    return buffer[0] && (!buffer[1]);
}


/**
 * Configures the communication with a Slave connected to Auxiliary I2C bus
 * */
esp_err_t MPU::setAuxI2CSlaveConfig(const auxi2c_slv_config_t &config) {
    // slaves' config registers are grouped as 3 regs in a row
    const uint8_t regAddr = config.slave * 3 + regs::I2C_SLV0_ADDR;
    // data for regs::I2C_SLVx_ADDR
    buffer[0] = config.rw << regs::I2C_SLV_RNW_BIT;
    buffer[0] |= config.addr;
    // data for regs::I2C_SLVx_REG
    buffer[1] = config.reg_addr;
    // data for regs::I2C_SLVx_CTRL
    if (MPU_ERR_CHECK(readByte(regAddr+2, buffer+2)))
            return err;
    if (config.rw == AUXI2C_READ) {
        buffer[2] &= 1 << regs::I2C_SLV_EN_BIT;  // keep enable bit, clear the rest
        buffer[2] |= config.reg_dis << regs::I2C_SLV_REG_DIS_BIT;
        buffer[2] |= config.swap_en << regs::I2C_SLV_BYTE_SW_BIT;
        buffer[2] |= config.end_of_word << regs::I2C_SLV_GRP_BIT;
        buffer[2] |= config.rxlength & 0xF;
    } else {  // AUXI2C_WRITE
        buffer[2] &= ~(1 << regs::I2C_SLV_REG_DIS_BIT | 0xF);  // clear length bits and register disable bit
        buffer[2] |= config.reg_dis << regs::I2C_SLV_REG_DIS_BIT;
        buffer[2] |= 0x1;  // set length to write 1 byte
        if (MPU_ERR_CHECK(writeByte(regs::I2C_SLV0_DO + config.slave, config.txdata)))
            return err;
    }
    if (MPU_ERR_CHECK(writeBytes(regAddr, 3, buffer)))
        return err;
    // sample_delay enable/disable
    if (MPU_ERR_CHECK(writeBit(regs::I2C_MST_DELAY_CRTL, config.slave, config.sample_delay_en)))
        return err;
    MPU_LOGVMSG(msgs::EMPTY, "Slave%d:: r/w: %s, addr: 0x%X, reg_addr: 0x%X, reg_dis: %d, %s: 0x%X, sample_delay_en: %d",
        config.slave, (config.rw == AUXI2C_READ ? "read" : "write"), config.addr, config.reg_addr,
        config.reg_dis, (config.rw == AUXI2C_READ ? "rxlength" : "txdata"), config.txdata, config.sample_delay_en);
    return err;
}

auxi2c_slv_config_t MPU::getAuxI2CSlaveConfig(auxi2c_slv_t slave) {
    auxi2c_slv_config_t config;
    const uint8_t regAddr = slave * 3 + regs::I2C_SLV0_ADDR;
    config.slave = slave;
    MPU_ERR_CHECK(readBytes(regAddr, 3, buffer));
    config.rw = (auxi2c_rw_t) ((buffer[0] >> regs::I2C_SLV_RNW_BIT) & 0x1);
    config.addr = buffer[0] & 0x7F;
    config.reg_addr = buffer[1];
    config.reg_dis = (buffer[2] >> regs::I2C_SLV_REG_DIS_BIT) & 0x1;
    if (config.rw == AUXI2C_READ) {
        config.swap_en = (buffer[2] >> regs::I2C_SLV_BYTE_SW_BIT) & 0x1;
        config.end_of_word = (auxi2c_eow_t) ((buffer[2] >> regs::I2C_SLV_GRP_BIT) & 0x1);
        config.rxlength = buffer[2] & 0xF;
    } else {
        MPU_ERR_CHECK(readByte(regs::I2C_SLV0_DO + slave, buffer+3));
        config.txdata = buffer[3];
    }
    MPU_ERR_CHECK(readByte(regs::I2C_MST_DELAY_CRTL, buffer+4));
    config.sample_delay_en = (buffer[4] >> slave) & 0x1;
    return config;
}


/**
 * Enable the Auxiliary I2C module to tranfer data with a slave at sample rate.
 * */
esp_err_t MPU::setAuxI2CSlaveEnabled(auxi2c_slv_t slave, bool enable) {
    const uint8_t regAddr = slave * 3 + regs::I2C_SLV0_CTRL;
    return MPU_ERR_CHECK(writeBit(regAddr, regs::I2C_SLV_EN_BIT, enable));
}

bool MPU::getAuxI2CSlaveEnabled(auxi2c_slv_t slave) {
    const uint8_t regAddr = slave * 3 + regs::I2C_SLV0_CTRL;
    MPU_ERR_CHECK(readBit(regAddr, regs::I2C_SLV_EN_BIT, buffer));
    return buffer[0];
}


/**
 * Note:
 * when enable = true, Auxiliar I2C Master is disabled, and Bypass enabled
 * when enable = false, Bypass is disabled, but the Auxiliar I2C Master is not enabled back,
 *   case needed, must be enabled again with setAuxI2CmasterEnabled();
 * */
esp_err_t MPU::setAuxI2CBypass(bool enable) {
    #ifdef CONFIG_MPU_SPI
    if (enable) {
        MPU_LOGWMSG(msgs::EMPTY, "Setting Aux I2C to bypass mode while MPU is connected via SPI");
    }
    #endif
    if (enable) {
        if (MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_EN_BIT, 0)))
            return err;
    }
    if (MPU_ERR_CHECK(writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_I2C_BYPASS_EN_BIT, enable)))
        return err;
    if (enable) {
        MPU_LOGVMSG(msgs::AUX_I2C_DISABLED, ", entering Bypass mode.");
    } else {
        MPU_LOGVMSG(msgs::EMPTY, "Leaving Bypass mode.");
    }
    return err;
}

bool MPU::getAuxI2CBypass() {
    MPU_ERR_CHECK(readBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_EN_BIT, buffer));
    MPU_ERR_CHECK(readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_I2C_BYPASS_EN_BIT, buffer+1));
    return (!buffer[0]) && buffer[1];
}


/**
 * Read data from slaves conected to Auxiliar I2C bus
 * 
 * Note:
 * Data is placed in these external sensor data registers according to I2C_SLV0_CTRL,
 * I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39, 42, 45, and 48). When
 * more than zero bytes are read (I2C_SLVx_LEN > 0) from an enabled slave (I2C_SLVx_EN = 1), the
 * slave is read at the Sample Rate (as defined in Register 25) or delayed rate (if specified in Register
 * 52 and 103). During each sample cycle, slave reads are performed in order of Slave number. If all
 * slaves are enabled with more than zero bytes to be read, the order will be Slave 0, followed by Slave
 * 1, Slave 2, and Slave 3.
 * 
 * If the sum of the read lengths of all SLVx transactions exceed the number of available
 * EXT_SENS_DATA registers, the excess bytes will be dropped. There are 24 EXT_SENS_DATA
 * registers and hence the total read lengths between all the slaves cannot be greater than 24 or some
 * bytes will be lost.
 * 
 * Note: set skip to 8 when using compass, compass data takes up the first 8 bytes.
 * */
esp_err_t MPU::readAuxI2CRxData(size_t length, uint8_t *data, size_t skip) {
    if (length + skip > 24) {
        MPU_LOGEMSG(msgs::INVALID_LENGTH, " %d, mpu has only 24 external sensor data registers!", length);
        return err = ESP_ERR_INVALID_SIZE;
    }
    // check if I2C Master is enabled, just for warning and debug
    #if CONFIG_MPU_LOG_LEVEL >= ESP_LOG_WARN
    const bool kAuxI2CEnabled = getAuxI2CEnabled();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    if (!kAuxI2CEnabled)
        MPU_LOGWMSG(msgs::AUX_I2C_DISABLED, ", better turn on.");
    #endif
    // read the specified amount of registers
    return MPU_ERR_CHECK(readBytes(regs::EXT_SENS_DATA_00 + skip, length, data));
}


/**
 * Restart Auxiliary I2C Master module, reset is asynchronous.
 * Note: This bit (I2C_MST_RST) should only be set when the I2C master has hung. If this bit
 * is set during an active I2C master transaction, the I2C slave will hang, which
 * will require the host to reset the slave.
 * */
esp_err_t MPU::restartAuxI2C() {
    return MPU_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_RESET_BIT, 1));
}


/**
 * Note: reading the register I2C_MST_STATUS clear all its bits
 * */
auxi2c_stat_t MPU::getAuxI2CStatus() {
    MPU_ERR_CHECK(readByte(regs::I2C_MST_STATUS, buffer));
    return (auxi2c_stat_t) buffer[0];
}


/**
 * Write to a slave a single byte just once (use for configuring a slave at initialization)
 * This function uses Slave 4 to perform single transfers to the slave device on Aux I2C.
 * 
 * Note: Auxiliary I2C Master must have already been configured before calling this function.
 * Note: The byte will be transfered at first sample take, so when sample rate is at minimum (4 Hz)
 *  it may take up to a quarter of a second to make the transfer.
 * 
 * @return 
 *      - ESP_ERR_INVALID_STATE  Auxiliary I2C Master not enabled
 *      - ESP_ERR_NOT_FOUND      Slave doesn't ACK the transfer
 *      - ESP_FAIL               Auxiliary I2C Master lost arbitration of the bus
 *      - + standard i2c driver error codes
 * */
esp_err_t MPU::auxI2CWriteByte(uint8_t devAddr, uint8_t regAddr, const uint8_t data) {
    // check for Aux I2C master enabled first
    const bool kAuxI2CEnabled = getAuxI2CEnabled();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    if (!kAuxI2CEnabled) {
        MPU_LOGEMSG(msgs::AUX_I2C_DISABLED, ", must enable first");
        return err = ESP_ERR_INVALID_STATE;
    }
    // data for regs::I2C_SLV4_ADDR
    buffer[0] = AUXI2C_WRITE << regs::I2C_SLV_RNW_BIT;
    buffer[0] |= devAddr & (0x7F);
    // data for regs::I2C_SLV4_REG
    buffer[1] = regAddr;
    // data for regs::I2C_SLV4_DO
    buffer[2] = data;
    // write configuration above to slave 4 registers
    if (MPU_ERR_CHECK(writeBytes(regs::I2C_SLV4_ADDR, 3, buffer)))
        return err;
    // clear status register before enable this transfer
    if (MPU_ERR_CHECK(readByte(regs::I2C_MST_STATUS, buffer+15)))
        return err;
    // enable transfer in slave 4
    if (MPU_ERR_CHECK(writeBit(regs::I2C_SLV4_CTRL, regs::I2C_SLV4_EN_BIT, 1)))
        return err;
    // check status until transfer is done
    TickType_t startTick = xTaskGetTickCount();
    TickType_t endTick = startTick + pdMS_TO_TICKS(1000);
    auxi2c_stat_t status;
    do {
        if (MPU_ERR_CHECK(readByte(regs::I2C_MST_STATUS, &status)))
            return err;
        if (status & (1 << regs::I2CMST_STAT_SLV4_NACK_BIT)) {
            MPU_LOGWMSG(msgs::AUX_I2C_SLAVE_NACK, "");
            return err = ESP_ERR_NOT_FOUND;
        }
        if (status & (1 << regs::I2CMST_STAT_LOST_ARB_BIT)) {
            MPU_LOGWMSG(msgs::AUX_I2C_LOST_ARB, "");
            return err = ESP_FAIL;
        }
        if (xTaskGetTickCount() >= endTick) {
            MPU_LOGEMSG(msgs::TIMEOUT, ". Aux I2C might've hung. Restart it.");
            return err = ESP_ERR_TIMEOUT;
        }
    } while (!(status & (1 << regs::I2C_SLV4_DONE_INT_BIT)));

    return err = ESP_OK;
}


/**
 * Read a single byte frome slave just once (use for configuring a slave at initialization)
 * This function uses Slave 4 to perform single transfers to the slave device on Aux I2C.
 * 
 * Note: Auxiliary I2C Master must have already been configured before calling this function.
 * Note: The byte will be transfered at first sample take, so when sample rate is at minimum (4 Hz)
 *  it may take up to a quarter of a second to make the transfer.
 * 
 * @return 
 *      - ESP_ERR_INVALID_STATE  Auxiliary I2C Master not enabled
 *      - ESP_ERR_NOT_FOUND      Slave doesn't ACK the transfer
 *      - ESP_FAIL               Auxiliary I2C Master lost arbitration of the bus
 *      - + standard i2c driver error codes
 * */
esp_err_t MPU::auxI2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
    // check for Aux I2C master enabled first
    const bool kAuxI2CEnabled = getAuxI2CEnabled();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    if (!kAuxI2CEnabled) {
        MPU_LOGEMSG(msgs::AUX_I2C_DISABLED, ", must enable first");
        return err = ESP_ERR_INVALID_STATE;
    }
    // data for regs::I2C_SLV4_ADDR
    buffer[0] = AUXI2C_READ << regs::I2C_SLV_RNW_BIT;
    buffer[0] |= devAddr & (0x7F);
    // data for regs::I2C_SLV4_REG
    buffer[1] = regAddr;
    // write configuration above to slave 4 registers
    if (MPU_ERR_CHECK(writeBytes(regs::I2C_SLV4_ADDR, 2, buffer)))
        return err;
    // clear status register before enable this transfer
    if (MPU_ERR_CHECK(readByte(regs::I2C_MST_STATUS, buffer+15)))
        return err;
    // enable transfer in slave 4
    if (MPU_ERR_CHECK(writeBit(regs::I2C_SLV4_CTRL, regs::I2C_SLV4_EN_BIT, 1)))
        return err;
    // check status until transfer is done
    TickType_t startTick = xTaskGetTickCount();
    TickType_t endTick = startTick + pdMS_TO_TICKS(1000);
    auxi2c_stat_t status;
    do {
        if (MPU_ERR_CHECK(readByte(regs::I2C_MST_STATUS, &status)))
            return err;
        if (status & (1 << regs::I2CMST_STAT_SLV4_NACK_BIT)) {
            MPU_LOGWMSG(msgs::AUX_I2C_SLAVE_NACK, "");
            return err = ESP_ERR_NOT_FOUND;
        }
        if (status & (1 << regs::I2CMST_STAT_LOST_ARB_BIT)) {
            MPU_LOGWMSG(msgs::AUX_I2C_LOST_ARB, "");
            return err = ESP_FAIL;
        }
        if (xTaskGetTickCount() >= endTick) {
            MPU_LOGEMSG(msgs::TIMEOUT, ". Aux I2C might've hung. Restart it.");
            return err = ESP_ERR_TIMEOUT;
        }
    } while (!(status & (1 << regs::I2C_SLV4_DONE_INT_BIT)));
    // get read value
    return MPU_ERR_CHECK(readByte(regs::I2C_SLV4_DI, data));
}


/**
 * Configure the active level of FSYNC pin that will cause an interrupt.
 * Use setFsyncEnabled() to enable / disable this interrupt.
 * */
esp_err_t MPU::setFsyncConfig(int_lvl_t level) {
    return MPU_ERR_CHECK(writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_LEVEL_BIT, level));
}

int_lvl_t MPU::getFsyncConfig() {
    MPU_ERR_CHECK(readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_LEVEL_BIT, buffer));
    return (int_lvl_t) buffer[0];
}


/**
 * Enable / disable FSYNC pin to cause an interrupt
 * @see setFsyncConfig()
 * 
 * Note: the interrupt status is located in I2C_MST_STATUS register, so use
 *  the method getAuxI2CStatus() which reads this register to get FSYNC status.
 *  Keep in mind that a read from I2C_MST_STATUS register clears all its status bits,
 *  so take care to miss status bits when using Auxiliary I2C bus too.
 * 
 * Note: It is possible to enable the FSYNC interrupt propagate to INT pin
 *  with setInterruptEnabled(), then the status can also be read with getInterruptStatus().
 * */

esp_err_t MPU::setFsyncEnabled(bool enable) {
    return MPU_ERR_CHECK(writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_INT_MODE_EN_BIT, enable));
}

bool MPU::getFsyncEnabled() {
    MPU_ERR_CHECK(readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_INT_MODE_EN_BIT, buffer));
    return buffer[0];
}


/**
 * Register dump
 * */
esp_err_t MPU::registerDump(uint8_t start, uint8_t end) {
    constexpr uint8_t kNumOfRegs = 128;
    if (end - start < 0 || start >= kNumOfRegs || end >= kNumOfRegs)
        return err = ESP_FAIL;
    printf(LOG_COLOR_W ">> " CONFIG_MPU_CHIP_MODEL " register dump:" LOG_RESET_COLOR "\n");
    uint8_t data;
    for (int i = start; i <= end; i++) {
        if (MPU_ERR_CHECK(readByte(i, &data))) {
            MPU_LOGEMSG("", "Reading Error.");
            return err;
        }
        printf("MPU: reg[ 0x%s%X ]  data( 0x%s%X )\n",
            i < 0x10 ? "0" : "", i, data < 0x10 ? "0" : "", data);
    }
    return err;
}


#if defined CONFIG_MPU_AK89xx
/**
 * Read a single byte from magnetometer
 * 
 * Note: it will check the communication protocol which the MPU is connected by,
 * in case of I2C, Auxiliary I2C bus will set to bypass mode and the reading will be performed directly (faster).
 * in case of SPI, the function will use Slave 4 of Auxiliary I2C bus to read the byte (slower).
 * */
esp_err_t MPU::compassReadByte(uint8_t regAddr, uint8_t *data) {
    // in case of I2C
    #if defined CONFIG_MPU_I2C
    const bool kPrevAuxI2CBypassState = getAuxI2CBypass();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    if (kPrevAuxI2CBypassState == false)
        if (MPU_ERR_CHECK(setAuxI2CBypass(true)))
            return err;
    if (MPU_ERR_CHECK(bus->readByte(COMPASS_I2CADDRESS, regAddr, data)))
        return err;
    if (kPrevAuxI2CBypassState == false)
        if (MPU_ERR_CHECK(setAuxI2CBypass(false)))
            return err;
    // in case of SPI
    #elif defined CONFIG_MPU_SPI
    return MPU_ERR_CHECK(auxI2CReadByte(COMPASS_I2CADDRESS, regAddr, data));
    #endif
    return err;
}

/**
 * Write a single byte to magnetometer
 * 
 * Note: it will check the communication protocol which the MPU is connected by,
 * in case of I2C, Auxiliary I2C bus will set to bypass mode and the reading will be performed directly (faster).
 * in case of SPI, the function will use Slave 4 of Auxiliary I2C bus to read the byte (slower).
 * */
esp_err_t MPU::compassWriteByte(uint8_t regAddr, const uint8_t data) {
    // in case of I2C
    #if defined CONFIG_MPU_I2C
    const bool kPrevAuxI2CBypassState = getAuxI2CBypass();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    if (kPrevAuxI2CBypassState == false)
        if (MPU_ERR_CHECK(setAuxI2CBypass(true)))
            return err;
    if (MPU_ERR_CHECK(bus->writeByte(COMPASS_I2CADDRESS, regAddr, data)))
        return err;
    if (kPrevAuxI2CBypassState == false)
        if (MPU_ERR_CHECK(setAuxI2CBypass(false)))
            return err;
    // in case of SPI
    #elif defined CONFIG_MPU_SPI
    return MPU_ERR_CHECK(auxI2CWriteByte(COMPASS_I2CADDRESS, regAddr, data));
    #endif
    return err;
}


/**
 * Initialize compass/magnetometer, initial configuration:
 *  - Mode: single measurement (permits variable sample rate)
 *  - Sensitivity: 0.15 uT/LSB  =  16-bit output
 * 
 * Note: to disable the compass: call compassSetMode(MAG_MODE_POWER_DOWN)
 * */
esp_err_t MPU::compassInit() {
    #ifdef CONFIG_MPU_I2C
    if (MPU_ERR_CHECK(setAuxI2CBypass(true)))
        return err;

    #elif CONFIG_MPU_SPI
    constexpr auxi2c_config_t kAuxI2CConfig = {
        .clock = AUXI2C_CLOCK_400KHZ,
        .multi_master_en = 1,
        .sample_delay = 0,
        .shadow_delay_en = 0,
        .wait_for_es = 0,
        .transition = AUXI2C_TRANS_RESTART
    };
    if (MPU_ERR_CHECK(setAuxI2CConfig(kAuxI2CConfig)))
        return err;
    if (MPU_ERR_CHECK(setAuxI2CEnabled(true)))
        return err;
    #endif

    /* configure the magnetometer */
    #ifdef CONFIG_MPU_AK8963
    if (MPU_ERR_CHECK(compassReset()))
        return err;
    #endif
    if (MPU_ERR_CHECK(compassSetMode(MAG_MODE_SINGLE_MEASURE)))
        return err;

    // finished configs, disable bypass mode
    #ifdef CONFIG_MPU_I2C
    if (MPU_ERR_CHECK(setAuxI2CBypass(false)))
        return err;
    #endif

    /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD (no idea why) */
    #ifdef CONFIG_MPU9150
    if (MPU_ERR_CHECK(setAuxVDDIOLevel(AUXVDDIO_LVL_VDD)))
        return err;
    #endif

    MPU_LOGV("Magnetometer configured successfully");
    return err;
}

/**
 * Test connection with Magnetometer by checking WHO_I_AM register
 * */
esp_err_t MPU::compassTestConnection() {
    const uint8_t wai = compassWhoAmI();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    return (wai == 0x48) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/**
 * Return value from WHO_I_AM register
 * should be 0x48 for AK8963 and AK8975
 * */
uint8_t MPU::compassWhoAmI() {
    MPU_ERR_CHECK(compassReadByte(regs::mag::WHO_I_AM, buffer));
    return buffer[0];
}


/**
 * Return value from magnetometer's INFO register
 * */
uint8_t MPU::compassGetInfo() {
    MPU_ERR_CHECK(compassReadByte(regs::mag::INFO, buffer));
    return buffer[0];
}


/**
 * Change magnetometer's measurement mode
 * 
 * Note: setting to MAG_MODE_POWER_DOWN will disable readings 
 *  from compass and disable (free) Aux I2C slaves 0 and 1.
 *  It will not disable Aux I2C Master I/F though!
 *  To enable back, use compassInit()
 * */
esp_err_t MPU::compassSetMode(mag_mode_t mode) {
    // keep previous sensitivity value
    #if defined CONFIG_MPU_AK8963
    const uint8_t kControl1Value = mode | (compassGetSensitivity() << regs::mag::CONTROL1_BIT_OUTPUT_BIT);
    if (MPU_ERR_CHECK(lastError()))
        return err;
    #else
    const uint8_t kControl1Value = mode;
    #endif
    /* POWER-DOWN */
    if (mode == MAG_MODE_POWER_DOWN) {
        if (MPU_ERR_CHECK(setAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE, false)))
            return err;
        if (MPU_ERR_CHECK(setAuxI2CSlaveEnabled(MAG_SLAVE_READ_DATA, false)))
            return err;
        if (MPU_ERR_CHECK(compassWriteByte(regs::mag::CONTROL1, kControl1Value)))
            return err;
        MPU_LOGVMSG(msgs::COMPASS_DISABLED, ", set to power-down mode");

    /* SINGLE MEASUREMENT */
    } else if (mode == MAG_MODE_SINGLE_MEASURE) {
        // set to power-down first
        if (MPU_ERR_CHECK(compassSetMode(MAG_MODE_POWER_DOWN)))
            return err;
        // set to single measurement
        if (MPU_ERR_CHECK(compassWriteByte(regs::mag::CONTROL1, kControl1Value)))
            return err;

        // slave 0 reads from magnetometer data register
        const auxi2c_slv_config_t kSlaveReadDataConfig = {
            .slave = MAG_SLAVE_READ_DATA,
            .addr = COMPASS_I2CADDRESS,
            .rw = AUXI2C_READ,
            .reg_addr = regs::mag::STATUS1,
            .reg_dis = 0,
            .sample_delay_en = 1,
            {
                {
                .swap_en = 0,
                .end_of_word = (auxi2c_eow_t) 0,
                .rxlength = 8
                }
            }
        };
        if (MPU_ERR_CHECK(setAuxI2CSlaveConfig(kSlaveReadDataConfig)))
            return err;

        // slave 1 changes mode to single measurement
        const auxi2c_slv_config_t kSlaveChgModeConfig = {
            .slave = MAG_SLAVE_CHG_MODE,
            .addr = COMPASS_I2CADDRESS,
            .rw = AUXI2C_WRITE,
            .reg_addr = regs::mag::CONTROL1,
            .reg_dis = 0,
            .sample_delay_en = 1,
            {.txdata = kControl1Value}
        };
        if (MPU_ERR_CHECK(setAuxI2CSlaveConfig(kSlaveChgModeConfig)))
            return err;
        // enable slaves
        if (MPU_ERR_CHECK(setAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE, true)))
            return err;
        if (MPU_ERR_CHECK(setAuxI2CSlaveEnabled(MAG_SLAVE_READ_DATA, true)))
            return err;
        MPU_LOGVMSG(msgs::EMPTY, "magnetometer mode set to single measurement");

    /* SELF-TEST */
    } else if (mode == MAG_MODE_SELF_TEST) {
        // set to power-down first
        if (MPU_ERR_CHECK(compassSetMode(MAG_MODE_POWER_DOWN)))
            return err;
        // set to self-test
        if (MPU_ERR_CHECK(compassWriteByte(regs::mag::CONTROL1, kControl1Value)))
            return err;
        MPU_LOGVMSG(msgs::EMPTY, "magnetometer mode set to self-test");

    /* FUSE-ROM ACCESS */
    } else if (mode == MAG_MODE_FUSE_ROM) {
        // set to power-down first
        if (MPU_ERR_CHECK(compassSetMode(MAG_MODE_POWER_DOWN)))
            return err;
        // set to fuse rom
        if (MPU_ERR_CHECK(compassWriteByte(regs::mag::CONTROL1, kControl1Value)))
            return err;
        MPU_LOGVMSG(msgs::EMPTY, "magnetometer mode set to fuse ROM");

    /* OTHER MODES */
    } else {
        MPU_LOGEMSG(msgs::NOT_SUPPORTED, " yet");
        return err = ESP_ERR_NOT_SUPPORTED;
    }

    return err = ESP_OK;
}

/**
 * Return magnetometer's measurement mode
 * */
mag_mode_t MPU::compassGetMode() {
    const auxi2c_slv_config_t kSlaveChgModeConfig = getAuxI2CSlaveConfig(MAG_SLAVE_CHG_MODE);
    MPU_ERR_CHECK(lastError());
    const bool kSlaveChgModeEnabled = getAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE);
    MPU_ERR_CHECK(lastError());
    mag_mode_t mode;
    // check if slave 1 is writing the mode to compass
    if (kSlaveChgModeEnabled && kSlaveChgModeConfig.addr == COMPASS_I2CADDRESS) {
        mode = (mag_mode_t) (kSlaveChgModeConfig.txdata & 0xF);
    // otherwise, get directly from the magnetometer register
    } else {
        MPU_ERR_CHECK(compassReadByte(regs::mag::CONTROL1, buffer));
        mode = (mag_mode_t) (buffer[0] & 0xF);
    }
    return mode;
}


/**
 * Return Magnetometer's sensitivity adjustment data for each axis
 * */
esp_err_t MPU::compassGetAdjustment(uint8_t* x, uint8_t* y, uint8_t* z) {
    mag_mode_t prevMode = compassGetMode();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    if (MPU_ERR_CHECK(compassSetMode(MAG_MODE_FUSE_ROM)))
        return err;
    if (MPU_ERR_CHECK(compassReadByte(regs::mag::ASAX, x)))
        return err;
    if (MPU_ERR_CHECK(compassReadByte(regs::mag::ASAY, y)))
        return err;
    if (MPU_ERR_CHECK(compassReadByte(regs::mag::ASAZ, z)))
        return err;
    // set back previous mode
    return MPU_ERR_CHECK(compassSetMode(prevMode));
}


/**
 * Compass self-test
 * */
bool MPU::compassSelfTest(raw_axes_t *result) {
    bool ret = true;
    mag_mode_t prevMode = compassGetMode();
    if (MPU_ERR_CHECK(lastError()))
        return err;
    uint8_t adjValue[3];
    MPU_ERR_CHECK(compassGetAdjustment(adjValue, adjValue+1, adjValue+2));
    // MPU_LOGD("adjValue: %d %d %d", adjValue[0], adjValue[1], adjValue[2]);
    MPU_ERR_CHECK(compassSetMode(MAG_MODE_POWER_DOWN));
    // read all data to reset status for any case, start clean
    for (int i = 0; i < 8; i++)
        MPU_ERR_CHECK(compassReadByte(regs::mag::STATUS1 + i, buffer));
    MPU_ERR_CHECK(compassWriteByte(regs::mag::ASTC, (1 << regs::mag::ASTC_SELF_TEST_BIT)));
    MPU_ERR_CHECK(compassSetMode(MAG_MODE_SELF_TEST));
    // wait for data-ready
    uint8_t status1;
    do {
        MPU_ERR_CHECK(compassReadByte(regs::mag::STATUS1, &status1));
    } while (!((status1 >> regs::mag::STATUS1_DATA_RDY_BIT) & 0x1));
    MPU_LOGD("status1: %#X", status1);
    for (int i = 0; i < 7; i++)
        MPU_ERR_CHECK(compassReadByte(regs::mag::HXL + i, buffer + i));
    MPU_LOGD("status2: %#X", buffer[6]);
    // convert data
    raw_axes_t data;
    if (result == nullptr)
        result = &data;
    result->x = math::magAdjust(buffer[1] << 8 | buffer[0], adjValue[0]);
    result->y = math::magAdjust(buffer[3] << 8 | buffer[2], adjValue[1]);
    result->z = math::magAdjust(buffer[5] << 8 | buffer[4], adjValue[2]);
    MPU_LOGD("raw self-test values: %+d %+d %+d", buffer[1] << 8 | buffer[0], buffer[3] << 8 | buffer[2], buffer[5] << 8 | buffer[4]);
    // check self-test data
    #if defined CONFIG_MPU_AK8975
    constexpr int16_t HX_MIN = -100;
    constexpr int16_t HX_MAX =  100;
    constexpr int16_t HY_MIN = -100;
    constexpr int16_t HY_MAX =  100;
    constexpr int16_t HZ_MIN = -1000;
    constexpr int16_t HZ_MAX = -300;
    if (result->x < HX_MIN || result->x > HX_MAX ||
        result->y < HY_MIN || result->y > HY_MAX ||
        result->z < HZ_MIN || result->z > HZ_MAX ) {
        ret = false;
    }
    #elif defined CONFIG_MPU_AK8963
    // HX_MIN[0] = 14-bit, [1] = 16-bit
    constexpr int16_t HX_MIN[2] = {-50, -200};
    constexpr int16_t HX_MAX[2] = { 50,  200};
    constexpr int16_t HY_MIN[2] = {-50, -200};
    constexpr int16_t HY_MAX[2] = { 50,  200};
    constexpr int16_t HZ_MIN[2] = {-800, -3200};
    constexpr int16_t HZ_MAX[2] = {-200, -800};
    mag_sensy_t sensy = compassGetSensitivity();
    MPU_ERR_CHECK(lastError());
    if (result->x < HX_MIN[sensy] || result->x > HX_MAX[sensy] ||
        result->y < HY_MIN[sensy] || result->y > HY_MAX[sensy] ||
        result->z < HZ_MIN[sensy] || result->z > HZ_MAX[sensy] ) {
        ret = false;
    }
    #endif
    // finish test
    MPU_ERR_CHECK(compassWriteByte(regs::mag::ASTC, 0x0));
    MPU_ERR_CHECK(compassSetMode(prevMode));
    return ret;
}


#ifdef CONFIG_MPU_AK8963
/**
 * Set magnetometer sensitivity
 * options:
 * - MAG_SENSITIVITY_0_6_uT  = 0,  // 0.6  uT/LSB  =  14-bit output
 * - MAG_SENSITIVITY_0_15_uT = 1,  // 0.15 uT/LSB  =  16-bit output
 * */
esp_err_t MPU::compassSetSensitivity(mag_sensy_t sensy) {
    auxi2c_slv_config_t slaveChgModeConfig = getAuxI2CSlaveConfig(MAG_SLAVE_CHG_MODE);
    if (MPU_ERR_CHECK(lastError()))
        return err;
    slaveChgModeConfig.txdata &= ~(1 << regs::mag::CONTROL1_BIT_OUTPUT_BIT);
    slaveChgModeConfig.txdata |= (sensy & 0x1) << regs::mag::CONTROL1_BIT_OUTPUT_BIT;
    if (MPU_ERR_CHECK(setAuxI2CSlaveConfig(slaveChgModeConfig)))
        return err;
    // write directly (assurance)
    return MPU_ERR_CHECK(compassWriteByte(regs::mag::CONTROL1, slaveChgModeConfig.txdata));
}

/**
 * Return magnetometer sensitivity
 * */
mag_sensy_t MPU::compassGetSensitivity() {
    auxi2c_slv_config_t slaveChgModeConfig = getAuxI2CSlaveConfig(MAG_SLAVE_CHG_MODE);
    MPU_ERR_CHECK(lastError());
    const bool kSlaveChgModeEnabled = getAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE);
    MPU_ERR_CHECK(lastError());
    mag_sensy_t sensy;
    // get from slave 1 config, if enabled
    if (kSlaveChgModeEnabled && slaveChgModeConfig.addr == COMPASS_I2CADDRESS) {
        sensy = (mag_sensy_t) ((slaveChgModeConfig.txdata >> regs::mag::CONTROL1_BIT_OUTPUT_BIT) & 0x1);
    // otherwise, get directly
    } else {
        MPU_ERR_CHECK(compassReadByte(regs::mag::CONTROL1, buffer));
        sensy = (mag_sensy_t) ((buffer[0] >> regs::mag::CONTROL1_BIT_OUTPUT_BIT) & 0x1);
    }
    return sensy;
}


/**
 * Soft reset AK8963
 * */
esp_err_t MPU::compassReset() {
    return MPU_ERR_CHECK(compassWriteByte(regs::mag::CONTROL2, 0x1));
}

#endif  // ak8963 stuff
#endif  // compass methods



}  // namespace mpu

}  // namespace emd
