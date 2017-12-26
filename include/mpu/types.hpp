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

#ifndef _MPU_TYPES_HPP_
#define _MPU_TYPES_HPP_

#include <stdint.h>
#include "sdkconfig.h"
#include "mpu/registers.hpp"


/* ^^^^^^^^^^^^^^^^^^^^^^
 * Embedded Motion Driver
 * ^^^^^^^^^^^^^^^^^^^^^^ */
namespace emd {

/* ^^^^^^^^^^^^^^^^^^^^^
 * Motion Processor Unit
 * ^^^^^^^^^^^^^^^^^^^^^ */
namespace mpu {

inline namespace types {
// MPU's I2C slave address
typedef enum {
    MPU_I2CADDRESS_AD0_LOW = 0x68,
    MPU_I2CADDRESS_AD0_HIGH = 0x69
} mpu_i2caddr_t;
static constexpr mpu_i2caddr_t MPU_DEFAULT_I2CADDRESS = MPU_I2CADDRESS_AD0_LOW;

// Communication Bus
#ifdef CONFIG_MPU_I2C
using mpu_bus_t = I2C_t;
using mpu_addr_handle_t = mpu_i2caddr_t;
static constexpr mpu_bus_t& MPU_DEFAULT_BUS = i2c0;
static constexpr mpu_addr_handle_t MPU_DEFAULT_ADDR_HANDLE = MPU_DEFAULT_I2CADDRESS;
#elif CONFIG_MPU_SPI
using mpu_bus_t = SPI_t;
using mpu_addr_handle_t = spi_device_handle_t;
static constexpr mpu_bus_t& MPU_DEFAULT_BUS = hspi;
static constexpr mpu_addr_handle_t MPU_DEFAULT_ADDR_HANDLE = nullptr;
#endif

// Maximum gyroscope sample rate
#if defined CONFIG_MPU6050
static constexpr uint16_t SAMPLE_RATE_MAX = 8000;
#elif defined CONFIG_MPU6500
static constexpr uint16_t SAMPLE_RATE_MAX = 32000;
#endif

// Gyro full-scale range
typedef enum {
    GYRO_FS_250DPS  = 0,
    GYRO_FS_500DPS  = 1,
    GYRO_FS_1000DPS = 2,
    GYRO_FS_2000DPS = 3
} gyro_fs_t;

// Accel full-scale range
typedef enum {
    ACCEL_FS_2G  = 0,
    ACCEL_FS_4G  = 1,
    ACCEL_FS_8G  = 2,
    ACCEL_FS_16G = 3
} accel_fs_t;

// Digital low-pass filter (based on gyro bandwidth)
typedef enum {
    DLPF_256HZ_NOLPF  = 0,
    DLPF_188HZ = 1,
    DLPF_98HZ  = 2,
    DLPF_42HZ  = 3,
    DLPF_20HZ  = 4,
    DLPF_10HZ  = 5,
    DLPF_5HZ   = 6,
#ifdef CONFIG_MPU6050
    DLPF_2100HZ_NOLPF = 7
#elif CONFIG_MPU6500
    DLPF_3600HZ_NOLPF = 7
#endif
} dlpf_t;

// Clock source
typedef enum {
    CLOCK_INTERNAL = 0,  // 20MHz for MPU6500 and 8MHz for MPU6050
    CLOCK_PLL = 3,  // Selects automatically best pll source (recommended)
#if defined CONFIG_MPU6050
    CLOCK_EXT32KHZ = 4,
    CLOCK_EXT19MHZ = 5,
#endif
    CLOCK_KEEP_RESET = 7
} clock_src_t;

// Fchoice (Frequency choice maybe ?) [MPU6500 and MPU9250 only]
#ifdef CONFIG_MPU6500
typedef enum {
    FCHOICE_0 = 0,
    FCHOICE_1 = 1,
    FCHOICE_2 = 2,
    FCHOICE_3 = 3
} fchoice_t;
#endif

// Low-power accelerometer wake-up rates
typedef enum {
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    LP_ACCEL_1_25HZ = 0,
    LP_ACCEL_5HZ    = 1,
    LP_ACCEL_20HZ   = 2,
    LP_ACCEL_40HZ   = 3
#elif defined CONFIG_MPU6500 || defined CONFIG_MPU9250
    LP_ACCEL_0_24HZ  = 0,
    LP_ACCEL_0_49HZ  = 1,
    LP_ACCEL_0_98HZ  = 2,
    LP_ACCEL_1_95HZ  = 3,
    LP_ACCEL_3_91HZ  = 4,
    LP_ACCEL_7_81HZ  = 5,
    LP_ACCEL_15_63HZ = 6,
    LP_ACCEL_31_25HZ = 7,
    LP_ACCEL_62_50HZ = 8,
    LP_ACCEL_125HZ   = 9,
    LP_ACCEL_250HZ   = 10,
    LP_ACCEL_500HZ   = 11
#endif
} lp_accel_rate_t;

// Accelerometer Digital High Pass Filter (only for motion detectors modules)
typedef enum {
    ACCEL_DHPF_RESET  = 0,  // This effectively disables the high pass filter. This mode may be toggled to quickly settle the filter.
    ACCEL_DHPF_5HZ    = 1,  // ON :
    ACCEL_DHPF_2_5HZ  = 2,  // ON :
    ACCEL_DHPF_1_25HZ = 3,  // ON :
    ACCEL_DHPF_0_63HZ = 4,  // ON state, the high pass filter will pass signals above the cut off frequency.
    ACCEL_DHPF_HOLD   = 7,  // The filter holds the present sample. The output will be the difference between the input sample and the held sample.
} accel_dhpf_t;

// Motion Detection counter decrement rate (Motion and FreeFall)
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
typedef enum {
    MOT_COUNTER_RESET = 0,  // when set, any non-qualifying sample will reset the corresponding counter to 0
    MOT_COUNTER_DEC_1 = 1,  // decrement counter in 1
    MOT_COUNTER_DEC_2 = 2,  // decrement counter in 2
    MOT_COUNTER_DEC_4 = 3   // decrement counter in 4
} mot_counter_t;
#endif

// Motion Detection configuration
typedef struct {
    uint8_t threshold;     // Motion threshold in LSB.
                           //  For MPU6000 / MPU6050 / MPU9150: 1LSB = 32mg, 255LSB = 8160mg
                           //  For MPU6500 / MPU9250: 1LSB = 4mg, 255LSB = 1020mg
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    uint8_t time;          // Duration in milliseconds that the accel data must exceed
                           //  the threshold before motion is reported. MAX = 255ms
    uint8_t accel_on_delay :2;  // Specifies in milliseconds the additional power-on delay applied to accelerometer data path modules. MAX = 3ms
                                //  More: The signal path contains filters which must be flushed on wake-up with new samples before
                                //  the detection modules begin operations. There is already a default built-in 4ms delay.
    mot_counter_t counter  :2;  // Configures the detection counter decrement rate.
#endif
} mot_config_t;

// Zero-motion configuration
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
typedef struct {
    uint8_t threshold;     // Motion threshold in LSB. 1LSB = 1mg, 255LSB = 1020mg
    uint8_t time;          // Duration in milliseconds that the accel data must exceed
                           //  the threshold before motion is reported. MAX = 255ms
} zrmot_config_t;
#endif

// Free-fall configuration
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
typedef struct {
    uint8_t threshold;     // Motion threshold in LSB. 1LSB = 1mg, 255LSB = 1020mg
    uint8_t time;          // Duration in milliseconds that the accel data must exceed
                           //  the threshold before motion is reported. MAX = 255ms
    uint8_t accel_on_delay :2;  // Specifies in milliseconds the additional power-on delay applied to accelerometer data path modules. MAX = 3ms
                                //  More: The signal path contains filters which must be flushed on wake-up with new samples before
                                //  the detection modules begin operations. There is already a default built-in 4ms delay.
    mot_counter_t counter  :2;  // Configures the detection counter decrement rate.
} ff_config_t;
#endif

// Motion Detection Status (MPU6000, MPU6050, MPU9150)
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
using mot_stat_t = uint8_t;
static constexpr mot_stat_t MOT_STAT_XNEG           {1 << regs::MOT_STATUS_X_NEG_BIT};
static constexpr mot_stat_t MOT_STAT_XPOS           {1 << regs::MOT_STATUS_X_POS_BIT};
static constexpr mot_stat_t MOT_STAT_YNEG           {1 << regs::MOT_STATUS_Y_NEG_BIT};
static constexpr mot_stat_t MOT_STAT_YPOS           {1 << regs::MOT_STATUS_Y_POS_BIT};
static constexpr mot_stat_t MOT_STAT_ZNEG           {1 << regs::MOT_STATUS_Z_NEG_BIT};
static constexpr mot_stat_t MOT_STAT_ZPOS           {1 << regs::MOT_STATUS_Z_POS_BIT};
static constexpr mot_stat_t MOT_STAT_ZEROMOTION     {1 << regs::MOT_STATUS_ZRMOT_BIT};
#endif

// Standby mode
using stby_en_t = uint8_t;
static constexpr stby_en_t STBY_EN_NONE             {0x0};
static constexpr stby_en_t STBY_EN_ACCEL_X          {1 << regs::PWR2_STBY_XA_BIT};
static constexpr stby_en_t STBY_EN_ACCEL_Y          {1 << regs::PWR2_STBY_YA_BIT};
static constexpr stby_en_t STBY_EN_ACCEL_Z          {1 << regs::PWR2_STBY_ZA_BIT};
static constexpr stby_en_t STBY_EN_ACCEL            {STBY_EN_ACCEL_X | STBY_EN_ACCEL_Y | STBY_EN_ACCEL_Z};
static constexpr stby_en_t STBY_EN_GYRO_X           {1 << regs::PWR2_STBY_XG_BIT};
static constexpr stby_en_t STBY_EN_GYRO_Y           {1 << regs::PWR2_STBY_YG_BIT};
static constexpr stby_en_t STBY_EN_GYRO_Z           {1 << regs::PWR2_STBY_ZG_BIT};
static constexpr stby_en_t STBY_EN_GYRO             {STBY_EN_GYRO_X | STBY_EN_GYRO_Y | STBY_EN_GYRO_Z};
static constexpr stby_en_t STBY_EN_TEMP             {1 << 6};
// This is a low power mode that allows quick enabling of the gyros.
// note: when set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled.
static constexpr stby_en_t STBY_EN_LOWPWR_GYRO_PLL_ON  {1 << 7};

// Auxiliary I2C Master clock speed
typedef enum {
    AUXI2C_CLOCK_348KHZ = 0,
    AUXI2C_CLOCK_333KHZ = 1,
    AUXI2C_CLOCK_320KHZ = 2,
    AUXI2C_CLOCK_308KHZ = 3,
    AUXI2C_CLOCK_296KHZ = 4,
    AUXI2C_CLOCK_286KHZ = 5,
    AUXI2C_CLOCK_276KHZ = 6,
    AUXI2C_CLOCK_267KHZ = 7,
    AUXI2C_CLOCK_258KHZ = 8,
    AUXI2C_CLOCK_500KHZ = 9,
    AUXI2C_CLOCK_471KHZ = 10,
    AUXI2C_CLOCK_444KHZ = 11,
    AUXI2C_CLOCK_421KHZ = 12,
    AUXI2C_CLOCK_400KHZ = 13,
    AUXI2C_CLOCK_381KHZ = 14,
    AUXI2C_CLOCK_364KHZ = 15
} auxi2c_clock_t;

// Auxiliary I2C Master’s transition from one slave read to the next slave read
typedef enum {
    AUXI2C_TRANS_RESTART = 0,
    AUXI2C_TRANS_STOP = 1
} auxi2c_trans_t;

// Auxiliary I2C Slaves slots
typedef enum {
    AUXI2C_SLAVE_0 = 0,  // NOTE: for MPU9150 & MPU9250:
    AUXI2C_SLAVE_1 = 1,  // The MPU uses SLAVE0 and SLAVE1 to read Compass data
                         // so do not use this slave slots when compass is enabled
    AUXI2C_SLAVE_2 = 2,
    AUXI2C_SLAVE_3 = 3
} auxi2c_slv_t;

// Auxiliary I2C operation
typedef enum {
    AUXI2C_WRITE = 0,
    AUXI2C_READ = 1
} auxi2c_rw_t;

// Auxiliary I2C, EOW = end of word, use for swap
// Note: External sensor data typically comes in as groups of two bytes. This bit is used to determine if the groups are from
//  the slave’s register address 0 and 1, 2 and 3, etc.., or if the groups are address 1 and 2, 3 and 4, etc..
typedef enum {
    AUXI2C_EOW_ODD_NUM  = 0,  // indicates slave register addresses 0 and 1 are grouped together (odd numbered register ends the word).
    AUXI2C_EOW_EVEN_NUM = 1   // indicates slave register addresses 1 and 2 are grouped together (even numbered register ends the word).
} auxi2c_eow_t;  // This allows byte swapping of registers that are grouped starting at any address.


// Auxiliary I2C Master configuration struct
typedef struct {
    auxi2c_clock_t clock :4;  // clock signal speed
    bool multi_master_en :1;  // enable if there is an another master driving the bus too.
    uint8_t sample_delay :5;  // number of samples to delay on Aux i2c trasactions, max = 31, formula: rate = (sample_rate / delay + 1)
                              // (e.g. sample_delay = 4, aux i2c transaction will occour after 4 samples.
                              // if sample rate is 1KHz, the Aux i2c transaction rate will be 1000 / (4 + 1) = 200 Hz)
                              // set zero if no delay is needed, so the transaction rate will be the same as sample rate.
    bool shadow_delay_en :1;  // delays shadowing of external sensor data until all data has been received.
    bool wait_for_es     :1;  // delays the data ready interrupt until external sensor data is loaded. (if data ready interrupt is enabled)
    auxi2c_trans_t transition :1;  // transition condition from one slave read to the next slave read, default is 'restart'
} auxi2c_config_t;

// Auxiliary I2C Slave configuration struct
typedef struct {
    auxi2c_slv_t slave;  // slave slot
    uint8_t addr   :7;   // slave device address
    auxi2c_rw_t rw :1;   // read/write flag
    uint8_t reg_addr;    // register address to read/write to
    bool reg_dis   :1;   // when set, the transaction does not write the register address, it will only read data, or write data
    bool sample_delay_en :1;  // enable delay specifided in master config, sample_delay, for this slave in specific.
    union {
        struct {  // when read
            bool swap_en     :1;  // enable swap of bytes when reading both the low and high byte of a word (see note below)
            auxi2c_eow_t end_of_word :1;  // define at which register address a word ends, for swap low and high bytes of the word (when swap enabled)
            uint8_t rxlength :4;  // number of bytes to read, when set to read, max = 15
        };
        // when write
        uint8_t txdata;  // data to transfer when slave is set to write
    };
} auxi2c_slv_config_t;
/** 
 * Note on SWAP:
 * Swap bytes may be needed when external data is in another order.
 * The option swap_en, swaps bytes when reading both the low and high byte of a word. 
 *
 * For example, if rxlength = 4, and if reg_addr = 0x1, and group = ODD_NUM
 *  1) The first byte read from address 0x1 will be stored at EXT_SENS_DATA_00.
 *  2) the second and third bytes will be read and swapped, so the data read from address 0x2
 *     will be stored at EXT_SENS_DATA_02, and the data read from address 0x3 will be stored at EXT_SENS_DATA_03,
 *  3) The last byte read from address 0x4 will be stored at EXT_SENS_DATA_04
 * 
 * Note there is nothing to swap after reading the first byte if reg_addr[bit 0] = 1,
 * or if the last byte read has a register address [bit 0] = 0. The opposite is true for 'group' = EVEN_NUM.
 * */


// Auxiliary I2C master status register data
using auxi2c_stat_t = uint8_t;
static constexpr auxi2c_stat_t AUXI2C_STAT_FSYNC      {1 << regs::I2CMST_STAT_PASS_THROUGH_BIT};
static constexpr auxi2c_stat_t AUXI2C_STAT_LOST_ARB   {1 << regs::I2CMST_STAT_LOST_ARB_BIT};
static constexpr auxi2c_stat_t AUXI2C_STAT_SLV4_DONE  {1 << regs::I2CMST_STAT_SLV4_DONE_BIT};
static constexpr auxi2c_stat_t AUXI2C_STAT_SLV4_NACK  {1 << regs::I2CMST_STAT_SLV4_NACK_BIT};
static constexpr auxi2c_stat_t AUXI2C_STAT_SLV3_NACK  {1 << regs::I2CMST_STAT_SLV3_NACK_BIT};
static constexpr auxi2c_stat_t AUXI2C_STAT_SLV2_NACK  {1 << regs::I2CMST_STAT_SLV2_NACK_BIT};
static constexpr auxi2c_stat_t AUXI2C_STAT_SLV1_NACK  {1 << regs::I2CMST_STAT_SLV1_NACK_BIT};
static constexpr auxi2c_stat_t AUXI2C_STAT_SLV0_NACK  {1 << regs::I2CMST_STAT_SLV0_NACK_BIT};

// Auxiliary I2C bus VDDIO level [MPU6050 / MPU9150 only]
#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
typedef enum {
    AUXVDDIO_LVL_VLOGIC = 0,
    AUXVDDIO_LVL_VDD    = 1
} auxvddio_lvl_t;
#endif

// Interrupt active level
typedef enum {
    INT_LVL_ACTIVE_HIGH = 0,
    INT_LVL_ACTIVE_LOW  = 1
} int_lvl_t;

// Interrupt drive state
typedef enum {
    INT_DRV_PUSHPULL  = 0,
    INT_DRV_OPENDRAIN = 1
} int_drive_t;

// Interrupt mode
typedef enum {
    INT_MODE_PULSE50US = 0,
    INT_MODE_LATCH     = 1
} int_mode_t;

// Interrupt clear mode
typedef enum {
    INT_CLEAR_STATUS_REG = 0,
    INT_CLEAR_ANYREAD    = 1
} int_clear_t;

// Interrupt configuration struct
typedef struct {
    int_lvl_t level   :1;
    int_drive_t drive :1;
    int_mode_t mode   :1;
    int_clear_t clear :1;
} int_config_t;

// Enable features to generate signal at Interrupt pin
using int_en_t = uint8_t;
static constexpr int_en_t INT_EN_NONE            {0x0};
static constexpr int_en_t INT_EN_MOTION_DETECT   {1 << regs::INT_ENABLE_MOTION_BIT};
static constexpr int_en_t INT_EN_FIFO_OVERFLOW   {1 << regs::INT_ENABLE_FIFO_OFLOW_BIT};
static constexpr int_en_t INT_EN_I2C_MST_FSYNC   {1 << regs::INT_ENABLE_I2C_MST_FSYNC_BIT};  // interrupts from I2C_MST_STATUS
static constexpr int_en_t INT_EN_PLL_READY       {1 << regs::INT_ENABLE_PLL_RDY_BIT};
static constexpr int_en_t INT_EN_DMP_READY       {1 << regs::INT_ENABLE_DMP_RDY_BIT};
static constexpr int_en_t INT_EN_RAWDATA_READY   {1 << regs::INT_ENABLE_RAW_DATA_RDY_BIT};
// freefall and zero motion only available to MPU6000 / MPU6050 / MPU9150
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
static constexpr int_en_t INT_EN_FREE_FALL       {1 << regs::INT_ENABLE_FREEFALL_BIT};
static constexpr int_en_t INT_EN_ZERO_MOTION     {1 << regs::INT_ENABLE_ZEROMOT_BIT};
#endif

#ifdef CONFIG_MPU6500
// MPU6500 Fifo size
typedef enum {
    FIFO_SIZE_512B = 0,
    FIFO_SIZE_1K   = 1,
    FIFO_SIZE_2K   = 2,
    FIFO_SIZE_4K   = 3
} fifo_size_t;
#endif

// DMP Interrupt mode
typedef enum {
    DMP_INT_MODE_PACKET  = 0,
    DMP_INT_MODE_GESTURE = 1
} dmp_int_mode_t;

// FIFO mode
typedef enum {
    FIFO_MODE_OVERWRITE = 0,  // when the fifo is full, additional writes will be written to the fifo, replacing the oldest data.
    FIFO_MODE_STOP_FULL = 1   // when the fifo is full, additional writes will not be written to fifo.
} fifo_mode_t;

// FIFO configuration, enable sensors to be written to FIFO
using fifo_config_t = uint16_t;
static constexpr fifo_config_t FIFO_CFG_NONE          {0x0};
static constexpr fifo_config_t FIFO_CFG_GYRO          {1 << regs::FIFO_XGYRO_EN_BIT | 1 << regs::FIFO_YGYRO_EN_BIT | 1 << regs::FIFO_ZGYRO_EN_BIT};
static constexpr fifo_config_t FIFO_CFG_ACCEL         {1 << regs::FIFO_ACCEL_EN_BIT};
static constexpr fifo_config_t FIFO_CFG_TEMPERATURE   {1 << regs::FIFO_TEMP_EN_BIT};
static constexpr fifo_config_t FIFO_CFG_SLAVE0        {1 << regs::FIFO_SLV_0_EN_BIT};
static constexpr fifo_config_t FIFO_CFG_SLAVE1        {1 << regs::FIFO_SLV_1_EN_BIT};
static constexpr fifo_config_t FIFO_CFG_SLAVE2        {1 << regs::FIFO_SLV_2_EN_BIT};
static constexpr fifo_config_t FIFO_CFG_SLAVE3        {1 << 8};
#if defined CONFIG_MPU_AK89xx
static constexpr fifo_config_t FIFO_CFG_COMPASS       {FIFO_CFG_SLAVE0};  // 8 bytes
#endif

// Enable DMP features
/* @note DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually exclusive.
 * @note DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
 * mutually exclusive.
 * @note DMP_FEATURE_PEDOMETER is always enabled.
 */

/* typedef uint16_t dmp_features_t;
static constexpr dmp_features_t DMP_FEATURE_TAP =            {0x001};
static constexpr dmp_features_t DMP_FEATURE_ANDROID_ORIENT = {0x002};
static constexpr dmp_features_t DMP_FEATURE_LP_QUAT =        {0x004};
static constexpr dmp_features_t DMP_FEATURE_PEDOMETER =      {0x008};
static constexpr dmp_features_t DMP_FEATURE_6X_LP_QUAT =     {0x010};
static constexpr dmp_features_t DMP_FEATURE_GYRO_CAL =       {0x020};
static constexpr dmp_features_t DMP_FEATURE_SEND_RAW_ACCEL = {0x040};
static constexpr dmp_features_t DMP_FEATURE_SEND_RAW_GYRO =  {0x080};
static constexpr dmp_features_t DMP_FEATURE_SEND_CAL_GYRO =  {0x100};

// DMP Tap axes
typedef uint8_t dmp_tap_axis_t;
static constexpr dmp_tap_axis_t DMP_TAP_X       {0x30};
static constexpr dmp_tap_axis_t DMP_TAP_Y       {0x0C};
static constexpr dmp_tap_axis_t DMP_TAP_Z       {0x03};
static constexpr dmp_tap_axis_t DMP_TAP_XYZ     {0x3F};
 */

// Generic axes struct to store sensors' data
template< class type_t>
struct axes_t {
    union {
        type_t xyz[3] = {0};
        struct {
            type_t x;
            type_t y;
            type_t z;
        };
    };
    type_t& operator[](int i) {
        return xyz[i];
    }
};

// Axes for gyroscope, accelerometer, magnetometer
using raw_axes_t = axes_t<int16_t>;
using float_axes_t = axes_t<float>;


// Sensors struct for fast reading all sensors at once
typedef struct {
    raw_axes_t accel;  // accelerometer
    raw_axes_t gyro;   // gyroscope
    int16_t temp;      // temperature
    uint8_t* extsens;  // external sensor buffer
#if defined CONFIG_MPU_AK89xx
    raw_axes_t mag;    // magnetometer
#endif
} sensors_t;


/**************
 * MAGNETOMETER
 * ************ */
#ifdef CONFIG_MPU_AK89xx
static constexpr uint8_t COMPASS_I2CADDRESS = 0xC;
static constexpr uint8_t COMPASS_SAMPLE_RATE_MAX = 100;  // 100 Hz

// Magnetometer operation modes
typedef enum {
    MAG_MODE_POWER_DOWN       = 0x0,
    MAG_MODE_SINGLE_MEASURE   = 0x1,
    MAG_MODE_SELF_TEST        = 0x8,
    MAG_MODE_FUSE_ROM         = 0xF,
#ifdef CONFIG_MPU_AK8963
    MAG_MODE_CONTINUOUS_8HZ   = 0x2,  // not supported yet
    MAG_MODE_CONTINUOUS_100HZ = 0x6,  // not supported yet
    MAG_MODE_EXTERNAL_TRIGGER = 0x4   // not supported
#endif
} mag_mode_t;

// Magnetometer sensor status 1
using mag_stat1_t = uint8_t;
static constexpr mag_stat1_t MAG_STAT1_DATA_RDY         {1 << regs::mag::STATUS1_DATA_RDY_BIT};
#ifdef CONFIG_MPU_AK8963
static constexpr mag_stat1_t MAG_STAT1_DATA_OVERRUN     {1 << regs::mag::STATUS1_DATA_OVERRUN_BIT};
#endif

// Magnetometer sensor status 2
using mag_stat2_t = uint8_t;
static constexpr mag_stat2_t MAG_STAT1_SENSOR_OVERFLOW  {1 << regs::mag::STATUS2_OVERFLOW_BIT};
#ifdef CONFIG_MPU_AK8975
static constexpr mag_stat2_t MAG_STAT1_DATA_OVERRUN     {1 << regs::mag::STATUS1_DATA_OVERRUN_BIT};
#endif

// Magnetometer sensitivity
#ifdef CONFIG_MPU_AK8963
typedef enum {
    MAG_SENSITIVITY_0_6_uT  = 0,  // 0.6  uT/LSB  =  14-bit output
    MAG_SENSITIVITY_0_15_uT = 1,  // 0.15 uT/LSB  =  16-bit output
} mag_sensy_t;
#endif

// Auxiliary I2C slaves that operate the Magnetometer (do not change)
static constexpr auxi2c_slv_t MAG_SLAVE_READ_DATA = AUXI2C_SLAVE_0;  // read measurement data
static constexpr auxi2c_slv_t MAG_SLAVE_CHG_MODE  = AUXI2C_SLAVE_1;  // change mode to single measure

static constexpr uint8_t MAG_DATA_LENGTH = 8;  // bytes

#endif  // Magnetometer stuff


// // MPU configuration cache
// typedef struct mpu_config_s {
//     gyro_fsr_t gyro_fsr;
//     accel_fsr_t accel_fsr;
//     uint8_t sensors;
//     dlpf_t dlpf;
//     clock_src_t clock_src;
//     uint16_t sample_rate;
//     uint8_t fifo_en_reg;
//     uint8_t int_en_reg;
//     bool bypass_mode;
//     bool accel_half;
//     bool lp_accel_mode;
//     bool int_motion_only;
//     // struct motion_int_cache_s cache;
//     int_lvl_t int_level;
//     int_mode_t int_mode;
//     bool dmp_on;
//     bool dmp_loaded;
//     uint16_t dmp_sample_rate;
// #ifdef AK89xx_SECONDARY
//     uint16_t mag_sample_rate;
//     // int8_t mag_sens_adj[3];
// #endif
// } config_t;

// // Information for self-test.
// typedef struct mpu_test_s {
//     uint32_t gyro_sens;
//     uint32_t accel_sens;
//     uint8_t reg_rate_div;
//     uint8_t reg_lpf;
//     uint8_t reg_gyro_fsr;
//     uint8_t reg_accel_fsr;
//     uint16_t wait_ms;
//     uint8_t packet_thresh;
//     float min_dps;
//     float max_dps;
//     float max_gyro_var;
//     float min_g;
//     float max_g;
//     float max_accel_var;
// #ifdef MPU6500
//     float max_g_offset;
//     uint16_t sample_wait_ms;
// #endif
// } test_t;
}  // namespace types

}  // namespace mpu

}  // namespace emd



#endif  /* end of include guard: _MPU_TYPES_HPP_ */
