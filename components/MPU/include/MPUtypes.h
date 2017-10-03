#ifndef _MPU_TYPES_H_
#define _MPU_TYPES_H_

#include "MPUregisters.h"
#include <stdint.h>

typedef enum {
    MPU_ADDRESS_AD0_LOW = 0x68,
    MPU_ADDRESS_AD0_HIGH = 0x69
} mpu_addr_t;

// Gyro full-scale-range
typedef enum {
    MPU_GYRO_FS_250DPS  = 0,
    MPU_GYRO_FS_500DPS  = 1,
    MPU_GYRO_FS_1000DPS = 2,
    MPU_GYRO_FS_2000DPS = 3
} mpu_gyro_fsr_t;

// Accel full-scale-range
typedef enum {
    MPU_ACCEL_FS_2G  = 0,
    MPU_ACCEL_FS_4G  = 1,
    MPU_ACCEL_FS_8G  = 2,
    MPU_ACCEL_FS_16G = 3
} mpu_accel_fsr_t;

// Digital low-pass filter (based on gyro bandwidth)
typedef enum {
    MPU_DLPF_265HZ_NOLPF  = 0,
    MPU_DLPF_188HZ = 1,
    MPU_DLPF_98HZ  = 2,
    MPU_DLPF_42HZ  = 3,
    MPU_DLPF_20HZ  = 4,
    MPU_DLPF_10HZ  = 5,
    MPU_DLPF_5HZ   = 6,
    MPU_DLPF_2100HZ_NOLPF = 7
} mpu_dlpf_t;

// Clock source
typedef enum {
    MPU_CLOCK_INTERNAL   = 0, // 20MHz for MPU6500 and 8MHz for MPU6050
    MPU_CLOCK_PLL = 3       // Selects automatically best pll source
#if defined CONFIG_MPU6050
    ,MPU_CLOCK_EXT32KHZ  = 4,
    MPU_CLOCK_EXT19MHZ   = 5,
    MPU_CLOCK_KEEP_RESET = 7
#endif
} mpu_clock_src_t;

// Low-power accel wake-up rates
typedef enum {
#if defined CONFIG_MPU6050
    MPU_LP_ACCEL_1_25HZ = 0,
    MPU_LP_ACCEL_5HZ    = 1,
    MPU_LP_ACCEL_20HZ   = 2,
    MPU_LP_ACCEL_40HZ   = 3
#elif defined CONFIG_MPU6500
    MPU_LP_ACCEL_0_24HZ  = 0,
    MPU_LP_ACCEL_0_49HZ  = 1,
    MPU_LP_ACCEL_0_98HZ  = 2,
    MPU_LP_ACCEL_1_95HZ  = 3,
    MPU_LP_ACCEL_3_91HZ  = 4,
    MPU_LP_ACCEL_7_81HZ  = 5,
    MPU_LP_ACCEL_15_63HZ = 6,
    MPU_LP_ACCEL_31_25HZ = 7,
    MPU_LP_ACCEL_62_50HZ = 8,
    MPU_LP_ACCEL_125HZ   = 9,
    MPU_LP_ACCEL_250HZ   = 10,
    MPU_LP_ACCEL_500HZ   = 11
#endif
} mpu_lp_accel_rate_t;

// I2C Master clock speed
typedef enum {
    MPU_I2C_MST_CLOCK_348KHZ = 0,
    MPU_I2C_MST_CLOCK_333KHZ = 1,
    MPU_I2C_MST_CLOCK_320KHZ = 2,
    MPU_I2C_MST_CLOCK_308KHZ = 3,
    MPU_I2C_MST_CLOCK_296KHZ = 4,
    MPU_I2C_MST_CLOCK_286KHZ = 5,
    MPU_I2C_MST_CLOCK_276KHZ = 6,
    MPU_I2C_MST_CLOCK_267KHZ = 7,
    MPU_I2C_MST_CLOCK_258KHZ = 8,
    MPU_I2C_MST_CLOCK_500KHZ = 9,
    MPU_I2C_MST_CLOCK_471KHZ = 10,
    MPU_I2C_MST_CLOCK_444KHZ = 11,
    MPU_I2C_MST_CLOCK_421KHZ = 12,
    MPU_I2C_MST_CLOCK_400KHZ = 13,
    MPU_I2C_MST_CLOCK_381KHZ = 14,
    MPU_I2C_MST_CLOCK_364KHZ = 15
} mpu_i2c_mst_clock_t;

// Interrupt active level
typedef enum {
    MPU_INT_LVL_ACTIVE_HIGH = 0,
    MPU_INT_LVL_ACTIVE_LOW  = 1
} mpu_int_lvl_t;

// Interrupt drive state
typedef enum {
    MPU_INT_DRV_PUSHPULL  = 0,
    MPU_INT_DRV_OPENDRAIN = 1
} mpu_int_drive_t;

// Interrupt mode
typedef enum {
    MPU_INT_MODE_PULSE50US = 0,
    MPU_INT_MODE_LATCH     = 1
} mpu_int_mode_t;

// Interrupt clear mode
typedef enum {
    MPU_INT_CLEAR_STATUS_REG = 0,
    MPU_INT_CLEAR_ANYREAD    = 1
} mpu_int_clear_t;

// Interrupt configuration struct
typedef struct {
    mpu_int_lvl_t level;
    mpu_int_mode_t mode;
    mpu_int_drive_t drive;
    mpu_int_clear_t clear;
    mpu_int_lvl_t fsyncLevel;
} mpu_int_config_t;

#ifdef CONFIG_MPU6500

// MPU6500 Fifo size
typedef enum {
    MPU6500_FIFO_SIZE_1K = 1,
    MPU6500_FIFO_SIZE_2K = 2,
    MPU6500_FIFO_SIZE_4K = 3,
} mpu6500_fifo_size_t;

#endif

// DMP Interrupt mode
typedef enum {
    DMP_INT_MODE_PACKET = 0,
    DMP_INT_MODE_GESTURE = 1
} dmp_int_mode_t;

// Enable Sensors to go into FIFO
#define MPU_FIFO_GYRO           (MPU_FIFO_XGYRO_EN_BIT | MPU_FIFO_YGYRO_EN_BIT | MPU_FIFO_ZGYRO_EN_BIT) 
#define MPU_FIFO_ACCEL          (MPU_FIFO_ACCEL_EN_BIT)
#define MPU_FIFO_TEMPERATURE    (MPU_FIFO_TEMP_EN_BIT)
#define MPU_FIFO_SLAVE0         (MPU_FIFO_SLV_0_EN_BIT)
#define MPU_FIFO_SLAVE1         (MPU_FIFO_SLV_1_EN_BIT)
#define MPU_FIFO_SLAVE2         (MPU_FIFO_SLV_2_EN_BIT)
#define MPU_FIFO_SLAVE3         (0x100)
typedef uint_fast16_t mpu_fifo_sensors_t;

// Enable features to generate signal at Interrupt pin
#define MPU_INT_FREEFALL        (MPU_INT_FREEFALL_EN_BIT)
#define MPU_INT_MOTION          (MPU_INT_MOTION_EN_BIT)
#define MPU_INT_ZEROMOTION      (MPU_INT_ZEROMOTION_EN_BIT)
#define MPU_INT_FIFO_OVERFLOW   (MPU_INT_FIFO_OFLOW_EN_BIT)
#define MPU_INT_FSYNC           (MPU_INT_FSYNC_INT_EN_BIT)
#define MPU_INT_PLL_READY       (MPU_INT_PLL_RDY_EN_BIT)
#define MPU_INT_DMP_READY       (MPU_INT_DMP_RDY_EN_BIT)
#define MPU_INT_RAWDATA_READY   (MPU_INT_RAW_DATA_RDY_EN_BIT)
typedef uint_fast8_t mpu_int_t;

// Enable DMP features 
/* @note DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually exclusive.
 * @note DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
 * mutually exclusive.
 * @note DMP_FEATURE_PEDOMETER is always enabled.
 */
#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)
typedef uint_fast16_t dmp_features_t;

// DMP Tap axes
#define DMP_TAP_X       (0x30)
#define DMP_TAP_Y       (0x0C)
#define DMP_TAP_Z       (0x03)
#define DMP_TAP_XYZ     (0x3F)
typedef uint_fast8_t dmp_tap_axis_t;

// Axis struct for gyro and accel
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t& operator[](int i) {
        if(i <= 0) return x;
        if(i == 1) return y;
        else return z;
    }
} mpu_axes_t;

// MPU configuration cache
typedef struct mpu_config_s {
    mpu_gyro_fsr_t gyro_fsr;
    mpu_accel_fsr_t accel_fsr;
    uint8_t sensors;
    mpu_dlpf_t dlpf;
    mpu_clock_src_t clock_src;
    uint16_t sample_rate;
    uint8_t fifo_en_reg;
    uint8_t int_en_reg;
    bool bypass_mode;
    bool accel_half;
    bool lp_accel_mode;
    bool int_motion_only;
    // struct motion_int_cache_s cache;
    mpu_int_lvl_t int_level;
    mpu_int_mode_t int_mode;
    bool dmp_on;
    bool dmp_loaded;
    uint16_t dmp_sample_rate;
#ifdef AK89xx_SECONDARY
    uint16_t mag_sample_rate;
    // int8_t mag_sens_adj[3];
#endif
} mpu_config_t;

// Information for self-test.
typedef struct mpu_test_s {
    uint32_t gyro_sens;
    uint32_t accel_sens;
    uint8_t reg_rate_div;
    uint8_t reg_lpf;
    uint8_t reg_gyro_fsr;
    uint8_t reg_accel_fsr;
    uint16_t wait_ms;
    uint8_t packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;
#ifdef MPU6500
    float max_g_offset;
    uint16_t sample_wait_ms;
#endif
} mpu_test_t;





#endif /* end of include guard: _MPU_TYPES_H_ */
