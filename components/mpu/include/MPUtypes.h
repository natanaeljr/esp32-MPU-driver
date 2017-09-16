#ifndef _MPU_TYPES_H_
#define _MPU_TYPES_H_

#include <stdint.h>

typedef enum {
    MPU_ADDRESS_AD0_LOW = 0x68,
    MPU_ADDRESS_AD0_HIGH = 0x69
} mpu_address_t;

// Gyro full-scale-range
typedef enum {
    MPU_GYRO_FSR_250DPS  = 0,
    MPU_GYRO_FSR_500DPS  = 1,
    MPU_GYRO_FSR_1000DPS = 2,
    MPU_GYRO_FSR_2000DPS = 3
} mpu_gyro_fsr_t;

// Accel full-scale-range
typedef enum {
    MPU_ACCEL_FSR_2G  = 0,
    MPU_ACCEL_FSR_4G  = 1,
    MPU_ACCEL_FSR_8G  = 2,
    MPU_ACCEL_FSR_16G = 3
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
    MPU_CLOCK_INTERNAL = 0, // 20MHz for MPU6500 and 8MHz for MPU6050
#if defined _MPU6500_
    MPU_CLOCK_PLL = 3       // Selects automatically best pll source
#elif defined _MPU6050_
    MPU_CLOCK_PLL_XG   = 1,
    MPU_CLOCK_PLL_YG   = 2,
    MPU_CLOCK_PLL_ZG   = 3,
    MPU_CLOCK_EXT32KHZ = 4,
    MPU_CLOCK_EXT19MHZ = 5,
    MPU_CLOCK_KEEP_RESET = 7
#endif
} mpu_clock_src_t;

// Low-power accel wake-up rates
typedef enum {
#if defined _MPU6050_
    MPU_LP_ACCEL_1_25HZ = 0,
    MPU_LP_ACCEL_5HZ    = 1,
    MPU_LP_ACCEL_20HZ   = 2,
    MPU_LP_ACCEL_40HZ   = 3
#elif defined _MPU6500_
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

#ifdef _MPU6500_
// MPU6500 Fifo size
typedef enum {
    MPU6500_FIFO_SIZE_1K = 1,
    MPU6500_FIFO_SIZE_2K = 2,
    MPU6500_FIFO_SIZE_4K = 3,
} mpu6500_fifo_size_t;
#endif

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
