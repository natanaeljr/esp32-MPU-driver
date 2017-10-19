#ifndef _MPU_REGISTERS_HPP_
#define _MPU_REGISTERS_HPP_

#include "mpu_define.hpp"


/*******************************************************************************
 * MPU commom registers for all models /////////////////////////////////////////
 ******************************************************************************/
#define MPU_REG_XG_OFFSET_H         (0x13)
#define MPU_REG_XG_OFFSET_L         (0x14)
#define MPU_REG_YG_OFFSET_H         (0x15)
#define MPU_REG_YG_OFFSET_L         (0x16)
#define MPU_REG_ZG_OFFSET_H         (0x17)
#define MPU_REG_ZG_OFFSET_L         (0x18)
#define MPU_REG_SMPLRT_DIV          (0x19) // [7:0]
//------------------------------------------------------------------------------
#define MPU_REG_CONFIG              (0x1A)
    #define MPU_CONFIG_FIFO_MODE_BIT        (6)
    #define MPU_CONFIG_EXT_SYNC_SET_BIT     (5) // [5:3]
    #define MPU_CONFIG_EXT_SYNC_SET_LENGTH  (3)
    #define MPU_CONFIG_DLPF_CFG_BIT         (2) // [2:0]
    #define MPU_CONFIG_DLPF_CFG_LENGTH      (3)
//------------------------------------------------------------------------------
#define MPU_REG_GYRO_CONFIG         (0x1B)
    #define MPU_GCONFIG_XG_ST_BIT           (7)
    #define MPU_GCONFIG_YG_ST_BIT           (6)
    #define MPU_GCONFIG_ZG_ST_BIT           (5)
    #define MPU_GCONFIG_ZG_ST_BIT           (5)
    #define MPU_GCONFIG_FS_SEL_BIT          (4) // [4:3]
    #define MPU_GCONFIG_FS_SEL_LENGTH       (2)
    #define MPU_GCONFIG_FCHOICE_B           (1) // [1:0]
    #define MPU_GCONFIG_FCHOICE_B_LENGTH    (2)
//------------------------------------------------------------------------------
#define MPU_REG_ACCEL_CONFIG        (0x1C)
    #define MPU_ACONFIG_XA_ST_BIT           (7)
    #define MPU_ACONFIG_YA_ST_BIT           (6)
    #define MPU_ACONFIG_ZA_ST_BIT           (5)
    #define MPU_ACONFIG_FS_SEL_BIT          (4) // [4:3]
    #define MPU_ACONFIG_FS_SEL_LENGTH       (2)
//------------------------------------------------------------------------------
#define MPU_REG_FF_THR               (0x1D)
#define MPU_REG_FF_DUR               (0x1E)
#define MPU_REG_MOTION_THR          (0x1F) // [7:0] // MPU9250_REG_WOM_THR
#define MPU_REG_MOTION_DUR          (0x20)
#define MPU_REG_ZRMOTION_THR         (0x21)
#define MPU_REG_ZRMOTION_DUR         (0x22)
//------------------------------------------------------------------------------
#define MPU_REG_FIFO_EN             (0x23)
    #define MPU_FIFO_TEMP_EN_BIT            (7)
    #define MPU_FIFO_XGYRO_EN_BIT           (6)
    #define MPU_FIFO_YGYRO_EN_BIT           (5)
    #define MPU_FIFO_ZGYRO_EN_BIT           (4)
    #define MPU_FIFO_ACCEL_EN_BIT           (3)
    #define MPU_FIFO_SLV_2_EN_BIT           (2)
    #define MPU_FIFO_SLV_1_EN_BIT           (1)
    #define MPU_FIFO_SLV_0_EN_BIT           (0)
//------------------------------------------------------------------------------
#define MPU_REG_I2C_MST_CTRL        (0x24)
    #define MPU_I2C_MST_MULT_EN_BIT         (7)
    #define MPU_I2C_MST_WAIT_FOR_ES_BIT     (6)
    #define MPU_I2C_MST_SLV_3_FIFO_EN_BIT   (5)
    #define MPU_I2C_MST_P_NSR_BIT           (4)
    #define MPU_I2C_MST_CLOCK_BIT           (3) // [3:0]
    #define MPU_I2C_MST_CLOCK_LENGTH        (4)
//------------------------------------------------------------------------------
#define MPU_REG_I2C_SLV0_ADDR       (0x25)
    #define MPU_I2C_SLV_RNW_BIT             (7) // same for all I2C_SLV registers
    #define MPU_I2C_SLV_ID_BIT              (6) // [6:0]
    #define MPU_I2C_SLV_ID_LENGTH           (7)
//------------------------------------------------------------------------------
#define MPU_REG_I2C_SLV0_REG        (0x26) // [7:0]
//------------------------------------------------------------------------------
#define MPU_REG_I2C_SLV0_CTRL       (0x27)
    #define MPU_I2C_SLV_EN_BIT              (7) // same for all I2C_SLV registers
    #define MPU_I2C_SLV_BYTE_SW_BIT         (6)
    #define MPU_I2C_SLV_REG_DIS_BIT         (5)
    #define MPU_I2C_SLV_GRP_BIT             (4)
    #define MPU_I2C_SLV_LEN_BIT             (3) // [3:0]
    #define MPU_I2C_SLV_LEN_LENGTH          (4)
//------------------------------------------------------------------------------
#define MPU_REG_I2C_SLV1_ADDR       (0x28) // see SLV0 for bit defines
#define MPU_REG_I2C_SLV1_REG        (0x29)
#define MPU_REG_I2C_SLV1_CTRL       (0x2A)
#define MPU_REG_I2C_SLV2_ADDR       (0x2B) // see SLV0 for bit defines
#define MPU_REG_I2C_SLV2_REG        (0x2C)
#define MPU_REG_I2C_SLV2_CTRL       (0x2D)
#define MPU_REG_I2C_SLV3_ADDR       (0x2E) // see SLV0 for bit defines
#define MPU_REG_I2C_SLV3_REG        (0x2F)
#define MPU_REG_I2C_SLV3_CTRL       (0x30)
#define MPU_REG_I2C_SLV4_ADDR       (0x31) // see SLV0 for bit defines
#define MPU_REG_I2C_SLV4_REG        (0x32)
#define MPU_REG_I2C_SLV4_DO         (0x33) // [7:0]
//------------------------------------------------------------------------------
#define MPU_REG_I2C_SLV4_CTRL       (0x34)
    #define MPU_I2C_SLV4_EN_BIT             (7)
    #define MPU_I2C_SLV4_DONE_INT_BIT       (6)
    #define MPU_I2C_SLV4_REG_DIS_BIT        (5)
    #define MPU_I2C_SLV4_MST_DELAY_BIT      (4) // [4:0]
    #define MPU_I2C_SLV4_MST_DELAY_LENGTH   (5)

//------------------------------------------------------------------------------
#define MPU_REG_I2C_SLV4_DI         (0x35) // [7:0]
//------------------------------------------------------------------------------
#define MPU_REG_I2C_MST_STATUS      (0x36)
    #define MPU_I2CMST_PASS_THROUGH_BIT     (7)
    #define MPU_I2CMST_SLV4_DONE_BIT        (6)
    #define MPU_I2CMST_LOST_ARB_BIT         (5)
    #define MPU_I2CMST_SLV4_NACK_BIT        (4)
    #define MPU_I2CMST_SLV3_NACK_BIT        (3)
    #define MPU_I2CMST_SLV2_NACK_BIT        (2)
    #define MPU_I2CMST_SLV1_NACK_BIT        (1)
    #define MPU_I2CMST_SLV0_NACK_BIT        (0)
//------------------------------------------------------------------------------
#define MPU_REG_INT_PIN_CFG         (0x37)
    #define MPU_INT_LEVEL_BIT               (7)
    #define MPU_INT_OPEN_BIT                (6)
    #define MPU_INT_LATCH_EN_BIT            (5)
    #define MPU_INT_ANYRD_2CLEAR_BIT        (4)
    #define MPU_INT_FSYNC_LEVEL_BIT         (3)
    #define MPU_INT_FSYNC_INT_MODE_EN_BIT   (2)
    #define MPU_INT_I2C_BYPASS_EN_BIT       (1)
//------------------------------------------------------------------------------
#define MPU_REG_INT_ENABLE          (0x38)
    #define MPU_INT_FREEFALL_EN_BIT         (7)
    #define MPU_INT_MOTION_EN_BIT           (6)
    #define MPU_INT_ZEROMOTION_EN_BIT       (5)
    #define MPU_INT_FIFO_OFLOW_EN_BIT       (4)
    #define MPU_INT_FSYNC_INT_EN_BIT        (3)
    #define MPU_INT_PLL_RDY_EN_BIT          (2)
    #define MPU_INT_DMP_RDY_EN_BIT          (1)
    #define MPU_INT_RAW_DATA_RDY_EN_BIT     (0)
//------------------------------------------------------------------------------
#define MPU_REG_DMP_INT_STATUS      (0x39)
    #define MPU_DMP_INT_STATUS_0            (0)
    #define MPU_DMP_INT_STATUS_1            (1)
    #define MPU_DMP_INT_STATUS_2            (2)
    #define MPU_DMP_INT_STATUS_3            (3)
    #define MPU_DMP_INT_STATUS_4            (4)
    #define MPU_DMP_INT_STATUS_5            (5)
//------------------------------------------------------------------------------
#define MPU_REG_INT_STATUS          (0x3A)
    #define MPU_INT_STATUS_FREEFALL_BIT     (7)
    #define MPU_INT_STATUS_MOT_BIT          (6)
    #define MPU_INT_STATUS_ZMOT_BIT         (5)
    #define MPU_INT_STATUS_FIFO_OFLOW_BIT   (4)
    #define MPU_INT_STATUS_I2C_MST_BIT      (3)
    #define MPU_INT_STATUS_PLL_RDY_BIT      (2)
    #define MPU_INT_STATUS_DMP_RDY_BIT      (1)
    #define MPU_INT_STATUS_RAW_DATA_RDY_BIT (0)
//------------------------------------------------------------------------------
#define MPU_REG_ACCEL_XOUT_H        (0x3B) // [15:0]
#define MPU_REG_ACCEL_XOUT_L        (0x3C)
#define MPU_REG_ACCEL_YOUT_H        (0x3D) // [15:0]
#define MPU_REG_ACCEL_YOUT_L        (0x3E)
#define MPU_REG_ACCEL_ZOUT_H        (0x3F) // [15:0]
#define MPU_REG_ACCEL_ZOUT_L        (0x40)
#define MPU_REG_TEMP_OUT_H          (0x41) // [15:0]
#define MPU_REG_TEMP_OUT_L          (0x42)
#define MPU_REG_GYRO_XOUT_H         (0x43) // [15:0]
#define MPU_REG_GYRO_XOUT_L         (0x44)
#define MPU_REG_GYRO_YOUT_H         (0x45) // [15:0]
#define MPU_REG_GYRO_YOUT_L         (0x46)
#define MPU_REG_GYRO_ZOUT_H         (0x47) // [15:0]
#define MPU_REG_GYRO_ZOUT_L         (0x48)
#define MPU_REG_EXT_SENS_DATA_00    (0x49) // Stores data read from Slave 0, 1, 2, and 3
#define MPU_REG_EXT_SENS_DATA_01    (0x4A)
#define MPU_REG_EXT_SENS_DATA_02    (0x4B)
#define MPU_REG_EXT_SENS_DATA_03    (0x4C)
#define MPU_REG_EXT_SENS_DATA_04    (0x4D)
#define MPU_REG_EXT_SENS_DATA_05    (0x4E)
#define MPU_REG_EXT_SENS_DATA_06    (0x4F)
#define MPU_REG_EXT_SENS_DATA_07    (0x50)
#define MPU_REG_EXT_SENS_DATA_08    (0x51)
#define MPU_REG_EXT_SENS_DATA_09    (0x52)
#define MPU_REG_EXT_SENS_DATA_10    (0x53)
#define MPU_REG_EXT_SENS_DATA_11    (0x54)
#define MPU_REG_EXT_SENS_DATA_12    (0x55)
#define MPU_REG_EXT_SENS_DATA_13    (0x56)
#define MPU_REG_EXT_SENS_DATA_14    (0x57)
#define MPU_REG_EXT_SENS_DATA_15    (0x58)
#define MPU_REG_EXT_SENS_DATA_16    (0x59)
#define MPU_REG_EXT_SENS_DATA_17    (0x5A)
#define MPU_REG_EXT_SENS_DATA_18    (0x5B)
#define MPU_REG_EXT_SENS_DATA_19    (0x5C)
#define MPU_REG_EXT_SENS_DATA_20    (0x5D)
#define MPU_REG_EXT_SENS_DATA_21    (0x5E)
#define MPU_REG_EXT_SENS_DATA_22    (0x5F)
#define MPU_REG_EXT_SENS_DATA_23    (0x60)
#define MPU_REG_MOT_DETECT_STATUS   (0x61)
#define MPU_REG_I2C_SLV0_DO         (0x63)
#define MPU_REG_I2C_SLV1_DO         (0x64)
#define MPU_REG_I2C_SLV2_DO         (0x65)
#define MPU_REG_I2C_SLV3_DO         (0x66)
//------------------------------------------------------------------------------
#define MPU_REG_I2C_MST_DELAY_CRTL  (0x67)
    #define MPU_I2CMST_ES_SHADOW_DLY_BIT    (7)
    #define MPU_I2CMST_SLV4_DLY_EN_BIT      (4)
    #define MPU_I2CMST_SLV3_DLY_EN_BIT      (3)
    #define MPU_I2CMST_SLV2_DLY_EN_BIT      (2)
    #define MPU_I2CMST_SLV1_DLY_EN_BIT      (1)
    #define MPU_I2CMST_SLV0_DLY_EN_BIT      (0)
//------------------------------------------------------------------------------
#define MPU_REG_SIGNAL_PATH_RESET   (0x68)
    #define MPU_SPATH_GYRO_RST_BIT          (2)
    #define MPU_SPATH_ACCEL_RST_BIT         (1)
    #define MPU_SPATH_TEMP_RST_BIT          (0)
//------------------------------------------------------------------------------
#define MPU_REG_MOT_DETECT_CTRL     (0x69) // ACCEL_INTEL_CTRL
    #define MPU_DETECT_ACCEL_INTEL_EN_BIT   (7)
    #define MPU_DETECT_ACCEL_INTEL_MODE_BIT (6)
//------------------------------------------------------------------------------
#define MPU_REG_USER_CTRL           (0x6A)
    #define MPU_USERCTRL_DMP_EN_BIT         (7)
    #define MPU_USERCTRL_FIFO_EN_BIT        (6)
    #define MPU_USERCTRL_I2C_MST_EN_BIT     (5)
    #define MPU_USERCTRL_I2C_IF_DIS_BIT     (4)
    #define MPU_USERCTRL_DMP_RESET_BIT      (3)
    #define MPU_USERCTRL_FIFO_RESET_BIT     (2)
    #define MPU_USERCTRL_I2C_MST_RESET_BIT  (1)
    #define MPU_USERCTRL_SIG_COND_RESET_BIT (0)
//------------------------------------------------------------------------------
#define MPU_REG_PWR_MGMT1           (0x6B)
    #define MPU_PWR1_DEVICE_RESET_BIT       (7)
    #define MPU_PWR1_SLEEP_BIT              (6)
    #define MPU_PWR1_CYCLE_BIT              (5)
    #define MPU_PWR1_GYRO_STANDBY_BIT       (4)
    #define MPU_PWR1_TEMP_DIS_BIT           (3)
    #define MPU_PWR1_CLKSEL_BIT             (2)
    #define MPU_PWR1_CLKSEL_LENGTH          (3)
//------------------------------------------------------------------------------
#define MPU_REG_PWR_MGMT2           (0x6C)
    #define MPU_PWR2_LP_WAKE_CTRL_BIT       (7)
    #define MPU_PWR2_LP_WAKE_CTRL_LENGTH    (2)
    #define MPU_PWR2_STBY_XA_BIT            (5)
    #define MPU_PWR2_STBY_YA_BIT            (4)
    #define MPU_PWR2_STBY_ZA_BIT            (3)
    #define MPU_PWR2_STBY_XG_BIT            (2)
    #define MPU_PWR2_STBY_YG_BIT            (1)
    #define MPU_PWR2_STBY_ZG_BIT            (0)
//------------------------------------------------------------------------------
#define MPU_REG_BANK_SEL            (0x6D)
    #define MPU_BANKSEL_PRFTCH_EN_BIT       (6)
    #define MPU_BANKSEL_CFG_USER_BANK_BIT   (5)
    #define MPU_BANKSEL_MEM_SEL_BIT         (4)
    #define MPU_BANKSEL_MEM_SEL_LENGTH      (5)
//------------------------------------------------------------------------------
#define MPU_REG_MEM_START_ADDR      (0x6E)
#define MPU_REG_MEM_R_W             (0x6F)
#define MPU_REG_PRGM_START_H        (0x70)
#define MPU_REG_PRGM_START_L        (0x71)
#define MPU_REG_FIFO_COUNT_H        (0x72) // [15:0]
#define MPU_REG_FIFO_COUNT_L        (0x73)
#define MPU_REG_FIFO_R_W            (0x74)
#define MPU_REG_WHO_AM_I            (0x75)





/*******************************************************************************
 * MPU6050 and MPU9150 commom registers
 ******************************************************************************/
#if defined CONFIG_MPU6050
#define MPU6050_REG_XG_OFFSET_TC        (0x00) //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
//------------------------------------------------------------------------------
#define MPU6050_REG_YG_OFFSET_TC        (0x01) //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
    #define MPU6050_TC_PWR_MODE_BIT         (7)
//------------------------------------------------------------------------------
#define MPU6050_REG_ZG_OFFSET_TC        (0x02) //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_REG_X_FINE_GAIN         (0x03) //[7:0] X_FINE_GAIN
#define MPU6050_REG_Y_FINE_GAIN         (0x04) //[7:0] Y_FINE_GAIN
#define MPU6050_REG_Z_FINE_GAIN         (0x05) //[7:0] Z_FINE_GAIN
#define MPU6050_REG_XA_OFFSET_H         (0x06) //[15:0] XA_OFFS
#define MPU6050_REG_XA_OFFSET_L_TC      (0x07)
#define MPU6050_REG_YA_OFFSET_H         (0x08) //[15:0] YA_OFFS
#define MPU6050_REG_YA_OFFSET_L_TC      (0x09)
#define MPU6050_REG_ZA_OFFSET_H         (0x0A) //[15:0] ZA_OFFS
#define MPU6050_REG_ZA_OFFSET_L_TC      (0x0B)
#define MPU6050_REG_SELF_TEST_X         (0x0D)
#define MPU6050_REG_SELF_TEST_Y         (0x0E)
#define MPU6050_REG_SELF_TEST_Z         (0x0F)
#define MPU6050_REG_SELF_TEST_A         (0x10)
#endif






/*******************************************************************************
 * MPU6500 and MPU9250 commom registers ////////////////////////////////////////
 ******************************************************************************/
#if defined CONFIG_MPU6500
#define MPU6500_REG_SELF_TEST_X_GYRO    (0x00) // XG_ST_DATA[7:0]
#define MPU6500_REG_SELF_TEST_Y_GYRO    (0x01) // YG_ST_DATA[7:0]
#define MPU6500_REG_SELF_TEST_Z_GYRO    (0x02) // ZG_ST_DATA[7:0]
#define MPU6500_REG_SELF_TEST_X_ACCEL   (0x0D)
#define MPU6500_REG_SELF_TEST_Y_ACCEL   (0x0E)
#define MPU6500_REG_SELF_TEST_Z_ACCEL   (0x0F)
//------------------------------------------------------------------------------
#define MPU6500_REG_ACCEL_CONFIG2       (0x1D)
    #define MPU6500_ACONFIG2_FIFO_SIZE_BIT          (7) // [7:6]
    #define MPU6500_ACONFIG2_FIFO_SIZE_LENGTH       (2)
    #define MPU6500_ACONFIG2_ACCEL_FCHOICE_B_BIT    (3)
    #define MPU6500_ACONFIG2_A_DLPF_CFG_BIT         (2) // [2:0]
    #define MPU6500_ACONFIG2_A_DLPF_CFG_LENGTH      (3)
//------------------------------------------------------------------------------
#define MPU6500_REG_LP_ACCEL_ODR        (0x1E)
    #define MPU6500_LPA_ODR_CLKSEL_BIT              (3) // [3:0]
    #define MPU6500_LPA_ODR_CLKSEL_LENGTH           (4)
//------------------------------------------------------------------------------
#define MPU6500_REG_XA_OFFSET_H         (0x77)
#define MPU6500_REG_XA_OFFSET_L         (0x78)
#define MPU6500_REG_YA_OFFSET_H         (0x7A)
#define MPU6500_REG_YA_OFFSET_L         (0x7B)
#define MPU6500_REG_ZA_OFFSET_H         (0x7D)
#define MPU6500_REG_ZA_OFFSET_L         (0x7C) // review this ones, might be wrong
#endif














#endif /* end of include guard: _MPU_REGISTERS_HPP_ */
