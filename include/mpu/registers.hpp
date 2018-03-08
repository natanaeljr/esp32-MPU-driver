// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu/registers.hpp
 * Define registers' for all MPU models.
 */

#ifndef _MPU_REGISTERS_HPP_
#define _MPU_REGISTERS_HPP_

#include <stdint.h>
#include "sdkconfig.h"

/*! MPU Driver namespace */
namespace mpud
{
/*! Registers namespace */
namespace regs
{
/*******************************************************************************
 * MPU commom registers for all models
 ******************************************************************************/
constexpr uint8_t XG_OFFSET_H = (0x13);
constexpr uint8_t XG_OFFSET_L = (0x14);
constexpr uint8_t YG_OFFSET_H = (0x15);
constexpr uint8_t YG_OFFSET_L = (0x16);
constexpr uint8_t ZG_OFFSET_H = (0x17);
constexpr uint8_t ZG_OFFSET_L = (0x18);
constexpr uint8_t SMPLRT_DIV  = (0x19);  // [7:0]
//------------------------------------------------------------------------------
constexpr uint8_t CONFIG                     = (0x1A);
constexpr uint8_t CONFIG_FIFO_MODE_BIT       = (6);
constexpr uint8_t CONFIG_EXT_SYNC_SET_BIT    = (5);  // [5:3]
constexpr uint8_t CONFIG_EXT_SYNC_SET_LENGTH = (3);
constexpr uint8_t CONFIG_DLPF_CFG_BIT        = (2);  // [2:0]
constexpr uint8_t CONFIG_DLPF_CFG_LENGTH     = (3);
//------------------------------------------------------------------------------
constexpr uint8_t GYRO_CONFIG              = (0x1B);
constexpr uint8_t GCONFIG_XG_ST_BIT        = (7);
constexpr uint8_t GCONFIG_YG_ST_BIT        = (6);
constexpr uint8_t GCONFIG_ZG_ST_BIT        = (5);
constexpr uint8_t GCONFIG_FS_SEL_BIT       = (4);  // [4:3]
constexpr uint8_t GCONFIG_FS_SEL_LENGTH    = (2);
constexpr uint8_t GCONFIG_FCHOICE_B        = (1);  // [1:0]
constexpr uint8_t GCONFIG_FCHOICE_B_LENGTH = (2);
//------------------------------------------------------------------------------
constexpr uint8_t ACCEL_CONFIG          = (0x1C);
constexpr uint8_t ACONFIG_XA_ST_BIT     = (7);
constexpr uint8_t ACONFIG_YA_ST_BIT     = (6);
constexpr uint8_t ACONFIG_ZA_ST_BIT     = (5);
constexpr uint8_t ACONFIG_FS_SEL_BIT    = (4);  // [4:3]
constexpr uint8_t ACONFIG_FS_SEL_LENGTH = (2);
constexpr uint8_t ACONFIG_HPF_BIT       = (2);  // [2:0]
constexpr uint8_t ACONFIG_HPF_LENGTH    = (3);
//------------------------------------------------------------------------------
constexpr uint8_t FF_THR       = (0x1D);
constexpr uint8_t FF_DUR       = (0x1E);
constexpr uint8_t MOTION_THR   = (0x1F);  // [7:0] // MPU9250_REG_WOM_THR
constexpr uint8_t MOTION_DUR   = (0x20);
constexpr uint8_t ZRMOTION_THR = (0x21);
constexpr uint8_t ZRMOTION_DUR = (0x22);
//------------------------------------------------------------------------------
constexpr uint8_t FIFO_EN           = (0x23);
constexpr uint8_t FIFO_TEMP_EN_BIT  = (7);
constexpr uint8_t FIFO_XGYRO_EN_BIT = (6);
constexpr uint8_t FIFO_YGYRO_EN_BIT = (5);
constexpr uint8_t FIFO_ZGYRO_EN_BIT = (4);
constexpr uint8_t FIFO_ACCEL_EN_BIT = (3);
constexpr uint8_t FIFO_SLV_2_EN_BIT = (2);
constexpr uint8_t FIFO_SLV_1_EN_BIT = (1);
constexpr uint8_t FIFO_SLV_0_EN_BIT = (0);
//------------------------------------------------------------------------------
constexpr uint8_t I2C_MST_CTRL                  = (0x24);
constexpr uint8_t I2CMST_CTRL_MULT_EN_BIT       = (7);
constexpr uint8_t I2CMST_CTRL_WAIT_FOR_ES_BIT   = (6);
constexpr uint8_t I2CMST_CTRL_SLV_3_FIFO_EN_BIT = (5);
constexpr uint8_t I2CMST_CTRL_P_NSR_BIT         = (4);
constexpr uint8_t I2CMST_CTRL_CLOCK_BIT         = (3);  // [3:0]
constexpr uint8_t I2CMST_CTRL_CLOCK_LENGTH      = (4);
//------------------------------------------------------------------------------
constexpr uint8_t I2C_SLV0_ADDR     = (0x25);
constexpr uint8_t I2C_SLV_RNW_BIT   = (7);  // same for all I2C_SLV registers
constexpr uint8_t I2C_SLV_ID_BIT    = (6);  // [6:0]
constexpr uint8_t I2C_SLV_ID_LENGTH = (7);
//------------------------------------------------------------------------------
constexpr uint8_t I2C_SLV0_REG = (0x26);  // [7:0]
//------------------------------------------------------------------------------
constexpr uint8_t I2C_SLV0_CTRL       = (0x27);
constexpr uint8_t I2C_SLV_EN_BIT      = (7);  // same for all I2C_SLV registers
constexpr uint8_t I2C_SLV_BYTE_SW_BIT = (6);
constexpr uint8_t I2C_SLV_REG_DIS_BIT = (5);
constexpr uint8_t I2C_SLV_GRP_BIT     = (4);
constexpr uint8_t I2C_SLV_LEN_BIT     = (3);  // [3:0]
constexpr uint8_t I2C_SLV_LEN_LENGTH  = (4);
//------------------------------------------------------------------------------
constexpr uint8_t I2C_SLV1_ADDR = (0x28);  // see SLV0 for bit defines
constexpr uint8_t I2C_SLV1_REG  = (0x29);
constexpr uint8_t I2C_SLV1_CTRL = (0x2A);
constexpr uint8_t I2C_SLV2_ADDR = (0x2B);  // see SLV0 for bit defines
constexpr uint8_t I2C_SLV2_REG  = (0x2C);
constexpr uint8_t I2C_SLV2_CTRL = (0x2D);
constexpr uint8_t I2C_SLV3_ADDR = (0x2E);  // see SLV0 for bit defines
constexpr uint8_t I2C_SLV3_REG  = (0x2F);
constexpr uint8_t I2C_SLV3_CTRL = (0x30);
constexpr uint8_t I2C_SLV4_ADDR = (0x31);  // see SLV0 for bit defines
constexpr uint8_t I2C_SLV4_REG  = (0x32);
constexpr uint8_t I2C_SLV4_DO   = (0x33);  // [7:0]
//------------------------------------------------------------------------------
constexpr uint8_t I2C_SLV4_CTRL             = (0x34);
constexpr uint8_t I2C_SLV4_EN_BIT           = (7);
constexpr uint8_t I2C_SLV4_DONE_INT_BIT     = (6);
constexpr uint8_t I2C_SLV4_REG_DIS_BIT      = (5);
constexpr uint8_t I2C_SLV4_MST_DELAY_BIT    = (4);  // [4:0]
constexpr uint8_t I2C_SLV4_MST_DELAY_LENGTH = (5);
//------------------------------------------------------------------------------
constexpr uint8_t I2C_SLV4_DI = (0x35);  // [7:0]
//------------------------------------------------------------------------------
constexpr uint8_t I2C_MST_STATUS               = (0x36);
constexpr uint8_t I2CMST_STAT_PASS_THROUGH_BIT = (7);
constexpr uint8_t I2CMST_STAT_SLV4_DONE_BIT    = (6);
constexpr uint8_t I2CMST_STAT_LOST_ARB_BIT     = (5);
constexpr uint8_t I2CMST_STAT_SLV4_NACK_BIT    = (4);
constexpr uint8_t I2CMST_STAT_SLV3_NACK_BIT    = (3);
constexpr uint8_t I2CMST_STAT_SLV2_NACK_BIT    = (2);
constexpr uint8_t I2CMST_STAT_SLV1_NACK_BIT    = (1);
constexpr uint8_t I2CMST_STAT_SLV0_NACK_BIT    = (0);
//------------------------------------------------------------------------------
constexpr uint8_t INT_PIN_CONFIG                = (0x37);
constexpr uint8_t INT_CFG_LEVEL_BIT             = (7);
constexpr uint8_t INT_CFG_OPEN_BIT              = (6);
constexpr uint8_t INT_CFG_LATCH_EN_BIT          = (5);
constexpr uint8_t INT_CFG_ANYRD_2CLEAR_BIT      = (4);
constexpr uint8_t INT_CFG_FSYNC_LEVEL_BIT       = (3);
constexpr uint8_t INT_CFG_FSYNC_INT_MODE_EN_BIT = (2);
constexpr uint8_t INT_CFG_I2C_BYPASS_EN_BIT     = (1);
constexpr uint8_t INT_CFG_CLOCKOUT_EN_BIT       = (0);
//------------------------------------------------------------------------------
constexpr uint8_t INT_ENABLE                   = (0x38);
constexpr uint8_t INT_ENABLE_FREEFALL_BIT      = (7);
constexpr uint8_t INT_ENABLE_MOTION_BIT        = (6);
constexpr uint8_t INT_ENABLE_ZEROMOT_BIT       = (5);
constexpr uint8_t INT_ENABLE_FIFO_OFLOW_BIT    = (4);
constexpr uint8_t INT_ENABLE_I2C_MST_FSYNC_BIT = (3);
constexpr uint8_t INT_ENABLE_PLL_RDY_BIT       = (2);
constexpr uint8_t INT_ENABLE_DMP_RDY_BIT       = (1);
constexpr uint8_t INT_ENABLE_RAW_DATA_RDY_BIT  = (0);
//------------------------------------------------------------------------------
constexpr uint8_t DMP_INT_STATUS   = (0x39);
constexpr uint8_t DMP_INT_STATUS_0 = (0);
constexpr uint8_t DMP_INT_STATUS_1 = (1);
constexpr uint8_t DMP_INT_STATUS_2 = (2);
constexpr uint8_t DMP_INT_STATUS_3 = (3);
constexpr uint8_t DMP_INT_STATUS_4 = (4);
constexpr uint8_t DMP_INT_STATUS_5 = (5);
//------------------------------------------------------------------------------
constexpr uint8_t INT_STATUS                  = (0x3A);
constexpr uint8_t INT_STATUS_FREEFALL_BIT     = (7);
constexpr uint8_t INT_STATUS_MOTION_BIT       = (6);
constexpr uint8_t INT_STATUS_ZEROMOT_BIT      = (5);
constexpr uint8_t INT_STATUS_FIFO_OFLOW_BIT   = (4);
constexpr uint8_t INT_STATUS_I2C_MST_BIT      = (3);
constexpr uint8_t INT_STATUS_PLL_RDY_BIT      = (2);
constexpr uint8_t INT_STATUS_DMP_RDY_BIT      = (1);
constexpr uint8_t INT_STATUS_RAW_DATA_RDY_BIT = (0);
//------------------------------------------------------------------------------
constexpr uint8_t ACCEL_XOUT_H     = (0x3B);  // [15:0]
constexpr uint8_t ACCEL_XOUT_L     = (0x3C);
constexpr uint8_t ACCEL_YOUT_H     = (0x3D);  // [15:0]
constexpr uint8_t ACCEL_YOUT_L     = (0x3E);
constexpr uint8_t ACCEL_ZOUT_H     = (0x3F);  // [15:0]
constexpr uint8_t ACCEL_ZOUT_L     = (0x40);
constexpr uint8_t TEMP_OUT_H       = (0x41);  // [15:0]
constexpr uint8_t TEMP_OUT_L       = (0x42);
constexpr uint8_t GYRO_XOUT_H      = (0x43);  // [15:0]
constexpr uint8_t GYRO_XOUT_L      = (0x44);
constexpr uint8_t GYRO_YOUT_H      = (0x45);  // [15:0]
constexpr uint8_t GYRO_YOUT_L      = (0x46);
constexpr uint8_t GYRO_ZOUT_H      = (0x47);  // [15:0]
constexpr uint8_t GYRO_ZOUT_L      = (0x48);
constexpr uint8_t EXT_SENS_DATA_00 = (0x49);  // Stores data read from Slave 0, 1, 2, and 3
constexpr uint8_t EXT_SENS_DATA_01 = (0x4A);
constexpr uint8_t EXT_SENS_DATA_02 = (0x4B);
constexpr uint8_t EXT_SENS_DATA_03 = (0x4C);
constexpr uint8_t EXT_SENS_DATA_04 = (0x4D);
constexpr uint8_t EXT_SENS_DATA_05 = (0x4E);
constexpr uint8_t EXT_SENS_DATA_06 = (0x4F);
constexpr uint8_t EXT_SENS_DATA_07 = (0x50);
constexpr uint8_t EXT_SENS_DATA_08 = (0x51);
constexpr uint8_t EXT_SENS_DATA_09 = (0x52);
constexpr uint8_t EXT_SENS_DATA_10 = (0x53);
constexpr uint8_t EXT_SENS_DATA_11 = (0x54);
constexpr uint8_t EXT_SENS_DATA_12 = (0x55);
constexpr uint8_t EXT_SENS_DATA_13 = (0x56);
constexpr uint8_t EXT_SENS_DATA_14 = (0x57);
constexpr uint8_t EXT_SENS_DATA_15 = (0x58);
constexpr uint8_t EXT_SENS_DATA_16 = (0x59);
constexpr uint8_t EXT_SENS_DATA_17 = (0x5A);
constexpr uint8_t EXT_SENS_DATA_18 = (0x5B);
constexpr uint8_t EXT_SENS_DATA_19 = (0x5C);
constexpr uint8_t EXT_SENS_DATA_20 = (0x5D);
constexpr uint8_t EXT_SENS_DATA_21 = (0x5E);
constexpr uint8_t EXT_SENS_DATA_22 = (0x5F);
constexpr uint8_t EXT_SENS_DATA_23 = (0x60);
constexpr uint8_t I2C_SLV0_DO      = (0x63);
constexpr uint8_t I2C_SLV1_DO      = (0x64);
constexpr uint8_t I2C_SLV2_DO      = (0x65);
constexpr uint8_t I2C_SLV3_DO      = (0x66);
//------------------------------------------------------------------------------
constexpr uint8_t I2C_MST_DELAY_CRTL       = (0x67);
constexpr uint8_t I2CMST_DLY_ES_SHADOW_BIT = (7);
constexpr uint8_t I2CMST_DLY_SLV4_EN_BIT   = (4);
constexpr uint8_t I2CMST_DLY_SLV3_EN_BIT   = (3);
constexpr uint8_t I2CMST_DLY_SLV2_EN_BIT   = (2);
constexpr uint8_t I2CMST_DLY_SLV1_EN_BIT   = (1);
constexpr uint8_t I2CMST_DLY_SLV0_EN_BIT   = (0);
//------------------------------------------------------------------------------
constexpr uint8_t SIGNAL_PATH_RESET   = (0x68);
constexpr uint8_t SPATH_GYRO_RST_BIT  = (2);
constexpr uint8_t SPATH_ACCEL_RST_BIT = (1);
constexpr uint8_t SPATH_TEMP_RST_BIT  = (0);
//------------------------------------------------------------------------------
constexpr uint8_t USER_CTRL                   = (0x6A);
constexpr uint8_t USERCTRL_DMP_EN_BIT         = (7);
constexpr uint8_t USERCTRL_FIFO_EN_BIT        = (6);
constexpr uint8_t USERCTRL_I2C_MST_EN_BIT     = (5);
constexpr uint8_t USERCTRL_I2C_IF_DIS_BIT     = (4);
constexpr uint8_t USERCTRL_DMP_RESET_BIT      = (3);
constexpr uint8_t USERCTRL_FIFO_RESET_BIT     = (2);
constexpr uint8_t USERCTRL_I2C_MST_RESET_BIT  = (1);
constexpr uint8_t USERCTRL_SIG_COND_RESET_BIT = (0);
//------------------------------------------------------------------------------
constexpr uint8_t PWR_MGMT1             = (0x6B);
constexpr uint8_t PWR1_DEVICE_RESET_BIT = (7);
constexpr uint8_t PWR1_SLEEP_BIT        = (6);
constexpr uint8_t PWR1_CYCLE_BIT        = (5);
constexpr uint8_t PWR1_GYRO_STANDBY_BIT = (4);
constexpr uint8_t PWR1_TEMP_DIS_BIT     = (3);
constexpr uint8_t PWR1_CLKSEL_BIT       = (2);
constexpr uint8_t PWR1_CLKSEL_LENGTH    = (3);
//------------------------------------------------------------------------------
constexpr uint8_t PWR_MGMT2                = (0x6C);
constexpr uint8_t PWR2_LP_WAKE_CTRL_BIT    = (7);
constexpr uint8_t PWR2_LP_WAKE_CTRL_LENGTH = (2);
constexpr uint8_t PWR2_STBY_XA_BIT         = (5);
constexpr uint8_t PWR2_STBY_YA_BIT         = (4);
constexpr uint8_t PWR2_STBY_ZA_BIT         = (3);
constexpr uint8_t PWR2_STBY_XG_BIT         = (2);
constexpr uint8_t PWR2_STBY_YG_BIT         = (1);
constexpr uint8_t PWR2_STBY_ZG_BIT         = (0);
constexpr uint8_t PWR2_STBY_XYZA_BITS      = (1 << PWR2_STBY_XA_BIT | 1 << PWR2_STBY_YA_BIT | 1 << PWR2_STBY_ZA_BIT);
constexpr uint8_t PWR2_STBY_XYZG_BITS      = (1 << PWR2_STBY_XG_BIT | 1 << PWR2_STBY_YG_BIT | 1 << PWR2_STBY_ZG_BIT);
//------------------------------------------------------------------------------
constexpr uint8_t BANK_SEL                  = (0x6D);
constexpr uint8_t BANKSEL_PRFTCH_EN_BIT     = (6);
constexpr uint8_t BANKSEL_CFG_USER_BANK_BIT = (5);
constexpr uint8_t BANKSEL_MEM_SEL_BIT       = (4);
constexpr uint8_t BANKSEL_MEM_SEL_LENGTH    = (5);
//------------------------------------------------------------------------------
constexpr uint8_t MEM_START_ADDR = (0x6E);
constexpr uint8_t MEM_R_W        = (0x6F);
constexpr uint8_t PRGM_START_H   = (0x70);
constexpr uint8_t PRGM_START_L   = (0x71);
constexpr uint8_t FIFO_COUNT_H   = (0x72);  // [15:0]
constexpr uint8_t FIFO_COUNT_L   = (0x73);
constexpr uint8_t FIFO_R_W       = (0x74);
constexpr uint8_t WHO_AM_I       = (0x75);

/*******************************************************************************
 * MPU6000, MPU6050 and MPU9150 registers
 ******************************************************************************/
#if defined CONFIG_MPU6050
constexpr uint8_t XG_OTP_OFFSET_TC = (0x00);  // [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
//------------------------------------------------------------------------------
constexpr uint8_t YG_OTP_OFFSET_TC = (0x01);  // [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
constexpr uint8_t TC_PWR_MODE_BIT  = (7);     // note: TC = temperature compensation, i think
//------------------------------------------------------------------------------
constexpr uint8_t ZG_OTP_OFFSET_TC = (0x02);  // [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
constexpr uint8_t X_FINE_GAIN      = (0x03);  // [7:0] X_FINE_GAIN
constexpr uint8_t Y_FINE_GAIN      = (0x04);  // [7:0] Y_FINE_GAIN
constexpr uint8_t Z_FINE_GAIN      = (0x05);  // [7:0] Z_FINE_GAIN
constexpr uint8_t XA_OFFSET_H      = (0x06);  // [15:1] XA_OFFS
constexpr uint8_t XA_OFFSET_L      = (0x07);  // note: TC: bit [0]
constexpr uint8_t YA_OFFSET_H      = (0x08);  // [15:1] YA_OFFS
constexpr uint8_t YA_OFFSET_L      = (0x09);  // note: TC: bit [0]
constexpr uint8_t ZA_OFFSET_H      = (0x0A);  // [15:1] ZA_OFFS
constexpr uint8_t ZA_OFFSET_L      = (0x0B);  // note: TC: bit [0]
constexpr uint8_t SELF_TEST_X      = (0x0D);
constexpr uint8_t SELF_TEST_Y      = (0x0E);
constexpr uint8_t SELF_TEST_Z      = (0x0F);
constexpr uint8_t SELF_TEST_A      = (0x10);
//------------------------------------------------------------------------------
constexpr uint8_t MOTION_DETECT_STATUS = (0x61);
constexpr uint8_t MOT_STATUS_X_NEG_BIT = (7);
constexpr uint8_t MOT_STATUS_X_POS_BIT = (6);
constexpr uint8_t MOT_STATUS_Y_NEG_BIT = (5);
constexpr uint8_t MOT_STATUS_Y_POS_BIT = (4);
constexpr uint8_t MOT_STATUS_Z_NEG_BIT = (3);
constexpr uint8_t MOT_STATUS_Z_POS_BIT = (2);
constexpr uint8_t MOT_STATUS_ZRMOT_BIT = (0);
//------------------------------------------------------------------------------
constexpr uint8_t MOTION_DETECT_CTRL            = (0x69);
constexpr uint8_t MOTCTRL_ACCEL_ON_DELAY_BIT    = (5);  // [5:4]
constexpr uint8_t MOTCTRL_ACCEL_ON_DELAY_LENGTH = (2);
constexpr uint8_t MOTCTRL_FF_COUNT_BIT          = (3);  // [3:2]
constexpr uint8_t MOTCTRL_FF_COUNT_LENGTH       = (2);
constexpr uint8_t MOTCTRL_MOT_COUNT_BIT         = (1);  // [1:0]
constexpr uint8_t MOTCTRL_MOT_COUNT_LENGTH      = (2);
//------------------------------------------------------------------------------
#endif

/*******************************************************************************
 * MPU6500 and MPU9250 registers
 ******************************************************************************/
#if defined CONFIG_MPU6500
constexpr uint8_t SELF_TEST_X_GYRO  = (0x00);  // XG_ST_DATA[7:0]
constexpr uint8_t SELF_TEST_Y_GYRO  = (0x01);  // YG_ST_DATA[7:0]
constexpr uint8_t SELF_TEST_Z_GYRO  = (0x02);  // ZG_ST_DATA[7:0]
constexpr uint8_t SELF_TEST_X_ACCEL = (0x0D);
constexpr uint8_t SELF_TEST_Y_ACCEL = (0x0E);
constexpr uint8_t SELF_TEST_Z_ACCEL = (0x0F);
//------------------------------------------------------------------------------
constexpr uint8_t ACCEL_CONFIG2                = (0x1D);
constexpr uint8_t ACONFIG2_FIFO_SIZE_BIT       = (7);  // [7:6]
constexpr uint8_t ACONFIG2_FIFO_SIZE_LENGTH    = (2);
constexpr uint8_t ACONFIG2_ACCEL_FCHOICE_B_BIT = (3);
constexpr uint8_t ACONFIG2_A_DLPF_CFG_BIT      = (2);  // [2:0]
constexpr uint8_t ACONFIG2_A_DLPF_CFG_LENGTH   = (3);
//------------------------------------------------------------------------------
constexpr uint8_t LP_ACCEL_ODR          = (0x1E);
constexpr uint8_t LPA_ODR_CLKSEL_BIT    = (3);  // [3:0]
constexpr uint8_t LPA_ODR_CLKSEL_LENGTH = (4);
//------------------------------------------------------------------------------
constexpr uint8_t ACCEL_INTEL_CTRL     = (0x69);
constexpr uint8_t ACCEL_INTEL_EN_BIT   = (7);
constexpr uint8_t ACCEL_INTEL_MODE_BIT = (6);
//------------------------------------------------------------------------------
constexpr uint8_t XA_OFFSET_H = (0x77);
constexpr uint8_t XA_OFFSET_L = (0x78);
constexpr uint8_t YA_OFFSET_H = (0x7A);
constexpr uint8_t YA_OFFSET_L = (0x7B);
constexpr uint8_t ZA_OFFSET_H = (0x7D);
constexpr uint8_t ZA_OFFSET_L = (0x7E);
#endif

/*******************************************************************************
 * MPU9150 and MPU9250 Magnetometer registers (AK89xx)
 ******************************************************************************/
#if defined CONFIG_MPU_AK89xx
/*! Magnetometer Registers namespace */
namespace mag
{
constexpr uint8_t WHO_I_AM = (0x00);
constexpr uint8_t INFO     = (0x01);
//------------------------------------------------------------------------------
constexpr uint8_t STATUS1              = (0x02);
constexpr uint8_t STATUS1_DATA_RDY_BIT = (0);
//------------------------------------------------------------------------------
constexpr uint8_t HXL = (0x03);
constexpr uint8_t HXH = (0x04);
constexpr uint8_t HYL = (0x05);
constexpr uint8_t HYH = (0x06);
constexpr uint8_t HZL = (0x07);
constexpr uint8_t HZH = (0x08);
//------------------------------------------------------------------------------
constexpr uint8_t STATUS2              = (0x09);
constexpr uint8_t STATUS2_OVERFLOW_BIT = (3);
//------------------------------------------------------------------------------
constexpr uint8_t CONTROL1             = (0x0A);
constexpr uint8_t CONTROL1_MODE_BIT    = (3);
constexpr uint8_t CONTROL1_MODE_LENGTH = (4);
//------------------------------------------------------------------------------
constexpr uint8_t ASTC               = (0x0C);
constexpr uint8_t ASTC_SELF_TEST_BIT = (6);
//------------------------------------------------------------------------------
constexpr uint8_t TEST1 = (0x0D);
constexpr uint8_t TEST2 = (0x0E);
//------------------------------------------------------------------------------
constexpr uint8_t I2CDIS               = (0x0F);
constexpr uint8_t I2CDIS_DISABLE_VALUE = (0x1B);
//------------------------------------------------------------------------------
constexpr uint8_t ASAX = (0x10);
constexpr uint8_t ASAY = (0x11);
constexpr uint8_t ASAZ = (0x12);

/*******************************************************************************
 * MPU9150 Magnetometer (AK8975)
 ******************************************************************************/
#if defined CONFIG_MPU_AK8975
constexpr uint8_t STATUS2_DATA_ERROR_BIT = (2);
#endif

/*******************************************************************************
 * MPU9250 Magnetometer (AK8963)
 ******************************************************************************/
#if defined CONFIG_MPU_AK8963
constexpr uint8_t STATUS1_DATA_OVERRUN_BIT = (1);
constexpr uint8_t STATUS2_BIT_OUTPUT_M_BIT = (4);
constexpr uint8_t CONTROL1_BIT_OUTPUT_BIT  = (4);
//------------------------------------------------------------------------------
constexpr uint8_t CONTROL2                = (0x0B);
constexpr uint8_t CONTROL2_SOFT_RESET_BIT = (0);
//------------------------------------------------------------------------------
#endif

}  // namespace mag
#endif  // defined AK89xx

}  // namespace regs

}  // namespace mpud

#endif /* end of include guard: _MPU_REGISTERS_HPP_ */
