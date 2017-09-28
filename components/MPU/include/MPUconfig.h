#ifndef _MPU_CONFIG_H_
#define _MPU_CONFIG_H_


/*******************************************************************************
 * SELECT MPU: uncomment one of the lines below to select a chip
 ******************************************************************************/
// #define _MPU9250_
// #define _MPU9150_
// #define _MPU6500_
#define _MPU6050_



/*******************************************************************************
 * DEBUG ERRORS: Logs any error within MPU methods. Uncomment for enable
 ******************************************************************************/
#define MPU_ERROR_LOGGER



// TODO: cache option


// Checks for FIFO corruption throught QUATERNION analyses (DMP mode only)
// TODO: elaborate
#define FIFO_CORRUPTION_CHECK








/*******************************************************************************
 * DO NOT EDIT FROM HERE ON
 ******************************************************************************/


/************************** MPU SELECTION CHECK *******************************/
#if !defined _MPU6050_ && !defined _MPU9150_ && !defined _MPU6500_ && !defined _MPU9250_
#error  Which MPU are you using? See 'MPUconfig.h' to select a MPU CHECK.
#endif
#if defined _MPU9250_ && defined _MPU9150_
#error MPU9250 and MPU9150 cannot both be defined. See 'MPUconfig.h' to select a MPU chip.
#endif
#if defined _MPU9250_ && defined _MPU6050_
#error MPU9250 and MPU6050 cannot both be defined. See 'MPUconfig.h' to select a MPU chip.
#endif
#if defined _MPU9150_ && defined _MPU6500_
#error MPU9150 and MPU6500 cannot both be defined. See 'MPUconfig.h' to select a MPU chip.
#endif
// MPU9250 is the same as MPU6500 + AK8963
// MPU9150 is the same as MPU6050 + AK8975
#if defined _MPU9250_
#ifndef _MPU6500_
#define _MPU6500_
#endif
#if defined AK8975_SECONDARY
#error MPU9250 and AK8975_SECONDARY cannot both be defined.
#elif !defined AK8963_SECONDARY
#define AK8963_SECONDARY
#endif
#elif defined _MPU9150_
#ifndef _MPU6050_
#define _MPU6050_
#endif
#if defined AK8963_SECONDARY
#error MPU9150 and AK8963_SECONDARY cannot both be defined.
#elif !defined AK8975_SECONDARY
#define AK8975_SECONDARY
#endif
#endif
// Compatible compass code
#if defined AK8975_SECONDARY || defined AK8963_SECONDARY
#define AK89xx_SECONDARY
#endif
/******************************************************************************/





#endif /* end of include guard: _MPU_CONFIG_H_ */
