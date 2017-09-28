#ifndef _MPU_DEFINE_H_
#define _MPU_DEFINE_H_

#include "sdkconfig.h"


/************************** MPU DEFINITION CHECK *******************************/
// MPU9250 is the same as MPU6500 + AK8963
// MPU9150 is the same as MPU6050 + AK8975
#if defined CONFIG_MPU9250
#define CONFIG_MPU6500 
#define AK8963_SECONDARY 
#elif defined CONFIG_MPU9150
#define CONFIG_MPU6050 
#define AK8975_SECONDARY 
#endif
// Compatible compass code
#if defined AK8975_SECONDARY || defined AK8963_SECONDARY
#define AK89xx_SECONDARY 
#endif
/******************************************************************************/




#endif /* end of include guard: _MPU_DEFINE_H_ */
