/* LIBRARIES */
#include <stdint.h>
// esp
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
// private
#include "I2Cbus.h"
#include "MPU.h"
#include "DMP.h"
// test
#include "tester.h"


/* DEFINES */
#ifdef __cplusplus
extern "C" {
    void app_main(void);
}
#endif


/* OBJECTS */
MPU_t MPU = MPU_t();
DMP_t DMP = DMP_t(MPU);


/** TESTS 
 * This is a very basic 'unity test' program to validate 
 * the MPU class implementation 
 */
void basicTest(void);
void sampleTest(void);


/* MAIN */
void app_main() {
    printf(LOG_BOLD("97") "\n[APP_MAIN]" LOG_RESET_COLOR "\n");
    // setup    
    I2Cbus0.begin(GPIO_NUM_21, GPIO_NUM_22, 400000U);
    I2Cbus0.setTimeout(100);
    MPU.setI2Cbus(I2Cbus0);
    MPU.setAddress(MPU_DEFAULT_ADDRESS);
    // tests to run
    basicTest();
    sampleTest();

    printf("\nDone!\n");
    vTaskDelay(portMAX_DELAY);
}




void basicTest() {
    /* First, check for communication */
    ASSERT_TRUE(MPU.testConnection());

    /* Test default initializing configs */
    ASSERT_ESP_OK(MPU.initialize());
    ASSERT_FALSE(MPU.getSleepStatus());
    ASSERT_EQUAL_INT(MPU.getClockSource(), MPU_CLOCK_PLL);
    ASSERT_EQUAL_INT(MPU.getGyroFullScale(), MPU_GYRO_FS_250DPS);
    ASSERT_EQUAL_INT(MPU.getAccelFullScale(), MPU_ACCEL_FS_2G);
    ASSERT_EQUAL_INT(MPU.getLowPassFilter(), MPU_DLPF_42HZ);

    /* Change initializing configs and test then */
    ASSERT_ESP_OK(MPU.setClockSource(MPU_CLOCK_INTERNAL));
    ASSERT_EQUAL_INT(MPU.getClockSource(), MPU_CLOCK_INTERNAL);
    ASSERT_ESP_OK(MPU.setGyroFullScale(MPU_GYRO_FS_2000DPS));
    ASSERT_EQUAL_INT(MPU.getGyroFullScale(), MPU_GYRO_FS_2000DPS);
    ASSERT_ESP_OK(MPU.setAccelFullScale(MPU_ACCEL_FS_16G));
    ASSERT_EQUAL_INT(MPU.getAccelFullScale(), MPU_ACCEL_FS_16G);
    ASSERT_ESP_OK(MPU.setLowPassFilter(MPU_DLPF_265HZ_NOLPF));
    ASSERT_EQUAL_INT(MPU.getLowPassFilter(), MPU_DLPF_265HZ_NOLPF);

    /* Test sleep, reset and initialize to default again */
    ASSERT_ESP_OK(MPU.sleep(true));
    ASSERT_TRUE(MPU.getSleepStatus());
    ASSERT_ESP_OK(MPU.reset());
    ASSERT_ESP_OK(MPU.initialize());
}


void sampleTest() {
    
}




