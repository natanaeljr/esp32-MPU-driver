/* LIBRARIES */
// c/c++
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
#include "MPU.h"
#include "I2Cbus.h"


/* DEFINES */
#ifdef __cplusplus
extern "C" {
    void app_main(void);
}
#endif

/* CONSTANTS */
static const char* TAG = "app";

/* OBJECTS */
MPU_t mpu = MPU_t();


/* MAIN */
void app_main() {
    printf(LOG_BOLD("97") "\n[APP_MAIN]" LOG_RESET_COLOR "\n");
    // setup    
    I2Cbus0.begin(GPIO_NUM_21, GPIO_NUM_22, 400000U);
    mpu.setI2Cbus(I2Cbus0);
    mpu.setAddress(MPU_DEFAULT_ADDRESS);

    if(!mpu.initialize()) {
        ESP_LOGD(TAG, "Initialize OK");
    }
    else {
        ESP_LOGE(TAG, "Initialize FAIL, error %#x", mpu.getLastError());
    }

    vTaskDelay(portMAX_DELAY);
}