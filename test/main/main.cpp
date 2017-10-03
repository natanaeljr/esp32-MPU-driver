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
#include "I2Cbus.h"
#include "MPU.h"
#include "DMP.h"


/* DEFINES */
#ifdef __cplusplus
extern "C" {
    void app_main(void);
}
#endif

/* CONSTANTS */
static const char* TAG = "app";

/* OBJECTS */
MPU_t MPU = MPU_t();
DMP_t DMP = DMP_t(MPU);


/* MAIN */
void app_main() {
    printf(LOG_BOLD("97") "\n[APP_MAIN]" LOG_RESET_COLOR "\n");
    // setup    
    I2Cbus0.begin(GPIO_NUM_21, GPIO_NUM_22, 400000U);
    MPU.setI2Cbus(I2Cbus0);
    MPU.setAddress(MPU_DEFAULT_ADDRESS);

    while(!MPU.testConnection()){
        ESP_LOGE(TAG, "testConnection FAIL, error %#X", MPU.getLastError());
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }

    while(1) {
        if(!MPU.initialize()) {
            ESP_LOGD(TAG, "initialize OK");
        }
        else {
            ESP_LOGE(TAG, "initialize FAIL, error %#X", MPU.getLastError());
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    vTaskDelay(portMAX_DELAY);
}