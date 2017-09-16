#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "i2cbus.h"
#include "MPU.h"

#ifdef __cplusplus
extern "C" {
    void app_main();
}
#endif
#define MPU_ADDRESS (0x68)

static const char* TAG = {"app_main"};


void test_task(void*) {
    // init I2C0 and reset MPU
    ESP_ERROR_CHECK(i2c0.begin(GPIO_NUM_21, GPIO_NUM_22, 400000));
    ESP_ERROR_CHECK(i2c0.write_bit(MPU_ADDRESS, 0x6B, 7, 1));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // read sleep mode status
    uint8_t data;
    ESP_ERROR_CHECK(i2c0.read_bit(MPU_ADDRESS, 0x6B, 6, &data));
    ESP_LOGI(TAG, "sleep = %d", data);
    // disable sleep mode
    ESP_ERROR_CHECK(i2c0.write_bit(MPU_ADDRESS, 0x6B, 6, 0));
    ESP_ERROR_CHECK(i2c0.read_bit(MPU_ADDRESS, 0x6B, 6, &data));
    ESP_LOGI(TAG, "sleep = %d", data);

    while(1){
        uint8_t buffer[2];
        int16_t az;
        ESP_ERROR_CHECK(i2c0.read_bytes(MPU_ADDRESS, 0x3F, 2, buffer));
        az = (buffer[0] << 8) | buffer[1];
        ESP_LOGI(TAG, "accel.Z=%+d", az);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}



void app_main() {
    printf(">> APP_MAIN\n");
    xTaskCreate(&test_task, "test_task", 4096, NULL, 2, NULL);
}
