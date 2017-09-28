#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <stdint.h>
// private libs
#include "I2Cbus.h"
#include "MPU.h"
#include "MPUdmp.h"

#ifdef __cplusplus
extern "C" {
    void app_main();
}
#endif

static const char* TAG = {"app_main"};


#define UNIT_CHECK(str, x, y)                                       \
    do {                                                            \
        esp_err_t err = x;                                          \
        if(err == y) { ESP_LOGD(TAG, str " OK");                    \
        } else ESP_LOGE(TAG, str " FAIL -> %#x (%d)", err, err);    \
    }while(0)

#define STR(x) #x



const gpio_num_t I2C_SDA = GPIO_NUM_21;
const gpio_num_t I2C_SCL = GPIO_NUM_22;

MPU_t mpu = MPU_t(I2Cbus0);

void app_main() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("\n\n>> APP_MAIN\n");

    I2Cbus0.begin(I2C_SDA, I2C_SCL, 400000L);

    while(true) {

        UNIT_CHECK("testConnection", mpu.testConnection(), true);
        UNIT_CHECK("reset", mpu.reset(), ESP_OK);
        UNIT_CHECK("sleep", mpu.getSleepStatus(), true);
        UNIT_CHECK("initialize", mpu.initialize(), ESP_OK);
        UNIT_CHECK("awake", mpu.getSleepStatus(), false);
        UNIT_CHECK("deviceID", mpu.getDeviceID(), 0x34);
        printf("\n");

        UNIT_CHECK("DMP initialize", mpu.dmp.initialize(), ESP_OK);
        ESP_LOGD(TAG, "DMP init getFeatures: %#X", mpu.dmp.getFeaturesEnabled());
        UNIT_CHECK("DMP getFeatures ", mpu.getLastError(), ESP_OK);
        UNIT_CHECK("DMP setFeatures ", mpu.dmp.setFeaturesEnabled(0), ESP_OK);
        UNIT_CHECK("DMP getFeatures ", mpu.dmp.getFeaturesEnabled(), (0 | DMP_FEATURE_PEDOMETER));

        ESP_LOGD(TAG, "DMP init getFIFOrate: %d", mpu.dmp.getFIFORate());
        UNIT_CHECK("DMP getFIFOrate", mpu.getLastError(), ESP_OK);
        UNIT_CHECK("DMP setFIFOrate", mpu.dmp.setFIFORate(10), ESP_OK);
        UNIT_CHECK("DMP getFIFOrate", mpu.dmp.getFIFORate(), 10);

        
        UNIT_CHECK("DMP getEnabled false", mpu.dmp.getEnabled(), false);
        UNIT_CHECK("DMP setEnabled", mpu.dmp.setEnabled(true), ESP_OK);
        UNIT_CHECK("DMP getEnabled true", mpu.dmp.getEnabled(), true);

        while(1) {
            while(!mpu.dmp.packetAvailable());
            ESP_LOGD(TAG, "FIFOcount: %d", mpu.getFIFOCount());

            // uint8_t packet[DMP_PACKET_SIZE_MAX];
            // UNIT_CHECK("DMP getFIFOPacket", mpu.dmp.getFIFOPacket(packet), ESP_OK);

            // mpu_axis_t gyro;
            // UNIT_CHECK("DMP getGyro", mpu.dmp.getGyro(&gyro, packet), ESP_OK);
            // ESP_LOGD(TAG, "gx=%+d gy=%+d gz=%+d", gyro.x, gyro.y, gyro.z);


            // mpu_axis_t accel;
            // UNIT_CHECK("DMP getAcccel", mpu.dmp.getAccel(&accel, packet), ESP_OK);
            // ESP_LOGD(TAG, "a.x=%+d a.y=%+d a.z=%+d", accel.x, accel.y, accel.z);


            // int32_t quat[4];
            // UNIT_CHECK("DMP getQuaternion", mpu.dmp.getQuaternion(quat, packet), ESP_OK);
            // ESP_LOGD(TAG, "q[0]=%+d  q[1]=%+d  q[2]=%+d  q[3]=%+d", quat[0], quat[1], quat[2], quat[3]);

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(portMAX_DELAY);
    }
    
}
