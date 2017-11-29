#include <stdio.h>
#include <stdint.h>
#include <math.h>
// esp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
// lib
#include "mpu.hpp"
#include "I2Cbus.hpp"


#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
#define I2C_CLOCK_SPEED 400000 // hz

#define GRAPH_LENGTH 31 // columns
#define REFRESH_RATE 10 // hz

/*
 * This example uses ANSI escape code to control the terminal screen
 * Your terminal has to support escape sequencies in order to the example work properly.
 */


void print_graph(int value, int min, int max) {
    int pos = (GRAPH_LENGTH) * ((float)(value - min) / (max - min));
    printf("\033[0K[\033[%dC+\033[%dC]\033[5C%+d", pos, (GRAPH_LENGTH - pos - 1), value);
}


extern "C" void app_main() {
    MPU_t MPU = MPU_t();
    mpu_axes_t accel;
    mpu_axes_t gyro;
    int temp;

    // setup
    ESP_ERROR_CHECK(I2Cbus0.begin(I2C_SDA, I2C_SCL, I2C_CLOCK_SPEED));
    MPU.setBus(I2Cbus0);
    MPU.setAddressHandle(MPU_DEFAULT_ADDRESS);
    ESP_ERROR_CHECK(MPU.initialize());
    ESP_ERROR_CHECK(MPU.setGyroFullScale(MPU_GYRO_FS_2000DPS));
    ESP_ERROR_CHECK(MPU.setAccelFullScale(MPU_ACCEL_FS_2G));

    printf( "\033[2J"   // clear the screen
            "\033[H"    // set cursor to 0;0
            "\033[?25l" // hide cursor
            LOG_COLOR(LOG_COLOR_BROWN)
            "\033[7m"   // reverse colors
            "> MPU example Raw Data Graph"
            LOG_RESET_COLOR
            "\n\n");
    fflush(stdout);

    // log configuration
    printf(LOG_COLOR(LOG_COLOR_GREEN) "");
    printf("Gyroscope range: %dº/s \n", (int)(250 * pow(2.0, MPU.getGyroFullScale())));
    printf("Accelerometer range: %dG\n", (int)(pow(2, MPU.getAccelFullScale() + 1)));
    printf("Low pass filter: %d\n\n", MPU.getLowPassFilter());
    printf(LOG_RESET_COLOR "");

    printf("X Acceleration\n");
    printf("Y Acceleration\n");
    printf("Z Acceleration\n\n");

    printf("X Rotation\n");
    printf("Y Rotation\n");
    printf("Z Rotation\n\n");

    printf("Temperature:\n");

    fflush(stdout);

    while(1) {
        /* read raw sensor measurements*/
        accel = MPU.getAcceleration();
        gyro = MPU.getRotation();
        temp = MPU.getTemperature();

        /* print gyro/accel graphs */
        printf("\033[7;20H"); // set graph position row:7 column:20
        print_graph(accel.x, -20000, 20000);

        printf("\033[8;20H"); // set graph position row:8 column:20
        print_graph(accel.y, -20000, 20000);

        printf("\033[9;20H");
        print_graph(accel.z, -20000, 20000);

        printf("\033[11;20H");
        print_graph(gyro.x, -5000, 5000);

        printf("\033[12;20H");
        print_graph(gyro.y, -5000, 5000);

        printf("\033[13;20H");
        print_graph(gyro.z, -5000, 5000);

        /* print temperature */
        printf("\033[15;14H" // row:15, column 14
               "\033[0K"); // clear inline to the right
        // Temperature in degrees Celsius
        // = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 35degC
        printf("%dºC  (%+d)", (int)((((float)(temp + 521) / 340) + 35)), temp);
    
        vTaskDelay(1000 / REFRESH_RATE / portTICK_PERIOD_MS);
    }

}







