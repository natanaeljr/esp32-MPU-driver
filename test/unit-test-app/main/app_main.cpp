#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "unity_config.h"
#include "sdkconfig.h"

#ifdef CONFIG_LOG_COLORS
#define ESC(x) "\033" x
#else
#define ESC(x) ""
#endif


void unityTask(void *pvParameters)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    unity_run_menu();
    while(1);
}


extern "C" void app_main()
{
    printf("\n"
        ESC("[37;40m") " MPU "
        ESC("[0m") " "
        ESC("[32;40m") " Unit "
        ESC("[42;30m") " test "
        ESC("[0m") "\n" );
    fflush(stdout);

    /* Uncomment to auto-start all tests */
    // UNITY_BEGIN();
    // unity_run_tests_with_filter("[MPU]");
    // UNITY_END();
    
    // Note: if unpinning this task, change the way run times are calculated in unity_platform
    xTaskCreatePinnedToCore(unityTask, "unityTask", 8192, NULL,
                            UNITY_FREERTOS_PRIORITY, NULL, UNITY_FREERTOS_CPU);
}
