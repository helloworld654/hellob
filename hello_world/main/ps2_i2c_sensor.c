#include "stdio.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void i2c_ps2_sensor_task(void *arg)
{
    while(true){
        printf("[%s] test\r\n",__func__);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}

void i2c_ps2_app_main(void)
{
    // ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_ps2_sensor_task, "i2c_ps2_sensor_task", 1024 * 2, NULL, 10, NULL);
}
