/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "hello_world_main.h"
#include "ps2_i2c_sensor.h"
#include "freertos_study.h"
#include "ble_at_cmd.h"

#define BLINK_GPIO    2

extern void gattc_app_main(void);
extern void gatts_app_main(void);
extern void uart_evnet_app_main(void);
extern void pwm_app_main(void);
extern void i2c_app_main(void);
uint8_t led_mode;

void app_main(void)
{
#if (defined(BLE_CAR_CLIENT) && BLE_CAR_CLIENT) || \
    (defined(BLE_CAR_SERVER) && BLE_CAR_SERVER)
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 0);  // set led off
    uint32_t led_interval = 0;
    led_mode = 0;
#endif

#if defined(BLE_CAR_CLIENT) && BLE_CAR_CLIENT
    gattc_app_main();
    i2c_ps2_app_main();
#endif

#if defined(BLE_CAR_SERVER) && BLE_CAR_SERVER
    pwm_app_main();
    gatts_app_main();
#endif

#if defined(FREERTOS_STUDY) && FREERTOS_STUDY
    task_create_test();
#endif

#if defined(BLE_AT_TEST) && BLE_AT_TEST
    at_cmd_task_init();
    uart_evnet_app_main();
#endif

    while(1)
    {
#if (defined(BLE_CAR_CLIENT) && BLE_CAR_CLIENT) || \
    (defined(BLE_CAR_SERVER) && BLE_CAR_SERVER)
        switch(led_mode){
            case 0:
                led_interval = 80;
                break;
            case 1:
                led_interval = 200;
                break;
            case 2:
                led_interval = 5000;
                break;
            case 3:
                led_interval = 80;
                break;
            default:
                break;
        }
        if(led_interval < 2000){
            gpio_set_level(BLINK_GPIO, 0);
            vTaskDelay(led_interval/portTICK_PERIOD_MS);
        }
        if(led_interval > 100){
            gpio_set_level(BLINK_GPIO, 1);
            vTaskDelay(led_interval/portTICK_PERIOD_MS);
        }
#else
        // printf("\r\n[%s] Hello world",__func__);
        vTaskDelay(1000/portTICK_PERIOD_MS);
#endif
    }
}
