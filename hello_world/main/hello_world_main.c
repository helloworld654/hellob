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

#define BLINK_GPIO    2

extern void gattc_app_main(void);
extern void gatts_app_main(void);
extern void uart_evnet_app_main(void);
extern void pwm_app_main(void);
extern void i2c_app_main(void);

void app_main(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

#if defined(BLE_CAR_CLIENT) && BLE_CAR_CLIENT
    gattc_app_main();
    i2c_app_main();
#endif

#if defined(BLE_CAR_SERVER) && BLE_CAR_SERVER
    gatts_app_main();
#endif

    printf("Hello world!\n");
    while(1)
    {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000/portTICK_PERIOD_MS);

        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
