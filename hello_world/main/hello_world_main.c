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

extern void gatts_app_main(void);
extern void uart_evnet_app_main(void);

void app_main(void)
{
    printf("Hello world!\n");
    gatts_app_main();
    uart_evnet_app_main();

    while(1)
    {
        printf("every one second\r\n");
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}
