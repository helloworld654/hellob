#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ps2_i2c_sensor.h"

extern uint8_t ble_client_is_connect(void);
extern void gattc_write_demo(uint8_t *p_data,uint8_t length);

void send_ps2_data_test(void)
{
    ps2_button_data ps2_data;
    memset(&ps2_data,0,sizeof(ps2_button_data));
    ps2_data.X_DATA = 123;
    ps2_data.Y_DATA = 224;
    ps2_data.Z = PS2_BUTTON_NORMAL;
    ps2_data.A = PS2_BUTTON_SINGLE;
    ps2_data.B = PS2_BUTTON_DOUBLE;
    ps2_data.C = PS2_BUTTON_LONG;
    gattc_write_demo(&ps2_data,sizeof(ps2_button_data));
}

void i2c_ps2_sensor_task(void *arg)
{
    while(true){
        if(ble_client_is_connect()){
            send_ps2_data_test();
            printf("[%s] already connected\r\n",__func__);
        }
        else{
            printf("[%s] not connect\r\n",__func__);
        }

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void i2c_ps2_app_main(void)
{
    // ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_ps2_sensor_task, "i2c_ps2_sensor_task", 1024 * 2, NULL, 10, NULL);
}
