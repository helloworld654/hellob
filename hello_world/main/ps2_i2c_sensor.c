#include "stdio.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern uint8_t ble_client_is_connect(void);
extern void gattc_write_demo(uint8_t *p_data,uint8_t length);

#define LEN    20
void i2c_ps2_sensor_task(void *arg)
{
    uint8_t data[LEN];
    for(uint8_t i=0;i<LEN;i++){
        data[i] = i+5;
    }
    while(true){
        if(ble_client_is_connect()){
            gattc_write_demo(data,LEN);
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
