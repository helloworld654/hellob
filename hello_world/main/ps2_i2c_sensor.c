#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ps2_i2c_sensor.h"
#include "i2c_protocol.h"

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
    uint8_t x_val,y_val,button_l,button_r,button_up,button_dn,button_joys;
    while(true){
#if 0
        if(ble_client_is_connect()){
            send_ps2_data_test();
            printf("[%s] already connected\r\n",__func__);
        }
        else{
            printf("[%s] not connect\r\n",__func__);
        }
#endif
        i2c_read_sensor_reg(JOYSTICK_LEFT_X_REG,&x_val,1);
        i2c_read_sensor_reg(JOYSTICK_LEFT_Y_REG,&y_val,1);
        i2c_read_sensor_reg(JOYSTICK_BUTTON_REG,&button_joys,1);

        i2c_read_sensor_reg(BUTOON_LEFT_REG,&button_l,1);
        i2c_read_sensor_reg(BUTOON_RIGHT_REG,&button_r,1);
        i2c_read_sensor_reg(BUTOON_UP_REG,&button_up,1);
        i2c_read_sensor_reg(BUTOON_DOWN_REG,&button_dn,1);

        printf("[%s] x_val:%d,y_val:%d,button_joys:%d,button_l:%d,button_r:%d,button_up:%d,button_dn:%d,\r\n",  \
                __func__,x_val,y_val,button_joys,button_l,button_r,button_up,button_dn);

        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void i2c_ps2_app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_ps2_sensor_task, "i2c_ps2_sensor_task", 1024 * 2, NULL, 10, NULL);
}
