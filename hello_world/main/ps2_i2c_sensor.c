#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ps2_i2c_sensor.h"
#include "i2c_protocol.h"

extern uint8_t ble_client_is_connect(void);
extern void gattc_write_demo(uint8_t *p_data,uint8_t length);

void get_joys_state(ps2_button_data *p_ps2_state)
{
    uint8_t x_val,y_val,button_l,button_r,button_up,button_dn,button_joys;
    memset(p_ps2_state,0,sizeof(ps2_button_data));

    i2c_read_sensor_reg(JOYSTICK_LEFT_X_REG,&x_val,1);
    i2c_read_sensor_reg(JOYSTICK_LEFT_Y_REG,&y_val,1);
    i2c_read_sensor_reg(JOYSTICK_BUTTON_REG,&button_joys,1);

    i2c_read_sensor_reg(BUTOON_LEFT_REG,&button_l,1);
    i2c_read_sensor_reg(BUTOON_RIGHT_REG,&button_r,1);
    i2c_read_sensor_reg(BUTOON_UP_REG,&button_up,1);
    i2c_read_sensor_reg(BUTOON_DOWN_REG,&button_dn,1);
    
    if(x_val!=JOY_NORMAL_VAL || y_val!=JOY_NORMAL_VAL || button_joys!=BUTTON_NORMAL_VAL || button_l!=BUTTON_NORMAL_VAL \
            || button_r!=BUTTON_NORMAL_VAL || button_up!=BUTTON_NORMAL_VAL || button_dn!=BUTTON_NORMAL_VAL )
    {
        p_ps2_state->changed = 1;
    }
    p_ps2_state->x_val = x_val;
    p_ps2_state->y_val = y_val;
    p_ps2_state->button_joys = button_joys;
    p_ps2_state->button_l = button_l;
    p_ps2_state->button_r = button_r;
    p_ps2_state->button_up = button_up;
    p_ps2_state->button_dn = button_dn;
}

void i2c_ps2_sensor_task(void *arg)
{
    ps2_button_data ps2_state;
    while(true){
        if(ble_client_is_connect()){
            get_joys_state(&ps2_state);
            if(ps2_state.changed){
                printf("[%s] send ps2 state to server\r\n",__func__);
                gattc_write_demo(&ps2_state,sizeof(ps2_button_data));
                // printf("[%s] x_val:%d,y_val:%d,button_joys:%d,button_l:%d,button_r:%d,button_up:%d,button_dn:%d,\r\n",
                // __func__,ps2_state.x_val,ps2_state.y_val,ps2_state.button_joys,ps2_state.button_l,ps2_state.button_r,ps2_state.button_up,ps2_state.button_dn);
            }
        }
        else{
            // printf("[%s] not connect\r\n",__func__);
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void i2c_ps2_app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_ps2_sensor_task, "i2c_ps2_sensor_task", 1024 * 2, NULL, 10, NULL);
}
