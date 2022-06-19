/* brushed dc motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use MCPWM module to control brushed dc motor.
 * This code is tested with L298 motor driver.
 * User may need to make changes according to the motor driver they use.
*/

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "hello_world_main.h"
#include "ps2_i2c_sensor.h"

#define MOTOR_PS2_QUEUE_SIZE    20

void *ps2_msg_queue_handle;

#define GPIO_PWM0A_OUT 19   //Set GPIO 19 as PWM0A
#define GPIO_PWM0B_OUT 18   //Set GPIO 18 as PWM0B
#define GPIO_PWM1A_OUT 15   //Set GPIO 15 as PWM1A
#define GPIO_PWM1B_OUT 14   //Set GPIO 14 as PWM1B
// #define GPIO_PWM2A_OUT 17   //Set GPIO 17(UART2-TX) as PWM2A
// #define GPIO_PWM2B_OUT 16   //Set GPIO 16(UART2-RX) as PWM2B

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

void pwm_example_config(void)
{
    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    
    pwm_config.frequency = 1000;    //frequency = 1000Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    pwm_config.frequency = 1000;    //frequency = 1000Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings
}

/*  
straight 1:forward,2:backward
turn 1:left,2:right
*/
void set_car_move(uint8_t straight,uint8_t turn)
{
    uint8_t car_speed,turn_speed;
    car_speed = 50;
    turn_speed = 20;
    if(straight==1 && turn==0){
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, car_speed);    //the left motor
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, car_speed);    //the right motor
    }
    else if(straight==2 && turn==0){
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, car_speed);
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, car_speed);
    }
    else if(straight==0 && turn==1){
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, car_speed);
    }
    else if(straight==0 && turn==2){
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0,car_speed);
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
    }

    else if(straight==1 && turn==1){
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0,car_speed-turn_speed);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, car_speed+turn_speed);
    }
    else if(straight==1 && turn==2){
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0,car_speed+turn_speed);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, car_speed-turn_speed);
    }
    else if(straight==2 && turn==1){
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0,car_speed-turn_speed);
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, car_speed+turn_speed);
    }
    else if(straight==2 && turn==2){
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0,car_speed+turn_speed);
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, car_speed-turn_speed);
    }
    else{
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
    }
}

static void pwm_motor_test(void)
{
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, 60.0);
    vTaskDelay(1000 / portTICK_RATE_MS);
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30.0);
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, 40.0);
    vTaskDelay(1000 / portTICK_RATE_MS);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
    vTaskDelay(2000 / portTICK_RATE_MS);
}

int send_ps2_data_to_motor_queue(uint8_t *p_data,uint32_t len)
{
    if(len != sizeof(ps2_button_data)){
        printf("[%s] Send to queue fail,because the len is not corrent\r\n",__func__);
        return 1;
    }
    if(xQueueSend(ps2_msg_queue_handle,p_data,500) != pdPASS){
        printf("[%s] Send to queue fail!\r\n",__func__);
        return 2;
    }
    return 0;
}

static void print_data_test(uint8_t *p_data,uint32_t len)
{
    for(uint8_t i=0;i<len;i++){
        printf("%02x ",p_data[i]);
    }
    printf("\r\n");
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    ps2_button_data ps2_data;
    uint8_t str_val,turn_val;
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();
    pwm_example_config();
    ps2_msg_queue_handle = xQueueCreate(MOTOR_PS2_QUEUE_SIZE,sizeof(ps2_button_data));
    while (1) {
        if(xQueueReceive(ps2_msg_queue_handle,&ps2_data,0xffffffff)){
            str_val = turn_val = 0;
            if(ps2_data.x_val < 100){
                // turn left
                turn_val = 1;
            }
            else if(ps2_data.x_val > 150){
                // turn right
                turn_val = 2;
            }
            if(ps2_data.y_val > 150){
                // forward
                str_val = 1;
            }
            else if(ps2_data.y_val < 100){
                // backward
                str_val = 2;
            }
            else{
                print_data_test(&ps2_data,sizeof(ps2_button_data));
            }
            set_car_move(str_val,turn_val);
            vTaskDelay(300/portTICK_PERIOD_MS);
            set_car_move(0,0);
        }
    }
}

void pwm_app_main(void)
{
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);
}
