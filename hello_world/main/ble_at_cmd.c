#include "ble_at_cmd.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AT_CMD_TASK_STACK_SIZE    2048
#define AT_CMD_TASK_PRIORITY    1

void *at_cmd_task_handl = NULL;

void at_cmd_func(void *arg)
{
    printf("[%s] enter\r\n",__func__);
    while(1){
        printf("\r\n[%s] hello world",__func__);
        vTaskDelay(1500/portTICK_PERIOD_MS);
    }
}

void at_cmd_task_init(void)
{
    xTaskCreate(at_cmd_func,"at_cmd_task",AT_CMD_TASK_STACK_SIZE,NULL,AT_CMD_TASK_PRIORITY,at_cmd_task_handl);  //the task stack size should more than 2048
}
