#include "ble_at_cmd.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define AT_CMD_TASK_STACK_SIZE    2048
#define AT_CMD_TASK_PRIORITY    1
#define AT_CMD_QUEUE_LENGTH    0x10
#define AT_CMD_MAX_STRING    64

typedef struct uart_data_rece_g{
    uint8_t len;
    uint8_t data[AT_CMD_MAX_STRING];
}uart_data_rece;

void *at_cmd_task_handl = NULL;
void *at_cmd_queue_handle = NULL;
static uint8_t at_cmd_task_is_ready = 0;

int send_uart_data_to_at_task(uint8_t *data,uint8_t len)
{
    uart_data_rece data_rece;
    if(at_cmd_task_is_ready){
        data_rece.len = len;
        if(len > AT_CMD_MAX_STRING){
            len = AT_CMD_MAX_STRING;
        }
        memcpy(data_rece.data,data,len);
        xQueueSend(at_cmd_queue_handle,&data_rece,500/portTICK_PERIOD_MS);
        return 0;
    }
    else{
        printf("[%s] The at cmd task is not ready\r\n",__func__);
        return 1;
    }
}

extern void parse_at_cmd(uint8_t *p_data,uint8_t length);
void at_cmd_func(void *arg)
{
    uint8_t i;
    uart_data_rece get_data;
    at_cmd_task_is_ready = 1;
    while(1){
        if(xQueueReceive(at_cmd_queue_handle,&get_data,0xffffffff)){
            parse_at_cmd(get_data.data,get_data.len);
        }
    }
}

void at_cmd_task_init(void)
{
    at_cmd_queue_handle = xQueueCreate(AT_CMD_QUEUE_LENGTH,sizeof(uart_data_rece));
    if(!at_cmd_queue_handle){
        printf("[%s] Create at cmd queue fail!\r\n",__func__);
    }
    xTaskCreate(at_cmd_func,"at_cmd_task",AT_CMD_TASK_STACK_SIZE,NULL,AT_CMD_TASK_PRIORITY,&at_cmd_task_handl);  //the task stack size should more than 2048
}
