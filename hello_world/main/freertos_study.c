#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos_study.h"

#define RTOS_TASK_STACK_SIZE    2048
#define RTOS_TASK_PRIORITY    1
#define RTOS_TEST_QUEUE_LEN    0x10

void *rtos_study_task_handle_A;
void *rtos_study_task_handle_B;
void *rtos_study_queue_handle;

static void rtos_study_task_a(void *arg)
{
    queue_item item_send;
    printf("[%s] enter\r\n",__func__);
    rtos_study_queue_handle = xQueueCreate(RTOS_TEST_QUEUE_LEN,sizeof(queue_item));
    if(!rtos_study_queue_handle){
        printf("create queue fail\r\n");
    }
    while(1){
        item_send.type = 0xa;
        printf("[%s] send item to queue\r\n",__func__);
        xQueueSend(rtos_study_queue_handle,&item_send,0x1000);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

static void rtos_study_task_b(void *arg)
{
    queue_item item_recv;
    printf("[%s] enter\r\n",__func__);
    while(1){
        if(rtos_study_queue_handle){
            if(xQueueReceive(rtos_study_queue_handle,&item_recv,1000)){
                printf("[%s] receive item:0x%x\r\n",__func__,item_recv.type);
            }
            else{
                printf("receive from queue timeout\r\n");
            }
        }
    }
}

void task_create_test(void)
{
    xTaskCreate(rtos_study_task_a,"study_task_A",RTOS_TASK_STACK_SIZE,NULL,RTOS_TASK_PRIORITY,rtos_study_task_handle_A);  // stack size should more than 2048
    xTaskCreate(rtos_study_task_b,"study_task_B",RTOS_TASK_STACK_SIZE,NULL,RTOS_TASK_PRIORITY,rtos_study_task_handle_B);
}
