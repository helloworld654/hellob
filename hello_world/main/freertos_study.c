#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos_study.h"

#define RTOS_TASK_STACK_SIZE    2048
#define RTOS_TASK_PRIORITY    1

void *rtos_study_task_handle_A;
void *rtos_study_task_handle_B;

#if defined(QUEUE_TEST) && QUEUE_TEST
#define RTOS_TEST_QUEUE_LEN    0x10
void *rtos_study_queue_handle;
#endif

#if defined(SEM_TEST) && SEM_TEST
void *rtos_study_sem_handle;
#endif

static void rtos_study_task_a(void *arg)
{
    printf("[%s] enter\r\n",__func__);
#if defined(QUEUE_TEST) && QUEUE_TEST
    queue_item item_send;
    rtos_study_queue_handle = xQueueCreate(RTOS_TEST_QUEUE_LEN,sizeof(queue_item));
#endif

#if defined(SEM_TEST) && SEM_TEST
    rtos_study_sem_handle = xSemaphoreCreateCounting(1,0);
#endif

    while(1){
#if defined(QUEUE_TEST) && QUEUE_TEST
        item_send.type = 0xa;
        printf("[%s] send item to queue\r\n",__func__);
        xQueueSend(rtos_study_queue_handle,&item_send,0x1000);
#endif

#if defined(SEM_TEST) && SEM_TEST
        printf("[%s] give the sem\r\n",__func__);
        xSemaphoreGive(rtos_study_sem_handle);
#endif
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

static void rtos_study_task_b(void *arg)
{
    printf("[%s] enter\r\n",__func__);
#if defined(QUEUE_TEST) && QUEUE_TEST
    queue_item item_recv;
#endif
    while(1){
#if defined(QUEUE_TEST) && QUEUE_TEST
        if(rtos_study_queue_handle){
            if(xQueueReceive(rtos_study_queue_handle,&item_recv,1000)){
                printf("[%s] receive item:0x%x\r\n",__func__,item_recv.type);
            }
            else{
                printf("receive from queue timeout\r\n");
            }
        }
#endif

#if defined(SEM_TEST) && SEM_TEST
        if(rtos_study_sem_handle){
            if(xSemaphoreTake(rtos_study_sem_handle,1000)){
                printf("[%s] take sem success\r\n",__func__);
            }
        }
#endif
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void task_create_test(void)
{
    xTaskCreate(rtos_study_task_a,"study_task_A",RTOS_TASK_STACK_SIZE,NULL,RTOS_TASK_PRIORITY,rtos_study_task_handle_A);  // stack size should more than 2048
    xTaskCreate(rtos_study_task_b,"study_task_B",RTOS_TASK_STACK_SIZE,NULL,RTOS_TASK_PRIORITY,rtos_study_task_handle_B);
}
