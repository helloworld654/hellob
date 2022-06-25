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

#if defined(MUTEX_TEST) && (MUTEX_TEST)
void *rtos_study_mutex_handle;
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

#if defined(MUTEX_TEST) && (MUTEX_TEST)
    rtos_study_mutex_handle = xSemaphoreCreateMutex();
#endif

    while(1){
#if defined(QUEUE_TEST) && QUEUE_TEST
        item_send.type = 0xa;
        printf("\r\n[%s] send item to queue",__func__);
        xQueueSend(rtos_study_queue_handle,&item_send,1000/portTICK_PERIOD_MS);
#endif

#if defined(SEM_TEST) && SEM_TEST
        printf("\r\n[%s] give the sem",__func__);
        xSemaphoreGive(rtos_study_sem_handle);
#endif

#if defined(MUTEX_TEST) && (MUTEX_TEST)
        printf("\r\n[%s] try to take mutex",__func__);
        if(xSemaphoreTake(rtos_study_mutex_handle,1000/portTICK_PERIOD_MS)){
            printf("\r\n[%s] take the mutex success",__func__);
            vTaskDelay(1500/portTICK_PERIOD_MS);
            printf("\r\n[%s] give the mutex",__func__);
            xSemaphoreGive(rtos_study_mutex_handle);
        }
        else{
            printf("\r\n[%s] take the mutex timeout",__func__);
        }
#endif
        // vTaskDelay(2500/portTICK_PERIOD_MS);
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
            if(xQueueReceive(rtos_study_queue_handle,&item_recv,1000/portTICK_PERIOD_MS)){
                printf("\r\n[%s] receive item:0x%x",__func__,item_recv.type);
            }
            else{
                printf("\r\nreceive from queue timeout");
            }
        }
#endif

#if defined(SEM_TEST) && SEM_TEST
        if(rtos_study_sem_handle){
            if(xSemaphoreTake(rtos_study_sem_handle,1000/portTICK_PERIOD_MS)){
                printf("\r\n[%s] take sem success",__func__);
            }
            else{
                printf("\r\n[%s] take sem timeout",__func__);
            }
        }
#endif

#if defined(MUTEX_TEST) && (MUTEX_TEST)
        if(rtos_study_mutex_handle){
            printf("\r\n[%s] try to take mutex",__func__);
            if(xSemaphoreTake(rtos_study_mutex_handle,1000/portTICK_PERIOD_MS)){
                printf("\r\n[%s] take the mutex success",__func__);
                vTaskDelay(500/portTICK_PERIOD_MS);
                printf("\r\n[%s] give the mutex",__func__);
                xSemaphoreGive(rtos_study_mutex_handle);
            }
            else{
                printf("\r\n[%s] take the mutex timeout",__func__);
            }
        }
#endif
        // vTaskDelay(5/portTICK_PERIOD_MS);
    }
}

void task_create_test(void)
{
    xTaskCreate(rtos_study_task_a,"study_task_A",RTOS_TASK_STACK_SIZE,NULL,RTOS_TASK_PRIORITY,rtos_study_task_handle_A);  // stack size should more than 2048
    xTaskCreate(rtos_study_task_b,"study_task_B",RTOS_TASK_STACK_SIZE,NULL,RTOS_TASK_PRIORITY,rtos_study_task_handle_B);
}
