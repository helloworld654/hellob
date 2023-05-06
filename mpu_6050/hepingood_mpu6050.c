#if 0
#include "stdio.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver_mpu6050.h"
#include "driver_mpu6050_interface.h"
#include "driver_mpu6050_basic.h"

#define MPU6050_TASK_STACK_SIZE    2048
#define MPU6050_TASK_PRIORITY    1
static void *mpu6050_stack_task_handle = NULL;

static void i2c_mpu6050_task(void *arg)
{
    printf("[%s] enter\r\n",__func__);
    while(1){
		// printf("hello worrrrd\r\n");
		vTaskDelay(1000/portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

int hepingood_mpu6050_init(void)
{
	printf("[%s] enter\r\n",__func__);
    mpu6050_interface_debug_print("print use interface\r\n");
    mpu6050_basic_init(MPU6050_ADDRESS_AD0_LOW);
    printf("pass\r\n");
    xTaskCreate(i2c_mpu6050_task, "mpu 6050 task", MPU6050_TASK_STACK_SIZE, NULL, MPU6050_TASK_PRIORITY, &mpu6050_stack_task_handle);
    return 0;
}
#else
#endif
