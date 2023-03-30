#include "stdio.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mpu_6050_sensor.h"
#include "i2c_protocol.h"

#define MPU6050_TASK_STACK_SIZE    2048
#define MPU6050_TASK_PRIORITY    1
void *mpu6050_stack_task_handle = NULL;

//  accl calcu for +/- 2g range
#define ACCL_MAX    (65536)
#define ACCL_MIDDLE    (32768)
#define ACCL_EVERY_G    (16384)

// return 0:success  other:fail
static uint8_t mpu6050_setting_init(void)
{
	int ret;
	ret = i2c_master_init();
	if(ret){
		printf("[%s] i2c master init fail,reason:0x%x\r\n",__func__,ret);
		return ret;
	}
    if(!i2c_write_byte_sensor_reg(MPU6050_SENSOR_ADDR, PWR_MGMT_1, 0x80)) {	//解除休眠状态
        vTaskDelay(100/portTICK_RATE_MS);
    	i2c_write_byte_sensor_reg(MPU6050_SENSOR_ADDR, PWR_MGMT_1, 0x00);
        i2c_write_byte_sensor_reg(MPU6050_SENSOR_ADDR, ACCEL_CONFIG,0);    //  +/-2g
        i2c_write_byte_sensor_reg(MPU6050_SENSOR_ADDR, SMPLRT_DIV, 0x07);
        i2c_write_byte_sensor_reg(MPU6050_SENSOR_ADDR, CONFIG, 0x06);
        i2c_write_byte_sensor_reg(MPU6050_SENSOR_ADDR, GYRO_CONFIG, 0x18);
        printf("mpu 6050 init success\n");
        return 0;
    }
    else{
        printf("[%s] mpu6050 i2c write failed\r\n", __func__);
        return 1;
    }
}

static uint16_t read_16bits_from_reg(uint8_t reg_addr)
{
	uint16_t read_value = 0;
	uint8_t read_byte;

	i2c_read_sensor_reg(MPU6050_SENSOR_ADDR, reg_addr, &read_byte, 1);
	read_value = read_value|(read_byte<<8);

	i2c_read_sensor_reg(MPU6050_SENSOR_ADDR, reg_addr+1, &read_byte, 1);
	read_value = read_value|read_byte;

	return read_value;
}

static float calcu_actual_accl(uint16_t reg_val)
{
	float result;
	if(reg_val < ACCL_MIDDLE){
		result = reg_val/(ACCL_EVERY_G*1.0);
	}
	else{
		result = (reg_val-ACCL_MAX)/(ACCL_EVERY_G*1.0);
	}
	return result;
}

void mpu6050_read_accl(MPU_ACCL_VAL *p_accl_val)
{
	uint16_t x,y,z;
	float accl_x,accl_y,accl_z;
	x = read_16bits_from_reg(ACCEL_XOUT_H);
	y = read_16bits_from_reg(ACCEL_YOUT_H);
	z = read_16bits_from_reg(ACCEL_ZOUT_H);
	accl_x = calcu_actual_accl(x);
	accl_y = calcu_actual_accl(y);
	accl_z = calcu_actual_accl(z);
	p_accl_val->x = accl_x;
	p_accl_val->y = accl_y;
	p_accl_val->z = accl_z;
	// printf("x:%u,  y:%u,  z:%u    ",x,y,z);
	// printf("accl_x:%.3f,  accl_y:%.3f,  accl_z:%.3f\r\n",accl_x,accl_y,accl_z);
}

static void i2c_mpu6050_task(void *arg)
{
    MPU_ACCL_VAL accl_val;
	int ret;
    while(1){
		mpu6050_read_accl(&accl_val);
		printf("acclx:%.3f,   accly:%.3f,   acclz:%.3f\r\n",accl_val.x,accl_val.y,accl_val.z);
		vTaskDelay(1000/portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

int mpu_6050_sensor_init(void)
{
	int ret;
	ret = mpu6050_setting_init();
	if(ret){
		printf("[%s] mpu6050 init fail,reason:%d\r\n",__func__,ret);
		return ret;
	}
    xTaskCreate(i2c_mpu6050_task, "mpu 6050 task", MPU6050_TASK_STACK_SIZE, NULL, MPU6050_TASK_PRIORITY, &mpu6050_stack_task_handle);
    return 0;
}
