#include "stdio.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu_6050.h"

//  accl calcu for +/- 2g range
#define ACCL_MAX    (65536)
#define ACCL_MIDDLE    (32768)
#define ACCL_EVERY_G    (16384)

// return 0:success  other:fail
uint8_t mpu6050_init(void)
{
    if(!i2c_write_sensor_reg(PWR_MGMT_1,0x80)) {	//解除休眠状态
        vTaskDelay(100/portTICK_RATE_MS);
    	i2c_write_sensor_reg(PWR_MGMT_1,0x00);
        i2c_write_sensor_reg(ACCEL_CONFIG,0);    //  +/-2g
        i2c_write_sensor_reg(SMPLRT_DIV,0x07);
        i2c_write_sensor_reg(CONFIG,0x06);
        i2c_write_sensor_reg(GYRO_CONFIG,0x18);
        printf("mpu 6050 init success\n");
        return 0;
    }
    else{
        printf("mpu write failed\n");
        return 1;
    }
}

static uint16_t read_16bits_from_reg(uint8_t reg_addr)
{
	uint16_t read_value = 0;
	uint8_t read_byte;

	i2c_read_sensor_reg(reg_addr,&read_byte,1);
	read_value = read_value|(read_byte<<8);

	i2c_read_sensor_reg(reg_addr+1,&read_byte,1);
	read_value = read_value|read_byte;

	return read_value;
}

float calcu_actual_accl(uint16_t reg_val)
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

extern uint8_t ble_client_is_connect(void);
extern void gattc_write_demo(uint8_t *p_data,uint8_t length);
void i2c_mpu6050_task(void *arg)
{
    MPU_ACCL_VAL accl_val;
    mpu6050_init();
    while(1){
        if(ble_client_is_connect()){
            // gattc_write_demo(data,10);
            mpu6050_read_accl(&accl_val);
            printf("acclx:%.3f,   accly:%.3f,   acclz:%.3f\r\n",accl_val.x,accl_val.y,accl_val.z);
        }
        vTaskDelay(500/portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}
