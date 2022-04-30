#include "stdio.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu_6050.h"
#include "i2c_example_main.h"

// return 0:success  other:fail
uint8_t mpu6050_init(void)
{
    uint8_t sensor_addr; 
	if(i2c_write_sensor_reg(MPU_PWR_MGMT1_REG,0X80))	//复位MPU6050
    {
        printf("[%s] mpu6050 write reg fail\r\n",__func__);
        return 2;
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
	i2c_write_sensor_reg(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	i2c_write_sensor_reg(MPU_GYRO_CFG_REG,3<<3);					//陀螺仪传感器,±2000dps
	i2c_write_sensor_reg(MPU_ACCEL_CFG_REG,0<<3);					//加速度传感器,±2g
	i2c_write_sensor_reg(MPU_SAMPLE_RATE_REG,50);						//设置采样率50Hz
	i2c_write_sensor_reg(MPU_INT_EN_REG,0X00);	//关闭所有中断
	i2c_write_sensor_reg(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	i2c_write_sensor_reg(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	i2c_write_sensor_reg(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	i2c_read_sensor_reg(MPU_DEVICE_ID_REG,&sensor_addr,1); 
	if(sensor_addr == MPU6050_SENSOR_ADDR)//器件ID正确
	{
		i2c_write_sensor_reg(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		i2c_write_sensor_reg(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		i2c_write_sensor_reg(MPU_SAMPLE_RATE_REG,50);						//设置采样率为50Hz
        printf("[%s] mpu6050 init success\r\n",__func__);
	    return 0;
 	}
    else{
        printf("[%s] mpu6050 init fail\r\n",__func__);
        return 1;
    }

}

void mpu6050_read_test(void)
{
    uint8_t data_read;
    i2c_read_sensor_reg(MPU_SAMPLE_RATE_REG,&data_read,1);
    printf("the value of MPU_SAMPLE_RATE_REG:0x%02x\r\n",data_read);
    i2c_read_sensor_reg(MPU_GYRO_CFG_REG,&data_read,1);
    printf("the value of MPU_GYRO_CFG_REG:0x%02x\r\n",data_read);
}
