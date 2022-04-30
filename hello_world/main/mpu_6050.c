#include "stdio.h"
#include "stdint.h"
#include "mpu_6050.h"
#include "i2c_example_main.h"

//****************************************
// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I			0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	0xD0	//IIC写入时的地址字节数据，+1为读取

// return 0:success  other:fail
uint8_t mpu6050_init(void)
{
    if(!i2c_write_sensor_reg(PWR_MGMT_1,0x00)) {	//解除休眠状态
        i2c_write_sensor_reg(SMPLRT_DIV,0x07);
        i2c_write_sensor_reg(CONFIG,0x06);
        i2c_write_sensor_reg(GYRO_CONFIG,0x18);
        i2c_write_sensor_reg(ACCEL_CONFIG,0x01);
        printf("mpu 6050 init success\n");
        return 0;
    }
    else{
        printf("mpu write failed\n");
        return 1;
    }
}

void mpu6050_read_test(void)
{
    uint8_t data_read;
    i2c_read_sensor_reg(GYRO_CONFIG,&data_read,1);
    printf("the value of GYRO_CONFIG:0x%x\r\n",data_read);
    i2c_read_sensor_reg(SMPLRT_DIV,&data_read,1);
    printf("the value of SMPLRT_DIV:0x%x\r\n",data_read);
}
