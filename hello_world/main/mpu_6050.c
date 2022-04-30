#include "stdio.h"
#include "stdint.h"
#include "mpu_6050.h"
#include "i2c_example_main.h"

#define WHO	    0x00
#define	SMPL	0x15
#define DLPF	0x16
#define INT_C	0x17
#define INT_S	0x1A
#define	TMP_H	0x1B
#define	TMP_L	0x1C
#define	GX_H	0x1D
#define	GX_L	0x1E
#define	GY_H	0x1F
#define	GY_L	0x20
#define GZ_H	0x21
#define GZ_L	0x22
#define PWR_M	0x3E

// return 0:success  other:fail
uint8_t mpu6050_init(void)
{
    if(!i2c_write_sensor_reg(PWR_M,0x80)) {
        i2c_write_sensor_reg(SMPL,0x07);
        i2c_write_sensor_reg(DLPF,0x1E);    //±2000°
        i2c_write_sensor_reg(INT_C,0x00);
        i2c_write_sensor_reg(PWR_M,0x00);
        printf("mpu 6050 init success\n");
        return 0;
    }
    else{
        printf("mpu write failed\n");
        return 1;
    }
}
