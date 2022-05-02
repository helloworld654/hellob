#ifndef __I2C_EXAMPLE_MAIN_H__
#define __I2C_EXAMPLE_MAIN_H__

typedef struct mpu_accl_val{
    float x;
    float y;
    float z;
}MPU_ACCL_VAL;

uint8_t i2c_write_sensor_reg(uint8_t reg_addr, uint8_t reg_data);

uint8_t i2c_read_sensor_reg(uint8_t reg_addr,uint8_t *data_rd, size_t size);

#endif
