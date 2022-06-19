#ifndef __PS2_SENSOR_H__
#define __PS2_SENSOR_H__

// #define JOYSTICK_I2C_ADDR         0x5A
#define JOYSTICK_LEFT_X_REG       0x10
#define JOYSTICK_LEFT_Y_REG       0x11

typedef enum{
    PS2_BUTTON_NORMAL,
    PS2_BUTTON_SINGLE,
    PS2_BUTTON_DOUBLE,
    PS2_BUTTON_LONG
}PS2_BUTTON_VAL;

typedef struct PS2_BUTTON_DATA_T{
    uint8_t X_DATA;
    uint8_t Y_DATA;
    uint8_t Z;
    uint8_t A;
    uint8_t B;
    uint8_t C;
    uint8_t D;
}ps2_button_data;

void i2c_ps2_app_main(void);

#endif
