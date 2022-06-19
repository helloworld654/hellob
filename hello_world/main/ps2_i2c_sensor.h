#ifndef __PS2_SENSOR_H__
#define __PS2_SENSOR_H__

// #define JOYSTICK_I2C_ADDR         0x5A
#define JOYSTICK_LEFT_X_REG       0x10    //uint8_t normal:128  left:<128  right:>128  max:255
#define JOYSTICK_LEFT_Y_REG       0x11    //uint8_t normal:128  down:<128  up:>128  max:255
#define JOYSTICK_BUTTON_REG       0x20    //normal:8  single click:1

#define BUTOON_LEFT_REG           0x24    //normal:8  single click:1  double click:0  long push:6
#define BUTOON_RIGHT_REG          0x23    //same as up state
#define BUTOON_UP_REG             0x22    //same as up state
#define BUTOON_DOWN_REG           0x21    //same as up state
//  the double click function is not correct

#define JOY_NORMAL_VAL    128
#define BUTTON_NORMAL_VAL   8

typedef enum{
    PS2_BUTTON_NORMAL,
    PS2_BUTTON_SINGLE,
    PS2_BUTTON_DOUBLE,
    PS2_BUTTON_LONG
}PS2_BUTTON_VAL;

typedef struct PS2_BUTTON_DATA_T{
    uint8_t x_val;
    uint8_t y_val;
    uint8_t button_joys;

    uint8_t button_l;
    uint8_t button_r;
    uint8_t button_up;
    uint8_t button_dn;

    uint8_t changed;
}ps2_button_data;

void i2c_ps2_app_main(void);

#endif
