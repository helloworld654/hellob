#include "stdio.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver_mpu6050.h"
#include "driver_mpu6050_interface.h"
#include "driver_mpu6050_basic.h"
#include "driver_mpu6050_dmp.h"

#define MPU6050_TASK_STACK_SIZE    256*32
#define MPU6050_TASK_PRIORITY    1
static void *mpu6050_stack_task_handle = NULL;

static volatile uint8_t gs_flag;           /**< flag */

/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6050_INTERRUPT_MOTION :
        {
            gs_flag |= 1 << 0;
            mpu6050_interface_debug_print("mpu6050: irq motion.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_FIFO_OVERFLOW :
        {
            mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_I2C_MAST :
        {
            mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DMP :
        {
            mpu6050_interface_debug_print("mpu6050: irq dmp\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DATA_READY :
        {
            mpu6050_interface_debug_print("mpu6050: irq data ready\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp tap callback
 * @param[in] count is the tap count
 * @param[in] direction is the tap direction
 * @note      none
 */
static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU6050_DMP_TAP_X_UP :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_X_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_UP :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_UP :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq z down with %d.\n", count);
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orient is the dmp orient
 * @note      none
 */
static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU6050_DMP_ORIENT_PORTRAIT :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_LANDSCAPE :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq reverse landscape.\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq unknown code.\n");
            
            break;
        }
    }
}

static void i2c_mpu6050_task(void *arg)
{
    int16_t gs_accel_raw[128][3];       /**< accel raw buffer */
    float gs_accel_g[128][3];           /**< accel g buffer */
    int16_t gs_gyro_raw[128][3];        /**< gyro raw buffer */
    float gs_gyro_dps[128][3];          /**< gyro dps buffer */
    int32_t gs_quat[128][4];            /**< quat buffer */
    float gs_pitch[128];                /**< pitch buffer */
    float gs_roll[128];                 /**< roll buffer */
    float gs_yaw[128];                  /**< yaw buffer */
    uint16_t len;
    uint8_t res;
    mpu6050_interface_debug_print("[%s] enter\r\n", __func__);
    while(1){
        res = mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                         gs_gyro_raw, gs_gyro_dps, 
                                         gs_quat,
                                         gs_pitch, gs_roll, gs_yaw,
                                         &len);
        if(res){
            mpu6050_interface_debug_print("[%s] Read dmp fail:%d\r\n", __func__, res);
            break;
        }
        mpu6050_interface_debug_print("gs_pitch:%d, gs_roll:%d, gs_yaw:%d\r\n", gs_pitch, gs_roll, gs_yaw);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

int hepingood_mpu6050_init(void)
{
    uint8_t res;
    mpu6050_interface_debug_print("[%s] Print use interface API\r\n", __func__);
    // mpu6050_basic_init(MPU6050_ADDRESS_AD0_LOW);  // success
    res = mpu6050_dmp_init(MPU6050_ADDRESS_AD0_LOW, a_receive_callback, a_dmp_tap_callback, a_dmp_orient_callback);  // fail, need debug
    if(res){
        mpu6050_interface_debug_print("mpu6050_dmp_init fail:%d\r\n", res);
    }
    xTaskCreate(i2c_mpu6050_task, "mpu 6050 task", MPU6050_TASK_STACK_SIZE, NULL, MPU6050_TASK_PRIORITY, &mpu6050_stack_task_handle);
    return 0;
}
