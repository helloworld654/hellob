#ifndef __HELLO_WORLD_MAIN_H__
#define __HELLO_WORLD_MAIN_H__

#define BLE_CAR_CLIENT    0

#define BLE_CAR_SERVER    0

#define FREERTOS_STUDY    1

#if (defined(BLE_CAR_CLIENT) && BLE_CAR_CLIENT) && \
    (defined(BLE_CAR_SERVER) && BLE_CAR_SERVER)
#error "cannot defined client and server at same time"
#endif

#endif
