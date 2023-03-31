#ifndef __FREERTOS_API_H__
#define __FREERTOS_API_H__

#include "stdio.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define free_delay_ms(ms)    vTaskDelay(ms/portTICK_RATE_MS)

#endif
