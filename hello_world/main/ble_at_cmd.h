#ifndef __BLE_AT_CMD_H__
#define __BLE_AT_CMD_H__

#include "stdint.h"

void at_cmd_task_init(void);

int send_uart_data_to_at_task(uint8_t *data,uint8_t len);

#endif
