#ifndef __USER_CMD_H__
#define __USER_CMD_H__

#include "stdint.h"

typedef void (*cmd_exec_func)(uint8_t argc,char *argv[]);

typedef struct cmd_struct
{
    char *cmd_str;
    cmd_exec_func cmd_func;
}cmd_struct;

uint32_t str_to_num(char *p_str,uint8_t length);

#endif
