
typedef void (*cmd_exec_func)(uint8_t argc,char *argv[]);

typedef struct cmd_struct
{
    char *cmd_str;
    cmd_exec_func cmd_func;
}cmd_struct;
