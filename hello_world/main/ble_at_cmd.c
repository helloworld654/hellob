#include "ble_at_cmd.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "user_cmd.h"
#include "esp_gap_ble_api.h"

#define AT_CMD_TASK_STACK_SIZE    2048
#define AT_CMD_TASK_PRIORITY    1
#define AT_CMD_QUEUE_LENGTH    0x10
#define AT_CMD_MAX_STRING    64

#define CMD_NUM    10
#define CMD_LENGTH    20
char *p_cmd[CMD_NUM] = {NULL};
uint8_t cmd_actual_num = 0;

extern void gatts_app_main(void);
extern void gattc_app_main(void);
extern void connect_to_peripheral(uint8_t *p_addr);
extern void gattc_write_demo(uint8_t *p_data,uint8_t length);
extern void gatts_notify_demo(uint8_t *p_data,uint8_t length);

void start_ble_peripheral(uint8_t argc,char *argv[])
{
    gatts_app_main();
}

void start_ble_central(uint8_t argc,char *argv[])
{
    gattc_app_main();
}

void establish_connection(uint8_t argc,char *argv[])
{
    uint8_t i,bt_addr[6];
    if(strlen(argv[1]) != 12){
        printf("[%s] the length of bt addr is wrong!!\r\n",__func__);
        return ;
    }
    str_to_hex(bt_addr,argv[1]);
    printf("connect to bt addr:");
    for(i=0;i<6;i++){
        printf("%02x ",bt_addr[i]);
    }
    printf("\r\n");
    connect_to_peripheral(bt_addr);
}

void gatt_write_cmd(uint8_t argc,char *argv[])
{
    uint8_t length,*p_data = NULL;
    length = strlen(argv[1]);
    p_data = (uint8_t *)malloc(sizeof(uint8_t)*(length/2));
    if(!str_to_hex(p_data,argv[1])){
        gattc_write_demo(p_data,length/2);
    }
    free(p_data);
}

void gatt_notify_cmd(uint8_t argc,char *argv[])
{
    uint8_t length,*p_data = NULL;
    length = strlen(argv[1]);
    p_data = (uint8_t *)malloc(sizeof(uint8_t)*(length/2));
    if(!str_to_hex(p_data,argv[1])){
        gatts_notify_demo(p_data,length/2);
    }
    free(p_data);
}

void cent_start_ble_scan(uint8_t argc,char *argv[])
{
    if(argc == 1){
        esp_ble_gap_start_scanning(0xffffffff);
    }
    else if(argc == 2){
        uint32_t scan_time = 100;
        esp_ble_gap_start_scanning(scan_time);
    }
}

void cent_stop_ble_scan(uint8_t argc,char *argv[])
{
    esp_ble_gap_stop_scanning();
}

cmd_struct cms_num_struct[] = {{"peri",start_ble_peripheral},{"cent",start_ble_central},{"conn",establish_connection},{"atw",gatt_write_cmd},{"atno",gatt_notify_cmd},
    {"scan1",cent_start_ble_scan},{"scan0",cent_stop_ble_scan}};
    
void user_cmd_init(void)
{
    uint8_t i;
    cmd_actual_num = sizeof(cms_num_struct)/sizeof(cms_num_struct[0]);
    for(i=0;i<CMD_NUM;i++){
        p_cmd[i] = (char *)malloc(sizeof(char)*CMD_LENGTH);
    }
}

void parse_at_cmd(uint8_t *p_data,uint8_t length)
{
    uint8_t i,j,cmd_index;
    if(length < 2)
        return ;
    cmd_index = 0;
    for(i=0;i<length-1;){
        if(p_data[i]==' ' || p_data[i]==','){
            i++;
            continue;
        }
        for(j=i+1;j<length;j++){
            if(p_data[j]==',' || p_data[j]==' ' || p_data[j]=='\r' || p_data[j]=='\n'){
                memcpy(p_cmd[cmd_index],p_data+i,j-i);
                p_cmd[cmd_index++][j-i] = '\0'; 
                break;
            }
            if(j == length-1){
                memcpy(p_cmd[cmd_index],p_data+i,j-i+1);
                p_cmd[cmd_index++][j-i+1] = '\0'; 
                break;
            }
        }
        i = j+1;
    }
    for(i=0;i<cmd_actual_num;i++){
        if(strcmp(p_cmd[0],cms_num_struct[i].cmd_str) == 0){
            cms_num_struct[i].cmd_func(cmd_index,p_cmd);
            break;
        }
    }
    if(i == cmd_actual_num){
        printf("[%s] Unknown at command\r\n",__func__);
    }
}

typedef struct uart_data_rece_g{
    uint8_t len;
    uint8_t data[AT_CMD_MAX_STRING];
}uart_data_rece;

void *at_cmd_task_handl = NULL;
void *at_cmd_queue_handle = NULL;
static uint8_t at_cmd_task_is_ready = 0;

int send_uart_data_to_at_task(uint8_t *data,uint8_t len)
{
    uart_data_rece data_rece;
    if(at_cmd_task_is_ready){
        data_rece.len = len;
        if(len > AT_CMD_MAX_STRING){
            len = AT_CMD_MAX_STRING;
        }
        memcpy(data_rece.data,data,len);
        xQueueSend(at_cmd_queue_handle,&data_rece,500/portTICK_PERIOD_MS);
        return 0;
    }
    else{
        printf("[%s] The at cmd task is not ready\r\n",__func__);
        return 1;
    }
}

void at_cmd_func(void *arg)
{
    uint8_t i;
    uart_data_rece get_data;
    at_cmd_task_is_ready = 1;
    while(1){
        if(xQueueReceive(at_cmd_queue_handle,&get_data,0xffffffff)){
            parse_at_cmd(get_data.data,get_data.len);
        }
    }
}

void at_cmd_task_init(void)
{
    user_cmd_init();
    at_cmd_queue_handle = xQueueCreate(AT_CMD_QUEUE_LENGTH,sizeof(uart_data_rece));
    if(!at_cmd_queue_handle){
        printf("[%s] Create at cmd queue fail!\r\n",__func__);
    }
    xTaskCreate(at_cmd_func,"at_cmd_task",AT_CMD_TASK_STACK_SIZE,NULL,AT_CMD_TASK_PRIORITY,&at_cmd_task_handl);  //the task stack size should more than 2048
}
