/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "user_cmd.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define CMD_NUM    10
#define CMD_LENGTH    20
char *p_cmd[CMD_NUM] = {NULL};
uint8_t cmd_actual_num = 0;

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

extern void gatts_app_main(void);
extern void gattc_app_main(void);
extern void connect_to_peripheral(uint8_t *p_addr);
extern void gattc_write_demo(uint8_t *p_data,uint8_t length);
extern void gatts_notify_demo(uint8_t *p_data,uint8_t length);

void print_paras(uint8_t argc,char *argv[])
{
    uint8_t i;
    printf("printf the paras:\r\n");
    for(i=0;i<argc;i++){
        printf("%s**",argv[i]);
    }
    printf("\r\n");
}

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
    for(i=0;i<12;i+=2){
        bt_addr[i/2] = str_to_num(argv[1]+i,2);
    }
    printf("connect to bt addr:");
    for(i=0;i<6;i++){
        printf("%02x ",bt_addr[i]);
    }
    printf("\r\n");
    connect_to_peripheral(bt_addr);
}

void gatt_write_cmd(uint8_t argc,char *argv[])
{
    uint8_t i,*p_data = NULL;
    uint8_t length;
    length = strlen(argv[1]);
    p_data = (uint8_t *)malloc(sizeof(uint8_t)*(length/2));
    for(i=0;i<length;i+=2){
        p_data[i/2] = str_to_num(argv[1]+i,2);
    }
    gattc_write_demo(p_data,length/2);
    free(p_data);
}

void gatt_notify_cmd(uint8_t argc,char *argv[])
{
    uint8_t i,*p_data = NULL;
    uint8_t length;
    length = strlen(argv[1]);
    p_data = (uint8_t *)malloc(sizeof(uint8_t)*(length/2));
    for(i=0;i<length;i+=2){
        p_data[i/2] = str_to_num(argv[1]+i,2);
    }
    gatts_notify_demo(p_data,length/2);
    free(p_data);
}

cmd_struct cms_num_struct[] = {{"peri",start_ble_peripheral},{"cent",start_ble_central},{"conn",establish_connection},{"atw",gatt_write_cmd},{"atno",gatt_notify_cmd}};

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
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    parse_at_cmd(dtmp,event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void user_cmd_init(void)
{
    uint8_t i;
    cmd_actual_num = sizeof(cms_num_struct)/sizeof(cms_num_struct[0]);
    for(i=0;i<CMD_NUM;i++){
        p_cmd[i] = (char *)malloc(sizeof(char)*CMD_LENGTH);
    }
}

void uart_evnet_app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    user_cmd_init();

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
