#include "user_cmd.h"
#include "stdio.h"
#include "string.h"

uint32_t str_to_num(char *p_str,uint8_t length)
{
    uint8_t i;
    uint32_t sum = 0;
    for(i=0;i<length;i++){
        if(p_str[i]>='0' && p_str[i]<='9'){
            sum = sum*16+(p_str[i]-48);
        }
        else if(p_str[i]>='A' && p_str[i]<='F'){
            sum = sum*16+(p_str[i]-55);
        }
        else if(p_str[i]>='a' && p_str[i]<='f'){
            sum = sum*16+(p_str[i]-87);
        }
    }
    return sum;
}

uint8_t str_to_hex(uint8_t *p_data,char *p_str)
{
    uint8_t str_len,i;
    str_len = strlen(p_str);
    if(str_len%2 != 0){
        printf("[%s] the length of string is not double!!\r\n",__func__);
        return 1;
    }
    for(i=0;i<str_len;i+=2){
        p_data[i/2] = str_to_num(p_str+i,2);
    }
    return 0;
}
