#include "user_cmd.h"

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
