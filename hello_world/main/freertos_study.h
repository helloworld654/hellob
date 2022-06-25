#ifndef __FREERTOS_STUDY_H__
#define __FREERTOS_STUDY_H__

typedef struct{
    uint8_t type;
    void *p_data;
}queue_item;

void task_create_test(void);

#endif
