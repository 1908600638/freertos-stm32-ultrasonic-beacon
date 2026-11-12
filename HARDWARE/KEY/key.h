#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

#define KEY0  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)
#define KEY1  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)
#define KEY2  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)
#define KEY3  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3)

// 按键返回值枚举
typedef enum {
    KEY_NONE  = 0,
    KEY0_PRES = 1,
    KEY1_PRES = 2,
    KEY2_PRES = 3,
    KEY3_PRES = 4
} KeyValue_t;

void KEY_Init(void);
u8 KEY_Scan(u8 mode);
#endif
