#include "stm32f10x.h"
#include "key.h"
#include "sys.h"
#include "delay.h"
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

// 去抖延时：FreeRTOS下让出CPU，裸机下忙等
static void key_debounce_delay(void)
{
#ifdef USE_FREERTOS
    vTaskDelay(pdMS_TO_TICKS(10));  // FreeRTOS: 让出CPU去抖
#else
    delay_ms(10);                   // 裸机: 忙等去抖
#endif
}

void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 使能 GPIOC 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// 按键扫描函数
// mode:0 单次按下; 1 支持连按
// 返回值：0无按键/1/2/3/4 对应四个按键
u8 KEY_Scan(u8 mode)
{
    static u8 key_up = 1; // 按键松开标志

    if (mode) key_up = 1;  // 支持连按模式则清零标志

    // 有按键按下且按键处于松开状态
    if (key_up && (KEY0 == 0 || KEY1 == 0 || KEY2 == 0 || KEY3 == 0))
    {
        key_debounce_delay();  // 去抖延时（RTOS或裸机自适应）
        key_up = 0;
        if      (KEY0 == 0) return KEY0_PRES;
        else if (KEY1 == 0) return KEY1_PRES;
        else if (KEY2 == 0) return KEY2_PRES;
        else if (KEY3 == 0) return KEY3_PRES;
    }
    // 所有按键全部松开则恢复标志
    else if (KEY0 && KEY1 && KEY2 && KEY3)
    {
        key_up = 1;
    }
    return KEY_NONE; // 无按键按下
}
