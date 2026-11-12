/**
 * motor.c — 步进电机驱动 (FreeRTOS版)
 *
 * 硬件: STM32F103ZET6, 标准外设库
 * 电机: 1.8°步距角, 16细分 (3200脉冲/圈)
 * 引脚: F0=使能  F1=方向  F2=脉冲
 * 加减速: 线性梯形 (加速→匀速→减速)
 * 延时:   FreeRTOS vTaskDelayUntil (0%忙等)
 */

#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"

/* GPIO 初始化 */
void MOTOR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOF, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
}

/* 角度→脉冲换算 */
uint16_t Stepper_AngleToPulses(float angle_deg)
{
    return (uint16_t)(angle_deg * STEP_PULSE_PER_DEG + 0.5f);
}

/* 线性加减速延时计算 */
uint8_t Stepper_CalcDelay(uint16_t pulse_index, uint16_t total_pulses)
{
    uint16_t accel_end;
    uint16_t decel_start;
    uint8_t  range;

    range = (uint8_t)(STEP_DELAY_MAX_MS - STEP_DELAY_MIN_MS);

    if (total_pulses > 2U * STEP_ACCEL_PULSES)
    {
        accel_end   = STEP_ACCEL_PULSES;
        decel_start = total_pulses - STEP_ACCEL_PULSES;
    }
    else
    {
        accel_end   = total_pulses / 3U;
        decel_start = total_pulses - accel_end;
    }

    if (pulse_index < accel_end)
    {
        return STEP_DELAY_MAX_MS
             - (uint8_t)((uint32_t)range * pulse_index / accel_end);
    }
    else if (pulse_index < decel_start)
    {
        return STEP_DELAY_MIN_MS;
    }
    else
    {
        uint16_t pos       = pulse_index - decel_start;
        uint16_t decel_len = total_pulses - decel_start;
        if (decel_len == 0U) { decel_len = 1U; }
        return STEP_DELAY_MIN_MS
             + (uint8_t)((uint32_t)range * pos / decel_len);
    }
}

/* 执行一次角度移动 (带加减速) */
void Stepper_Run(uint8_t dir, float angle_deg)
{
    uint16_t   pulses;
    uint16_t   i;
    TickType_t xLastWakeTime;
    uint8_t    dly;

    pulses = Stepper_AngleToPulses(angle_deg);
    if (pulses == 0U) { return; }

    MOTOR_DIR = dir;
    xLastWakeTime = xTaskGetTickCount();

    for (i = 0U; i < pulses; i++)
    {
        dly = Stepper_CalcDelay(i, pulses);
        MOTOR_PUL = 1U;
        vTaskDelayUntil(&xLastWakeTime, (TickType_t)dly);
        MOTOR_PUL = 0U;
        vTaskDelayUntil(&xLastWakeTime, (TickType_t)dly);
    }
}

