#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

/* 步进电机引脚 (GPIOF) */
#define MOTOR_EN   PFout(0)   /* F0: 使能信号 */
#define MOTOR_DIR  PFout(1)   /* F1: 方向控制  1=正转 0=反转 */
#define MOTOR_PUL  PFout(2)   /* F2: 脉冲输入 */

/* 步进电机机械参数 */
#define STEP_ANGLE_DEG      1.8f       /* 步距角 (度)                     */
#define STEP_MICROSTEP      16U        /* 驱动细分倍数                    */
#define STEP_PULSE_PER_REV  3200U      /* 每圈脉冲数 = 360/1.8 * 16      */
#define STEP_PULSE_PER_DEG  8.888889f  /* 每度脉冲数 = 3200/360           */

/* 加减速参数 */
#define STEP_ACCEL_PULSES   30U        /* 加速段脉冲数 (线性加减速)       */
#define STEP_DELAY_MIN_MS   1U         /* 最高速: 半脉冲延时 (ms)         */
#define STEP_DELAY_MAX_MS   4U         /* 起步速: 半脉冲延时 (ms)         */

/* API */
void MOTOR_Init(void);

/* 角度转脉冲数 */
uint16_t Stepper_AngleToPulses(float angle_deg);

/* 计算第n个脉冲的加减速延时(ms) */
uint8_t  Stepper_CalcDelay(uint16_t pulse_index, uint16_t total_pulses);

/* 移动指定角度: dir=1正转 0反转, angle支持小数 (内部调用FreeRTOS延时) */
void Stepper_Run(uint8_t dir, float angle_deg);

#endif
