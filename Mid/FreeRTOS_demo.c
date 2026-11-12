/**
 * FreeRTOS_demo.c — 超声波定位系统 主任务文件
 *
 * 硬件:  STM32F103ZET6 + 步进电机 + 超声波模块 + TJC串口屏
 * 功能:  超声波扫描定位, 串口屏命令驱动, 校准模式
 * 协议:  TJC USART HMI (ASCII指令 + 0xFF 0xFF 0xFF 结束符)
 *
 * 任务列表:
 *   TIME    - CPU使用率统计, 每5秒打印一次
 *   RecvMsg - 超声波USART3数据接收与滤波
 *   TASK_KEY- 按键扫描(KEY0~KEY3)
 *   SCAN    - 串口屏命令驱动扫描定位
 *   CAL     - 串口屏校准模式
 *
 * 串口屏命令 (UART4回传, 3字节帧 0xFF XX 0xFE):
 *   0x01 - 开始扫描定位
 *   0x03 - 复位/清除数据
 *   0x05 - 进入校准模式
 *   0x11~0x16 - 校准第1~6个值
 *   0x20 - 确认当前校准值
 *   0x21 - 校准完成
 */

#include "FreeRTOS_demo.h"
#include "key.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "motor.h"

/* 任务声明与参数 */

/* 超声波数据接收任务 (USART3中断 -> 环形缓冲 -> 数据滤波) */
void recvMsg(void *pvParameters);
#define Recv_TASK_STACK_SIZE 256   /* 栈大小(words), 含滤波数组 */
#define Recv_TASK_PRIORITY 4       /* 较高优先级, 保证数据不丢 */
TaskHandle_t Recv_Task_handle;

/* CPU运行时间统计任务 (调试用, 实际产品可删除) */
void timeTask(void *pvParameters);
#define Time_TASK_STACK_SIZE 256   /* stats_buf[512]需大栈 */
#define Time_TASK_PRIORITY 3
TaskHandle_t Time_Task_handle;

/* 按键扫描任务 (静态函数, 仅本文件使用) */
static void task_key(void *pvParam);
#define KEY_STACK 128U             /* 栈128words, 仅扫描按键够用 */
#define KEY_PRIO  2

/* 扫描定位任务 (核心任务, 串口屏命令驱动) */
void task_scan(void *pvParameters);
#define SCAN_STACK 512U            /* 大栈: 含采样数组和角度数据 */
#define SCAN_PRIO  3

/* 系统常量 */

#define SAMPLE_COUNT  8U           /* recvMsg滑动窗口样本数 */
#define ALIGN_THRESH  20U          /* 数据对齐阈值(mm), 窗口极差小于此值认为稳定 */
#define MAX_DIST_MM   3000U        /* 超声波最大有效距离 */
#define MAX_ANGLES    7U           /* 最多扫描7个角度(pass 1~7) */
#define RANGE_CNT     6U           /* 距离区间数量(对应列1~6) */

/* 全局变量 */

TaskHandle_t task_key_handle;      /* 按键任务句柄 */
TaskHandle_t Task_Scan_handle;     /* 扫描任务句柄 */

/*
 * 超声波距离数据
 * recvMsg任务持续更新, 其他任务通过read_distance()读取
 * volatile: 多任务共享, 防止编译器优化导致读取过期值
 */
static volatile uint32_t g_distance_mm    = 0U;  /* 最新有效距离(mm) */
static volatile uint8_t  g_distance_valid = 0U;  /* 距离有效标志, 1=有效 */
static volatile uint32_t g_raw_count      = 0U;  /* 原始帧计数(调试) */

/*
 * 校准值存储
 * 索引 1~6 对应列1~6 的实测距离(mm)
 * 默认值为硬编码区间中心值 (未校准时使用)
 * g_calibrated: 0=未校准(用默认值), 1=已校准(用实测值)
 */
uint32_t g_cal_val[7] = {
    0U,      /* [0] 未使用 */
    1488U,   /* [1] 列1中心: (1475+1496)/2 向上取整 */
    1637U,   /* [2] 列2中心: (1630+1645)/2 */
    1785U,   /* [3] 列3中心: (1775+1795)/2 */
    1930U,   /* [4] 列4中心: (1910+1940)/2 向上取整 */
    2085U,   /* [5] 列5中心: (2050+2100)/2 向上取整 */
    2230U    /* [6] 列6中心: (2210+2240)/2 向上取整 */
};
uint8_t g_calibrated = 0U;         /* 校准标志: 0=未校准, 1=已校准 */
volatile uint8_t g_cal_mode_active = 0U; /* 校准进行中, recvMsg暂停发送n0 */

/* 模式2校准值 (索引1~4, 列7~10, 默认区间中心) */
uint32_t g_cal_val2[5] = {0U, 2373U, 2518U, 2664U, 2816U};
uint8_t  g_calibrated2 = 0U;
volatile uint8_t g_cal2_mode_active = 0U;

/* 辅助函数 */

/**
 * 阻塞读取一次有效超声波数据
 * 先清除旧标志, 然后轮询等待 recvMsg 产生新数据
 * @param timeout_ms 超时时间(ms), 超时返回0
 * @return 距离值(mm), 超时或无数据返回0
 */
static uint32_t read_distance(uint32_t timeout_ms)
{
    uint32_t waited = 0U;
    g_distance_valid = 0U;              /* 清除旧数据, 等待新一轮稳定读数 */

    while (g_distance_valid == 0U && waited < timeout_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(10));  /* 10ms轮询一次 */
        waited += 10U;
    }

    if (g_distance_valid == 0U) return 0U;  /* 超时: 无有效数据 */
    return g_distance_mm;                    /* 返回最新距离 */
}

/**
 * 距离 -> 列号映射
 * 已校准: 找最近的校准值, 允许偏差±25mm
 * 未校准: 使用硬编码阈值区间
 * @param d 超声波距离(mm)
 * @return 列号(1~6), 不属于任何列返回0
 */
static uint8_t get_range(uint32_t d)
{
    uint8_t k;

    if (g_calibrated)
    {
        /* === 校准模式: 找最接近的校准值 === */
        uint32_t min_d = 9999U;
        uint8_t  best  = 0U;

        for (k = 1U; k <= RANGE_CNT; k++)
        {
            uint32_t diff = (d > g_cal_val[k])
                          ? (d - g_cal_val[k])
                          : (g_cal_val[k] - d);
            if (diff < min_d) { min_d = diff; best = k; }
        }

        if (min_d <= 25U) return best;     /* 在校准值±25mm范围内 */
        return 0U;                          /* 偏差过大, 不属于任何列 */
    }

    /* === 未校准模式: 硬编码阈值 === */
    /* 这些区间需要根据实际物理布局调整 */
    if      (d >= 1475U && d <= 1496U) return 1U;   /* 列1: ~1485mm */
    else if (d >= 1630U && d <= 1645U) return 2U;   /* 列2: ~1637mm */
    else if (d >= 1775U && d <= 1795U) return 3U;   /* 列3: ~1785mm */
    else if (d >= 1910U && d <= 1940U) return 4U;   /* 列4: ~1925mm */
    else if (d >= 2050U && d <= 2100U) return 5U;   /* 列5: ~2075mm */
    else if (d >= 2210U && d <= 2240U) return 6U;   /* 列6: ~2225mm */
    return 0U;
}

/* 模式2 get_range: 4列(7~10), 校准后用校准值, 未校准用硬编码 */
static uint8_t get_range2(uint32_t d)
{
    uint8_t k;
    if (g_calibrated2)
    {
        uint32_t min_d = 9999U;
        uint8_t  best  = 0U;
        for (k = 1U; k <= 4U; k++)
        {
            uint32_t diff = (d > g_cal_val2[k]) ? (d - g_cal_val2[k]) : (g_cal_val2[k] - d);
            if (diff < min_d) { min_d = diff; best = k; }
        }
        if (min_d <= 25U) return (uint8_t)(best + 6U);  /* 列7~10 = 索引+6 */
        return 0U;
    }
    if      (d >= 2350U && d <= 2395U) return 7U;
    else if (d >= 2495U && d <= 2540U) return 8U;
    else if (d >= 2640U && d <= 2685U) return 9U;
    else if (d >= 2790U && d <= 2840U) return 10U;
    return 0U;
}

/* FreeRTOS 钩子函数 */

/**
 * 栈溢出钩子
 * 发生栈溢出时进入此函数, 关中断死循环, 防止更严重的错误
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("STACK OVERFLOW: %s\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    while (1) { ; }                     /* 死循环, 等待看门狗复位 */
}

/**
 * FreeRTOS启动函数
 * 在main.c中调用, 创建全部5个任务并启动调度器
 */
void FreeRTOS_Start(void)
{
    /* 创建任务 (按优先级从高到低) */
    xTaskCreate(timeTask,       "TIME",    Time_TASK_STACK_SIZE,
                NULL, Time_TASK_PRIORITY,  &Time_Task_handle);
    xTaskCreate(recvMsg,        "RecvMsg", Recv_TASK_STACK_SIZE,
                NULL, Recv_TASK_PRIORITY, &Recv_Task_handle);
    xTaskCreate(task_key,       "TASK_KEY",KEY_STACK,
                NULL, KEY_PRIO,           &task_key_handle);
    xTaskCreate(task_scan,      "SCAN",    SCAN_STACK,
                NULL, SCAN_PRIO,          &Task_Scan_handle);
    /* 打印任务信息表 (通过USART1输出到PC串口助手) */
    printf("\r\n========== FreeRTOS Task Info ==========\r\n");
    printf("%-12s %8s %6s\r\n", "Name", "Stack(w)", "Prio");
    printf("%-12s %8u %6u\r\n", "TIME",    Time_TASK_STACK_SIZE, Time_TASK_PRIORITY);
    printf("%-12s %8u %6u\r\n", "RecvMsg", Recv_TASK_STACK_SIZE, Recv_TASK_PRIORITY);
    printf("%-12s %8u %6u\r\n", "TASK_KEY",KEY_STACK,            KEY_PRIO);
    printf("%-12s %8u %6u\r\n", "SCAN",    SCAN_STACK,           SCAN_PRIO);
    printf("----------------------------------------\r\n");
    printf("FreeHeap: %u bytes\r\n", xPortGetFreeHeapSize());
    printf("========================================\r\n\r\n");

    vTaskStartScheduler();              /* 启动调度器, 永不返回 */
}

/* ================================================================
 * timeTask — CPU运行时间统计 (调试用)
 * 每5秒通过USART1输出各任务CPU占用率
 * 依赖 FreeRTOSConfig.h 中 configGENERATE_RUN_TIME_STATS=1
 * ================================================================ */
void timeTask(void *pvParameters)
{
    char stats_buf[512];                /* 运行时间统计缓冲区(需大栈) */
    vTaskDelay(pdMS_TO_TICKS(3000));    /* 启动后等3秒再开始统计 */

    while (1)
    {
        vTaskGetRunTimeStats(stats_buf);
        printf("\r\n========== CPU Usage ==========\r\n");
        printf("Task          AbsTime       %%CPU\r\n");
        printf("%s", stats_buf);
        printf("========================================\r\n\r\n");
        vTaskDelay(pdMS_TO_TICKS(5000));/* 每5秒输出一次 */
    }
}

/* ================================================================
 * task_key — 按键扫描任务
 *
 * 按键定义 (key.h):
 *   KEY0 - 切换LED状态
 *   KEY1 - 步进电机正转90度 (测试用)
 *   KEY2 - 预留 (功能由串口屏命令替代)
 *   KEY3 - 关闭LED和蜂鸣器
 *
 * 扫描周期: 20ms (消抖由KEY_Scan内部处理)
 * ================================================================ */
static void task_key(void *pvParam)
{
    u8 key_val = 0U;

    while (1)
    {
        key_val = KEY_Scan(0);          /* 0=不等待, 立即返回 */

        switch (key_val)
        {
            case KEY0_PRES:             /* KEY0: 切换LED */
                LED = !LED;
                printf("KEY0\r\n");
                break;

            case KEY1_PRES:             /* KEY1: 电机正转90度(调试) */
                printf("KEY1: Motor +90deg\r\n");
                Stepper_Run(1, 90.0f);  /* dir=1正转, 90度 */
                printf("KEY1: Done\r\n");
                break;

            case KEY2_PRES:             /* KEY2: 预留, 无操作 */
                break;

            case KEY3_PRES:             /* KEY3: 紧急关闭 */
                LED = 0;
                BEEP = 0;               /* PE5 蜂鸣器 */
                printf("KEY3\r\n");
                break;

            default: break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  /* 20ms扫描间隔 */
    }
}

/* ================================================================
 * move_between — 步进电机相对移动
 *
 * 用于在扫描过程中从当前角度移动到目标角度
 * 步进电机方向约定:
 *   dir=0: 正转 (角度增大, 向远端扫描)
 *   dir=1: 反转 (角度减小, 向0度归位)
 *
 * 各pass对应的绝对角度:
 *   pass1 =  7.5°, pass2 = 22.5°, pass3 = 37.5°
 *   pass4 = 52.5°, pass5 = 67.5°, pass6 = 82.5°
 *   pass7 = 97.5°
 *  公式: angle = 7.5 + (pass-1) * 15
 *
 * @param curr   当前所在pass号
 * @param target 目标pass号
 * ================================================================ */
static void move_between(uint8_t curr, uint8_t target)
{
    if (target == curr) return;         /* 已在目标位置 */

    /* 每pass间距15度 */
    float delta = (float)((target > curr)
                 ? (target - curr)
                 : (curr - target)) * 15.0f;

    /* 目标更大→正转, 目标更小→反转 */
    uint8_t dir = (target > curr) ? 0U : 1U;

    Stepper_Run(dir, delta);
}

/* ================================================================
 * task_scan — 扫描定位任务 (核心)
 *
 * 工作流程:
 *   1. 等待串口屏发送 0xFF 0x01 0xFE (开始扫描)
 *   2. 阶段1: 从pass1开始逐角度扫描, 每个角度读3次判断有无回波
 *      有回波→记录该pass, 继续
 *      无回波且之前有记录→停止扫描
 *   3. 阶段2: 根据有回波的角度数量采用不同策略:
 *       0个: 强制输出最近区间坐标
 *       1个: 采15次投票确定列号
 *       2个: 比较两个角度的距离稳定度(极差), 取更稳定的
 *       3+个: 只采集中间角度
 *   4. 蜂鸣器提示, 发送结果到串口屏, 归零
 *
 * 坐标计算公式: site = (pass-1)*10 + range
 *   pass=行号(A=1,B=2,...), range=列号(1~6)
 *   例: pass=3,range=4 → site=24 → C4
 * ================================================================ */
void task_scan(void *pvParameters)
{
    uint8_t  cmd = CMD_NONE;          /* 当前命令 (由UART4 ISR写入) */

    uint8_t  active_pass[MAX_ANGLES]; /* 阶段1记录的有回波的pass号列表 */
    uint8_t  active_cnt = 0U;         /* 有回波的pass数量 */
    uint8_t  last_pass  = 0U;         /* 当前电机所在的pass号, 用于归零计算 */
    uint8_t  p, k;                    /* 循环变量 */

    uint8_t  cal_mode = 0U;           /* 模式1校准标志 */
    uint8_t  cal_idx  = 0U;           /* 正在校准的列号(1~6) */
    uint32_t cal_sum  = 0U;           /* 校准累加 */
    uint8_t  cal_n    = 0U;           /* 校准有效样本数 */

    uint8_t  cal2_mode = 0U;          /* 模式2校准标志 */
    uint8_t  cal2_idx  = 0U;          /* 模式2校准列号(1~4→列7~10) */
    uint32_t cal2_sum  = 0U;          /* 模式2校准累加 */
    uint8_t  cal2_n    = 0U;          /* 模式2校准有效数 */

    /* 模式2扫描状态 */
    #define MAX_ANGLES2 12U
    uint8_t  active_pass2[MAX_ANGLES2];
    uint8_t  active_cnt2;
    uint8_t  last_pass2;

    uint32_t d;                       /* 临时距离值 */
    uint8_t  r;                       /* 临时列号 */

    while (1)
    {
        /* UART4 ISR已解析命令到 g_uart4_cmd, 统一在此处理所有命令 */
        cmd = g_uart4_cmd;
        if (cmd != CMD_NONE)
        {
            g_uart4_cmd = CMD_NONE;   /* 立即消费, 避免竞态 */

            /* ---- 复位 ---- */
            if (cmd == CMD_RESET)
            {
                active_cnt = 0U;
                last_pass  = 0U;
                cal_mode = 0U;
                cal_idx  = 0U;
                g_calibrated = 0U;
                g_cal_mode_active = 0U;
            }

            /* ---- 进入校准模式 ---- */
            else if (cmd == CMD_CAL_MODE)
            {
                cal_mode = 1U;
                cal_idx  = 0U;
                g_cal_mode_active = 1U;     /* 暂停recvMsg的n0发送 */
                BEEP = 1;
                Stepper_Run(0, 37.5f);      /* 转到区域C */
                BEEP = 0;
            }

            /* ---- 校准值1~6 (模式1) ---- */
            else if (cmd >= CMD_CAL_VAL_BASE && cmd <= CMD_CAL_VAL_BASE + 5U && cal_mode)
            {
                cal_idx = cmd - CMD_CAL_VAL_BASE + 1U;
                cal_sum = 0U; cal_n = 0U;
                for (k = 0U; k < 10U; k++)
                {
                    uint32_t dv = read_distance(100U);
                    if (dv > 100U && dv < 3000U) { cal_sum += dv; cal_n++; }
                }
                if (cal_n > 0U)
                    uart4_send_cal_val(cal_idx, (uint16_t)(cal_sum / (uint32_t)cal_n));
            }

            /* ---- 进入校准模式2 ---- */
            else if (cmd == CMD_CAL2_MODE)
            {
                cal2_mode = 1U;
                cal2_idx  = 0U;
                g_cal2_mode_active = 1U;
                Stepper_Run(0, 41.25f);     /* 转到区域M */
            }

            /* ---- 校准值1~4 (模式2, 0x21~0x24) ---- */
            else if (cmd >= CMD_CAL2_VAL_BASE && cmd <= CMD_CAL2_VAL_BASE + 3U && cal2_mode)
            {
                cal2_idx = cmd - CMD_CAL2_VAL_BASE + 1U;
                cal2_sum = 0U; cal2_n = 0U;
                for (k = 0U; k < 10U; k++)
                {
                    uint32_t dv = read_distance(100U);
                    if (dv > 100U && dv < 3000U) { cal2_sum += dv; cal2_n++; }
                }
                if (cal2_n > 0U)
                    uart4_send_cal_val(cal2_idx, (uint16_t)(cal2_sum / (uint32_t)cal2_n));
            }

            /* ---- 确认校准值 (模式1) ---- */
            else if (cmd == CMD_CAL_CONFIRM && cal_mode && cal_idx > 0U)
            {
                cal_sum = 0U; cal_n = 0U;
                for (k = 0U; k < 10U; k++) {
                    uint32_t dv = read_distance(100U);
                    if (dv > 100U && dv < 3000U) { cal_sum += dv; cal_n++; }
                }
                if (cal_n > 0U) g_cal_val[cal_idx] = cal_sum / (uint32_t)cal_n;
            }

            /* ---- 确认校准值 (模式2) ---- */
            else if (cmd == CMD_CAL2_CONFIRM && cal2_mode && cal2_idx > 0U)
            {
                cal2_sum = 0U; cal2_n = 0U;
                for (k = 0U; k < 10U; k++) {
                    uint32_t dv = read_distance(100U);
                    if (dv > 100U && dv < 3000U) { cal2_sum += dv; cal2_n++; }
                }
                if (cal2_n > 0U) g_cal_val2[cal2_idx] = cal2_sum / (uint32_t)cal2_n;
            }

            /* ---- 校准完成 (模式1) ---- */
            else if (cmd == CMD_CAL_FINISH && cal_mode)
            {
                cal_mode = 0U;
                g_calibrated = 1U;
                g_cal_mode_active = 0U;
                Stepper_Run(1, 37.5f);
                printf("[CAL1] Complete: %u %u %u %u %u %u\r\n",
                       (unsigned int)g_cal_val[1],(unsigned int)g_cal_val[2],
                       (unsigned int)g_cal_val[3],(unsigned int)g_cal_val[4],
                       (unsigned int)g_cal_val[5],(unsigned int)g_cal_val[6]);
            }

            /* ---- 校准完成 (模式2) ---- */
            else if (cmd == CMD_CAL2_FINISH && cal2_mode)
            {
                cal2_mode = 0U;
                g_calibrated2 = 1U;
                g_cal2_mode_active = 0U;
                Stepper_Run(1, 41.25f);     /* 归零 */
                printf("[CAL2] Complete: %u %u %u %u\r\n",
                       (unsigned int)g_cal_val2[1],(unsigned int)g_cal_val2[2],
                       (unsigned int)g_cal_val2[3],(unsigned int)g_cal_val2[4]);
            }

            /* ---- 开始扫描定位 (模式1) ---- */
            else if (cmd == CMD_SCAN_START)
            {

            /* ========================================================
             * 阶段1: 逐个角度扫描, 找出有超声波回波的角度
             *
             * 扫描策略:
             *   - 从pass1(7.5°)开始, 每步转15°
             *   - 每个角度稳定100ms后读3次
             *   - 3次中至少1次有数据 → 记录该pass
             *   - 3次全无数据 且 之前已记录过 → 停止扫描
             *   - 电机保持在最后位置, 不归零 (节省时间)
             * ======================================================== */
            active_cnt = 0U;
            last_pass  = 0U;
            {
                uint8_t  zero_count;    /* 连续0的次数 */
                uint32_t rd;            /* 单次读数 */

                for (p = 1U; p <= MAX_ANGLES; p++)
                {
                    /* 计算本次转动角度 */
                    float step = (p == 1U) ? 7.5f : 15.0f;
                    Stepper_Run(0, step);       /* dir=0正转 */
                    last_pass = p;

                    vTaskDelay(pdMS_TO_TICKS(100)); /* 等待稳定 */

                    /* 连续读3次, 统计全0的次数 */
                    zero_count = 0U;
                    for (k = 0U; k < 3U; k++)
                    {
                        rd = read_distance(100U);   /* 100ms超时 */
                        if (rd == 0U) zero_count++; /* 无回波计数 */
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }

                    if (zero_count < 3U)
                    {
                        /* 有回波: 记录该pass */
                        active_pass[active_cnt++] = p;
                    }
                    else if (active_cnt > 0U)
                    {
                        /* 无回波且之前有过回波: 目标区域已过, 停止 */
                        break;
                    }
                    /* 无回波且之前也无: 继续扫描下一个角度 */
                }
            }

            /* 阶段2: 根据有回波角度数量, 选择不同策略确定坐标 */

            if (active_cnt == 0U)
            {
                /* ----------------------------------------------------
                 * 0个角度有数据: 强读一次, 用平均距离找最近区间
                 * 保证总是输出坐标, 不空手而归
                 * ---------------------------------------------------- */
                d = read_distance(50U);
                r = get_range(d);

                /* 阈值区间无匹配: 用6个区间中心做最近邻匹配 */
                if (r == 0U && d > 0U)
                {
                    uint32_t centers[7] = {0U, 1488U,1637U,1785U,
                                              1930U,2085U,2230U};
                    uint32_t min_d = 9999U;
                    for (k = 1U; k <= RANGE_CNT; k++)
                    {
                        uint32_t diff = (d > centers[k])
                                      ? (d - centers[k])
                                      : (centers[k] - d);
                        if (diff < min_d) { min_d = diff; r = k; }
                    }
                }

                if (r > 0U)
                {
                    uint8_t site = (last_pass - 1U) * 10U + r;
                    BEEP = 1; vTaskDelay(pdMS_TO_TICKS(200)); BEEP = 0;
                    uart4_send_distance((uint16_t)site);
                    uart4_send_location(site);
                }
                Stepper_Run(1, 7.5f + (float)(last_pass - 1U) * 15.0f);
            }
            else if (active_cnt == 1U)
            {
                /* ----------------------------------------------------
                 * 1个角度有数据: 直接到此角度采15次, 投票确定列号
                 * 投票无结果则用平均距离找最近区间中心
                 * ---------------------------------------------------- */
                uint8_t  target_p = active_pass[0U];    /* 目标pass */
                uint8_t  vote[RANGE_CNT + 1] = {0U};    /* 投票箱[1~6] */
                uint8_t  best_r = 0U, best_v = 0U;      /* 最佳列号/票数 */
                uint32_t sum_mm  = 0U;                  /* 距离累加(用于平均值) */
                uint8_t  sum_n   = 0U;                  /* 有效样本计数 */

                move_between(last_pass, target_p);
                last_pass = target_p;
                vTaskDelay(pdMS_TO_TICKS(200));         /* 稳定200ms */

                /* 采集15次 */
                for (k = 0U; k < 8U; k++)
                {
                    d = read_distance(50U);
                    r = get_range(d);
                    if (r > 0U) { vote[r]++; sum_mm += d; sum_n++; }
                }

                /* 找最高票 */
                for (k = 1U; k <= RANGE_CNT; k++)
                    if (vote[k] > best_v) { best_v = vote[k]; best_r = k; }

                /* 投票无结果: 平均距离找最近区间中心 */
                if (best_r == 0U && sum_n > 0U)
                {
                    uint32_t centers[7] = {0U, 1488U,1637U,1785U,
                                              1930U,2085U,2230U};
                    uint32_t avg = sum_mm / (uint32_t)sum_n;
                    uint32_t min_d = 9999U;
                    for (k = 1U; k <= RANGE_CNT; k++)
                    {
                        uint32_t diff = (avg > centers[k])
                                      ? (avg - centers[k])
                                      : (centers[k] - avg);
                        if (diff < min_d) { min_d = diff; best_r = k; }
                    }
                }

                if (best_r > 0U)
                {
                    uint8_t site = (target_p - 1U) * 10U + best_r;
                    BEEP = 1; vTaskDelay(pdMS_TO_TICKS(200)); BEEP = 0;
                    uart4_send_distance((uint16_t)site);
                    uart4_send_location(site);
                }
                Stepper_Run(1, 7.5f + (float)(target_p - 1U) * 15.0f);
            }
            else if (active_cnt == 2U)
            {
                /* ----------------------------------------------------
                 * 2个角度有数据: 比较原始距离稳定性 (不依赖阈值区间)
                 *
                 * 算法:
                 *   每个角度采集最多15个原始距离值(>100且<3000mm)
                 *   计算极差 (max-min)
                 *   极差更小的角度 → 数据更稳定 → 选它
                 *   用选中角度的平均距离找最近区间中心 → 确定列号
                 *
                 * 原理: 超声波正对目标时回波稳定(极差小)
                 *       偏角时回波跳动大(极差大)
                 * ---------------------------------------------------- */
                uint8_t  best_p   = 0U;         /* 最佳pass */
                uint8_t  best_r   = 0U;         /* 最佳列号 */
                uint32_t best_mm  = 0U;         /* 最佳角度的平均距离 */
                uint32_t best_rng = 9999U;      /* 最小极差(初始极大) */
                uint8_t  ap;

                for (ap = 0U; ap < 2U; ap++)    /* 遍历两个角度 */
                {
                    uint8_t  cur_p  = active_pass[ap];
                    uint32_t smp[8];           /* 原始距离样本 */
                    uint8_t  sn = 0U;           /* 有效样本数 */
                    uint32_t fmin, fmax, rng;

                    move_between(last_pass, cur_p);
                    last_pass = cur_p;
                    vTaskDelay(pdMS_TO_TICKS(200));

                    /* 采集原始距离, 只过滤明显异常值 */
                    for (k = 0U; k < 15U && sn < 8U; k++)
                    {
                        d = read_distance(50U);
                        if (d > 100U && d < 3000U) { smp[sn++] = d; }
                    }

                    if (sn < 3U) continue;      /* 样本太少, 跳过 */

                    /* 计算极差 = 最大值 - 最小值 */
                    fmin = smp[0U]; fmax = smp[0U];
                    for (k = 1U; k < sn; k++)
                    {
                        if (smp[k] < fmin) fmin = smp[k];
                        if (smp[k] > fmax) fmax = smp[k];
                    }
                    rng = fmax - fmin;

                    /* 极差更小 → 更稳定 → 选当前角度 */
                    if (rng < best_rng)
                    {
                        best_rng = rng;
                        best_p   = cur_p;
                        /* 计算选中角度的平均距离 */
                        {
                            uint32_t sum = 0U;
                            for (k = 0U; k < sn; k++) sum += smp[k];
                            best_mm = sum / (uint32_t)sn;
                        }
                    }
                }

                /* 用选中角度的平均距离找最近区间中心 */
                if (best_rng < 9999U)
                {
                    uint32_t centers[7] = {0U, 1488U,1637U,1785U,
                                              1930U,2085U,2230U};
                    uint32_t min_d = 9999U;
                    for (k = 1U; k <= RANGE_CNT; k++)
                    {
                        uint32_t diff = (best_mm > centers[k])
                                      ? (best_mm - centers[k])
                                      : (centers[k] - best_mm);
                        if (diff < min_d) { min_d = diff; best_r = k; }
                    }
                }

                if (best_r > 0U)
                {
                    uint8_t site = (best_p - 1U) * 10U + best_r;
                    BEEP = 1; vTaskDelay(pdMS_TO_TICKS(200)); BEEP = 0;
                    uart4_send_distance((uint16_t)site);
                    uart4_send_location(site);
                }
                /* 从当前位置(last_pass)归零 */
                Stepper_Run(1, 7.5f + (float)(last_pass - 1U) * 15.0f);
            }
            else
            {
                /* ----------------------------------------------------
                 * 3个及以上角度有数据: 取中间角度, 采15次投票+fallback
                 * 例: [2,3,4,5] → 取3 (中间偏前)
                 * ---------------------------------------------------- */
                uint8_t mid_pass = active_pass[active_cnt / 2U];
                uint8_t vote[RANGE_CNT + 1] = {0U};
                uint8_t best_r = 0U, best_v = 0U;
                uint32_t sum_mm  = 0U;
                uint8_t  sum_n   = 0U;

                move_between(last_pass, mid_pass);
                last_pass = mid_pass;
                vTaskDelay(pdMS_TO_TICKS(200));

                for (k = 0U; k < 8U; k++)
                {
                    d = read_distance(50U);
                    r = get_range(d);
                    if (r > 0U) { vote[r]++; sum_mm += d; sum_n++; }
                }
                for (k = 1U; k <= RANGE_CNT; k++)
                    if (vote[k] > best_v) { best_v = vote[k]; best_r = k; }

                if (best_r == 0U && sum_n > 0U)
                {
                    uint32_t centers[7] = {0U, 1488U,1637U,1785U,
                                              1930U,2085U,2230U};
                    uint32_t avg = sum_mm / (uint32_t)sum_n;
                    uint32_t min_d = 9999U;
                    for (k = 1U; k <= RANGE_CNT; k++)
                    {
                        uint32_t diff = (avg > centers[k])
                                      ? (avg - centers[k])
                                      : (centers[k] - avg);
                        if (diff < min_d) { min_d = diff; best_r = k; }
                    }
                }

                if (best_r > 0U)
                {
                    uint8_t site = (mid_pass - 1U) * 10U + best_r;
                    BEEP = 1; vTaskDelay(pdMS_TO_TICKS(200)); BEEP = 0;
                    uart4_send_distance((uint16_t)site);
                    uart4_send_location(site);
                }
                Stepper_Run(1, 7.5f + (float)(mid_pass - 1U) * 15.0f);
            }
        }  /* end else-if (cmd == CMD_SCAN_START) */

            /* ---- 开始扫描定位 (模式2) ---- */
            else if (cmd == CMD_SCAN_START2)
            {
            /* 模式2: H~T共12区, 起始3.75°, 步进7.5°, 列7~10 */
            #define PASS2_DEG(p) ((p)==1U?3.75f:(3.75f+(float)((p)-1U)*7.5f))
            active_cnt2 = 0U; last_pass2 = 0U;
            {
                uint8_t zc; uint32_t rd;
                for (p = 1U; p <= MAX_ANGLES2; p++) {
                    Stepper_Run(0, (p==1U)?3.75f:7.5f); last_pass2 = p;
                    vTaskDelay(pdMS_TO_TICKS(100));
                    zc = 0U;
                    for (k = 0U; k < 3U; k++) {
                        rd = read_distance(50U);
                        if (rd == 0U) zc++;
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    if (zc < 3U) active_pass2[active_cnt2++] = p;
                    else if (active_cnt2 > 0U) break;
                }
            }
            /* --- 阶段2 --- */
            if (active_cnt2 == 0U) {
                d = 0U; for (k = 0U; k < 5U; k++) {
                    d = read_distance(100U); if (d > 100U) break;
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                r = get_range2(d);
                if (r > 0U) { uint8_t s = (last_pass2-1U)*10U+(r-6U);
                    BEEP=1;vTaskDelay(pdMS_TO_TICKS(200));BEEP=0;
                    uart4_send_distance((uint16_t)s);uart4_send_location2(s); }
                Stepper_Run(1, PASS2_DEG(last_pass2));
            } else if (active_cnt2 == 1U) {
                uint8_t vote[4]={0U},br=0U,bv=0U,tp=active_pass2[0U];
                { float to=PASS2_DEG(tp),ca=PASS2_DEG(last_pass2);
                  if(to>ca)Stepper_Run(0,to-ca);else if(ca>to)Stepper_Run(1,ca-to);last_pass2=tp; }
                vTaskDelay(pdMS_TO_TICKS(200));
                for(k=0U;k<15U;k++){d=read_distance(50U);r=get_range2(d);if(r>=7U&&r<=10U)vote[r-7U]++;}
                for(k=0U;k<4U;k++)if(vote[k]>bv){bv=vote[k];br=(uint8_t)(k+7U);}
                if(br>0U){uint8_t s=(tp-1U)*10U+(br-6U);
                    BEEP=1;vTaskDelay(pdMS_TO_TICKS(200));BEEP=0;
                    uart4_send_distance((uint16_t)s);uart4_send_location2(s);}
                Stepper_Run(1,PASS2_DEG(tp));
            } else if (active_cnt2 >= 3U) {
                uint8_t mp=active_pass2[active_cnt2/2U],vote[4]={0U},br=0U,bv=0U;
                { float to=PASS2_DEG(mp),ca=PASS2_DEG(last_pass2);
                  if(to>ca)Stepper_Run(0,to-ca);else if(ca>to)Stepper_Run(1,ca-to);last_pass2=mp; }
                vTaskDelay(pdMS_TO_TICKS(200));
                for(k=0U;k<15U;k++){d=read_distance(50U);r=get_range2(d);if(r>=7U&&r<=10U)vote[r-7U]++;}
                for(k=0U;k<4U;k++)if(vote[k]>bv){bv=vote[k];br=(uint8_t)(k+7U);}
                if(br>0U){uint8_t s=(mp-1U)*10U+(br-6U);
                    BEEP=1;vTaskDelay(pdMS_TO_TICKS(200));BEEP=0;
                    uart4_send_distance((uint16_t)s);uart4_send_location2(s);}
                Stepper_Run(1,PASS2_DEG(mp));
            } else {
                /* 2角度: 比极差 */
                uint8_t bp2=0U,br2=0U,ap;
                uint32_t brng=9999U,bmm2=0U;
                for(ap=0U;ap<2U;ap++){uint8_t cp=active_pass2[ap],sn=0U;uint32_t smp[15],fmin,fmax,rng;
                    { float to=PASS2_DEG(cp),ca=PASS2_DEG(last_pass2);
                      if(to>ca)Stepper_Run(0,to-ca);else if(ca>to)Stepper_Run(1,ca-to);last_pass2=cp; }
                    vTaskDelay(pdMS_TO_TICKS(200));
                    for(k=0U;k<30U&&sn<15U;k++){d=read_distance(50U);if(d>100U&&d<3000U)smp[sn++]=d;}
                    if(sn<3U)continue;
                    fmin=smp[0U];fmax=smp[0U];
                    for(k=1U;k<sn;k++){if(smp[k]<fmin)fmin=smp[k];if(smp[k]>fmax)fmax=smp[k];}
                    rng=fmax-fmin;
                    if(rng<brng){brng=rng;bp2=cp;uint32_t sum=0U;for(k=0U;k<sn;k++)sum+=smp[k];bmm2=sum/(uint32_t)sn;}
                }
                if(brng<9999U){br2=get_range2(bmm2);if(br2>0U){uint8_t s=(bp2-1U)*10U+(br2-6U);
                    BEEP=1;vTaskDelay(pdMS_TO_TICKS(200));BEEP=0;
                    uart4_send_distance((uint16_t)s);uart4_send_location2(s);}}
                Stepper_Run(1,PASS2_DEG(last_pass2));
            }
            }  /* end CMD_SCAN_START2 */

        }  /* end if (cmd != CMD_NONE) */

        vTaskDelay(pdMS_TO_TICKS(50));  /* 任务循环间隔50ms */
    }
}

/* ================================================================
 * (task_calibrate 已合并到 task_scan 中)
 * ================================================================
 *
 * 校准目的:
 *   题目中四分之一圆的圆心到列1~6的距离相等,
 *   校准可消除安装误差, 提高坐标识别准确率
 *
 * 校准流程:
 *   1. 串口屏发 0xFF 0x05 0xFE → 电机自动转到区域C(37.5°)
 *   2. 将超声波放在列1位置 → 发 0xFF 0x11 0xFE → 自动采10次取平均
 *      → 结果发到串口屏 n0.val=平均距离
 *   3. 确认无误 → 发 0xFF 0x20 0xFE → 保存为 cal_val[1]
 *   4. 重复步骤2~3 校准列2~6 (0x12~0x16)
 *   5. 发 0xFF 0x21 0xFE → 校准完成, 电机归零
 *
 * 串口屏控件映射:
 *   校准值1 → n0.val, 校准值2 → n1.val, ..., 校准值6 → n5.val
 *
 * 注意:
 *   - 校准过程中不要发送扫描命令
 *   - 校准值存储在 g_cal_val[1~6] 中, 掉电丢失
 *   - 复位命令 0x03 可清除校准数据
 * ================================================================ */
void task_calibrate(void *pvParameters)
{
    uint8_t  cmd = CMD_NONE;          /* 当前命令 (由UART4 ISR写入) */
    uint8_t  cal_idx = 0U;            /* 正在校准的列号(1~6), 0=未选中 */
    uint8_t  cal_mode = 0U;           /* 校准模式标志: 1=在校准中 */
    uint32_t sum_mm;                  /* 距离累加 */
    uint8_t  k, n;                    /* 循环变量, 有效样本数 */

    while (1)
    {
        /* UART4 ISR已解析命令到 g_uart4_cmd, 此处直接读取 */
        cmd = g_uart4_cmd;
        if (cmd != CMD_NONE)
        {
            /* 仅处理本任务关心的命令, 其他忽略 */
            if (cmd != CMD_CAL_MODE && cmd != CMD_CAL_CONFIRM
                && cmd != CMD_CAL_FINISH && cmd != CMD_RESET
                && !(cmd >= CMD_CAL_VAL_BASE && cmd <= CMD_CAL_VAL_BASE + 5U))
                cmd = CMD_NONE;
            else
                g_uart4_cmd = CMD_NONE;  /* 消费命令 */
        }

        /* ---- 复位: 清除校准数据, 退出校准模式 ---- */
        if (cmd == CMD_RESET)
        {
            cmd = CMD_NONE;
            if (cal_mode) { Stepper_Run(1, 37.5f); }  /* 正在校准→归零 */
            cal_mode = 0U;
            cal_idx  = 0U;
            g_calibrated = 0U;          /* 恢复使用硬编码阈值 */
        }

        /* ---- 进入校准模式: 电机转到区域C ---- */
        if (cmd == CMD_CAL_MODE)
        {
            cmd = CMD_NONE;
            cal_mode = 1U;
            cal_idx  = 0U;
            Stepper_Run(0, 37.5f);      /* 正转37.5°, 到pass3(区域C) */
            printf("[CAL] Enter calibration mode, moved to area C\r\n");
        }

        /* ---- 校准第N个值 (0x11~0x16, 对应列1~6) ---- */
        if (cal_mode && cmd >= CMD_CAL_VAL_BASE
            && cmd <= CMD_CAL_VAL_BASE + 5U)
        {
            cal_idx = cmd - CMD_CAL_VAL_BASE + 1U;  /* 1~6 */
            cmd = CMD_NONE;

            /* 采集10次取平均 */
            sum_mm = 0U;
            n = 0U;
            for (k = 0U; k < 10U; k++)
            {
                uint32_t d = read_distance(100U);
                if (d > 100U && d < 3000U) { sum_mm += d; n++; }
            }

            if (n > 0U)
            {
                uint16_t avg = (uint16_t)(sum_mm / (uint32_t)n);
                /* 发送到对应串口屏控件: idx=1→n0, idx=2→n1, ... */
                uart4_send_cal_val(cal_idx, avg);
                printf("[CAL] idx=%u avg=%u mm\r\n", cal_idx, avg);
            }
        }

        /* ---- 确认当前校准值: 再采10次确认, 写入 g_cal_val ---- */
        if (cal_mode && cmd == CMD_CAL_CONFIRM && cal_idx > 0U)
        {
            cmd = CMD_NONE;

            /* 再读10次确认值 */
            sum_mm = 0U;
            n = 0U;
            for (k = 0U; k < 10U; k++)
            {
                uint32_t d = read_distance(100U);
                if (d > 100U && d < 3000U) { sum_mm += d; n++; }
            }

            if (n > 0U)
            {
                g_cal_val[cal_idx] = sum_mm / (uint32_t)n;
                printf("[CAL] Confirmed idx=%u val=%u\r\n",
                       cal_idx, (unsigned int)g_cal_val[cal_idx]);
            }
            cal_idx = 0U;               /* 重置, 等待下一个校准命令 */
        }

        /* ---- 校准完成: 启用校准数据, 电机归零 ---- */
        if (cal_mode && cmd == CMD_CAL_FINISH)
        {
            cmd = CMD_NONE;
            cal_mode = 0U;
            g_calibrated = 1U;          /* 启用校准值 */
            Stepper_Run(1, 37.5f);      /* 反转37.5°, 回到0度 */
            printf("[CAL] Calibration complete, return to 0\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(50));  /* 任务循环间隔50ms */
    }
}

/* ================================================================
 * recvMsg — 超声波数据接收任务
 *
 * 数据流: 超声波模块 → USART3(PB10/PB11,9600bps) → 环形缓冲 → 本任务
 *
 * 帧格式 (4字节):
 *   0x5A | 距离高8位 | 距离低8位 | 校验和
 *   校验和 = 前3字节 XOR
 *
 * 滤波策略:
 *   8样本滑动窗口, 窗口内极差 ≤ ALIGN_THRESH(20mm) 认为数据稳定
 *   稳定后更新 g_distance_mm 和 g_distance_valid
 *
 * 输出:
 *   实时距离通过UART4发送到串口屏: n0.val=<距离>\xFF\xFF\xFF
 *   同时通过USART1 printf 输出调试信息
 * ================================================================ */
void recvMsg(void *pvParameters)
{
    uint8_t  byte;                     /* 从USART3环形缓冲读取的字节 */
    uint8_t  rx_buf[4];               /* 接收帧缓冲: [0]=0x5A [1]=H [2]=L [3]=XOR */
    uint8_t  idx = 0U;                /* 帧解析状态: 0~3 */
    uint32_t samples[SAMPLE_COUNT];   /* 滑动窗口样本 (8个) */
    uint8_t  sample_idx  = 0U;        /* 当前样本写入位置 */
    uint8_t  sample_full = 0U;        /* 窗口是否已满 (满后才开始输出) */

    while (1)
    {
        uint8_t got_data = 0U;

        /* 从USART3环形缓冲读取 (ISR写入, 此处消费) */
        while (usart3_ring_get(&byte) != 0U)
        {
            got_data = 1U;

            /* 帧同步状态机 */
            if (idx == 0U)
            {
                /* 状态0: 等待帧头 0x5A */
                if (byte == 0x5AU) { rx_buf[0] = byte; idx = 1U; }
            }
            else if (idx == 1U)
            {
                /* 状态1: 接收距离高字节 */
                rx_buf[1] = byte; idx = 2U;
            }
            else if (idx == 2U)
            {
                /* 状态2: 接收距离低字节 */
                rx_buf[2] = byte; idx = 3U;
            }
            else
            {
                /* 状态3: 接收校验和, 验证并处理 */
                rx_buf[3] = byte; idx = 0U;

                /* XOR校验: 前3字节异或应等于第4字节 */
                if ((rx_buf[0] ^ rx_buf[1] ^ rx_buf[2]) == rx_buf[3])
                {
                    /* 重新组装距离值 */
                    uint32_t raw_dist = ((uint32_t)rx_buf[1] << 8)
                                      | rx_buf[2];

                    if (raw_dist <= MAX_DIST_MM)
                    {
                        g_raw_count++;                          /* 调试计数 */
                        samples[sample_idx++] = raw_dist;       /* 写入窗口 */
                        if (sample_idx >= SAMPLE_COUNT)
                            { sample_idx = 0U; sample_full = 1U; }

                        if (sample_full)
                        {
                            /* 窗口已满: 检查数据对齐度 */
                            uint32_t vmin = 0xFFFFFFFFU, vmax = 0U, sum = 0U;
                            uint8_t i;

                            for (i = 0U; i < SAMPLE_COUNT; i++)
                            {
                                uint32_t val = samples[i];
                                if (val < vmin) vmin = val;
                                if (val > vmax) vmax = val;
                                sum += val;
                            }

                            if ((vmax - vmin) <= ALIGN_THRESH)
                            {
                                /* 数据稳定: 输出平均距离 */
                                g_distance_mm    = sum / SAMPLE_COUNT;
                                g_distance_valid = 1U;

                                /* 实时发送距离到串口屏 (校准模式下暂停) */
                                if (!g_cal_mode_active && !g_cal2_mode_active)
                                    uart4_send_distance((uint16_t)g_distance_mm);
                            }
                            else
                            {
                                /* 数据波动大: 无效 */
                                g_distance_valid = 0U;
                            }
                        }
                    }
                }
            }
        }

        /* 无数据时短暂休眠, 避免空转 */
        if (got_data == 0U) { vTaskDelay(pdMS_TO_TICKS(50)); }
    }
}
