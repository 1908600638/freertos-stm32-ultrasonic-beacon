#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"

#define USART_REC_LEN   200
#define EN_USART1_RX    1

extern u8  USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;

/* USART3 接收环形缓冲 (超声波模块) */
#define USART3_RING_SIZE  256U
void usart3_ring_put(uint8_t byte);
uint8_t usart3_ring_get(uint8_t *byte);

/* USART3 数据到达信号量 (FreeRTOS任务阻塞等待) */
#include "FreeRTOS.h"
#include "semphr.h"
extern SemaphoreHandle_t g_usart3_sem;

/* UART4 接收环形缓冲 (串口屏回传命令) */
#define UART4_RING_SIZE   64U
void uart4_ring_put(uint8_t byte);
uint8_t uart4_ring_get(uint8_t *byte);

/* 串口屏命令标志 (UART4 ISR解析后写入, 任务轮询读取) */
extern volatile uint8_t g_uart4_cmd;

#define CMD_NONE          0U
#define CMD_SCAN_START    1U    /* 0xFF 0x01 0xFE: 模式1开始扫描 */
#define CMD_SCAN_START2   2U    /* 0xFF 0x02 0xFE: 模式2开始扫描 */
#define CMD_RESET         3U    /* 0xFF 0x03 0xFE: 复位清除 */
#define CMD_CAL_MODE      5U    /* 0xFF 0x05 0xFE: 模式1进入校准 */
#define CMD_CAL2_MODE     6U    /* 0xFF 0x06 0xFE: 模式2进入校准 */
#define CMD_CAL_VAL_BASE  0x11U /* 0x11~0x16: 模式1校准值1~6 */
#define CMD_CAL2_VAL_BASE 0x21U /* 0x21~0x24: 模式2校准值1~4 */
#define CMD_CAL_CONFIRM   0x20U /* 0xFF 0x20 0xFE: 模式1确认 */
#define CMD_CAL_FINISH    0x21U /* 0xFF 0x21 0xFE: 模式1完成 / 模式2校准值1 */
#define CMD_CAL2_CONFIRM  0x30U /* 0xFF 0x30 0xFE: 模式2确认 */
#define CMD_CAL2_FINISH   0x31U /* 0xFF 0x31 0xFE: 模式2完成 */

/* 模式1校准值 (索引1~6) */
extern uint32_t g_cal_val[7];
extern uint8_t  g_calibrated;
/* 模式2校准值 (索引1~4, 对应列7~10) */
extern uint32_t g_cal_val2[5];
extern uint8_t  g_calibrated2;

void uart1_init(u32 bound);
void uart3_init(u32 bound);
void uart4_init(u32 bound);

/* UART4 发送接口 (串口屏) */
void uart4_send_byte(uint8_t ch);
void uart4_send_distance(uint16_t distance_mm);
void uart4_send_location(uint8_t site);
void uart4_send_location2(uint8_t site);  /* 模式2: H~T + 7~10 */

/* 校准值发送 (n0~n5.val=) */
void uart4_send_cal_val(uint8_t idx, uint16_t val);

#endif
