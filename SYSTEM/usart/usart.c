#include "sys.h"
#include "usart.h"
#include "delay.h"
#include <string.h>
#include <stdio.h>

#if 1
#pragma import(__use_no_semihosting)
struct __FILE
{
	int handle;

};
FILE __stdout;
void _sys_exit(int x)
{
	x = x;
}
int fputc(int ch, FILE *f)
{
	while((USART1->SR&0X40)==0);
    USART1->DR = (u8) ch;
	return ch;
}
#endif

/* USART3 接收环形缓冲 (超声波模块) */
static uint8_t usart3_ring_buf[USART3_RING_SIZE];
static volatile uint32_t usart3_ring_head = 0;
static volatile uint32_t usart3_ring_tail = 0;

void usart3_ring_put(uint8_t byte)
{
    uint32_t next = (usart3_ring_head + 1) & (USART3_RING_SIZE - 1);
    if (next != usart3_ring_tail)
    {
        usart3_ring_buf[usart3_ring_head] = byte;
        __DMB();
        usart3_ring_head = next;
    }
}

uint8_t usart3_ring_get(uint8_t *byte)
{
    if (usart3_ring_head == usart3_ring_tail)
        return 0;
    *byte = usart3_ring_buf[usart3_ring_tail];
    __DMB();
    usart3_ring_tail = (usart3_ring_tail + 1) & (USART3_RING_SIZE - 1);
    return 1;
}

/* UART4 接收环形缓冲 (串口屏回传命令) */
static uint8_t uart4_ring_buf[UART4_RING_SIZE];
static volatile uint32_t uart4_ring_head = 0;
static volatile uint32_t uart4_ring_tail = 0;

void uart4_ring_put(uint8_t byte)
{
    uint32_t next = (uart4_ring_head + 1) & (UART4_RING_SIZE - 1);
    if (next != uart4_ring_tail)
    {
        uart4_ring_buf[uart4_ring_head] = byte;
        __DMB();
        uart4_ring_head = next;
    }
}

uint8_t uart4_ring_get(uint8_t *byte)
{
    if (uart4_ring_head == uart4_ring_tail)
        return 0;
    *byte = uart4_ring_buf[uart4_ring_tail];
    __DMB();
    uart4_ring_tail = (uart4_ring_tail + 1) & (UART4_RING_SIZE - 1);
    return 1;
}

void uart1_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
  uint8_t rx_data;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    rx_data = USART_ReceiveData(USART1);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

void uart3_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t recv_byte = USART_ReceiveData(USART3);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        usart3_ring_put(recv_byte);
    }
}

void uart4_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_DeInit(UART4);
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  USART_Cmd(UART4, ENABLE);
}

/* UART4 命令帧解析 (ISR中直接解析, 避免多任务抢环形缓冲) */
static uint8_t  u4_cmd_buf[3];
static uint8_t  u4_cmd_idx = 0U;
volatile uint8_t g_uart4_cmd = CMD_NONE;   /* 全局命令, 任务轮询此标志 */

void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        uint8_t recv_byte = USART_ReceiveData(UART4);
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);

        /* 同时保留环形缓冲(兼容旧代码, 可删除) */
        uart4_ring_put(recv_byte);

        /* 帧解析: 0xFF | CMD | 0xFE */
        if (u4_cmd_idx == 0U)
        {
            if (recv_byte == 0xFFU) { u4_cmd_buf[0] = recv_byte; u4_cmd_idx = 1U; }
        }
        else if (u4_cmd_idx == 1U)
        {
            u4_cmd_buf[1] = recv_byte; u4_cmd_idx = 2U;
        }
        else
        {
            u4_cmd_buf[2] = recv_byte; u4_cmd_idx = 0U;
            if (u4_cmd_buf[0] == 0xFFU && u4_cmd_buf[2] == 0xFEU)
            {
                g_uart4_cmd = u4_cmd_buf[1];   /* 写入命令, 任务轮询读取 */
            }
        }
    }
}

void usart_send_char(uint8_t ch)
{
  USART_SendData(USART1, ch);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void usart_send_string(uint8_t *str)
{
  while(*str)
  {
    usart_send_char(*str++);
  }
}

/* UART4 发送函数 */

void uart4_send_byte(uint8_t ch)
{
    USART_SendData(UART4, (uint16_t)ch);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
}

void uart4_send_distance(uint16_t distance_mm)
{
    char cmd[32];
    uint8_t len;
    uint8_t i;

    len = sprintf(cmd, "n0.val=%u", distance_mm);

    for (i = 0U; i < len; i++)
    {
        uart4_send_byte((uint8_t)cmd[i]);
    }

    uart4_send_byte(0xFF);
    uart4_send_byte(0xFF);
    uart4_send_byte(0xFF);

    printf("[UART4 TX] %s\r\n", cmd);
}

void uart4_send_location(uint8_t site)
{
    char cmd[32];
    char row;
    uint8_t col;
    uint8_t len;
    uint8_t i;

    if (site == 0U || site > 56U) return;

    row = 'A' + (char)((site - 1U) / 10U);
    col = ((site - 1U) % 10U) + 1U;

    len = sprintf(cmd, "t2.txt=\"\xC7\xF8\xD3\xF2:%c%d\"", row, col);

    for (i = 0U; i < len; i++)
    {
        uart4_send_byte((uint8_t)cmd[i]);
    }

    uart4_send_byte(0xFF);
    uart4_send_byte(0xFF);
    uart4_send_byte(0xFF);

    printf("[UART4 TX] %s\r\n", cmd);
}

/*
 * UART4 发送校准值到串口屏 (idx:1~6 → n0~n5.val=<val>)
 */
void uart4_send_cal_val(uint8_t idx, uint16_t val)
{
    char cmd[32];
    uint8_t len;
    uint8_t i;

    len = sprintf(cmd, "n%u.val=%u", (unsigned int)(idx - 1U), (unsigned int)val);

    for (i = 0U; i < len; i++)
    {
        uart4_send_byte((uint8_t)cmd[i]);
    }

    uart4_send_byte(0xFF);
    uart4_send_byte(0xFF);
    uart4_send_byte(0xFF);

    printf("[UART4 TX] %s\r\n", cmd);
}

/*
 * 模式2区域文本: site→H7~T10
 * pass=(site-1)/10+1  → H(1) I(2) ... T(12)
 * col=(site-1)%10+7   → 7 8 9 10
 */
void uart4_send_location2(uint8_t site)
{
    char cmd[32], row;
    uint8_t pass, col, len, i;
    const char rows[] = "HIJKLMNPQRST";   /* 12个, 跳过O */

    if (site == 0U) return;
    pass = (site - 1U) / 10U + 1U;
    col  = ((site - 1U) % 10U) + 7U;
    if (pass > 12U) return;

    row = rows[pass - 1U];
    len = sprintf(cmd, "t2.txt=\"\xC7\xF8\xD3\xF2:%c%d\"", row, col);

    for (i = 0U; i < len; i++)
        uart4_send_byte((uint8_t)cmd[i]);

    uart4_send_byte(0xFF);
    uart4_send_byte(0xFF);
    uart4_send_byte(0xFF);

    printf("[UART4 TX] %s\r\n", cmd);
}
