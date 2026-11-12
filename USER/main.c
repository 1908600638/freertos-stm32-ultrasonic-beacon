#include "led.h"
#include "delay.h"
#include "key.h"
#include "motor.h"
#include "sys.h"
#include "usart.h"
#include "stdlib.h"
#include "FreeRTOS_demo.h"
int main(void)
{
	delay_init();	    	 //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart1_init(115200);      //串口1初始化(调试)
	uart3_init(9600);	 //串口3初始化(超声波, PB10/PB11)
	uart4_init(115200);      //串口4初始化(串口屏, PC10/PC11)
	LED_Init();          //LED初始化
	KEY_Init();          //按键初始化
	MOTOR_Init();		 //步进电机初始化
	FreeRTOS_Start();    //启动FreeRTOS调度器
	while(1)
	{
		// 被FreeRTOS接管，不会执行到这里
	}
}
