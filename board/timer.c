#include "timer.h"
#include "usart.h"
#include "IMU_ADIS16xxx.h"
#include "ReportIMU.h"
#include "GPS.h"



/*TIM_Period--1000   TIM_Prescaler--71 -->中断周期为1ms*/
void TIM2_Configuration()
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		NVIC_InitTypeDef   			 NVIC_InitStructure;	
		RCC_ClocksTypeDef RCC_Clocks;
	
		RCC_GetClocksFreq(&RCC_Clocks);
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period=20000;		 								/* 自动重装载寄存器周期的值(计数值) */
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= (uint16_t) ((RCC_Clocks.SYSCLK_Frequency /2) / 100000) - 1; /* 时钟预分频数 100000hz */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* 采样分频 */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* 向上计数模式 */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);							    		/* 清除溢出中断标志 */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM2, ENABLE);																		/* 开启时钟 */
	
		//定时中断   													
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; /* 抢占优先级 */	
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
    	  
}
//对84MHz（TIM3时钟源为168MHz/2）进行TIM_scale分频后作为计数时钟
void TIM3_Init(u32 TIM_scale, u32 TIM_Period)//TIM_Period为16位的数
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  	NVIC_InitTypeDef  NVIC_InitStructure; 
	 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  
	
  	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);	
	
  	TIM_TimeBaseStructure.TIM_Period = TIM_Period;//计数器重装值
  	TIM_TimeBaseStructure.TIM_Prescaler = 0;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  	TIM_PrescalerConfig(TIM3, (TIM_scale-1), TIM_PSCReloadMode_Immediate);
  	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  	TIM_Cmd(TIM3, ENABLE);
}

//中断定时 50ms 20Hz
void TIM2_IRQHandler(void)
{	
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		//UART_ReportIMU((int16_t)(yaw*1800/3.14f), (int16_t)(pitch*1800/3.14f), (int16_t)(roll*1800/3.14f), 0, 0, 0, 0);
		printf("@6,%.9lf,%.9lf,%.2lfe\r\n", GPSMsg.lat, GPSMsg.lon, (yaw*180/3.1416f));//data upload 
		//printf("@6,%.12lf,%.12lf,%.1lfe\r\n", GPSMsg.lat*100, GPSMsg.lon, test_r);//data upload 
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update); 		
	}		
  	
}
//中断定时 50ms 20Hz
void TIM3_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
	{			
		//UART_ReportIMU((int16_t)(yaw*1800/3.14f), (int16_t)(pitch*1800/3.14f), (int16_t)(roll*1800/3.14f), 0, 0, 0, 0);
		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update); 
	}		
   	
}

