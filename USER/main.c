
#include "main.h"
#include "timer.h"
#include "usart.h"
#include "IMU_ADIS16xxx.h"
#include "led.h"
#include "key.h"
#include "GPS.h"
#include "ReportIMU.h"

// #include "arm_math.h"

static __IO uint32_t TimingDelay;

int main(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);/*!< 3 bits for pre-emption priority 1 bits for subpriority */
	usart2_init();
	printf("Usart2 init complete\r\n");
	KEY_init();
 	IMU_init();
// 	IMU_BE10();

	Delay(500);
	GPS_init();
	TIM2_Configuration();
		
	while(1)
	{
	}
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
 */ 
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}
/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}


