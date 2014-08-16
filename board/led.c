#include "led.h"

void LED_Init(void)
{
  	GPIO_InitTypeDef  GPIO_InitStructure;

  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//   	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
//   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;        //PA4
//   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//   	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//   	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;         //PB0
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;          //PC0
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
// 		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;         //PD0
//   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//   	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//   	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void LEDOff(Led_Def Led)//LEDÃð
{
  	LED_PORT[Led]->BSRRL=LED_PIN[Led];
}

void LEDOn(Led_Def Led)//LEDÁÁ
{
  	LED_PORT[Led]->BSRRH=LED_PIN[Led];  
}

void LEDTog(Led_Def Led)//LED×´Ì¬·­×ª
{
  	LED_PORT[Led]->ODR^=LED_PIN[Led];
	
}



