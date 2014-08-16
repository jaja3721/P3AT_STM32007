

#include "key.h"

//k1:pf8	k2:pf9	k3:pf10	k4:pf7	k5:pf6


void KEY_init()
{
	
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//I/O配置

	//M1、M2 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPS-PV(PA01)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
}

 //返回M1的值
uint8_t Get_M1()
 {
	return (GPIO_ReadInputDataBit(GPIOA,  GPIO_Pin_6));
 }
  //返回M2的值
uint8_t Get_M2()
 {
	return (GPIO_ReadInputDataBit(GPIOA,  GPIO_Pin_7));
 }
 
  //返回GPS-PV的值
uint8_t get_PV()
 {
	return GPIO_ReadInputDataBit(GPIOA,  GPIO_Pin_1) ;
 }
 
 
