

// 连接：PA04-->INT	PB13-->CLK	PB14-->MOSI	PB15-->MISO
//sps 814.3

#include "IMU_ADIS16xxx.h"
#include "main.h"
#include "arm_math.h"
#include "math.h"
#include "GPS.h"

#define imu_div  10

static uint16_t count;
static uint16_t data[15];

volatile float roll,pitch,yaw;
volatile int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
float gps_dir_rad;//GPS角度，以弧度为单位

void spi2_init(void);
void RST_init(void);
void RST_IMU(void);
void EXTI_init(void);
int16_t map_14bits_to_16bits(uint16_t data_14bits);
void ADI_BurstReadMode(void);



//中断服务函数
void EXTI0_IRQHandler() //外部中断
 {
	 float t11,t12,t13,t21,t22,t23,t31,t32,t33;
	 float r11, r12, r13, r21, r22, r23;
	 float aax, aay, aaz, ggx, ggy, ggz, mmx, mmy, mmz;	 
	 float recipNorm;
		if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
		{	  
			count++;	
			 if(count==imu_div)
				 {
					 
					count=0;
					ADI_BurstReadMode();	
					 ggx=gx/1145.916f;//*0.05f*3.1415/180
					 ggy=gz/1145.916f;
					 ggz=-gy/1145.916f;
					 
					 aax=ax;//300
					 aay=az;
					 aaz=-ay;
					 
					 mmx=mx;
					 mmy=mz;
					 mmz=-my;
					 
					//MadgwickAHRSupdate(ggx, ggy, ggz, aax, aay, aaz, mmx, mmy, mmz);
					 MadgwickAHRSupdateIMU(ggx, ggy, ggz, aax, aay, aaz);
					 
					//更新方向余弦矩阵
					t11=q0*q0+q1*q1-q2*q2-q3*q3;
					t12=2.0f*(q1*q2+q0*q3);
					t13=2.0f*(q1*q3-q0*q2);
					//t21=2.0f*(q1*q2-q0*q3);
					//t22=q0*q0-q1*q1+q2*q2-q3*q3;
					t23=2.0f*(q2*q3+q0*q1);
					//t31=2.0f*(q1*q3+q0*q2);
					//t32=2.0f*(q2*q3-q0*q1);
					t33=q0*q0-q1*q1-q2*q2+q3*q3;
					//求出欧拉角
					roll = atan2(t23,t33);			
					pitch = -asin(t13);
					yaw = atan2(t12,t11);
					
						//根据GPS输出的航向修正yaw角
					if (GPS_dir_flag)
					{
						GPS_dir_flag=0;
						gps_dir_rad=GPSMsg.trk_gnd*PI/180;
						
						if (GPSMsg.trk_gnd < 180)
						{
							if (abs(gps_dir_rad-yaw)>PI)
								yaw +=(2*PI);
							
							yaw=gps_dir_rad*0.8 + yaw*0.2;
							
							if (yaw > PI)
								yaw -= (2*PI);
								
						}
						else
						{
							gps_dir_rad -= (2*PI);
							
							if (abs(gps_dir_rad-yaw)>PI)
								gps_dir_rad +=(2*PI);
							
							yaw=gps_dir_rad*0.8 + yaw*0.2;
							
							if (yaw > PI)
								yaw -= (2*PI);
						}

						//更新四元数
						r11=arm_cos_f32(roll/2);
						r12=arm_cos_f32(pitch/2);
						r13=arm_cos_f32(yaw/2);
						r21=arm_sin_f32(roll/2);
						r22=arm_sin_f32(pitch/2);
						r23=arm_sin_f32(yaw/2);						
						q0=r11*r12*r13+ r21*r22*r23;
						q1=r21*r12*r13- r11*r22*r23;
						q2=r11*r22*r13+ r21*r12*r23;
						q3=r11*r12*r23- r21*r22*r13;
						
						// 四元数归一化
						recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
						q0 *= recipNorm;
						q1 *= recipNorm;
						q2 *= recipNorm;
						q3 *= recipNorm;
				 } 
			 }
			 EXTI_ClearITPendingBit(EXTI_Line0);	
		}	
	
 }

void IMU_init()
{
		spi2_init();	
		RST_init();
		RST_IMU();
		Delay(20);
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
		SPI_I2S_SendData(SPI2,0xB904);
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
		SPI_I2S_ReceiveData(SPI2);
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
		SPI_I2S_SendData(SPI2,0xB802);
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
		SPI_I2S_ReceiveData(SPI2);
		
		Delay(40);
		EXTI_init();
}

//初始化设置 精准自动校准：BE10 恢复出厂设置：BE02
void IMU_BE10()
{
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPI2,0xBE10);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
	SPI_I2S_ReceiveData(SPI2);
}
void spi2_init()
{	
	SPI_InitTypeDef    SPI_InitStruct;
	GPIO_InitTypeDef   GPIO_InitStructure;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType    = GPIO_OType_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13 ,GPIO_AF_SPI2 );
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2 );
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2 );

	SPI_StructInit(&SPI_InitStruct);
	SPI_I2S_DeInit(SPI2);
	SPI_InitStruct.SPI_Direction=	SPI_Direction_2Lines_FullDuplex;//双向
	SPI_InitStruct.SPI_Mode		=	SPI_Mode_Master;//MSTA=1选择主设备，SSI=1
	SPI_InitStruct.SPI_DataSize	=	SPI_DataSize_16b;
	/*极性配置*/
	SPI_InitStruct.SPI_CPOL		=	SPI_CPOL_High;//SPI Clock Polarity
	/*相位配置*/
	SPI_InitStruct.SPI_CPHA		=	SPI_CPHA_2Edge;//SPI Clock Active Edge
	SPI_InitStruct.SPI_NSS		=	SPI_NSS_Soft; //SSM=1
	/*波特率为CPU频率的64分频*/
	SPI_InitStruct.SPI_BaudRatePrescaler	=	SPI_BaudRatePrescaler_64;//波特率128分频
	SPI_InitStruct.SPI_FirstBit	=	SPI_FirstBit_MSB ;//每帧先发送MSB
//	SPI_InitStruct.SPI_CRCPolynomial	= 7	 ;//CRC 校验

 	SPI_Init(SPI2,&SPI_InitStruct); /*初始化SPI1*/
  //  SPI_ITConfig(SPI2,SPI_I2S_IT_RXNE,ENABLE);
	SPI_Cmd(SPI2, ENABLE);	/*使能SPI1*/
}
void RST_init()
{
	//PB1
	GPIO_InitTypeDef   GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_1);
	
}
void RST_IMU()
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	Delay(40);
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
}
void EXTI_init()
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//I/O配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	
	//IMU
	/* Enable and set EXTI4 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; /* 抢占优先级 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* 响应优先级 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
}

/*
 输入值： uint16_t
 输出值： int16_t
 功能：14bits无符号 转换为 16bits 有符号格式
 */
 int16_t map_14bits_to_16bits(uint16_t data_14bits)
 {
		unsigned short tp;
   	if(data_14bits & 0x2000)
   	tp=(unsigned)data_14bits | 0xC000;
   	else
   	tp=(unsigned)data_14bits & 0x3FFF;
   	return tp;
 }
 
/*输入值：
 输出值：
 功能：突发式读取传感器值*/
 /*依次输出：SUPPLY_OUT  XGYRO_OUT YGYRO_OUT ZGYRO_OUT XACCL_OUT 
			YACCL_OUT ZACCL_OUT  XTEMP_OUT YTEMP_OUT ZTEMP_OUT  AUX_ADC*/
  void ADI_BurstReadMode()
 {
		uint8_t i;
		
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_FLAG_TXE)==RESET);
			SPI_I2S_SendData(SPI2,0x3E00);
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_FLAG_RXNE)==RESET);
			SPI_I2S_ReceiveData(SPI2);
		for(i=0;i<12;i++)
		{
			while(SPI_I2S_GetFlagStatus(SPI2,SPI_FLAG_TXE)==RESET);
				SPI_I2S_SendData(SPI2,0x0000);
			while(SPI_I2S_GetFlagStatus(SPI2,SPI_FLAG_RXNE)==RESET);
				data[i]=SPI_I2S_ReceiveData(SPI2);
		}
 
		gx=map_14bits_to_16bits(data[1]);
		gy=map_14bits_to_16bits(data[2]);
		gz=map_14bits_to_16bits(data[3]);
		ax=map_14bits_to_16bits(data[4]);
		ay=map_14bits_to_16bits(data[5]);
		az=map_14bits_to_16bits(data[6]); 
		mx=map_14bits_to_16bits(data[7]);
		my=map_14bits_to_16bits(data[8]);
		mz=map_14bits_to_16bits(data[9]);	
}


