#ifndef	_REPORTIMU
#define	_REPORTIMU
#include "stm32f4xx.h"
void UART_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz, int16_t hx,int16_t hy,int16_t hz);
#endif

//=====================================================================================================
// End of file
//=====================================================================================================
