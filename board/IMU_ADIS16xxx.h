
#ifndef IMU_ADIS16xxx_H
#define IMU_ADIS16xxx_H

#include "stm32f4xx.h"
#include "MadgwickAHRS.h"
//#include "MahonyAHRS.h"

extern volatile float roll,pitch,yaw;
extern volatile int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
void IMU_init(void);
void IMU_BE10(void);

#endif

