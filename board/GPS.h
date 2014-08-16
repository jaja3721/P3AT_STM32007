
#ifndef __GPS_H
#define __GPS_H
#include "stm32f4xx.h"
#define Length_GPS 256

typedef struct
{
	uint8_t  Len_Header;
	uint16_t ID;//42:BESTPOS	99:BESTVEL
	uint16_t Len_Msg;//The length in bytes of the body of the message, not including the header nor the CRC
	double lat;//ά��
	double lon;//����
	float deviation;//���
	double hor_spd;//��С ��λ��m/s
	double trk_gnd;//���� ��λ����
}GPSMsgTypeDef;

extern uint8_t volatile GPS_pos_flag;
extern uint8_t volatile GPS_vel_flag;
extern uint8_t volatile GPS_dir_flag;
extern GPSMsgTypeDef GPSMsg;
extern float gps_dir[21];

void GPS_init(void);
//uint8_t get_PV(void);

#endif
