


#ifndef __GPRS_H__
#define __GPRS_H__

#include "bsp.h"



/******************************************************
�������ĵ�ַ   ���ù�֮��ʹ�����ù���  δ���ù�ʹ��Ĭ�ϵ�5700000999
*/

uint8_t send_server(uint8_t * data,uint16_t count);

uint8_t connect(void);

uint8_t device_cmd(uint8_t cmd);


#endif

