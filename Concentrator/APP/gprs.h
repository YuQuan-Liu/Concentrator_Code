


#ifndef __GPRS_H__
#define __GPRS_H__

#include "bsp.h"

#define USE_DNS          //ʹ��DNS��ȡIP��ַ   



/******************************************************
�������ĵ�ַ   ���ù�֮��ʹ�����ù���  δ���ù�ʹ��Ĭ�ϵ�5700000999
*/

ErrorStatus send_server(uint8_t * data,uint16_t count);

ErrorStatus connect(void);


ErrorStatus Device_Cmd(FunctionalState NewState);

void check_str(uint8_t * start,uint8_t * end);  //��start��ʼ��end�������ַ��� ��ͷ��0x00�滻�ɡ�\r��

void change_connect(uint8_t state);  //�ı����ߵ�״̬

#endif

