


#ifndef __GPRS_H__
#define __GPRS_H__

#include "bsp.h"



/******************************************************
集中器的地址   配置过之后使用配置过的  未配置过使用默认的5700000999
*/

uint8_t send_server(uint8_t * data,uint16_t count);

uint8_t connect(void);

uint8_t device_cmd(uint8_t cmd);


#endif

