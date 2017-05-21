


#ifndef __GPRS_H__
#define __GPRS_H__

#include "bsp.h"

#define USE_DNS          //使用DNS获取IP地址   



/******************************************************
集中器的地址   配置过之后使用配置过的  未配置过使用默认的5700000999
*/

ErrorStatus send_server(uint8_t * data,uint16_t count);

ErrorStatus connect(void);


ErrorStatus Device_Cmd(FunctionalState NewState);

void check_str(uint8_t * start,uint8_t * end);  //将start开始的end结束的字符串 开头的0x00替换成‘\r’

void change_connect(uint8_t state);  //改变在线的状态

#endif

