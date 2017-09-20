

#ifndef READMETER_H
#define READMETER_H

#include "stm32f10x_conf.h"

void meter_control(uint8_t * p_buf,uint16_t msg_size);
void meter_control_meter(uint8_t * p_buf,uint16_t msg_size);
void meter_control_cjq(uint8_t * p_buf,uint16_t msg_size);



void meter_read(uint8_t * p_buf,uint16_t msg_size);
void meter_read_meters(uint8_t * p_buf,uint16_t msg_size);
void meter_read_188(uint8_t * p_buf,uint16_t msg_size);
void meter_read_eg(uint8_t * p_buf,uint16_t msg_size);

void meter_read_cjqs(uint8_t * p_buf,uint16_t msg_size);
void meter_read_188_cjq(uint8_t * p_buf,uint16_t msg_size);
void meter_read_eg_cjq(uint8_t * p_buf,uint16_t msg_size);



#endif