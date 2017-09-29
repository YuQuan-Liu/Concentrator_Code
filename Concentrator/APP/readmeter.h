

#ifndef READMETER_H
#define READMETER_H

#include "stm32f10x_conf.h"

void meter_control(uint8_t * p_frame,uint16_t frame_len);
void meter_control_meter(uint8_t * p_frame,uint16_t frame_len);
void meter_control_cjq(uint8_t * p_frame,uint16_t frame_len);



void meter_read(uint8_t * p_frame,uint16_t frame_len);

void meter_read_c_all(uint8_t * p_frame,uint16_t frame_len);
void meter_read_c_channel(uint8_t * p_frame,uint16_t frame_len);
void meter_read_c_meter(uint8_t * p_frame,uint16_t frame_len);

uint8_t write_frame_cjq(uint8_t * p_cjqaddr,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn,uint8_t server_seq_);

uint8_t meter_read_save(uint32_t block_meter,uint8_t * meter_read,uint8_t * meter_status);


#endif