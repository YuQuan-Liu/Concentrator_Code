

#ifndef READMETER_H
#define READMETER_H

#include "stm32f10x_conf.h"

void meter_control(uint8_t * p_frame,uint16_t frame_len);
void meter_control_meter(uint8_t * p_frame,uint16_t frame_len);
void meter_control_cjq(uint8_t * p_frame,uint16_t frame_len);



void meter_read(uint8_t * p_frame,uint16_t frame_len);
void meter_read_m_all(uint8_t * p_frame,uint16_t frame_len);
void meter_read_m_channel(uint8_t * p_frame,uint16_t frame_len);
void meter_read_m_meter(uint8_t * p_frame,uint16_t frame_len);

void meter_read_c_all(uint8_t * p_frame,uint16_t frame_len);
void meter_read_c_channel(uint8_t * p_frame,uint16_t frame_len);
void meter_read_c_meter(uint8_t * p_frame,uint16_t frame_len);

uint8_t meter_read_single(uint32_t block_meter, uint8_t *p_meteraddr,uint8_t meter_type,uint8_t * meter_read,uint8_t * meter_status);
uint8_t meter_read_save(uint32_t block_meter,uint8_t * meter_read,uint8_t * meter_status);
uint8_t meter_read_frame_send(uint8_t * p_meteraddr,uint8_t meter_type);

void send_meter_data_single(uint8_t * p_meteraddr,uint8_t * meter_read,uint8_t * meter_status,uint8_t meter_type, uint8_t desc);
void send_meter_data_channel(uint32_t block_cjq_,uint16_t frame_times,uint16_t frame_times_start,uint8_t meter_type, uint8_t desc);
void send_meter_data_all(uint8_t desc);

uint8_t cjq_relay_control(uint8_t cmd,uint8_t cjq);

#endif