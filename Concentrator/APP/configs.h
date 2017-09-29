#ifndef CONFIGS_H
#define CONFIGS_H

#include "stm32f10x_conf.h"

void param_config(uint8_t * p_buf,uint16_t msg_size);
uint32_t search_cjq(uint8_t * p_cjqaddr);         //查找Flash中是否已包含此采集器
uint32_t add_cjq(uint8_t * p_cjqaddr);    //添加采集器
uint8_t delete_cjqs(void);   //删除所有的采集器
uint32_t delete_cjq(uint8_t * p_cjqaddr);         //删除某一个采集器  暂时先不加

uint32_t search_meter(uint32_t block_cjq,uint8_t * p_meteraddr);  //在采集器下查找是否已经包含此表
uint32_t add_meters(uint8_t * p_buf);
uint32_t add_single_meter(uint32_t block_cjq,uint8_t * p_meteraddr, uint8_t meter_type);
uint32_t delete_meters(uint8_t * p_buf);
uint32_t delete_single_meter(uint32_t block_cjq,uint8_t * p_meteraddr);


void param_query(uint8_t * p_buf,uint16_t msg_size);

uint8_t * ack_mulit_header(uint8_t *p_buf,uint8_t frame_type,uint16_t len,uint8_t afn,uint8_t seq_,uint8_t fn);

void ack_query_cjq(uint8_t desc,uint8_t server_seq_);
void ack_query_ip(uint8_t desc,uint8_t server_seq_);

void ack_query_meter_all(uint8_t desc,uint8_t server_seq_);
void ack_query_meter_channel(uint32_t block_cjq_,uint16_t frame_times,uint16_t frame_times_start,uint8_t desc,uint8_t server_seq_);
void ack_query_meter_single(uint8_t *p_cjqaddr,uint8_t * p_meteraddr,uint8_t desc,uint8_t server_seq_);

//给CJQ发送确认帧  1~LORA  0~485
void device_ack_cjq(uint8_t desc,uint8_t server_seq_,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn);

//发送确认帧  1 发送给M590E服务器   0发送给485
void device_ack(uint8_t desc,uint8_t server_seq_,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn);


#endif