#ifndef CONFIGS_H
#define CONFIGS_H

#include "stm32f10x_conf.h"

void param_config(uint8_t * buf_frame,uint8_t desc);
uint32_t search_cjq(uint8_t * cjqaddr);         //����Flash���Ƿ��Ѱ����˲ɼ���
uint32_t add_cjq(uint8_t * cjqaddr);    //��Ӳɼ���
uint32_t delete_cjq(uint32_t block_cjq);         //ɾ���ɼ���

uint32_t search_meter(uint32_t block_cjq,uint8_t * meteraddr);  //�ڲɼ����²����Ƿ��Ѿ������˱�
uint32_t add_meter(uint32_t block_cjq,uint8_t * meteraddr);
uint32_t delete_meter(uint32_t block_cjq,uint32_t block_meter);



void param_query(uint8_t * buf_frame,uint8_t desc);

void ack_query_mbus(uint8_t desc,uint8_t server_seq_);
void ack_query_cjq(uint8_t desc,uint8_t server_seq_);
void ack_query_meter(uint8_t metertype,uint8_t * meteraddr,uint8_t desc,uint8_t server_seq_);
void ack_query_addr(uint8_t desc,uint8_t server_seq_);
void ack_query_ip(uint8_t desc,uint8_t server_seq_);
void ack_query_di_seq(uint8_t desc,uint8_t server_seq_);
void ack_query_ack_action(uint8_t desc,uint8_t server_seq_);
void ack_query_protocol(uint8_t desc,uint8_t server_seq_);


void device_ack(uint8_t desc,uint8_t server_seq_);  //����ȷ��֡  1 ���͸�M590E������  0 ���͸�485

#endif