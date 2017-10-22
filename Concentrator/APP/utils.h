#ifndef UTILS_H
#define UTILS_H

#include "stm32f10x_conf.h"


uint8_t check_cs(uint8_t * start,uint16_t len);
uint8_t check_eor(uint8_t * start,uint16_t len);
uint8_t check_frame(uint8_t * start);

uint8_t check_lora_data2frame(uint8_t * p_buf_start,uint8_t * p_buf_end);
uint8_t check_xintian_frame(uint8_t * p_buf_start,uint8_t * p_buf_end);
uint8_t check_lora_ok_frame(uint8_t * p_buf_start,uint8_t * p_buf_end);

uint8_t check_meter_frame(uint8_t * p_buf_start,uint8_t * p_buf_end);
uint8_t check_188_frame(uint8_t * p_buf_start,uint8_t * p_buf_end);


//将start开始的end结束的字符串 开头的0x00替换成‘\r’
void replace_str00(uint8_t * start,uint8_t * end);


uint8_t add_server_seq(void);
uint8_t add_cjq_seq(void);

uint8_t delayms(uint32_t timeout);

uint8_t cjq_data_tome(uint8_t * p_frame,uint8_t frame_len);

uint8_t lock_gprs(void);
uint8_t unlock_gprs(void);
uint8_t lock_lora(void);
uint8_t unlock_lora(void);
uint8_t lock_cjq(void);
uint8_t unlock_cjq(void);
uint8_t lock_meter(void);
uint8_t unlock_meter(void);
uint8_t lock_mem4k(void);
uint8_t unlock_mem4k(void);
uint8_t * get_mem4k(void);

uint8_t * get_membuf(void);
uint8_t put_membuf(uint8_t * p_membuf);

uint8_t * get_memisr(void);
uint8_t put_memisr(uint8_t * p_membuf);


uint8_t wait_lora_ok(uint32_t timeout);
uint8_t signal_lora_ok(void);

uint8_t wait_heartbeat(uint32_t timeout);
uint8_t signal_heartbeat(void);

uint8_t wait_serverack(uint32_t timeout);
uint8_t signal_serverack(void);

uint8_t wait_sendgprs(uint32_t timeout);
uint8_t signal_sendgprs(void);

uint8_t wait_cjqack(uint32_t timeout);
uint8_t signal_cjqack(void);



uint8_t wait_q_cjq_usart(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout);
uint8_t post_q_cjq_usart(uint8_t * p_mem, uint16_t msg_size);

uint8_t wait_q_meter_usart(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout);
uint8_t post_q_meter_usart(uint8_t * p_mem, uint16_t msg_size);

uint8_t wait_q_lora_usart(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout);
uint8_t post_q_lora_usart(uint8_t * p_mem, uint16_t msg_size);

uint8_t wait_q_cjq(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout);
uint8_t post_q_cjq(uint8_t * p_mem, uint16_t msg_size);

uint8_t wait_q_meter(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout);
uint8_t post_q_meter(uint8_t * p_mem, uint16_t msg_size);

uint8_t wait_q_read(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout);
uint8_t post_q_read(uint8_t * p_mem, uint16_t msg_size);

uint8_t wait_q_conf(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout);
uint8_t post_q_conf(uint8_t * p_mem, uint16_t msg_size);

uint32_t get_timestamp(void);


#endif