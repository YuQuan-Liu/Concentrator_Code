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
uint8_t check_eg_meter_frame(uint8_t * p_buf_start,uint8_t * p_buf_end);


//将start开始的end结束的字符串 开头的0x00替换成‘\r’
void replace_str00(uint8_t * start,uint8_t * end);


uint8_t addSEQ(void);

uint8_t delayms(uint32_t timeout);


uint8_t lock_gprs();
uint8_t unlock_gprs();
uint8_t lock_lora();
uint8_t unlock_lora();
uint8_t lock_cjq();
uint8_t unlock_cjq();
uint8_t lock_meter();
uint8_t unlock_meter();
uint8_t lock_mem4k();
uint8_t unlock_mem4k();
uint8_t *get_mem4k();

uint8_t *get_membuf();
uint8_t put_membuf(uint8_t * p_membuf);

uint8_t *get_memisr();
uint8_t put_memisr(uint8_t * p_membuf);


uint8_t wait_lora_ok(uint32_t timeout);
uint8_t signal_lora_ok();

uint8_t wait_heartbeat(uint32_t timeout);
uint8_t signal_heartbeat();

uint8_t wait_serverack(uint32_t timeout);
uint8_t signal_serverack();

uint8_t wait_sendgprs(uint32_t timeout);
uint8_t signal_sendgprs();

uint8_t wait_cjqack(uint32_t timeout);
uint8_t signal_cjqack();


#endif