#ifndef UTILS_H
#define UTILS_H

#include "stm32f10x_conf.h"

void set_ip(uint8_t * p_ip);
void set_port(uint16_t port);
void set_device_addr(uint8_t * p_addr);
void set_slave(uint8_t slave_);
void set_di_seq(uint8_t di_seq_);
void set_ack_valve(uint8_t ack_valve_);
void set_protocol(uint8_t set_protocol_);
void set_connect_state(uint8_t connect_);
void set_readding(uint8_t reading_);
void set_lora_test(uint8_t lora_test_);
void set_simcard(uint8_t simcard_);
void set_meter_baud(uint8_t meter_baud_);

uint8_t *get_ip();
uint16_t get_port();
uint8_t *get_device_addr();
uint8_t get_slave();
uint8_t get_di_seq();
uint8_t get_ack_valve();
uint8_t get_protocol();
uint8_t get_connect_state();
uint8_t get_readding();
uint8_t get_lora_test();
uint8_t get_simcard();
uint8_t get_version();
uint8_t get_meter_baud();

#endif