
#include "device_params.h"


uint8_t ip_[4] = {139,129,40,74};
uint16_t port_ = 3333;

uint8_t deviceaddr[5] = {0x99,0x09,0x00,0x00,0x57};      //��������ַ

uint8_t simcard = 0xff;    //0xff~�ƶ�  0xaa~��ͨ
uint8_t ack_valve = 0xff;  //��Ӧ������~0xaa    �Ȳ�����Ӧ��~0xff
uint8_t slave = 0xbb; //0xaa~mbus   0xff~485   0xbb~���߲ɼ���  0xcc~���߲ɼ���
uint8_t di_seq = 0xff; //DI0 DI1 ˳��   0xAA~DI1��ǰ(ǧ��ͨ)   0xFF~DI0��ǰ(default)  
uint8_t protocol = 0xff;  //Э������ 0xFF~188(Default)  0x11~EG 

volatile uint8_t connectstate = 0;       //0 didn't connect to the server   1 connect to the server
volatile uint8_t reading = 0;   //0 didn't reading meters    1  reading meters
volatile uint8_t lora_test = 0;  //ÿ3s����TEST��LORA

uint8_t version = 100;    //�汾��100��ʼ��

void set_ip(uint8_t * p_ip){
  ip_[0] = *(p_ip+3);
  ip_[1] = *(p_ip+2);
  ip_[2] = *(p_ip+2);
  ip_[3] = *(p_ip);
}

uint8_t *get_ip(){
  return ip_;
}

void set_port(uint16_t port){
  port_ = port;
}

uint16_t get_port(){
  return port_;
}

void set_device_addr(uint8_t * p_addr){
  deviceaddr[0] = *(p_addr);
  deviceaddr[1] = *(p_addr+1);
  deviceaddr[2] = *(p_addr+2);
  deviceaddr[3] = *(p_addr+3);
  deviceaddr[4] = *(p_addr+4);
}

uint8_t *get_device_addr(){
  return deviceaddr;
}

void set_slave(uint8_t slave_){
  slave = slave_;
}

uint8_t get_slave(){
  return slave;
}

void set_di_seq(uint8_t di_seq_){
  di_seq = di_seq_;
}

uint8_t get_di_seq(){
    return di_seq;
}

void set_ack_valve(uint8_t ack_valve_){
  ack_valve = ack_valve;
}

uint8_t get_ack_valve(){
  return ack_valve;
}

void set_protocol(uint8_t protocol_){
  protocol = protocol_;
}

uint8_t get_protocol(){
  return protocol;
}

void set_connect_state(uint8_t connect_){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  connectstate = connect_;
  CPU_CRITICAL_EXIT();
}

uint8_t get_connect_state(){
  return connectstate;
}

void set_readding(uint8_t reading_){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  reading = reading_;
  CPU_CRITICAL_EXIT();
  
}

uint8_t get_readding(){
  return reading;
}

void set_lora_test(uint8_t lora_test_){
  lora_test = lora_test_;
}

uint8_t get_lora_test(){
  return lora_test;
}

void set_simcard(uint8_t simcard_){
  simcard = simcard_;
}

uint8_t get_simcard(){
  return simcard;
}

uint8_t get_version(){
  return version;
}