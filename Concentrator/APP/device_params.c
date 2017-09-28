
#include "device_params.h"
#include "os.h"

uint8_t ip_[4] = {139,129,40,74};
uint16_t port_ = 3333;

uint8_t deviceaddr[5] = {0x99,0x09,0x00,0x00,0x57};      //集中器地址

uint8_t simcard = 0xff;    //0xff~移动  0xaa~联通
uint8_t ack_valve = 0xff;  //先应答后操作~0xaa    先操作后应答~0xff
uint8_t slave = 0xbb; //0xaa~mbus   0xff~485   0xbb~有线采集器  0xcc~无线采集器
uint8_t di_seq = 0xff; //DI0 DI1 顺序   0xAA~DI1在前(千宝通)   0xFF~DI0在前(default)  
uint8_t protocol = 0xff;  //协议类型 0xFF~188(Default)  0xEE~bad-188 

volatile uint8_t connectstate = 0;       //0 didn't connect to the server   1 connect to the server
volatile uint8_t reading = 0;   //0 didn't reading meters    1  reading meters
volatile uint8_t lora_test = 0;  //每3s发送TEST到LORA

uint8_t version = 100;    //版本从100开始算
uint8_t meter_baud = 0x24;  //96H——9600 bps; 48H——4800 bps; 24H——2400 bps; 12H——1200 bps

//正在抄表的采集器地址  在抄表时判断是不是找我  海大协议只有低2位有效
uint8_t cjqaddr[5] = {0x01,0x00,0x00,0x00,0x00};   

uint8_t device_mode = 0xFF;   //0xFF~无线   0xAA~有线

uint8_t data_seq = 0;  //记录数据的序列号 等待ack

void set_ip(uint8_t * p_ip){
  ip_[0] = *(p_ip+3);
  ip_[1] = *(p_ip+2);
  ip_[2] = *(p_ip+2);
  ip_[3] = *(p_ip);
}

uint8_t * get_ip(void){
  return ip_;
}

void set_port(uint16_t port){
  port_ = port;
}

uint16_t get_port(void){
  return port_;
}

void set_device_addr(uint8_t * p_addr){
  deviceaddr[0] = *(p_addr);
  deviceaddr[1] = *(p_addr+1);
  deviceaddr[2] = *(p_addr+2);
  deviceaddr[3] = *(p_addr+3);
  deviceaddr[4] = *(p_addr+4);
}

uint8_t * get_device_addr(void){
  return deviceaddr;
}

void set_slave(uint8_t slave_){
  slave = slave_;
}

uint8_t get_slave(void){
  return slave;
}

void set_di_seq(uint8_t di_seq_){
  di_seq = di_seq_;
}

uint8_t get_di_seq(void){
    return di_seq;
}

void set_ack_valve(uint8_t ack_valve_){
  ack_valve = ack_valve;
}

uint8_t get_ack_valve(void){
  return ack_valve;
}

void set_protocol(uint8_t protocol_){
  protocol = protocol_;
}

uint8_t get_protocol(void){
  return protocol;
}

void set_connect_state(uint8_t connect_){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  connectstate = connect_;
  CPU_CRITICAL_EXIT();
}

uint8_t get_connect_state(void){
  return connectstate;
}

void set_readding(uint8_t reading_){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  reading = reading_;
  CPU_CRITICAL_EXIT();
  
}

uint8_t get_readding(void){
  return reading;
}

void set_lora_test(uint8_t lora_test_){
  lora_test = lora_test_;
}

uint8_t get_lora_test(void){
  return lora_test;
}

void set_simcard(uint8_t simcard_){
  simcard = simcard_;
}

uint8_t get_simcard(void){
  return simcard;
}

uint8_t get_version(void){
  return version;
}

void set_meter_baud(uint8_t meter_baud_){
  meter_baud = meter_baud_;
}

uint8_t get_meter_baud(void){
   return meter_baud;
}

void set_cjq_addr(uint8_t * p_addr){
  switch(get_protocol()){
  case 0xEE:
  case 0xFF:
    cjqaddr[0] = *(p_addr);
    cjqaddr[1] = *(p_addr+1);
    cjqaddr[2] = *(p_addr+2);
    cjqaddr[3] = *(p_addr+3);
    cjqaddr[4] = *(p_addr+4);
    break;
  }
}

uint8_t * get_cjq_addr(void){
  return cjqaddr;
}

void set_device_mode(uint8_t device_mode_){
  device_mode = device_mode_;
}

uint8_t get_device_mode(void){
  return device_mode;
}

void set_data_seq(uint8_t data_seq_){
  data_seq = data_seq_;
}

uint8_t get_data_seq(void){
  return data_seq;
}
