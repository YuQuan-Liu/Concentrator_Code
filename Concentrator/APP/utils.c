
#include "utils.h"
#include "os.h"

uint8_t local_seq = 0;  //本地序列号
extern uint8_t cjqaddr[5];
extern uint8_t cjqaddr_eg[2];
/**
 * 检查CS  + 
 */
uint8_t check_cs(uint8_t * start,uint16_t len){
  uint16_t i;
  uint8_t cs = 0;
  for(i = 0;i < len;i++){
    cs += *(start+i);
  }
  return cs;
}

/**
 * 检查CS ^
 */
uint8_t check_eor(uint8_t * start,uint16_t len){
  uint16_t i;
  uint8_t cs = 0;
  for(i = 0;i < len;i++){
    cs ^= *(start+i);
  }
  return cs;
}

/*
if the frame is ok  return the length of the frame
if the frame is error return 0;
*/
uint8_t check_frame(uint8_t * start){
  uint16_t len1;
  uint16_t len2;
  
  uint8_t * s;
  s = start;
  len1 = *(uint16_t *)(s + 1);
  len2 = *(uint16_t *)(s + 3);
  
  if(len1 == len2){
    len1 = len1 >> 2;
    if(*(s+len1 + 6) == check_cs(s+6,len1) && *(s+len1+7) == 0x16){
      return len1+8;
    }
  }
  return 0;
}


/**
  * 增加 本地序列号  返回增加前的序列号
  * 发送心跳 发送数据时调用
  */
uint8_t addSEQ(void){
  uint8_t seq_ = 0;
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  seq_ = local_seq;
  local_seq++;
  local_seq = local_seq & 0x0F;
  CPU_CRITICAL_EXIT();
  return seq_;
}


void cjqaddr2eg(void){
  //6 55 35
  uint32_t cjqaddr_eg_ = (cjqaddr[2]&0x0F) * 10000 + (cjqaddr[1]>>4) * 1000 + (cjqaddr[1]&0x0F) * 100 + (cjqaddr[0]>>4) * 10 + (cjqaddr[0]&0x0F) ;
  
  cjqaddr_eg[0] = *((uint8_t *)(&cjqaddr_eg_));
  cjqaddr_eg[1] = *((uint8_t *)(&cjqaddr_eg_) + 1);
}