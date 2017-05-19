
#include "utils.h"


uint8_t check_cs(uint8_t * start,uint16_t len){
  uint16_t i;
  uint8_t cs = 0;
  for(i = 0;i < len;i++){
    cs += *(start+i);
  }
  return cs;
}


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