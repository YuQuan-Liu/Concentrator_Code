

#include "configs.h"
#include "serial.h"
#include "frame.h"
#include "gprs.h"
#include "os.h"
#include "spi_flash.h"
#include "utils.h"
#include "device_params.h"


void param_config(uint8_t * p_buf,uint8_t desc){
  
  uint8_t server_seq_ = *(p_buf + SEQ_POSITION) & 0x0F;
  
  uint16_t port = 0;
  uint8_t temp_u8 = 0;
  uint8_t * mem4k = 0;
  
  if(lock_mem4k()){
    mem4k = get_mem4k();
  }else{
    return;
  }
  
  switch(*(p_buf + FN_POSITION)){
  case FN_IP_PORT:
    
    set_ip(p_buf + DATA_POSITION);
    set_port(*((uint16_t *)(p_buf + DATA_POSITION +4)));
    
    port = get_port();
    
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_CON_IP1 - sFLASH_CON_START_ADDR),get_ip(),4);
    Mem_Copy(mem4k + (sFLASH_CON_PORT_ - sFLASH_CON_START_ADDR),&port,2);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    device_ack(desc,server_seq_);
    
    break;
  case FN_ADDR:
    
    set_device_addr(p_buf + DATA_POSITION);
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_DEVICE_ADDR - sFLASH_CON_START_ADDR),get_device_addr(),5);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    
    device_ack(desc,server_seq_);
    break;
  case FN_METER:
    if(*(p_buf + DATA_POSITION) == 0x00){//删除表
      if(delete_meters(p_buf)){
        device_ack(desc,server_seq_);
      }
    }
    
    if(*(p_buf + DATA_POSITION) == 0x01){//添加表
      if(add_meters(p_buf)){
        device_ack(desc,server_seq_);
      }
    }
    break;
  case FN_CJQ:
    if(*(p_buf + DATA_POSITION) == 0xAA){  //删除全部采集器  即重新初始化FLASH POOL
      sFLASH_PoolInit();
      device_ack(desc,server_seq_);
    }
    
    if(*(p_buf + DATA_POSITION) == 0x55){  //添加采集器
      if(add_cjq(p_buf + DATA_POSITION + 1)){
        device_ack(desc,server_seq_);
      }
    }
    
    //删除某一个采集器的功能先不加
    break;
  case FN_MBUS:
    switch(*(p_buf + DATA_POSITION)){
    case 0xAA:
    case 0xBB:
    case 0xCC:
    case 0xFF:
      set_slave(*(p_buf + DATA_POSITION));
      temp_u8 = get_slave();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_METER_MBUS - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_DI_SEQ:
    switch(*(p_buf + DATA_POSITION)){
    case 0xAA:
    case 0xFF:
      set_di_seq(*(p_buf + DATA_POSITION));
      temp_u8 = get_di_seq();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_READMETER_DI_SEQ - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_ACK_ACTION:
    switch(*(p_buf + DATA_POSITION)){
    case 0xAA:
    case 0xFF:
      set_ack_valve(*(p_buf + DATA_POSITION));
      temp_u8 = get_ack_valve();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_ACK_ACTION - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_PROTOCOL:
    switch(*(p_buf + DATA_POSITION)){
    case 0x11:
    case 0xFF:
      set_protocol(*(p_buf + DATA_POSITION));
      temp_u8 = get_protocol();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_PROTOCOL - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_BAUD:
    //96H——9600 bps;      48H——4800 bps;    24H——2400 bps;     12H——1200 bps;
    switch(*(p_buf + DATA_POSITION)){
    case 0x12:
    case 0x24:
    case 0x48:
    case 0x96:
      set_meter_baud(*(p_buf + DATA_POSITION));
      temp_u8 = get_meter_baud();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_METER_BAUD - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_ERASE:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF:
      sFLASH_PoolInit();
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_RESET:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF:
      device_ack(desc,server_seq_);
      device_cmd(DISABLE);
      *((uint8_t *)0) = 0x00;  //迫使系统重启
      break;
    }
    break;
  case FN_LORA_SEND:
    switch(*(p_buf + DATA_POSITION)){
    case 0x01:
      set_lora_test(1);
      break;
    default:
      set_lora_test(0);
      break;
    }
    device_ack(desc,server_seq_);
    break;
  case FN_SYN:
    //TODO... CJQ JZQ 同步
    break:
  }
  
  unlock_mem4k();
}

void param_query(uint8_t * p_buf,uint8_t desc){
  uint8_t server_seq_ = *(p_buf + SEQ_POSITION) & 0x0F;
  switch(*(p_buf + FN_POSITION)){
  case FN_IP_PORT:
    ack_query_ip(desc,server_seq_);
    break;
  case FN_ADDR: 
    ack_query_addr(desc,server_seq_);
    break;
  case FN_METER:
    ack_query_meter(p_buf + DATA_POSITION,p_buf + DATA_POSITION + 5,desc,server_seq_);
    break;
  case FN_CJQ:
    ack_query_cjq(desc,server_seq_);
    break;
  case FN_MBUS:
    ack_query_mbus(desc,server_seq_);
    break;
  case FN_DI_SEQ:
    ack_query_di_seq(desc,server_seq_);
    break;
  case FN_ACK_ACTION:
    ack_query_ack_action(desc,server_seq_);
    break;
  case FN_PROTOCOL:
    ack_query_protocol(desc,server_seq_);
    break;
  case FN_VERSION:
    ack_query_version(desc,server_seq_);
    break;
  case FN_BAUD:
    ack_query_baud(desc,server_seq_);
    break;
  }
}


void device_ack(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x27;//(9 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x27;//(9 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_ACK;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_ACK;
    
    *p_buf++ = check_cs(p_buf_+6,9);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      if(lock_gprs()){
        send_server(p_buf_,17);
        unlock_gprs();
      }
      break;
    default:
      if(lock_cjq()){
        write_cjq(p_buf_,17);
        lock_cjq();
      }
      break;
    }
    put_membuf(p_buf_);
  }
}

void device_ack_lora(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x3B;//(14 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x3B;//(14 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_ACK;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_ACK;
    
    //TODO...
//    if(protocol == 0x01){
//      *buf_frame++ = cjqaddr_eg[0];
//      *buf_frame++ = cjqaddr_eg[1];
//      *buf_frame++ = 0x00;
//      *buf_frame++ = 0x00;
//      *buf_frame++ = 0x00;
//    }else{
//      *buf_frame++ = cjqaddr[0];
//      *buf_frame++ = cjqaddr[1];
//      *buf_frame++ = cjqaddr[2];
//      *buf_frame++ = cjqaddr[3];
//      *buf_frame++ = cjqaddr[4];
//    }
    
    *p_buf++ = check_cs(p_buf_+6,14);
    *p_buf++ = FRAME_END;
    
    if(lock_lora()){
      write_lora(p_buf_,22);
      unlock_lora();
    }
    put_membuf(p_buf_);
  }
}


void ack_query_addr(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x27;//(9 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x27;//(9 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_ADDR;
    
    *p_buf++ = check_cs(p_buf_+6,9);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,17);
      break;
    default:
      write_cjq(p_buf_,17);
      break;
    }
    put_membuf(p_buf_);
  }
}

void ack_query_ip(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x3F;//(15 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x3F;//(15 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_IP_PORT;
    
    p_temp = get_ip();
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[0];
    
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = port_;
    p_buf = (uint8_t *)p_buf_16;
    
    *p_buf++ = check_cs(p_buf+6,15);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,23);
      break;
    default:
      write_cjq(p_buf_,23);
      break;
    }
    put_membuf(p_buf_);
  }
}

void ack_query_mbus(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_MBUS;
    
    *p_buf++ = get_slave();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}

void ack_query_di_seq(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_DI_SEQ;
    
    *p_buf++ = get_di_seq();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}


void ack_query_ack_action(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_ACK_ACTION;
    
    *p_buf++ = get_ack_valve();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}


void ack_query_protocol(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_PROTOCOL;
    
    *p_buf++ = get_protocol();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}

void ack_query_version(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_VERSION;
    
    *p_buf++ = get_version();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}

void ack_query_baud(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_BAUD;
    
    *p_buf++ = get_meter_baud();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}


void ack_query_cjq(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;
  
  uint16_t cjq_count;
  uint32_t block_cjq;
  uint8_t cjq_addr[5];
  uint16_t i;
  uint8_t j;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
    sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
    
    *p_buf++ = FRAME_HEAD;
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = ((9+cjq_count*5) << 2) | 0x03;
    *p_buf_16++ = ((9+cjq_count*5) << 2) | 0x03;
    p_buf = (uint8_t *)p_buf_16;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_CJQ;
    
    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer(cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);
      for(j = 0;j < 5;j++){
        *p_buf++ = cjq_addr[j];
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
    }
    
    *p_buf++ = check_cs(p_buf_+6,9+cjq_count*5);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,17+cjq_count*5);
      break;
    default:
      write_cjq(p_buf_,17+cjq_count*5);
      break;
    }
    put_membuf(p_buf_);
  }
}


uint8_t * ack_mulit_header(uint8_t *p_buf,uint8_t frame_type,uint16_t len,uint8_t afn,uint8_t seq_,uint8_t fn){
  uint8_t * p_temp;
  uint16_t * p_buf_16;
  
  *p_buf++ = FRAME_HEAD;
  p_buf_16 = (uint16_t *)p_buf;
  *p_buf_16++ = len;
  *p_buf_16++ = len;
  p_buf = (uint8_t *)p_buf_16;
  *p_buf++ = FRAME_HEAD;
  
  *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  p_temp = get_device_addr();
  *p_buf++ = p_temp[0];
  *p_buf++ = p_temp[1];
  *p_buf++ = p_temp[2];
  *p_buf++ = p_temp[3];
  *p_buf++ = p_temp[4];
  
  *p_buf++ = afn;
  switch(frame_type){
  case 0://单帧 0
    *p_buf++ = ZERO_BYTE | SINGLE | CONFIRM | seq_;
    break;
  case 1://首帧 1
    *p_buf++ = ZERO_BYTE | MUL_FIRST | CONFIRM | seq_;
    break;
  case 2://中间帧 2
    *p_buf++ = ZERO_BYTE | MUL_MIDDLE | CONFIRM | seq_;
    break;
  case 3://尾帧 3
    *p_buf++ = ZERO_BYTE | MUL_LAST | CONFIRM | seq_;
    break;
  }
  *p_buf++ = fn;
}

void ack_query_meter(uint8_t *p_cjqaddr,uint8_t * p_meteraddr,uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;
  uint16_t i;
  uint8_t j;
  uint8_t k;
  uint8_t frame_meter_count = 0;
  uint8_t meter_addr[7];
  
  uint32_t block_cjq = 0;
  uint32_t block_meter = 0;
  uint16_t cjqmeter_count = 0;
  uint8_t frame_data_len = 0;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    block_cjq = search_cjq(p_cjqaddr);
    if(block_cjq){
      if(Mem_Cmp(p_meteraddr,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7)){  //all meter under this cjq
        
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
        sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
        if(cjqmeter_count > 0){
          times = cjqmeter_count/10;
          remain = cjqmeter_count%10;
          times_count = 0;
          times_ = times;
          if(remain > 0){
            times_ = times_ + 1;
          }
          
          for(i=0;i< times_;i++){
            p_buf = p_buf_;
            frame_data_len = 0;
            frame_meter_count = 0;
            if(times_==1){//单帧 0
              frame_meter_count = cjqmeter_count;
              frame_data_len = 9+7*frame_meter_count+5+4;
              p_buf = ack_mulit_header(p_buf,0,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
            }else{
              if(i==0){//首帧 1
                frame_meter_count = 10;
                frame_data_len = 9+7*frame_meter_count+5+4;
                p_buf = ack_mulit_header(p_buf,1,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
              }else{
                if(i==times-1){//尾帧 3
                  frame_meter_count = remain;
                  frame_data_len = 9+7*frame_meter_count+5+4;
                  p_buf = ack_mulit_header(p_buf,3,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
                }else{//中间帧 2
                  frame_meter_count = 10;
                  frame_data_len = 9+7*frame_meter_count+5+4;
                  p_buf = ack_mulit_header(p_buf,2,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
                }
              }
            }
            
            //帧的数据域
            p_buf_16 = (uint16_t *)p_buf;
            *p_buf_16++ = times_;  //总次数
            *p_buf_16++ = i+1;  //当前次数
            p_buf = (uint8_t *)p_buf_16;
            //采集器地址
            for(j=0;j<5;j++){
              *p_buf++ = p_cjqaddr[j];
            }
            //表数据
            for(k=0;k<frame_meter_count;k++){
              sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
              for(j=0;j<7;j++){
                *p_buf++ = meter_addr[j];
              }
              sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
            }
            
            *p_buf++ = check_cs(p_buf+6,frame_data_len);
            *p_buf++ = FRAME_END;
            
            switch(desc){
            case 0x01:
              send_server(p_buf_,frame_data_len+8);
              break;
            default:
              write_cjq(p_buf_,frame_data_len+8);
              break;
            }
            delayms(100);
          }
        }
      }else{  //the single meter under the cjq
        if(search_meter(p_meteraddr)){  //get the meter
          *p_buf++ = FRAME_HEAD;
          *p_buf++ = 0x67;//((9+12+4) << 2) | 0x03;
          *p_buf++ = 0x00;
          *p_buf++ = 0x67;//((9+12+4) << 2) | 0x03;
          *p_buf++ = 0x00;
          *p_buf++ = FRAME_HEAD;
          
          *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
          /**/
          p_temp = get_device_addr();
          *p_buf++ = p_temp[0];
          *p_buf++ = p_temp[1];
          *p_buf++ = p_temp[2];
          *p_buf++ = p_temp[3];
          *p_buf++ = p_temp[4];
          
          *p_buf++ = AFN_QUERY;
          *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
          *p_buf++ = FN_METER;
          
          *p_buf++ = 0x00;
          *p_buf++ = 0x00;
          *p_buf++ = 0x00;
          *p_buf++ = 0x00;
          for(i=0;i<5;i++){
            *p_buf++ = p_cjqaddr[i];
          }
          for(i=0;i<7;i++){
            *p_buf++ = p_meteraddr[i];
          }
          
          *p_buf++ = check_cs(p_buf+6,25);
          *p_buf++ = FRAME_END;
          
          switch(desc){
          case 0x01:
            send_server(p_buf_,17+5+7+4);
            break;
          default:
            write_cjq(p_buf_,17+5+7+4);
            break;
          }
        }
      }
    }
    put_membuf(p_buf_);
  }
  
  
}













//有采集器  返回此采集器的block地址   没有则返回0x000000
uint32_t search_cjq(uint8_t * p_cjqaddr){
  uint32_t block_current = 0;
  uint16_t cjq_count = 0;
  uint8_t cjq_addr[5];
  
  uint16_t i = 0;
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //采集器数量
  sFLASH_ReadBuffer((uint8_t *)&block_current,sFLASH_CJQ_Q_START,3);  //采集器队列头
  
  for(i = 0;i < cjq_count;i++){
    
    sFLASH_ReadBuffer(cjq_addr,block_current+CJQ_FLASH_INDEX_ADDR,5);
    if(Mem_Cmp(p_cjqaddr,cjq_addr,5)){
      
      return block_current;
    }
    
    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+METER_FLASH_INDEX_ADDR,3);
  }
  return 0x000000;
}

uint32_t add_cjq(uint8_t * p_cjqaddr){
  
  uint32_t block_last = 0;
  uint32_t block_new = 0;
  uint16_t cjq_count = 0;
  uint32_t meter_count = 0;
  uint8_t * mem4k = 0;
  
  block_last = search_cjq(p_cjqaddr);
  if(block_last){  //已经有这个采集器了
    return block_last;
  }
  //没有这个采集器
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,sFLASH_CJQ_Q_LAST,3);
  
  cjq_count++;  //采集器数量++
  
  //获取一个flash块  并配置相应信息
  block_new = GetFlash();  
  if(block_new){  //得到FLASH BLOCK
    //**************配置采集器块**************//
    sFLASH_WritePage(p_cjqaddr,block_new + CJQ_FLASH_INDEX_ADDR,5);  //采集器地址
    sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_METERCOUNT,2);  //采集器表数  (uint8_t *)0 是地址0x00000000处的值。
    sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_FIRSTMETER,3);  
    sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_LASTMETER,3);   //第一块表和最后一块表都指向了0x000000
    
    //**************配置总配置块**************//
    mem4k = get_mem4k();
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    if(block_last == 0x000000){
      //this is the first cjq
      //cjq Q 的开始和结尾都指向新添加的采集器块
      Mem_Copy(mem4k + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
      Mem_Copy(mem4k + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
      Mem_Copy(mem4k + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    }else{
      //不是第一个
      //将cjq Q 的结尾指向新添加的采集器块
      Mem_Copy(mem4k + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
      Mem_Copy(mem4k + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
      //将原来最后一个采集器的下一个采集器指向新添加的采集器块
      sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);
    }
    //将新添加的采集器块中的上一个采集器  指向原来的最后一个采集器
    sFLASH_WritePage((uint8_t *)&block_last,block_new + CJQ_FLASH_INDEX_PREVCJQ,3);  //上一个采集器
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
  }
  //没有得到FLASH BLOCK
  return block_new;
}


uint32_t add_meters(uint8_t * p_buf){
  uint8_t metercount = 0;
  uint16_t frame_len = 0;
  uint8_t i = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0; //表类型
  uint8_t result = 0;
  
  frame_len = check_frame(p_buf);
  meter_type = *(p_buf+18);
  //frame 中 8~帧头+帧尾  6~控制域地址域 3~AFN+SEQ+FN
  //frame 数据域 : 运行标志(1) 表类型(1) 采集器地址(5)  表地址(7)...
  metercount = (frame_len - 17 - 7)/7;
  block_cjq = search_cjq(p_buf + 19);
  if(block_cjq){
    for(i = 0;i < metercount;i++){
      if(add_single_meter(block_cjq,p_buf+24+i*7,meter_type)){
        result++;
      }
    }
    if(result != metercount){
      result = 0;
    }
  }
  return result;
}

uint32_t add_single_meter(uint32_t block_cjq, uint8_t * p_meteraddr, uint8_t meter_type){
  uint32_t block_last = 0;
  uint32_t block_new = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  uint32_t meter_read = 0;
  uint8_t * mem4k = 0;
  
  block_last = search_meter(block_cjq,p_meteraddr);
  if(block_last){  //已经有这个表了
    return block_last;
  }
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,block_cjq+CJQ_FLASH_INDEX_LASTMETER,3);
  
  meter_count++;  //采集器下的表数量++
  meter_all++;  //所有表数量++
  
  //获取一个flash块  并配置相应信息
  block_new = GetFlash();  
  if(block_new){  //得到FLASH BLOCK
    //**************配置表块**************//
    sFLASH_WritePage(p_meteraddr,block_new + METER_FLASH_INDEX_ADDR,7);  //表地址
    sFLASH_WritePage((uint8_t *)&meter_type,block_new + METER_FLASH_INDEX_TYPE,1);  //表类型
    sFLASH_WritePage((uint8_t *)&meter_read,block_new + METER_FLASH_INDEX_READ,4);  //表读数  meter_read = 0
    sFLASH_WritePage((uint8_t *)&meter_read,block_new + METER_FLASH_INDEX_METERSTATE,2);  //表状态  meter_read = 0
    
    //**************配置采集器块**************//
    mem4k = get_mem4k();
    sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //读取采集器所在Sector
    if(block_last == 0x000000){//first meter
      //采集器表的开始和结尾都指向新添加的表的块
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_FIRSTMETER,(uint8_t *)&block_new,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_new,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);  //采集器下的数目++
      
    }else{//将采集器表的结尾指向新添加的表的块
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_new,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);  //采集器下的数目++
      
      //原来最后一个表的下一个表指向新添加的表的块
      sFLASH_WritePage((uint8_t *)&block_new,block_last + FLASH_POOL_NEXT_INDEX,3);  
    }
    //将配置好的Flash块重新写入到Flash中。
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
    //新添加的表的块的上一个表 指向原来的最后一个表
    sFLASH_WritePage((uint8_t *)&block_last,block_new + METER_FLASH_INDEX_PREVMETER,3);
    
    //**************配置总配置块**************//
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);//all meter
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
  }
  //没有得到FLASH BLOCK
  return block_new;
}

uint32_t search_meter(uint32_t block_cjq,uint8_t * p_meteraddr){
  uint32_t block_current = 0;
  uint16_t meter_count = 0;
  uint8_t meter_addr[7];
  uint16_t i;
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_current,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
  
  for(i = 0;i < meter_count;i++){
    sFLASH_ReadBuffer(meter_addr,block_current+METER_FLASH_INDEX_ADDR,7);
    if(Mem_Cmp(p_meteraddr,meter_addr,7)){
      return block_current;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+FLASH_POOL_NEXT_INDEX,3);
  }
  return 0x000000;
}


uint32_t delete_meters(uint8_t * p_buf){
  uint8_t metercount = 0;
  uint16_t frame_len = 0;
  uint8_t i = 0;
  uint32_t block_cjq = 0;
  uint8_t result = 0;
  
  frame_len = check_frame(p_buf);
  //frame 中 8~帧头+帧尾  6~控制域地址域 3~AFN+SEQ+FN
  //frame 数据域 : 运行标志(1) 表类型(1) 采集器地址(5)  表地址(7)...
  metercount = (frame_len - 17 - 7)/7;
  block_cjq = search_cjq(p_buf + 19);
  if(block_cjq){
    for(i = 0;i < metercount;i++){
      if(delete_single_meter(block_cjq,p_buf+24+i*7)){
        result++;
      }
    }
    if(result != metercount){
      result = 0;
    }
  }
  return result;
}
uint32_t delete_single_meter(uint32_t block_cjq,uint8_t * p_meteraddr){
  uint32_t block_current = 0;
  uint32_t block_after = 0;
  uint32_t block_before = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  uint8_t * mem4k = 0;
  
  block_current = search_meter(block_cjq,p_meteraddr);
  if(block_current == 0x000000){  //没有这个表
    return 1;
  }
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_after,block_current+FLASH_POOL_NEXT_INDEX,3);
  sFLASH_ReadBuffer((uint8_t *)&block_before,block_current+METER_FLASH_INDEX_PREVMETER,3);
  
  if(PutFlash(block_current)){
    meter_count--;
    meter_all--;
    
    mem4k = get_mem4k();
    if(meter_count == 0){
      //采集器下唯一的表  block_before、block_after  都为0x000000
      sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //读取所在Sector
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_FIRSTMETER,(uint8_t *)&block_after,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_before,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);
      sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
      sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
    }else{
      if(block_before == 0x000000 || block_after == 0x000000){//要删除的这个是第一个  或者是最后一个
        if(block_before == 0x000000){
          //修改后一个的before 为block_before
          sFLASH_ReadBuffer(mem4k,(block_after/0x1000)*0x1000,0x1000);  //读取所在Sector
          Mem_Copy(mem4k+block_after%0x1000 + METER_FLASH_INDEX_PREVMETER,(uint8_t *)&block_before,3);
          sFLASH_EraseSector((block_after/0x1000)*0x1000);
          sFLASH_WriteBuffer(mem4k,(block_after/0x1000)*0x1000,0x1000);
          //修改采集器第一个表地址为  block_after  更新采集器表数目
          sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //读取所在Sector
          Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_FIRSTMETER,(uint8_t *)&block_after,3);
          Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);
          sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
          sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
        }
        if(block_after == 0x000000){
          //修改前一个的next 为block_after
          sFLASH_ReadBuffer(mem4k,(block_before/0x1000)*0x1000,0x1000);  //读取所在Sector
          Mem_Copy(mem4k+block_before%0x1000 + FLASH_POOL_NEXT_INDEX,(uint8_t *)&block_after,3);
          sFLASH_EraseSector((block_before/0x1000)*0x1000);
          sFLASH_WriteBuffer(mem4k,(block_before/0x1000)*0x1000,0x1000);
          //修改采集器的最后一个表地址为  block_before  更新采集器表数目
          sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //读取所在Sector
          Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_before,3);
          Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);
          sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
          sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
        }
      }else{
        //要删除的这个在中间
        //修改前一个的next 为block_after
        sFLASH_ReadBuffer(mem4k,(block_before/0x1000)*0x1000,0x1000);  //读取所在Sector
        Mem_Copy(mem4k+block_before%0x1000 + FLASH_POOL_NEXT_INDEX,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(mem4k,(block_before/0x1000)*0x1000,0x1000);
        //修改后一个的before 为block_before
        sFLASH_ReadBuffer(mem4k,(block_after/0x1000)*0x1000,0x1000);  //读取所在Sector
        Mem_Copy(mem4k+block_after%0x1000 + METER_FLASH_INDEX_PREVMETER,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(mem4k,(block_after/0x1000)*0x1000,0x1000);
        //更新采集器表数目
        sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //读取所在Sector
        Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
      }
    }
    //更新全部表数目
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    return 1;
  }else{
    return 0;
  }
}





















