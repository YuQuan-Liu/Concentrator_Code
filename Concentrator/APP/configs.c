

#include "utils.h"
#include "device_params.h"
#include "configs.h"
#include "serial.h"
#include "frame.h"
#include "gprs.h"
#include "os.h"
#include "spi_flash.h"
#include "readmeter.h"


void param_config(uint8_t * p_buf,uint16_t msg_size){
  uint8_t * p_temp = 0;
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
    p_temp = get_ip();
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_CON_IP4 - sFLASH_CON_START_ADDR),p_buf + DATA_POSITION,4);
    Mem_Copy(mem4k + (sFLASH_CON_PORT_ - sFLASH_CON_START_ADDR),&port,2);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
    break;
  case FN_ADDR:

    set_device_addr(p_buf + DATA_POSITION);
    p_temp = get_device_addr();
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_DEVICE_ADDR - sFLASH_CON_START_ADDR),p_temp,5);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
    break;
  case FN_METER:
    if(*(p_buf + DATA_POSITION) == 0x00){//删除表
      if(delete_meters(p_buf)){
        device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      }
    }

    if(*(p_buf + DATA_POSITION) == 0x01){//添加表
      if(add_meters(p_buf)){
        device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      }
    }
    break;
  case FN_CJQ:
    if(*(p_buf + DATA_POSITION) == 0xAA){  //删除全部采集器  即重新初始化FLASH POOL
      if(delete_cjqs()){
        device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      }
    }

    if(*(p_buf + DATA_POSITION) == 0x55){  //添加采集器
      if(add_cjq(p_buf + DATA_POSITION + 1)){
        device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      }
    }

    //删除某一个采集器的功能先不加
    break;
  case FN_MBUS:
    switch(*(p_buf + DATA_POSITION)){
    case 0xAA:
    case 0xBB:
    case 0xFF:
      set_slave(*(p_buf + DATA_POSITION));
      temp_u8 = get_slave();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_METER_MBUS - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
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

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
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

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_PROTOCOL:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF: //188
    case 0xEE: //188 bad
      set_protocol(*(p_buf + DATA_POSITION));
      temp_u8 = get_protocol();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_PROTOCOL - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
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

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_DEVICE_MODE:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF: //无线
    case 0xAA: //有线
      set_device_mode(*(p_buf + DATA_POSITION));
      temp_u8 = get_device_mode();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_DEVICE_MODE - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_SIMCARD:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF: //移动
    case 0xAA: //联通
      set_simcard(*(p_buf + DATA_POSITION));
      temp_u8 = get_simcard();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_SIMCARD - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_ERASE:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF:
      sFLASH_PoolInit();
      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_RESET:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF:
      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
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
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
    break;
  case FN_SYN:
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
    sync_data2cjq(p_buf + DATA_POSITION);//同步CJQ所有通道数据到 采集器
    break;
  }

  unlock_mem4k();
}

void param_query(uint8_t * p_buf,uint16_t msg_size){
  uint8_t server_seq_ = *(p_buf + SEQ_POSITION) & 0x0F;
  uint32_t block_cjq = 0;
  uint8_t temp = 0;
  uint8_t * p_temp = 0;
  switch(*(p_buf + FN_POSITION)){
  case FN_IP_PORT:
    ack_query_ip(*(p_buf+msg_size),server_seq_);
    break;
  case FN_METER:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF: //全部表
      ack_query_meter_all(*(p_buf+msg_size),server_seq_);
      break;
    case 0xAA: //单个采集器通道
      block_cjq = search_cjq(p_buf + DATA_POSITION+1);
      if(block_cjq){
        ack_query_meter_channel(block_cjq,0,0,*(p_buf+msg_size),server_seq_);
      }
      break;
    case 0x11: //单个表
      ack_query_meter_single(p_buf + DATA_POSITION+1,p_buf + DATA_POSITION + 6,*(p_buf+msg_size),server_seq_);
      break;
    }
    break;
  case FN_CJQ:
    ack_query_cjq(*(p_buf+msg_size),server_seq_);
    break;
  case FN_ADDR:
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_QUERY,FN_ADDR);
    break;
  case FN_MBUS:
    temp = get_slave();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_MBUS);
    break;
  case FN_DI_SEQ:
    temp = get_di_seq();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_DI_SEQ);
    break;
  case FN_ACK_ACTION:
    temp = get_ack_valve();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_ACK_ACTION);
    break;
  case FN_PROTOCOL:
    temp = get_protocol();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_PROTOCOL);
    break;
  case FN_VERSION:
    temp = get_version();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_VERSION);
    break;
  case FN_BAUD:
    temp = get_meter_baud();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_BAUD);
    break;
  case FN_DEVICE_MODE:
    temp = get_device_mode();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_DEVICE_MODE);
    break;
  case FN_SIMCARD:
    temp = get_simcard();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_DEVICE_MODE);
    break;
  case FN_READING:
    temp = get_readding();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_READING);
    break;
  case FN_SYN:
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
    check_sync_data2cjq(p_buf+DATA_POSITION,*(p_buf+msg_size),server_seq_);//检查 CJQ当前通道与 JZQ 同步情况
    break;
  case FN_ALL_READDATA:
    p_temp = get_membuf();
    if(p_temp > 0){
      send_meter_data_all(*(p_buf + msg_size),p_temp);
    }
    break;
  }
}

void device_ack(uint8_t desc,uint8_t server_seq_,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn){
  uint8_t * p_temp = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;
  uint8_t i = 0;

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    p_buf = (uint8_t *)p_buf_16;
    *p_buf++ = FRAME_HEAD;

    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];

    *p_buf++ = afn;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = fn;

    for(i = 0;i < data_len;i++){
      *p_buf++ = p_data[i];
    }

    *p_buf++ = check_cs(p_buf_+6,9+data_len);
    *p_buf++ = FRAME_END;

    switch(desc){
    case 0x01:
      if(lock_gprs()){
        send_server(p_buf_,17+data_len);
        unlock_gprs();
      }
      break;
    default:
      if(lock_cjq()){
        write_cjq(p_buf_,17+data_len);
        unlock_cjq();
      }
      break;
    }
    put_membuf(p_buf_);
  }
}



void device_ack_cjq(uint8_t desc,uint8_t server_seq_,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn){
  uint8_t * p_temp = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;
  uint8_t i = 0;

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    p_buf = (uint8_t *)p_buf_16;
    *p_buf++ = FRAME_HEAD;

    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
    /**/
    p_temp = get_cjq_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];

    *p_buf++ = afn;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = fn;

    for(i = 0;i < data_len;i++){
      *p_buf++ = p_data[i];
    }

    *p_buf++ = check_cs(p_buf_+6,9+data_len);
    *p_buf++ = FRAME_END;

    switch(desc){
    case 0x01:
      if(lock_lora()){
        write_lora(p_buf_,17+data_len);
        unlock_lora();
      }
      break;
    default:
      if(lock_cjq()){
        write_cjq(p_buf_,17+data_len);
        unlock_cjq();
      }
      break;
    }
    put_membuf(p_buf_);
  }
}



void ack_query_ip(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;

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
    *p_buf_16++ = get_port();
    p_buf = (uint8_t *)p_buf_16;

    *p_buf++ = check_cs(p_buf_+6,15);
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


void ack_query_cjq(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;

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


uint8_t * ack_mulit_header(uint8_t *p_buf,uint8_t *p_device_addr,uint8_t frame_type,uint16_t len,uint8_t afn,uint8_t seq_,uint8_t fn){
  uint16_t * p_buf_16 = 0;

  *p_buf++ = FRAME_HEAD;
  p_buf_16 = (uint16_t *)p_buf;
  *p_buf_16++ = len;
  *p_buf_16++ = len;
  p_buf = (uint8_t *)p_buf_16;
  *p_buf++ = FRAME_HEAD;

  *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  //p_temp = get_device_addr();
  *p_buf++ = p_device_addr[0];
  *p_buf++ = p_device_addr[1];
  *p_buf++ = p_device_addr[2];
  *p_buf++ = p_device_addr[3];
  *p_buf++ = p_device_addr[4];

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
  return p_buf;
}


void ack_query_meter_all(uint8_t desc,uint8_t server_seq_){
  uint16_t cjq_count = 0;
  uint32_t block_cjq = 0;
  uint16_t meter_count = 0;

  uint16_t send_times = 0;  //所有表 发送总次数
  uint16_t sended = 0;  //已经发送的
  uint8_t remains = 0;
  uint16_t c = 0;

  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //采集器数量
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //采集器队列头

  //获取所有通道所有的表加起来一共要发送多少次
  for(c = 0;c < cjq_count;c++){
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%5;
    send_times = send_times + meter_count/5;
    if(remains){
      send_times = send_times + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }

  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //采集器队列头

  for(c = 0;c < cjq_count;c++){
    ack_query_meter_channel(block_cjq,send_times,sended,desc,server_seq_);

    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%5;
    sended = sended + meter_count/5;
    if(remains){
      sended = sended + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }
}

void ack_query_meter_channel(uint32_t block_cjq_,uint16_t frame_times,uint16_t frame_times_start,uint8_t desc,uint8_t server_seq_){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;
  uint16_t i = 0;
  uint8_t j = 0;
  uint8_t k = 0;
  uint8_t frame_meter_count = 0;
  uint8_t meter_addr[7];
  uint8_t cjq_addr[5];

  uint32_t block_cjq = block_cjq_;
  uint32_t block_meter = 0;
  uint16_t cjqmeter_count = 0;
  uint8_t frame_data_len = 0;
  uint8_t frame_seq = 0;

  uint16_t times = 0;
  uint8_t remain = 0;
  uint16_t times_ = 0;      //一共要发送多少帧

  uint8_t ack = 0;  //发送数据后是否得到ack
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
    sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);
    if(cjqmeter_count > 0){
      times = cjqmeter_count/5;
      remain = cjqmeter_count%5;
      times_ = times;
      if(remain > 0){
        times_ = times_ + 1;
      }

      for(i=0;i< times_;i++){
        p_buf = p_buf_;
        frame_seq = add_server_seq();
        set_server_data_seq(frame_seq);
        
        frame_data_len = 0;
        frame_meter_count = 0;
        if(times_==1){//单帧 0
          frame_meter_count = cjqmeter_count;
          frame_data_len = 9+7*frame_meter_count+5+4;
          p_buf = ack_mulit_header(p_buf,get_device_addr(),0,(frame_data_len << 2) | 0x03,AFN_QUERY,frame_seq,FN_METER);
        }else{
          if(i==0){//首帧 1
            frame_meter_count = 5;
            frame_data_len = 9+7*frame_meter_count+5+4;
            p_buf = ack_mulit_header(p_buf,get_device_addr(),1,(frame_data_len << 2) | 0x03,AFN_QUERY,frame_seq,FN_METER);
          }else{
            if(i==times_-1){//尾帧 3
              frame_meter_count = remain;
              frame_data_len = 9+7*frame_meter_count+5+4;
              p_buf = ack_mulit_header(p_buf,get_device_addr(),3,(frame_data_len << 2) | 0x03,AFN_QUERY,frame_seq,FN_METER);
            }else{//中间帧 2
              frame_meter_count = 5;
              frame_data_len = 9+7*frame_meter_count+5+4;
              p_buf = ack_mulit_header(p_buf,get_device_addr(),2,(frame_data_len << 2) | 0x03,AFN_QUERY,frame_seq,FN_METER);
            }
          }
        }

        //帧的数据域
        p_buf_16 = (uint16_t *)p_buf;
        if(frame_times){
          *p_buf_16++ = frame_times;  //总次数
        }else{
          *p_buf_16++ = times_;
        }
        *p_buf_16++ = i+1+frame_times_start;  //当前次数
        p_buf = (uint8_t *)p_buf_16;
        //采集器地址
        for(j=0;j<5;j++){
          *p_buf++ = cjq_addr[j];
        }
        //表数据
        for(k=0;k<frame_meter_count;k++){
          sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
          for(j=0;j<7;j++){
            *p_buf++ = meter_addr[j];
          }
          sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
        }

        *p_buf++ = check_cs(p_buf_+6,frame_data_len);
        *p_buf++ = FRAME_END;

        for(k = 0;k < 3;k++){
          ack = 0;
          switch(desc){
          case 0x01:
            if(lock_gprs()){
              write_server(p_buf_,p_buf-p_buf_);
              unlock_gprs();
            }
            break;
          case 0x00:
            if(lock_cjq()){
              write_cjq(p_buf_,p_buf-p_buf_);
              unlock_cjq();
            }
            break;
          }
          
          if(wait_serverack(10000)){ //wait ack
            ack = 1;
            break;
          }
          delayms(100);
        }
        if(!ack){
          break;
        }
      }
    }
    put_membuf(p_buf_);
  }
}

void ack_query_meter_single(uint8_t *p_cjqaddr,uint8_t * p_meteraddr,uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t i = 0;

  uint32_t block_cjq = 0;


  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    block_cjq = search_cjq(p_cjqaddr);
    if(block_cjq){
      if(search_meter(block_cjq,p_meteraddr)){  //get the meter
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

        *p_buf++ = check_cs(p_buf_+6,25);
        *p_buf++ = FRAME_END;

        switch(desc){
        case 0x01:
          if(lock_lora()){
            write_lora(p_buf_,17+5+7+4);
            unlock_lora();
          }
          break;
        case 0x00:
          if(lock_cjq()){
            write_cjq(p_buf_,17+5+7+4);
            unlock_cjq();
          }
          break;
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

    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+FLASH_POOL_NEXT_INDEX,3);
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
    //sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_FIRSTMETER,3);
    //sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_LASTMETER,3);   //第一块表和最后一块表都指向了0xFFFFFF

    //**************配置总配置块**************//
    mem4k = get_mem4k();
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    if(block_last == sFLASH_POINT_END){
      //this is the first cjq
      //cjq Q 的开始和结尾都指向新添加的采集器块
      Mem_Copy(mem4k + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
      Mem_Copy(mem4k + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
      Mem_Copy(mem4k + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    }else{
      //不是第一个
      //将cjq Q 的结尾指向新添加的采集器块
      Mem_Copy(mem4k + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
      Mem_Copy(mem4k + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      //将原来最后一个采集器的下一个采集器指向新添加的采集器块
      sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);
    }
    //将新添加的采集器块中的上一个采集器  指向原来的最后一个采集器
    sFLASH_WritePage((uint8_t *)&block_last,block_new + CJQ_FLASH_INDEX_PREVCJQ,3);  //上一个采集器
  }
  //没有得到FLASH BLOCK
  return block_new;
}

uint8_t delete_cjqs(void){

  uint8_t * mem4k = 0;
  uint32_t temp32 = 0;
  mem4k = get_mem4k();

  sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);//保留以前的配置
  sFLASH_PoolInit();

  //the pool start
  temp32 = sFLASH_POOL_START_ADDR;
  Mem_Copy(mem4k + (sFLASH_POOL - sFLASH_CON_START_ADDR),&temp32,3);
  //pool free
  temp32 = 2044;
  Mem_Copy(mem4k + (sFLASH_POOL_FREE - sFLASH_CON_START_ADDR),&temp32,2);
  //pool used
  temp32 = 0x000000;
  Mem_Copy(mem4k + (sFLASH_POOL_USED - sFLASH_CON_START_ADDR),&temp32,2);
  //pool all
  temp32 = 2044;
  Mem_Copy(mem4k + (sFLASH_POOL_ALL - sFLASH_CON_START_ADDR),&temp32,2);

  //CJQ Q
  temp32 = sFLASH_POINT_END;
  Mem_Copy(mem4k + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),&temp32,3);
  temp32 = 0x000000;
  Mem_Copy(mem4k + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),&temp32,2);
  temp32 = sFLASH_POINT_END;
  Mem_Copy(mem4k + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),&temp32,3);

  //Meter Q
  temp32 = sFLASH_POINT_END;
  Mem_Copy(mem4k + (sFLASH_METER_Q_START - sFLASH_CON_START_ADDR),&temp32,3);
  temp32 = 0x000000;
  Mem_Copy(mem4k + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),&temp32,2);

  temp32 = 0xABCDAA;
  Mem_Copy(mem4k + (sFLASH_POOL_INIT - sFLASH_CON_START_ADDR),&temp32,1);

  sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

  return 1;
}

uint32_t add_meters(uint8_t * p_buf){
  uint8_t metercount = 0;
  uint16_t frame_len = 0;
  uint8_t i = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0; //表类型
  uint8_t result = 0;

  frame_len = check_frame(p_buf);
  meter_type = *(p_buf+DATA_POSITION+1);
  //frame 中 8~帧头+帧尾  6~控制域地址域 3~AFN+SEQ+FN
  //frame 数据域 : 运行标志(1) 表类型(1) 采集器地址(5)  表地址(7)...
  metercount = (frame_len - 17 - 7)/7;
  block_cjq = search_cjq(p_buf + DATA_POSITION+2);
  if(block_cjq){
    for(i = 0;i < metercount;i++){
      if(add_single_meter(block_cjq,p_buf+22+i*7,meter_type)){
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
    if(block_last == sFLASH_POINT_END){//first meter
      //采集器表的开始和结尾都指向新添加的表的块
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_FIRSTMETER,(uint8_t *)&block_new,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_new,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);  //采集器下的数目++
      //将配置好的Flash块重新写入到Flash中。
      sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
      sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
    }else{//将采集器表的结尾指向新添加的表的块
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_new,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);  //采集器下的数目++
      //将配置好的Flash块重新写入到Flash中。
      sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
      sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
      //原来最后一个表的下一个表指向新添加的表的块
      sFLASH_WritePage((uint8_t *)&block_new,block_last + FLASH_POOL_NEXT_INDEX,3);
    }
    
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
  uint16_t i = 0;

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
  block_cjq = search_cjq(p_buf + DATA_POSITION+2);
  if(block_cjq){
    for(i = 0;i < metercount;i++){
      if(delete_single_meter(block_cjq,p_buf+22+i*7)){
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
  if(!block_current){  //没有这个表
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
      //采集器下唯一的表  block_before、block_after  都为0xFFFFFF
      sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //读取所在Sector
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_FIRSTMETER,(uint8_t *)&block_after,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_before,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);
      sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
      sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
    }else{
      if(block_before == sFLASH_POINT_END || block_after == sFLASH_POINT_END){//要删除的这个是第一个  或者是最后一个
        if(block_before == sFLASH_POINT_END){
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
        if(block_after == sFLASH_POINT_END){
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




void sync_data2cjq(uint8_t * p_cjqaddr){
  //1删除采集器中所有的数据
  //2添加采集器通道
  //3下载通道中的所有的数据

  uint32_t block_cjq = 0;
  uint8_t cjq_seq = 0;
  uint16_t c = 0;
  uint8_t i = 0;
  uint8_t frame_data[6];
  uint8_t deletecjq_ok = 0;  //删除采集器内的数据是否OK
  uint8_t addcjq_ok = 0;  //删除采集器内的数据是否OK

  //删除所有
  frame_data[0] = 0xAA;
  cjq_seq = add_cjq_seq();
  set_cjq_data_seq(cjq_seq);
  set_cjq_addr(p_cjqaddr);
  if(write_frame_cjq(p_cjqaddr, frame_data, 1,AFN_CONFIG,FN_CJQ,cjq_seq)){ //删除所有的采集器
    if(wait_cjqack(20000)){
      deletecjq_ok = 1;
    }
  }
  if(!deletecjq_ok){  //删除采集器失败
    return;
  }

  for(c = 1;c <= 3;c++){
    *p_cjqaddr = c;
    block_cjq = search_cjq(p_cjqaddr);
    if(block_cjq){
      //添加通道
      addcjq_ok = 0;
      frame_data[0] = 0x55;
      for(i = 0;i < 5;i++){
        frame_data[i+1] = p_cjqaddr[i];
      }
      cjq_seq = add_cjq_seq();
      set_cjq_data_seq(cjq_seq);
      set_cjq_addr(p_cjqaddr);
      if(write_frame_cjq(p_cjqaddr, frame_data, 6,AFN_CONFIG,FN_CJQ,cjq_seq)){ //添加采集器通道
        if(wait_cjqack(10000)){
          addcjq_ok = 1;
        }
      }
      if(!addcjq_ok){
        break;
      }
    }
  }
  if(!addcjq_ok){  //添加所有的采集器通道失败
    return;
  }

  //添加每个采集器通道的表
  for(c = 1;c <= 3;c++){
    *p_cjqaddr = c;
    block_cjq = search_cjq(p_cjqaddr);
    if(block_cjq){
      //添加通道内所有的表
      if(!addcjq_meter_data(block_cjq)){
        break;
      }
    }
  }
}


uint8_t addcjq_meter_data(uint32_t block_cjq_){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;

  uint8_t frame_meter_count = 0;
  uint8_t meter_addr[7];
  uint8_t cjq_addr[5];
  uint32_t block_cjq = block_cjq_;
  uint32_t block_meter = 0;
  uint16_t cjqmeter_count = 0;
  uint8_t frame_data_len = 0;
  uint16_t times = 0;
  uint8_t remain = 0;
  uint16_t times_ = 0;      //一共要发送多少帧
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t k = 0;
  uint8_t cjq_seq = 0;
  uint8_t addmeter_ok = 0;
  uint8_t meter_type = 0;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
    sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);
    sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);
    if(cjqmeter_count > 0){
      times = cjqmeter_count/5;
      remain = cjqmeter_count%5;
      times_ = times;
      if(remain > 0){
        times_ = times_ + 1;
      }

      for(i=0;i< times_;i++){
        p_buf = p_buf_;
        frame_data_len = 0;
        frame_meter_count = 0;
        
        cjq_seq = add_cjq_seq();
        set_cjq_data_seq(cjq_seq);
        if(times_==1){//单帧 0
          frame_meter_count = cjqmeter_count;
          frame_data_len = 9+7*frame_meter_count+5+2;
          p_buf = ack_mulit_header(p_buf,cjq_addr,0,(frame_data_len << 2) | 0x03,AFN_CONFIG,cjq_seq,FN_METER);
        }else{
          if(i==0){//首帧 1
            frame_meter_count = 5;
            frame_data_len = 9+7*frame_meter_count+5+2;
            p_buf = ack_mulit_header(p_buf,cjq_addr,1,(frame_data_len << 2) | 0x03,AFN_CONFIG,cjq_seq,FN_METER);
          }else{
            if(i==times_-1){//尾帧 3
              frame_meter_count = remain;
              frame_data_len = 9+7*frame_meter_count+5+2;
              p_buf = ack_mulit_header(p_buf,cjq_addr,3,(frame_data_len << 2) | 0x03,AFN_CONFIG,cjq_seq,FN_METER);
            }else{//中间帧 2
              frame_meter_count = 5;
              frame_data_len = 9+7*frame_meter_count+5+2;
              p_buf = ack_mulit_header(p_buf,cjq_addr,2,(frame_data_len << 2) | 0x03,AFN_CONFIG,cjq_seq,FN_METER);
            }
          }
        }

        //frame 数据域 : 运行标志(1) 表类型(1) 采集器地址(5)  表地址(7)...
        *p_buf++ = 0x01;   //添加表
        *p_buf++ = meter_type; //表类型

        for(j = 0;j < 5;j++){
          *p_buf++ = cjq_addr[j];
        }

        //表数据
        for(k=0;k<frame_meter_count;k++){
          sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
          for(j=0;j<7;j++){
            *p_buf++ = meter_addr[j];
          }
          sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
        }

        *p_buf++ = check_cs(p_buf_+6,frame_data_len);
        *p_buf++ = FRAME_END;

        for(j = 0;j < 3;j++){
          switch(get_device_mode()){
          case 0xFF:
            if(lock_lora()){
              write_lora(p_buf_,p_buf - p_buf_);
              unlock_lora();
            }
            break;
          case 0xAA:
            if(lock_cjq()){
              write_cjq(p_buf_,p_buf - p_buf_);
              unlock_cjq();
            }
            break;
          }
          if(wait_cjqack(10000)){
            addmeter_ok = 1;
            break;
          }
        }

        if(!addmeter_ok){
          break;
        }

        delayms(100);
      }
    }
    put_membuf(p_buf_);
  }
  return addmeter_ok;
}


void check_sync_data2cjq(uint8_t * p_cjqaddr,uint8_t desc,uint8_t server_seq_){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  uint8_t frame_data[6];
  uint16_t meter_count = 0;  //集中器中当前采集器通道表的数量
  uint16_t cjq_return_meter_count = 0;   //当前采集器返回的表的数量
  uint16_t no_meter_count = 0;   //当前采集器返回的表 集中器中没有的数量
  
  uint16_t all_frames = 0;
  uint16_t this_frame = 0;
  uint8_t frame_metercount = 0;
  
  uint8_t i = 0;
  uint8_t cjq_seq = 0;
  uint8_t timeout_count = 0;
  uint8_t * p_response = 0;
  uint16_t msg_size = 0;
  
  uint8_t * p_meteraddr = 0;
  uint8_t seq = 255;
  uint8_t seq_ = 0;
  
  uint32_t send_ts = 0;  //发送指令的时钟
  
  block_cjq = search_cjq(p_cjqaddr);
  if(block_cjq){
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);

    frame_data[0] = 0xAA;
    for(i = 0;i < 5;i++){
      frame_data[i+1] = p_cjqaddr[i];
    }
    cjq_seq = add_cjq_seq();
    set_cjq_data_seq(cjq_seq);
    set_cjq_addr(p_cjqaddr);
    if(write_frame_cjq(p_cjqaddr, frame_data, 6,AFN_QUERY,FN_METER,cjq_seq)){ //查询当前采集器通道下所有的表
      send_ts = get_timestamp();
      timeout_count = 10;
      while(timeout_count > 0){
        if(wait_q_cjq_ts(&p_response,&msg_size,10000,send_ts)){
          seq_ = *(p_response + SEQ_POSITION) & 0x0F; // 当前帧的seq_
          
          if(desc == 1){
            switch(get_device_mode()){
            case 0xFF:
              device_ack_cjq(1,seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);  //发往LORA
              break;
            case 0xAA:
              device_ack_cjq(0,seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);  //发往485
              break;
            }
          }else{
            device_ack_cjq(1,seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);  //485调试  发往LORA
          }
          
          if(seq == seq_){
            continue;
          }
          seq = seq_;
          
          //帧的长度msg_size  计算此帧中一共有多少表 frame_metercount
          //一共有多少帧all_frames  这是第几帧this_frame
          all_frames = *(p_response + DATA_POSITION) | *(p_response +DATA_POSITION+1)<<8;
          this_frame = *(p_response + DATA_POSITION+2) | *(p_response + DATA_POSITION+3)<<8;
          frame_metercount = (msg_size-8-9-5-4)/7;
          cjq_return_meter_count = cjq_return_meter_count + frame_metercount;
          for(i = 0;i < frame_metercount;i++){
            //从帧中获取  表地址 
            p_meteraddr = p_response + DATA_POSITION + 5 + 4 + 7 * i;
            block_meter = search_meter(block_cjq,p_meteraddr);
            if(!block_meter){  //是否找到采集器返回的表
              no_meter_count++;
            }
          }
          put_membuf(p_response);
          if(all_frames == this_frame){
            break;
          }
        }else{
          timeout_count--;
        }
      }

      if(timeout_count){  //接收所有的采集器返回数据
        frame_data[0] = meter_count & 0xFF;
        frame_data[1] = meter_count >> 8;
        frame_data[2] = cjq_return_meter_count & 0xFF;
        frame_data[3] = cjq_return_meter_count >> 8;
        frame_data[4] = no_meter_count & 0xFF;
        frame_data[5] = no_meter_count >> 8;
        device_ack(desc,server_seq_,frame_data,6,AFN_QUERY,FN_SYN);
      }else{  //等待超时
        frame_data[0] = 0xFF;
        frame_data[1] = 0xFF;
        frame_data[2] = 0xFF;
        frame_data[3] = 0xFF;
        frame_data[4] = 0xFF;
        frame_data[5] = 0xFF;
        device_ack(desc,server_seq_,frame_data,6,AFN_QUERY,FN_SYN);
      }
    }
  }
}
