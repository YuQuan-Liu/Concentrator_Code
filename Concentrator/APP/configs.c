

#include "configs.h"
#include "serial.h"
#include "frame.h"
#include "gprs.h"
#include "os.h"
#include "spi_flash.h"
#include "utils.h"

extern uint8_t ip[17];                 //the server ip
extern uint8_t port[8];  
extern uint8_t ip1;
extern uint8_t ip2;
extern uint8_t ip3;
extern uint8_t ip4;
extern uint16_t port_;

extern uint8_t deviceaddr[5];
extern uint8_t cjqaddr[5];
extern volatile uint8_t lora_send;

extern uint8_t slave_mbus; //0xaa mbus   0xff  485   0xBB~采集器

extern uint8_t config_flash[];  //配置处理Flash使用的数组  Sector==4K  需要一个4K的数组
extern uint8_t *meterdata;  //使用海大协议抄表时存放返回的信息  使用config_flash
extern uint8_t di_seq; //DI0 DI1 顺序   0xAA~DI1在前(千宝通)   0xFF~DI0在前(default)  
extern uint8_t ack_action;  //先应答后操作~0xaa    先操作后应答~0xff
extern uint8_t protocol;  //协议类型 0xFF~188(Default)  1~EG 

extern OS_MEM MEM_Buf;
extern OS_MUTEX MUTEX_CONFIGFLASH;    //是否可以使用 config_flash  4K 数组配置FLASH

void param_config(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint8_t ip_port_[6];
  
  uint16_t i = 0;
  uint16_t cjq_count = 0;
  uint32_t block_cjq = 0;   //cjq block 地址
  uint32_t block_cjq_next = 0;   //cjq block 地址删除时  先查出来下一个的地址 然后在删除
  uint32_t block_meter = 0;  //meter block 地址
  uint8_t server_seq_ = *(buf_frame + SEQ_POSITION) & 0x0F;
  
  switch(*(buf_frame + FN_POSITION)){
  case FN_IP_PORT:
    ip1 = *(buf_frame + DATA_POSITION + 3);
    ip2 = *(buf_frame + DATA_POSITION + 2);
    ip3 = *(buf_frame + DATA_POSITION + 1);
    ip4 = *(buf_frame + DATA_POSITION);
    port_ = *((uint16_t *)(buf_frame + DATA_POSITION +4));
    
    //ip
    Mem_Clr(ip,17);
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*(buf_frame + DATA_POSITION + 3));
    Str_Cat(ip,ip_port_);
    Str_Cat(ip,".");
    
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*(buf_frame + DATA_POSITION + 2));
    Str_Cat(ip,ip_port_);
    Str_Cat(ip,".");
    
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*(buf_frame + DATA_POSITION + 1));
    Str_Cat(ip,ip_port_);
    Str_Cat(ip,".");
    
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*(buf_frame + DATA_POSITION));
    Str_Cat(ip,ip_port_);
    
    
    //port
    Mem_Clr(port,8);
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*((uint16_t *)(buf_frame + DATA_POSITION +4)));
    Str_Cat(port,",");
    Str_Cat(port,ip_port_);
    Str_Cat(port,"\r");
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_CON_IP - sFLASH_CON_START_ADDR),ip,17);
    Mem_Copy(config_flash + (sFLASH_CON_PORT - sFLASH_CON_START_ADDR),port,8);
    
    Mem_Copy(config_flash + (sFLASH_CON_IP1 - sFLASH_CON_START_ADDR),&ip1,1);
    Mem_Copy(config_flash + (sFLASH_CON_IP2 - sFLASH_CON_START_ADDR),&ip2,1);
    Mem_Copy(config_flash + (sFLASH_CON_IP3 - sFLASH_CON_START_ADDR),&ip3,1);
    Mem_Copy(config_flash + (sFLASH_CON_IP4 - sFLASH_CON_START_ADDR),&ip4,1);
    Mem_Copy(config_flash + (sFLASH_CON_PORT_ - sFLASH_CON_START_ADDR),&port_,2);
    
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    device_ack(desc,server_seq_);
    
    break;
  case FN_ADDR:
    Mem_Clr(deviceaddr,5);
    deviceaddr[0] = *(buf_frame + DATA_POSITION);
    deviceaddr[1] = *(buf_frame + DATA_POSITION + 1);
    deviceaddr[2] = *(buf_frame + DATA_POSITION + 2);
    deviceaddr[3] = *(buf_frame + DATA_POSITION + 3);
    deviceaddr[4] = *(buf_frame + DATA_POSITION + 4);  //与新天协议有出入 协议第5位默认为0x00
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_DEVICE_ADDR - sFLASH_CON_START_ADDR),deviceaddr,5);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    
    break;
  case FN_METER:
    
    break;
  case FN_CJQ:
   
    break;
  case FN_MBUS:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //the slave is mbus
      slave_mbus = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xBB){
      //the slave is 采集器
      slave_mbus = 0xBB;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //the slave is 485
      slave_mbus = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_METER_MBUS - sFLASH_CON_START_ADDR),&slave_mbus,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_DI_SEQ:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //千宝通使用的大表模块
      di_seq = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //默认
      di_seq = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_READMETER_DI_SEQ - sFLASH_CON_START_ADDR),&di_seq,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_ACK_ACTION:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //骏普阀控表模式
      ack_action = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //默认
      ack_action = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_ACK_ACTION - sFLASH_CON_START_ADDR),&ack_action,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_PROTOCOL:
    if(*(buf_frame + DATA_POSITION) == 0x01){
      //EG protocol
      protocol = 0x01;
    }else{
      //188协议
      protocol = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_PROTOCOL - sFLASH_CON_START_ADDR),&protocol,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_ERASE:
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
      if(err != OS_ERR_NONE){
        //获取MUTEX过程中 出错了...
        //return 0xFFFFFF;
        return;
      }
      //处理Config Flash 块
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      di_seq = 0xFF;
      Mem_Copy(config_flash + (sFLASH_POOL_INIT - sFLASH_CON_START_ADDR),&di_seq,1);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
      
      device_ack(desc,server_seq_);
      Device_Cmd(DISABLE);
      *((uint8_t *)0) = 0x00;  //迫使系统重启
    }
    break;
  case FN_RESET:
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      device_ack(desc,server_seq_);
      Device_Cmd(DISABLE);
      *((uint8_t *)0) = 0x00;  //迫使系统重启
    }
    break;
  case FN_LORA_SEND:
    if(*(buf_frame + DATA_POSITION) == 0x01){
      //开始发送LORA测试
      lora_send = 0x01;
    }else{
      //停止发送LORA测试
      lora_send = 0x00;
    }
    device_ack(desc,server_seq_);
    break;
  }
  
}

void param_query(uint8_t * buf_frame,uint8_t desc){
  uint8_t server_seq_ = *(buf_frame + SEQ_POSITION) & 0x0F;
  switch(*(buf_frame + FN_POSITION)){
  case FN_IP_PORT:
    ack_query_ip(desc,server_seq_);
    break;
  case FN_ADDR: 
    ack_query_addr(desc,server_seq_);
    break;
  case FN_METER:
    
    break;
  case FN_CJQ:
    
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
  }
}


void device_ack(uint8_t desc,uint8_t server_seq_){
  uint8_t ack[17];
  uint8_t * buf_frame;
  
  buf_frame = ack;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_ACK;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_ACK;
  
  *buf_frame++ = check_cs(ack+6,9);
  *buf_frame++ = FRAME_END;
  
  if(desc){
    //to gprs
    send_server(ack,17);
  }else{
    //to 485
    Write_485_2(ack,17);
  }
  
}

void device_ack_lora(uint8_t desc,uint8_t server_seq_){
  uint8_t ack[17];
  uint8_t * buf_frame;
  
  buf_frame = ack;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x3B;//(14 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x3B;//(14 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_ACK;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_ACK;
  
  *buf_frame++ = cjqaddr[0];
  *buf_frame++ = cjqaddr[1];
  *buf_frame++ = cjqaddr[2];
  *buf_frame++ = cjqaddr[3];
  *buf_frame++ = cjqaddr[4];
  
  *buf_frame++ = check_cs(ack+6,14);
  *buf_frame++ = FRAME_END;
  
  
  Write_LORA(ack,22);
  
}


void ack_query_addr(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame;
  uint8_t * buf_frame_;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_ADDR;
  
  *buf_frame++ = check_cs(buf_frame_+6,9);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to gprs
    send_server(buf_frame_,17);
  }else{
    //to 485
    Write_485_2(buf_frame_,17);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
}

void ack_query_ip(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  uint16_t * buf_frame_16 = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x3F;//(15 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x3F;//(15 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_IP_PORT;
  
  *buf_frame++ = ip4;
  *buf_frame++ = ip3;
  *buf_frame++ = ip2;
  *buf_frame++ = ip1;
  
  buf_frame_16 = (uint16_t *)buf_frame;
  *buf_frame_16++ = port_;
  buf_frame = (uint8_t *)buf_frame_16;
  
  *buf_frame++ = check_cs(buf_frame_+6,15);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to gprs
    send_server(buf_frame_,23);
  }else{
    //to 485
    Write_485_2(buf_frame_,23);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}

void ack_query_mbus(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_MBUS;
  
  *buf_frame++ = slave_mbus;
  *buf_frame++ = check_cs(buf_frame_+6,10);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to gprs
    send_server(buf_frame_,18);
  }else{
    //to 485
    Write_485_2(buf_frame_,18);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}

void ack_query_di_seq(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_DI_SEQ;
  
  *buf_frame++ = di_seq;
  *buf_frame++ = check_cs(buf_frame_+6,10);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to gprs
    send_server(buf_frame_,18);
  }else{
    //to 485
    Write_485_2(buf_frame_,18);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}


void ack_query_ack_action(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_ACK_ACTION;
  
  *buf_frame++ = ack_action;
  *buf_frame++ = check_cs(buf_frame_+6,10);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to gprs
    send_server(buf_frame_,18);
  }else{
    //to 485
    Write_485_2(buf_frame_,18);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}


void ack_query_protocol(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_PROTOCOL;
  
  *buf_frame++ = protocol;
  *buf_frame++ = check_cs(buf_frame_+6,10);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to gprs
    send_server(buf_frame_,18);
  }else{
    //to 485
    Write_485_2(buf_frame_,18);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}
