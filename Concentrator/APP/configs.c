

#include "configs.h"


extern uint8_t ip[17];                 //the server ip
extern uint8_t port[8];  
extern uint8_t ip1;
extern uint8_t ip2;
extern uint8_t ip3;
extern uint8_t ip4;
extern uint16_t port_;
extern uint8_t device_test; //0x00~测试过了~www.xcxdtech.com   0xFF~未测试~avenger0422.vicp.cc

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
    block_cjq = search_cjq(buf_frame + DATA_POSITION + 10);
    block_meter = search_meter(block_cjq,buf_frame + DATA_POSITION + 3);
    if(block_cjq != 0xFFFFFF){
      if(*(buf_frame + DATA_POSITION + 16) == 0x00){
        //删除表
        if(block_meter == 0xFFFFFF){
          //没有这个表 do nothing
        }else{
          //有这个表  删除这个表
          if(delete_meter(block_cjq,block_meter) == 0xFFFFFF){
            return;
          }
        }
        device_ack(desc,server_seq_);
      }
      
      if(*(buf_frame + DATA_POSITION + 16) == 0x01){
        //添加表
        if(block_meter == 0xFFFFFF){
          //没有这个表 添加
          if(add_meter(block_cjq,buf_frame + DATA_POSITION + 3) == 0xFFFFFF){
            return;
          }
        }else{
          //有这个表 do nothing
        }
        device_ack(desc,server_seq_);
      }
    }
    
    break;
  case FN_CJQ:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //删除全部采集器  即清空集中器中的采集器表信息
      sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
      
      for(i = 0;i < cjq_count;i++){
        sFLASH_ReadBuffer((uint8_t *)&block_cjq_next,block_cjq+3,3);
        
        if(delete_cjq(block_cjq) == 0xFFFFFF){
          return;
        }
        block_cjq = block_cjq_next;
      }
      device_ack(desc,server_seq_);
    }
    
    if(*(buf_frame + DATA_POSITION) == 0x55){
      //添加
      if(search_cjq(buf_frame + DATA_POSITION + 1) == 0xFFFFFF){
        //添加这个采集器
        if(add_cjq(buf_frame + DATA_POSITION + 1) == 0xFFFFFF){
          return;
        }
      }else{
        //已经有这个采集器了
      }
      device_ack(desc,server_seq_);
    }
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
  }
  
}

uint32_t search_meter(uint32_t block_cjq,uint8_t * meteraddr){
  uint32_t block_current = 0;
  uint16_t meter_count = 0;
  uint8_t meter_addr[7];
  uint16_t i;
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&block_current,block_cjq+12,3);
  
  for(i = 0;i < meter_count;i++){
    sFLASH_ReadBuffer(meter_addr,block_current+6,7);
    if(Mem_Cmp(meteraddr,meter_addr,7) == DEF_YES){
      
      return block_current;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+3,3);
  }
  return 0xFFFFFF;
}

uint32_t add_meter(uint32_t block_cjq,uint8_t * meteraddr){
  uint32_t block_last = 0;
  uint32_t block_new = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  uint32_t meter_read = 0;
  OS_ERR err;
  CPU_TS ts;
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,block_cjq+15,3);
  
  meter_count++;  //采集器下的表数量++
  meter_all++;  //所有表数量++
  
  block_new = GetFlash();  
  if(block_new == 0xFFFFFF){
    return 0xFFFFFF;
  }
  sFLASH_WritePage(meteraddr,block_new + 6,7);  //表地址
  sFLASH_WritePage((uint8_t *)(meteraddr - 3),block_new + 13,1);  //表类型
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 14,4);  //表读数  meter_read = 0
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 22,2);  //表状态  meter_read = 0
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    return 0xFFFFFF;
  }
  
  if(block_last == 0xFFFFFF){
    //first meter
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
    
    //采集器表的开始和结尾都指向新添加的表的块
    Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);  //采集器下的数目++
    
    //将配置好的Flash块重新写入到Flash中。
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
  }else{
    //将采集器表的结尾指向新添加的表的块
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);  //采集器下的数目++
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    //原来最后一个表的下一个表指向新添加的表的块
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);  
    //新添加的表的块的上一个表 指向原来的最后一个表
    sFLASH_WritePage((uint8_t *)&block_last,block_new + 18,3);
    
  }
  
  sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
  Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
  sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  return block_new;
}

uint32_t delete_meter(uint32_t block_cjq,uint32_t block_meter){
  uint32_t block_after = 0;
  uint32_t block_before = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  OS_ERR err;
  CPU_TS ts;
  
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_after,block_meter+3,3);
  sFLASH_ReadBuffer((uint8_t *)&block_before,block_meter+18,3);
  
  meter_count--;
  meter_all--;
  
  PutFlash(block_meter);
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    return 0xFFFFFF;
  }
  
  if(meter_count == 0){
    //采集器下唯一的表  block_before、block_after  都为0xFFFFFF
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
    Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_after,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_before,3);
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    //更新全部表数目
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //要删除的这个是第一个  或者是最后一个
      if(block_before == 0xFFFFFF){
        //修改后一个的before 为block_before
        sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_after%0x1000 + 18,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //修改采集器第一个表地址为  block_after  更新采集器表数目
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_after,3);
        Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //更新全部表数目
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      if(block_after == 0xFFFFFF){
        //修改前一个的next 为block_after
        sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //修改采集器的最后一个表地址为  block_before  更新采集器表数目
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_before,3);
        Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //更新全部表数目
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
    }else{
      //要删除的这个在中间
      //修改前一个的next 为block_after
      sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseSector((block_before/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //修改后一个的before 为block_before
      sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_after%0x1000 + 18,(uint8_t *)&block_before,3);
      sFLASH_EraseSector((block_after/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //更新采集器表数目
      sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
      sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //更新全部表数目
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    }
  }
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  return block_meter;
  
}

//有采集器  返回此采集器的block地址   没有则返回0xFFFFFF
uint32_t search_cjq(uint8_t * cjqaddr){
  uint32_t block_current = 0;
  uint16_t cjq_count;
  uint8_t cjq_addr[6];
  
  uint16_t i;
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_current,sFLASH_CJQ_Q_START,3);
  
  for(i = 0;i < cjq_count;i++){
    
    sFLASH_ReadBuffer(cjq_addr,block_current+6,6);
    if(Mem_Cmp(cjqaddr,cjq_addr,6) == DEF_YES){
      
      return block_current;
    }
    
    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+3,3);
  }
  return 0xFFFFFF;
}

uint32_t add_cjq(uint8_t * cjqaddr){
  
  uint32_t block_last = 0;
  uint32_t block_new = 0;
  uint16_t cjq_count = 0;
  uint16_t meter_count = 0;
  OS_ERR err;
  CPU_TS ts;
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,sFLASH_CJQ_Q_LAST,3);
  
  cjq_count++;  //采集器数量++
  
  //获取一个flash块  并配置相应信息
  block_new = GetFlash();  
  sFLASH_WritePage(cjqaddr,block_new + 6,6);  //采集器地址
  sFLASH_WritePage((uint8_t *)&meter_count,block_new + 18,2);  //采集器表数  (uint8_t *)0 是地址0x00000000处的值。
  //第一块表和最后一块表都指向了0xFFFFFF
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    return 0xFFFFFF;
  }
  sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
  
  if(block_last == 0xFFFFFF){
    //this is the first cjq
    //cjq Q 的开始和结尾都指向新添加的采集器块
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
  }else{
    //不是第一个
    //将cjq Q 的结尾指向新添加的采集器块
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    //将原来最后一个采集器的下一个采集器指向新添加的采集器块
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);
    //将新添加的采集器块中的上一个采集器  指向原来的最后一个采集器
    sFLASH_WritePage((uint8_t *)&block_last,block_new + 20,3);  //上一个采集器
  }
  
  sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  
  return block_new;
}

uint32_t delete_cjq(uint32_t block_cjq){
  uint16_t meter_count = 0;
  uint32_t block_meter = 0;
  uint32_t block_meter_next = 0;
  uint32_t block_after = 0;  //下一个采集器
  uint32_t block_before = 0;  //上一个采集器
  uint16_t cjq_count = 0;
  uint16_t i = 0;
  OS_ERR err;
  CPU_TS ts;
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
  sFLASH_ReadBuffer((uint8_t *)&block_after,block_cjq+3,3);
  sFLASH_ReadBuffer((uint8_t *)&block_before,block_cjq+20,3);
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  
  cjq_count--;
  
  //将这个采集器下的表Q清空  将采集器删除
  for(i = 0;i < meter_count;i++){
    
    sFLASH_ReadBuffer((uint8_t *)&block_meter_next,block_meter+3,3);  //获取下一个表的block地址
    if(delete_meter(block_cjq,block_meter) == 0xFFFFFF){
      continue;
    }
    block_meter = block_meter_next;
  }
  //将采集器删除
  PutFlash(block_cjq);
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    return 0xFFFFFF;
  }
  
  if(cjq_count == 0){
    //这个采集器是唯一的一个   block_before、block_after  都为0xFFFFFF
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //要删除的这个是第一个  或者是最后一个
      if(block_before == 0xFFFFFF){
        //修改后一个的before 为block_before
        sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_after%0x1000 + 20,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //修改采集器Q的start 为block_after  更新采集器数量
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
        Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      
      if(block_after == 0xFFFFFF){
        //修改前一个的next 为block_after
        sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //修改采集器Q的end 为block_before  更新采集器数量
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      
    }else{
      //要删除的这个在中间
      //修改前一个的next 为block_after
      sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseSector((block_before/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //修改后一个的before 为block_before
      sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_after%0x1000 + 20,(uint8_t *)&block_before,3);
      sFLASH_EraseSector((block_after/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //修改control blocks
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    }
  }
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  return block_cjq;
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
    ack_query_meter(*(buf_frame + DATA_POSITION),buf_frame + DATA_POSITION + 1,desc,server_seq_);
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
  }
}

void Task_LED(void *p_arg){
  OS_ERR err;
  uint8_t cnt = 0;
  uint8_t readingbeat[17];  //抄全部表时的心跳
  uint8_t *buf_frame = 0;
  
  while(DEF_TRUE){
    //LED2
    if(reading == 0){
      GPIO_SetBits(GPIOB,GPIO_Pin_8);
      OSTimeDly(1000,
                    OS_OPT_TIME_DLY,
                    &err);
      GPIO_ResetBits(GPIOB,GPIO_Pin_8);
      OSTimeDly(1000,
                    OS_OPT_TIME_DLY,
                    &err);
      cnt = 0;
    }else{
      GPIO_SetBits(GPIOB,GPIO_Pin_8);
      OSTimeDly(100,
                    OS_OPT_TIME_DLY,
                    &err);
      GPIO_ResetBits(GPIOB,GPIO_Pin_8);
      OSTimeDly(100,
                    OS_OPT_TIME_DLY,
                    &err);
      if(readingall){
        cnt++;
        if(cnt >= 15){
          cnt = 0;
          buf_frame = readingbeat;
          *buf_frame++ = FRAME_HEAD;
          //buf_frame_16 = (uint16_t *)buf_frame;
          *buf_frame++ = 0x27;//(9 << 2) | 0x03;
          *buf_frame++ = 0x00;
          *buf_frame++ = 0x27;//(9 << 2) | 0x03;
          *buf_frame++ = 0x00;
          //buf_frame = (uint8_t *)buf_frame_16;
          *buf_frame++ = FRAME_HEAD;
          
          *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_START | START_FUN_REQ1;
          /**/
          *buf_frame++ = deviceaddr[0];
          *buf_frame++ = deviceaddr[1];
          *buf_frame++ = deviceaddr[2];
          *buf_frame++ = deviceaddr[3];
          *buf_frame++ = deviceaddr[4];
          
          *buf_frame++ = AFN_FAKE;
          *buf_frame++ = ZERO_BYTE |SINGLE | local_seq;
          *buf_frame++ = readingall_progress;//FN_HEARTBEAT;
          
          *buf_frame++ = check_cs(readingbeat+6,9);
          *buf_frame++ = FRAME_END;
          send_server(readingbeat,17);
        }
      }
    }
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
    //to m590e
    send_server(ack,17);
  }else{
    //to 485
    Server_Write_485(ack,17);
  }
  
}

void ack_query_cjq(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame;
  uint8_t * buf_frame_;
  uint16_t * buf_frame_16;
  
  uint16_t cjq_count;
  uint32_t block_cjq;
  uint16_t i;
  uint8_t cjq_addr[6];
  uint8_t j;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  
  *buf_frame++ = FRAME_HEAD;
  buf_frame_16 = (uint16_t *)buf_frame;
  *buf_frame_16++ = ((9+cjq_count*6) << 2) | 0x03;
  *buf_frame_16++ = ((9+cjq_count*6) << 2) | 0x03;
  buf_frame = (uint8_t *)buf_frame_16;
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
  *buf_frame++ = FN_CJQ;
  
  for(i = 0;i < cjq_count;i++){
    sFLASH_ReadBuffer(cjq_addr,block_cjq+6,6);
    for(j = 0;j < 6;j++){
      *buf_frame++ = cjq_addr[j];
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
  }
  
  *buf_frame++ = check_cs(buf_frame_+6,9+cjq_count*6);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,17+cjq_count*6);
  }else{
    //to 485
    Server_Write_485(buf_frame_,17+cjq_count*6);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
  
}

void ack_query_meter(uint8_t metertype,uint8_t * meteraddr,uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame;
  uint8_t * buf_frame_;
  uint16_t * buf_frame_16;
  uint16_t cjqmeter_count;
  uint16_t allmeter_count;
  uint16_t cjq_count;
  
  uint16_t times;
  uint8_t remain;
  uint16_t times_;      //一共要发送多少帧
  uint16_t times_count; //发送了多少帧了
  
  uint16_t len = 0;  //当前帧的数据长度
  
  uint32_t block_cjq;
  uint32_t block_meter;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type;   //表的类型
  uint8_t meter_status; //表的状态
  uint8_t meter_fount = 0;  //是否找到了这个表
  
  uint16_t i = 0;       //计数采集器
  uint16_t j = 0;       //计数采集器下的表
  uint8_t k = 0;        //计数copy采集器、表地址
  
  uint16_t meter_count = 0;  //计数一帧中的数据体个数
  uint16_t meter_count_ = 0;    //保持一帧中数据体的个数
  uint8_t header = 0;   //一帧的帧头是否已准备
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&allmeter_count,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //没有采集器。。。
    return;
  }
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  
  buf_frame_ = buf_frame;
  
  if(Mem_Cmp(meteraddr,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7) == DEF_YES){
    //全部表
    times = allmeter_count/10;
    remain = allmeter_count%10;
    times_count = 0;
    times_ = times;
    if(remain > 0){
      times_ = times_ + 1;
    }
    
    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+21,1);
        
        if(header == 0){
          header = 1;
          times_count++;
          *buf_frame++ = FRAME_HEAD;
          if(times_ == 1){
            //单帧
            meter_count = allmeter_count;
            meter_count_ = meter_count;
            len = ((9+17*meter_count_+1) << 2) | 0x03;    //+1  because of  *buf_frame++ = metertype;
            
          }else{
            //多帧
            if(times_count == 1){
              //首帧
              meter_count = 10;
              meter_count_ = meter_count;
              len = ((9+17*meter_count_+1) << 2) | 0x03;
              
            }else{
              if(times_count == times_){
                //尾帧
                if(remain == 0){
                  meter_count = 10; 
                  meter_count_ = meter_count;
                }else{
                  meter_count = remain;
                  meter_count_ = meter_count;
                }
                len = ((9+17*meter_count_+1) << 2) | 0x03;
                
              }else{
                //中间帧
                meter_count = 10;
                meter_count_ = meter_count;
                len = ((9+17*meter_count_+1) << 2) | 0x03;
                
              }
            }
          }
          
          buf_frame_16 = (uint16_t *)buf_frame;
          *buf_frame_16++ = len;
          *buf_frame_16++ = len;
          buf_frame = (uint8_t *)buf_frame_16;
          *buf_frame++ = FRAME_HEAD;
          
          *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
          *buf_frame++ = deviceaddr[0];
          *buf_frame++ = deviceaddr[1];
          *buf_frame++ = deviceaddr[2];
          *buf_frame++ = deviceaddr[3];
          *buf_frame++ = deviceaddr[4];
              
          *buf_frame++ = AFN_QUERY;
          
          if(times_ == 1){
            //单帧
            *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
          }else{
            //多帧
            if(times_count == 1){
              //首帧
              *buf_frame++ = ZERO_BYTE |MUL_FIRST | server_seq_;
            }else{
              if(times_count == times_){
                //尾帧
                *buf_frame++ = ZERO_BYTE |MUL_LAST | server_seq_;
              }else{
                //中间帧
               *buf_frame++ = ZERO_BYTE |MUL_MIDDLE | server_seq_;
              }
            }
          }
          
          *buf_frame++ = FN_METER;
          *buf_frame++ = metertype;
        }
        
        *buf_frame++ = 0x00;
        *buf_frame++ = 0x00;
        for(k=0;k<7;k++){
          *buf_frame++ = meter_addr[k];
        }
        for(k=0;k<6;k++){
          *buf_frame++ = cjq_addr[k];
        }
        *buf_frame++ = 0x00;
        *buf_frame++ = 0x01;
        
        meter_count--;
        if(meter_count == 0){
          header = 0;   //为下一帧做准备
          //发送这一帧
          
          *buf_frame++ = check_cs(buf_frame_+6,9+17*meter_count_+1);
          *buf_frame++ = FRAME_END;
          
          if(desc){
            //to m590e
            send_server(buf_frame_,17+17*meter_count_+1);
          }else{
            //to 485
            Server_Write_485(buf_frame_,17+17*meter_count_+1);
          }
          OSTimeDly(100,
                    OS_OPT_TIME_DLY,
                    &err);
          
          buf_frame = buf_frame_;
        }
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
  }else{
    //单个表
    for(i = 0;meter_fount == 0 && i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        if(Mem_Cmp(meteraddr,meter_addr,7) == DEF_YES && meter_type == metertype){
          //找到这个表了。。。
          sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+21,1);
          meter_fount = 1; 
          break;
        }
        
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
    }
    
    if(meter_fount){
      //找到这个表了
      *buf_frame++ = FRAME_HEAD;
      *buf_frame++ = 0x6F;//((9+17+1) << 2) | 0x03;
      *buf_frame++ = 0x00;
      *buf_frame++ = 0x6F;//((9+17+1) << 2) | 0x03;
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
      *buf_frame++ = FN_METER;
      
      *buf_frame++ = metertype;
      *buf_frame++ = 0x00;
      *buf_frame++ = 0x00;
      for(i=0;i<7;i++){
        *buf_frame++ = meter_addr[i];
      }
      for(i=0;i<6;i++){
        *buf_frame++ = cjq_addr[i];
      }
      *buf_frame++ = 0x00;
      *buf_frame++ = 0x01;
      
      *buf_frame++ = check_cs(buf_frame_+6,27);
      *buf_frame++ = FRAME_END;
      
      if(desc){
        //to m590e
        send_server(buf_frame_,35);
      }else{
        //to 485
        Server_Write_485(buf_frame_,35);
      }
    }
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
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
    //to m590e
    send_server(buf_frame_,17);
  }else{
    //to 485
    Server_Write_485(buf_frame_,17);
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
    //to m590e
    send_server(buf_frame_,23);
  }else{
    //to 485
    Server_Write_485(buf_frame_,23);
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
    //to m590e
    send_server(buf_frame_,18);
  }else{
    //to 485
    Server_Write_485(buf_frame_,18);
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
    //to m590e
    send_server(buf_frame_,18);
  }else{
    //to 485
    Server_Write_485(buf_frame_,18);
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
    //to m590e
    send_server(buf_frame_,18);
  }else{
    //to 485
    Server_Write_485(buf_frame_,18);
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
    //to m590e
    send_server(buf_frame_,18);
  }else{
    //to 485
    Server_Write_485(buf_frame_,18);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}
