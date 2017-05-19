

#include "configs.h"


extern uint8_t ip[17];                 //the server ip
extern uint8_t port[8];  
extern uint8_t ip1;
extern uint8_t ip2;
extern uint8_t ip3;
extern uint8_t ip4;
extern uint16_t port_;
extern uint8_t device_test; //0x00~���Թ���~www.xcxdtech.com   0xFF~δ����~avenger0422.vicp.cc

void param_config(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint8_t ip_port_[6];
  
  uint16_t i = 0;
  uint16_t cjq_count = 0;
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_cjq_next = 0;   //cjq block ��ַɾ��ʱ  �Ȳ������һ���ĵ�ַ Ȼ����ɾ��
  uint32_t block_meter = 0;  //meter block ��ַ
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
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
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
    deviceaddr[4] = *(buf_frame + DATA_POSITION + 4);  //������Э���г��� Э���5λĬ��Ϊ0x00
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
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
        //ɾ����
        if(block_meter == 0xFFFFFF){
          //û������� do nothing
        }else{
          //�������  ɾ�������
          if(delete_meter(block_cjq,block_meter) == 0xFFFFFF){
            return;
          }
        }
        device_ack(desc,server_seq_);
      }
      
      if(*(buf_frame + DATA_POSITION + 16) == 0x01){
        //��ӱ�
        if(block_meter == 0xFFFFFF){
          //û������� ���
          if(add_meter(block_cjq,buf_frame + DATA_POSITION + 3) == 0xFFFFFF){
            return;
          }
        }else{
          //������� do nothing
        }
        device_ack(desc,server_seq_);
      }
    }
    
    break;
  case FN_CJQ:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //ɾ��ȫ���ɼ���  ����ռ������еĲɼ�������Ϣ
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
      //���
      if(search_cjq(buf_frame + DATA_POSITION + 1) == 0xFFFFFF){
        //�������ɼ���
        if(add_cjq(buf_frame + DATA_POSITION + 1) == 0xFFFFFF){
          return;
        }
      }else{
        //�Ѿ�������ɼ�����
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
      //the slave is �ɼ���
      slave_mbus = 0xBB;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //the slave is 485
      slave_mbus = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_METER_MBUS - sFLASH_CON_START_ADDR),&slave_mbus,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_DI_SEQ:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //ǧ��ͨʹ�õĴ��ģ��
      di_seq = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //Ĭ��
      di_seq = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_READMETER_DI_SEQ - sFLASH_CON_START_ADDR),&di_seq,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_ACK_ACTION:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //���շ��ر�ģʽ
      ack_action = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //Ĭ��
      ack_action = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
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
      //188Э��
      protocol = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
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
        //��ȡMUTEX������ ������...
        //return 0xFFFFFF;
        return;
      }
      //����Config Flash ��
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      di_seq = 0xFF;
      Mem_Copy(config_flash + (sFLASH_POOL_INIT - sFLASH_CON_START_ADDR),&di_seq,1);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
      
      device_ack(desc,server_seq_);
      Device_Cmd(DISABLE);
      *((uint8_t *)0) = 0x00;  //��ʹϵͳ����
    }
    break;
  case FN_RESET:
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      device_ack(desc,server_seq_);
      Device_Cmd(DISABLE);
      *((uint8_t *)0) = 0x00;  //��ʹϵͳ����
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
  
  meter_count++;  //�ɼ����µı�����++
  meter_all++;  //���б�����++
  
  block_new = GetFlash();  
  if(block_new == 0xFFFFFF){
    return 0xFFFFFF;
  }
  sFLASH_WritePage(meteraddr,block_new + 6,7);  //���ַ
  sFLASH_WritePage((uint8_t *)(meteraddr - 3),block_new + 13,1);  //������
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 14,4);  //�����  meter_read = 0
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 22,2);  //��״̬  meter_read = 0
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //��ȡMUTEX������ ������...
    return 0xFFFFFF;
  }
  
  if(block_last == 0xFFFFFF){
    //first meter
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    
    //�ɼ�����Ŀ�ʼ�ͽ�β��ָ������ӵı�Ŀ�
    Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);  //�ɼ����µ���Ŀ++
    
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
  }else{
    //���ɼ�����Ľ�βָ������ӵı�Ŀ�
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);  //�ɼ����µ���Ŀ++
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    //ԭ�����һ�������һ����ָ������ӵı�Ŀ�
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);  
    //����ӵı�Ŀ����һ���� ָ��ԭ�������һ����
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
    //��ȡMUTEX������ ������...
    return 0xFFFFFF;
  }
  
  if(meter_count == 0){
    //�ɼ�����Ψһ�ı�  block_before��block_after  ��Ϊ0xFFFFFF
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_after,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_before,3);
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    //����ȫ������Ŀ
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //Ҫɾ��������ǵ�һ��  ���������һ��
      if(block_before == 0xFFFFFF){
        //�޸ĺ�һ����before Ϊblock_before
        sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_after%0x1000 + 18,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //�޸Ĳɼ�����һ�����ַΪ  block_after  ���²ɼ�������Ŀ
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_after,3);
        Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //����ȫ������Ŀ
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      if(block_after == 0xFFFFFF){
        //�޸�ǰһ����next Ϊblock_after
        sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //�޸Ĳɼ��������һ�����ַΪ  block_before  ���²ɼ�������Ŀ
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_before,3);
        Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //����ȫ������Ŀ
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
    }else{
      //Ҫɾ����������м�
      //�޸�ǰһ����next Ϊblock_after
      sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseSector((block_before/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //�޸ĺ�һ����before Ϊblock_before
      sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_after%0x1000 + 18,(uint8_t *)&block_before,3);
      sFLASH_EraseSector((block_after/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //���²ɼ�������Ŀ
      sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
      sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //����ȫ������Ŀ
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    }
  }
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  return block_meter;
  
}

//�вɼ���  ���ش˲ɼ�����block��ַ   û���򷵻�0xFFFFFF
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
  
  cjq_count++;  //�ɼ�������++
  
  //��ȡһ��flash��  ��������Ӧ��Ϣ
  block_new = GetFlash();  
  sFLASH_WritePage(cjqaddr,block_new + 6,6);  //�ɼ�����ַ
  sFLASH_WritePage((uint8_t *)&meter_count,block_new + 18,2);  //�ɼ�������  (uint8_t *)0 �ǵ�ַ0x00000000����ֵ��
  //��һ�������һ���ָ����0xFFFFFF
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //��ȡMUTEX������ ������...
    return 0xFFFFFF;
  }
  sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
  
  if(block_last == 0xFFFFFF){
    //this is the first cjq
    //cjq Q �Ŀ�ʼ�ͽ�β��ָ������ӵĲɼ�����
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
  }else{
    //���ǵ�һ��
    //��cjq Q �Ľ�βָ������ӵĲɼ�����
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    //��ԭ�����һ���ɼ�������һ���ɼ���ָ������ӵĲɼ�����
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);
    //������ӵĲɼ������е���һ���ɼ���  ָ��ԭ�������һ���ɼ���
    sFLASH_WritePage((uint8_t *)&block_last,block_new + 20,3);  //��һ���ɼ���
  }
  
  sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  
  return block_new;
}

uint32_t delete_cjq(uint32_t block_cjq){
  uint16_t meter_count = 0;
  uint32_t block_meter = 0;
  uint32_t block_meter_next = 0;
  uint32_t block_after = 0;  //��һ���ɼ���
  uint32_t block_before = 0;  //��һ���ɼ���
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
  
  //������ɼ����µı�Q���  ���ɼ���ɾ��
  for(i = 0;i < meter_count;i++){
    
    sFLASH_ReadBuffer((uint8_t *)&block_meter_next,block_meter+3,3);  //��ȡ��һ�����block��ַ
    if(delete_meter(block_cjq,block_meter) == 0xFFFFFF){
      continue;
    }
    block_meter = block_meter_next;
  }
  //���ɼ���ɾ��
  PutFlash(block_cjq);
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //��ȡMUTEX������ ������...
    return 0xFFFFFF;
  }
  
  if(cjq_count == 0){
    //����ɼ�����Ψһ��һ��   block_before��block_after  ��Ϊ0xFFFFFF
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //Ҫɾ��������ǵ�һ��  ���������һ��
      if(block_before == 0xFFFFFF){
        //�޸ĺ�һ����before Ϊblock_before
        sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_after%0x1000 + 20,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //�޸Ĳɼ���Q��start Ϊblock_after  ���²ɼ�������
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
        Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      
      if(block_after == 0xFFFFFF){
        //�޸�ǰһ����next Ϊblock_after
        sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //�޸Ĳɼ���Q��end Ϊblock_before  ���²ɼ�������
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      
    }else{
      //Ҫɾ����������м�
      //�޸�ǰһ����next Ϊblock_after
      sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseSector((block_before/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //�޸ĺ�һ����before Ϊblock_before
      sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_after%0x1000 + 20,(uint8_t *)&block_before,3);
      sFLASH_EraseSector((block_after/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //�޸�control blocks
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
  uint8_t readingbeat[17];  //��ȫ����ʱ������
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
  uint16_t times_;      //һ��Ҫ���Ͷ���֡
  uint16_t times_count; //�����˶���֡��
  
  uint16_t len = 0;  //��ǰ֡�����ݳ���
  
  uint32_t block_cjq;
  uint32_t block_meter;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type;   //�������
  uint8_t meter_status; //���״̬
  uint8_t meter_fount = 0;  //�Ƿ��ҵ��������
  
  uint16_t i = 0;       //�����ɼ���
  uint16_t j = 0;       //�����ɼ����µı�
  uint8_t k = 0;        //����copy�ɼ��������ַ
  
  uint16_t meter_count = 0;  //����һ֡�е����������
  uint16_t meter_count_ = 0;    //����һ֡��������ĸ���
  uint8_t header = 0;   //һ֡��֡ͷ�Ƿ���׼��
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&allmeter_count,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //û�вɼ���������
    return;
  }
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  
  buf_frame_ = buf_frame;
  
  if(Mem_Cmp(meteraddr,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7) == DEF_YES){
    //ȫ����
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
            //��֡
            meter_count = allmeter_count;
            meter_count_ = meter_count;
            len = ((9+17*meter_count_+1) << 2) | 0x03;    //+1  because of  *buf_frame++ = metertype;
            
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              meter_count = 10;
              meter_count_ = meter_count;
              len = ((9+17*meter_count_+1) << 2) | 0x03;
              
            }else{
              if(times_count == times_){
                //β֡
                if(remain == 0){
                  meter_count = 10; 
                  meter_count_ = meter_count;
                }else{
                  meter_count = remain;
                  meter_count_ = meter_count;
                }
                len = ((9+17*meter_count_+1) << 2) | 0x03;
                
              }else{
                //�м�֡
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
            //��֡
            *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              *buf_frame++ = ZERO_BYTE |MUL_FIRST | server_seq_;
            }else{
              if(times_count == times_){
                //β֡
                *buf_frame++ = ZERO_BYTE |MUL_LAST | server_seq_;
              }else{
                //�м�֡
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
          header = 0;   //Ϊ��һ֡��׼��
          //������һ֡
          
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
    //������
    for(i = 0;meter_fount == 0 && i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        if(Mem_Cmp(meteraddr,meter_addr,7) == DEF_YES && meter_type == metertype){
          //�ҵ�������ˡ�����
          sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+21,1);
          meter_fount = 1; 
          break;
        }
        
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
    }
    
    if(meter_fount){
      //�ҵ��������
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
