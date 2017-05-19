

#include "os.h"
#include "stm32f10x_conf.h"
#include "tasks.h"
#include "lib_str.h"
#include "serial.h"
#include "spi_flash.h"
#include "gprs.h"
#include "frame.h"
#include "frame_188.h"
#include "readeg.h"
//#include "stdlib.h"

extern OS_MEM MEM_Buf;
extern OS_MEM MEM_ISR;

extern OS_Q Q_485_2;            //�ɼ��������͹���������
extern OS_Q Q_LORA;
extern OS_Q Q_Read;            //��������Queue
extern OS_Q Q_ReadData;        //���ͳ���ָ���  �²㷵�س�������
extern OS_Q Q_Config;         //��������Queue
extern OS_Q Q_Deal;         //������յ��ķ��������͹���������

extern OS_SEM SEM_HeartBeat;    //���շ���������Task to HeartBeat Task  ���յ������Ļ�Ӧ
extern OS_SEM SEM_ACKData;     //�����������ݵ�ACK
extern OS_SEM SEM_Send;      //got the '>'  we can send the data now  ���Է�������

extern OS_TMR TMR_CJQTIMEOUT;    //�򿪲ɼ���֮�� 20���ӳ�ʱ �Զ��ر�ͨ��

extern volatile uint8_t reading;
extern volatile uint8_t connectstate; 
extern uint8_t deviceaddr[5];

extern uint8_t slave_mbus; //0xaa mbus   0xff  485   0xBB~�ɼ���

extern uint8_t config_flash[];  //���ô���Flashʹ�õ�����  Sector==4K  ��Ҫһ��4K������
extern uint8_t *meterdata;  //ʹ�ú���Э�鳭��ʱ��ŷ��ص���Ϣ  ʹ��config_flash
extern OS_MUTEX MUTEX_CONFIGFLASH;    //�Ƿ����ʹ�� config_flash  4K ��������FLASH
extern uint8_t di_seq; //DI0 DI1 ˳��   0xAA~DI1��ǰ(ǧ��ͨ)   0xFF~DI0��ǰ(default)  
extern uint8_t ack_action;  //��Ӧ������~0xaa    �Ȳ�����Ӧ��~0xff
extern uint8_t protocol;  //Э������ 0xFF~188(Default)  1~EG 



extern uint8_t * volatile server_ptr;      //�ж��б���M590E ������������
extern uint8_t * volatile server_ptr_;     //��¼�жϵĿ�ʼָ��


uint8_t heart_seq = 0;  //��¼���������к� �ȴ�ack
uint8_t data_seq = 0;  //��¼���ݵ����к� �ȴ�ack

uint8_t local_seq = 0;  //�������к�
uint8_t server_seq = 0;  //�����������к�  ����ʱ  ��ͬ�������к�

uint8_t fe[4] = {0xFE,0xFE,0xFE,0xFE};  //����ʱǰ�淢�͵�4��0xFE




uint8_t readingall = 0;   //�Ƿ����ڳ�ȫ����
uint8_t readingall_progress = 0;  //���ڳ�ȫ����Ľ������

void Task_485_2(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  
  uint8_t * buf = 0;   //the buf used put the data in 
  uint8_t * buf_;       //keep the buf's ptr  used to release the buf
  uint8_t start_recv = 0;
  uint8_t frame_len = 0;      //the frame's length
  uint8_t data;         //the data get from the queue
  uint8_t * mem_ptr;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t header_count; //count 0x68 L L 0x68
  uint8_t header_ok;    //the header is received ok
  uint16_t len1;
  uint16_t len2;
    
  
  while(DEF_TRUE){
    //�յ�0x68֮��  ���200ms û���յ�����  ����Ϊ��ʱ��
    mem_ptr = OSQPend(&Q_485_2,200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    
    if(err == OS_ERR_TIMEOUT){
      if(start_recv == 1){
        buf = buf_;
        start_recv = 0;
        frame_len = 0;
      }
      continue;
    }
    
    data = *mem_ptr;
    OSMemPut(&MEM_ISR,mem_ptr,&err);
    
    if(buf == 0){
      buf = OSMemGet(&MEM_Buf,
                     &err);
      if(err == OS_ERR_NONE){
        //get the buf
        buf_ = buf;
        Mem_Set(buf_,0x00,256); //clear the buf
      }else{
        //didn't get the buf
        asm("NOP");
        continue;
      }
      
      
    }
  }
}


void Task_LORA(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  
  uint8_t * buf = 0;   //the buf used put the data in 
  uint8_t * buf_;       //keep the buf's ptr  used to release the buf
  uint8_t start_recv = 0;
  uint8_t frame_len = 0;      //the frame's length
  uint8_t data;         //the data get from the queue
  uint8_t * mem_ptr;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t header_count; //count 0x68 L L 0x68
  uint8_t header_ok;    //the header is received ok
  uint16_t len1;
  uint16_t len2;
    
  
  while(DEF_TRUE){
    //�յ�0x68֮��  ���200ms û���յ�����  ����Ϊ��ʱ��
    mem_ptr = OSQPend(&Q_LORA,200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    
    if(err == OS_ERR_TIMEOUT){
      if(start_recv == 1){
        buf = buf_;
        start_recv = 0;
        frame_len = 0;
      }
      continue;
    }
    
    data = *mem_ptr;
    OSMemPut(&MEM_ISR,mem_ptr,&err);
    
    if(buf == 0){
      buf = OSMemGet(&MEM_Buf,
                     &err);
      if(err == OS_ERR_NONE){
        //get the buf
        buf_ = buf;
        Mem_Set(buf_,0x00,256); //clear the buf
      }else{
        //didn't get the buf
        asm("NOP");
        continue;
      }
      
      
    }
  }
}





void Task_Server(void *p_arg){
  OS_ERR err;
  
  uint8_t * buf_server_task = 0;
  uint8_t * buf_server_task_ = 0;
  uint8_t getbuffail = 0;
  uint8_t connectfail = 0;
  
  while(DEF_TRUE){
    if(connectstate == 1){
      if(buf_server_task == 0){
        buf_server_task = OSMemGet(&MEM_Buf,
                       &err);
        
        if(err == OS_ERR_NONE){
          //get the buf
          getbuffail = 0;
          buf_server_task_ = buf_server_task;
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          Server_Post2Buf(buf_server_task_);
        }else{
          //didn't get the buf
          getbuffail ++;
          if(getbuffail >= 20){
            //20��û�л�ȡ��buf  
            Device_Cmd(DISABLE);
            *((uint8_t *)0) = 0x00;  //��ʹϵͳ����
          }
          continue;
        }
      }
      
      if(server_ptr - server_ptr_ > 0){
        OSTimeDly(4,
             OS_OPT_TIME_DLY,
             &err);
        
        buf_server_task = server_ptr;
        check_str(buf_server_task_,buf_server_task);  //���ε�����ǰ��0x00
        if(Str_Str(buf_server_task_,"\n>")){
          
          buf_server_task = buf_server_task_;
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          Server_Post2Buf(buf_server_task_);
          
          OSSemPost(&SEM_Send,
                    OS_OPT_POST_1,
                    &err);
          continue;
        }
        
        if(Str_Str(buf_server_task_,"RECEIVE")){
          //oh it's the data 
          OSQPost(&Q_Deal,
                  buf_server_task_,
                  buf_server_task-buf_server_task_,
                  OS_OPT_POST_FIFO,
                  &err);
          buf_server_task = 0;
          Server_Post2Buf(0);
          continue;
        }
        
        //CLOSED
        if(Str_Str(buf_server_task_,"CLOSE")){
          
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          OSMemPut(&MEM_Buf,buf_server_task_,&err);
          buf_server_task = 0;
          buf_server_task_ = 0;
          Server_Post2Buf(0);
          change_connect(0);
          continue;
        }
        //+PDP: DEACT\r\n
        if(Str_Str(buf_server_task_,"DEACT")){
          
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          OSMemPut(&MEM_Buf,buf_server_task_,&err);
          buf_server_task = 0;
          buf_server_task_ = 0;
          Server_Post2Buf(0);
          change_connect(0);
          continue;
        }
        
        if(Str_Str(buf_server_task_,"ERROR")){
          
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          OSMemPut(&MEM_Buf,buf_server_task_,&err);
          buf_server_task = 0;
          buf_server_task_ = 0;
          Server_Post2Buf(0);
          change_connect(0);
          continue;
        }
        
        //don't know what's that
        buf_server_task = buf_server_task_;
        Mem_Set(buf_server_task_,0x00,256); //clear the buf
        Server_Post2Buf(buf_server_task_);
        
      }else{
        OSTimeDly(4,
             OS_OPT_TIME_DLY,
             &err);
      }
    }else{
      
      if(buf_server_task != 0){
        Mem_Set(buf_server_task_,0x00,256); //clear the buf
        OSMemPut(&MEM_Buf,buf_server_task_,&err);
        
        buf_server_task = 0;
        buf_server_task_ = 0;
        
        Server_Post2Buf(0);
      }
      
      Device_Cmd(DISABLE);
      Device_Cmd(ENABLE);
      if(connect() == SUCCESS){
        connectfail = 0;
      }else{
        connectfail++;
        if(connectfail > 20){
          Device_Cmd(DISABLE);
          *((uint8_t *)0) = 0x00;  //��ʹϵͳ����
        }
      }
    }
  }
  
}



void Task_DealServer(void *p_arg){
  CPU_TS ts;
  OS_ERR err;
  uint8_t * buf_copy = 0;
  uint8_t * buf_ptr_ = 0;
  uint16_t msg_size = 0;
  
  uint8_t * start = 0;
  uint8_t server_seq_ = 0;
  uint16_t len = 0;
  
  while(DEF_TRUE){
    /*
    \r\n+TCPRECV:0,**,0x68 L L 0x68 C A     \r\n
    */
    buf_ptr_ = OSQPend(&Q_Deal,
            0,
            OS_OPT_PEND_BLOCKING,
            &msg_size,
            &ts,
            &err);
    
    start = Str_Str(buf_ptr_,"\r\n\x68") + 2;
    
    //check the frame
    len = check_frame(start);
    
      if(len){
        //the frame is ok
        switch(*(start+AFN_POSITION)){
        case AFN_ACK:
          //the ack of the server
            
          server_seq_ = *(start+SEQ_POSITION) & 0x0F;  //��ø�֡�����к�
          
          if(server_seq_ == heart_seq){
            OSSemPost(&SEM_HeartBeat,
                    OS_OPT_POST_1,
                    &err);
          }else{
            if(server_seq_ == data_seq){
              OSSemPost(&SEM_ACKData,
                        OS_OPT_POST_1,
                        &err);
            }else{
              //������Ӧ��֡
            }
          }
          break;
        case AFN_CONFIG:
        case AFN_QUERY:
          buf_copy = OSMemGet(&MEM_Buf,&err);
          if(buf_copy != 0){
            Mem_Copy(buf_copy,start,len);
            *(buf_copy + len) = 0x01;  //��ʶ��һ֡���Է�����
            OSQPost(&Q_Config,buf_copy,len,OS_OPT_POST_1,&err);
            if(err != OS_ERR_NONE){
              OSMemPut(&MEM_Buf,buf_copy,&err);
            }
          }
          break;
        case AFN_CONTROL:
        case AFN_CURRENT:
          
          server_seq_ = *(start + SEQ_POSITION) & 0x0F;
          if(*(start+FN_POSITION) == 0x05){
            //ƥ�����к�
            server_seq = server_seq_;
            device_ack(0x01,server_seq_);
          }else{
            if(server_seq != server_seq_){
              //�µĳ���ָ��  ack & read
              buf_copy = OSMemGet(&MEM_Buf,&err);
              if(buf_copy != 0){
                Mem_Copy(buf_copy,start,len);
                *(buf_copy + len) = 0x01;  //��ʶ��һ֡���Է�����
                server_seq = server_seq_;
                device_ack(0x01,server_seq_);
                OSQPost(&Q_Read,buf_copy,len,OS_OPT_POST_1,&err);
                if(err != OS_ERR_NONE){
                  OSMemPut(&MEM_Buf,buf_copy,&err);
                }
              }
            }else{
              device_ack(0x01,server_seq_);
            }
          }
          
          
          break;
        case AFN_LINK_TEST:
          //never will come to here
          break;
        case AFN_HISTORY:
          //don't support
          break;
          
        }
      }
    OSMemPut(&MEM_Buf,buf_ptr_,&err);
  }
}



void addSEQ(void){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  local_seq++;
  local_seq = local_seq & 0x0F;
  CPU_CRITICAL_EXIT();
}

void Task_HeartBeat(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * buf_frame;
  uint8_t heart_ack = 0;
  uint8_t i;
  uint8_t beat[17];
  
  while(DEF_TRUE){
    buf_frame = beat;
    *buf_frame++ = FRAME_HEAD;
    //buf_frame_16 = (uint16_t *)buf_frame;
    *buf_frame++ = 0x27;//(9 << 2) | 0x03;
    *buf_frame++ = 0x00;
    *buf_frame++ = 0x27;//(9 << 2) | 0x03;
    *buf_frame++ = 0x00;
    //buf_frame = (uint8_t *)buf_frame_16;
    *buf_frame++ = FRAME_HEAD;
    
    *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_START | START_FUN_TEST;
    /**/
    *buf_frame++ = deviceaddr[0];
    *buf_frame++ = deviceaddr[1];
    *buf_frame++ = deviceaddr[2];
    *buf_frame++ = deviceaddr[3];
    *buf_frame++ = deviceaddr[4];
    
    *buf_frame++ = AFN_LINK_TEST;
    *buf_frame++ = ZERO_BYTE |SINGLE | CONFIRM | local_seq;
    heart_seq = local_seq;
    addSEQ();
    *buf_frame++ = FN_HEARTBEAT;
    
    *buf_frame++ = check_cs(beat+6,9);
    *buf_frame++ = FRAME_END;
    if(connectstate){
      for(i = 0;connectstate && i < 3;i++){
        heart_ack = 0;
        if(send_server(beat,17)){
          OSSemPend(&SEM_HeartBeat,
                    5000,
                    OS_OPT_PEND_BLOCKING,
                    &ts,
                    &err);
          if(err == OS_ERR_NONE){
            heart_ack = 1;
            break;
          }
        }
      }
      if(heart_ack){
        OSTimeDly(120000,
                    OS_OPT_TIME_DLY,
                    &err);
      }else{
        change_connect(0);
      }
    }else{
      OSTimeDly(100,
                    OS_OPT_TIME_DLY,
                    &err);
    }
  }
}

void power_cmd(FunctionalState NewState){
  if(NewState != DISABLE){
    //�򿪵�Դ
  }
}

void Task_Read(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint16_t msg_size;
  uint8_t * buf_frame;
  
  while(DEF_TRUE){
    buf_frame = OSQPend(&Q_Read,
                        0,
                        OS_OPT_PEND_BLOCKING,
                        &msg_size,
                        &ts,
                        &err);
    switch(protocol){
    case 0xFF:
      //188
      switch(*(buf_frame+AFN_POSITION)){
        case AFN_CONTROL:
          power_cmd(ENABLE);
          meter_control(buf_frame,*(buf_frame+msg_size));
          power_cmd(DISABLE);
          break;
        case AFN_CURRENT:
          power_cmd(ENABLE);
          meter_read_188(buf_frame,*(buf_frame+msg_size));
          power_cmd(DISABLE);
          break;
      }
      break;
    case 0x01:
      //EG 
      power_cmd(ENABLE);
      meter_read_eg(buf_frame,*(buf_frame+msg_size));
      power_cmd(DISABLE);
      break;
    }
    
    
    
    OSMemPut(&MEM_Buf,buf_frame,&err);
  }
}

uint8_t mbus_power(FunctionalState NewState){
  /**/
  OS_ERR err;
  if(NewState != DISABLE){
    
    GPIO_SetBits(GPIOA,GPIO_Pin_0);
    OSTimeDly(1500,
                  OS_OPT_TIME_DLY,
                  &err);
  }else{
    GPIO_ResetBits(GPIOA,GPIO_Pin_0);
  }
  return 1;
}

/*
������Э���
*/
void meter_read_eg(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  //��ȡconfig_flash��ʹ��Ȩ
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //��ȡMUTEX������ ������...
    //return 0xFFFFFF;
    return;
  }
  meterdata = config_flash; //�����ص�������Ϣ�����config_flash
  Device_Read(ENABLE);
  switch (*(buf_frame + DATA_POSITION)){
    case 0xAA:
      meter_single_eg(buf_frame);
      send_data_eg(1,desc);
    break;
    case 0x00:
      meter_cjq_eg(buf_frame);
      if(meterdata[2] != 0xFF){
        send_data_eg(*(buf_frame + DATA_POSITION + 3),desc);
      }else{
        send_cjqtimeout_eg(desc);
      }
      
    break;
  }
  Device_Read(DISABLE);
  
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}

/*
����ɼ�����ַΪ FF FF FF FF FF FF ��ʾ����������ֱ�ӽӵı�

���������ʱ��
�ȷ��ʹ�ָ���ɼ���͸������  ���յ���Ӧ֮��
���Է��ͳ�������ָ��
ָ��ʱ����û���յ����صı��ָ��  �ظ�3��

�ص�ָ���ɼ�����͸������ 

*/
void meter_read_188(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_meter = 0;  //meter block ��ַ
  
  uint16_t cjq_count = 0;
  uint16_t cjqmeter_count = 0;
  
  uint16_t i = 0;
  uint16_t j = 0;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type = 0;
  
  uint8_t meter_fount = 0;
  //��ѯ�Ƿ��������
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //û�вɼ���������
    return;
  }
  
  Device_Read(ENABLE);
  if(Mem_Cmp(buf_frame+16,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7) == DEF_YES){
    //��ȫ����
    readingall = 1;
    /**/
    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      
      if(cjq_open(cjq_addr,block_cjq) == 0){
        //û�д򿪲ɼ���
        OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
      
        if(err != OS_ERR_NONE){
          //��ȡMUTEX������ ������...
          //return 0xFFFFFF;
          return;
        }
        for(j=0;j < cjqmeter_count;j++){
          sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
          *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x80;
          
          //�����úõ�Flash������д�뵽Flash�С�
          sFLASH_EraseSector((block_meter/0x1000)*0x1000);
          sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
          
          sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
        }
        
        OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
      }else{
        for(j=0;j < cjqmeter_count;j++){
          sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
          sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
          
          meter_read_single(meter_addr,block_meter,meter_type,desc);
          sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
        }
        cjq_close(cjq_addr,block_cjq);
      }
      
      readingall_progress = (i+1)/cjq_count;  //��ȫ����ʱ�����ϱ��Ľ���  ��ǰ�ڼ����ɼ���/�ɼ�������
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
    readingall = 0;
    meter_send(1,0,desc);
  }else{
    //��������
    for(i = 0;meter_fount == 0 && i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        if(Mem_Cmp(buf_frame + 16,meter_addr,7) == DEF_YES && meter_type == *(buf_frame + 15)){
          //�ҵ�������ˡ�����
          if(cjq_open(cjq_addr,block_cjq) == 0){
            //û�д򿪲ɼ���
            OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
          
            if(err != OS_ERR_NONE){
              //��ȡMUTEX������ ������...
              //return 0xFFFFFF;
              return;
            }
            sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
            *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x80;
            
            //�����úõ�Flash������д�뵽Flash�С�
            sFLASH_EraseSector((block_meter/0x1000)*0x1000);
            sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
            
            OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
          }else{
            meter_read_single(meter_addr,block_meter,meter_type,desc);
            //send the data;
            meter_send(0,block_meter,desc);
            cjq_close(cjq_addr,block_cjq);
          }
          
          meter_fount = 1; 
          break;
        }
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
  }
  Device_Read(DISABLE);
}

//ֻ�ܶ���
void meter_read_single(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc){
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[16];
    uint8_t read[4];
    uint8_t half[4];
    uint8_t i = 0;
    uint8_t j = 0;
    uint16_t msg_size = 0;
    uint8_t * buf_readdata = 0;
    uint8_t success = 0;
    uint8_t st_l = 0;
    uint8_t st_h = 0;
    uint8_t * buf_frame = 0;
    
    buf_frame = buf_frame_;
    *buf_frame++ = FRAME_HEAD;
    *buf_frame++ = meter_type;
    for(i=0;i<7;i++){
      *buf_frame++ = *(meter_addr + i);
    }
    *buf_frame++ = 0x01; //C
    *buf_frame++ = 0x03; //len
    if(di_seq == 0xFF){
      //Ĭ�ϵ�λ��ǰ
      *buf_frame++ = DATAFLAG_RD_L;
      *buf_frame++ = DATAFLAG_RD_H;
    }else{
      //ǧ��ͨʹ�õ�˳�򡣡������ʹ��
      *buf_frame++ = DATAFLAG_RD_H;
      *buf_frame++ = DATAFLAG_RD_L;
    }
    
    *buf_frame++ = 0x01;
    *buf_frame++ = check_cs(buf_frame_,11+3);
    *buf_frame++ = FRAME_END;
    
    for(i =0;i<4;i++){
      read[i] = 0x00;
      half[i] = 0x00;
    }
    
    for(i = 0;success == 0 && i < 1;i++){
      Slave_Write(fe,4);
      Slave_Write(buf_frame_,13+3);
      buf_readdata = OSQPend(&Q_ReadData,1200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        continue;
      }
      //���յ���ȷ������
      //�жϱ��ַ
      if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
        success = 1;
        //��ȡST
        st_l = *(buf_readdata + 31);
        st_h = *(buf_readdata + 32);
        for(j = 0;j < 4;j++){
          read[j] = *(buf_readdata + 14 + j);
          half[j] = *(buf_readdata + 19 + j);
        }
      }
      
      OSMemPut(&MEM_Buf,buf_readdata,&err);
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    
    sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    if(success == 0){
      //����ʧ��  ��flash  return nack
      *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x40;
    }else{
      Mem_Copy(config_flash+block_meter%0x1000 + 22,&st_l,1);  //��¼st��Ϣ
      Mem_Copy(config_flash+block_meter%0x1000 + 23,&st_h,1);  //��¼st��Ϣ
      Mem_Copy(config_flash+block_meter%0x1000 + 14,read,4);        //����
      Mem_Copy(config_flash+block_meter%0x1000 + 24,half,4);        //��λ
    }
    
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((block_meter/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}

//all = 1 ����ȫ����  all = 0 ���ͱ���Ӧ�ı�
void meter_send(uint8_t all,uint32_t block_meter_,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  uint16_t * buf_frame_16 = 0;
  uint16_t cjqmeter_count = 0;
  uint16_t allmeter_count = 0;
  uint16_t cjq_count = 0;
  
  uint16_t times = 0;
  uint8_t remain = 0;
  uint16_t times_ = 0;      //һ��Ҫ���Ͷ���֡
  uint16_t times_count = 0; //�����˶���֡��
  
  
  uint8_t meter_addr[7];
  uint8_t meter_read[4];
  uint32_t block_cjq = 0;
  uint32_t block_meter = 0;
  
  uint8_t meter_type = 0;   //�������
  uint8_t st_l = 0; //���״̬
  uint8_t st_h = 0; //���״̬
  
  uint16_t i = 0;       //�����ɼ���
  uint16_t j = 0;       //�����ɼ����µı�
  uint8_t k = 0;        //����copy�ɼ��������ַ
  
  uint16_t meter_count = 0;  //����һ֡�е����������
  uint16_t meter_count_ = 0;    //����һ֡��������ĸ���
  uint8_t header = 0;   //һ֡��֡ͷ�Ƿ���׼��
  
  uint16_t len = 0; 
  
  block_meter = block_meter_;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  
  buf_frame_ = buf_frame;
  
  if(all == 0){
    sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
    sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
    sFLASH_ReadBuffer((uint8_t *)&meter_read,block_meter+14,4);
    sFLASH_ReadBuffer((uint8_t *)&st_l,block_meter+22,1);
    sFLASH_ReadBuffer((uint8_t *)&st_h,block_meter+23,1);
    
    *buf_frame++ = FRAME_HEAD;
    buf_frame_16 = (uint16_t *)buf_frame;
    *buf_frame_16++ = 0x73;//((9+14+5) << 2) | 0x03;
    *buf_frame_16++ = 0x73;//((9+14+5) << 2) | 0x03;
    buf_frame = (uint8_t *)buf_frame_16;
    *buf_frame++ = FRAME_HEAD;
    
    *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    *buf_frame++ = deviceaddr[0];
    *buf_frame++ = deviceaddr[1];
    *buf_frame++ = deviceaddr[2];
    *buf_frame++ = deviceaddr[3];
    *buf_frame++ = deviceaddr[4];
    
    *buf_frame++ = AFN_CURRENT;
    *buf_frame++ = ZERO_BYTE |SINGLE | local_seq;
    *buf_frame++ = FN_CURRENT_METER;
    
    data_seq = local_seq;
    addSEQ();
    
    *buf_frame++ = 0x00;
    *buf_frame++ = 0x00;
    *buf_frame++ = 0x00;
    *buf_frame++ = 0x00;
    *buf_frame++ = meter_type;
    
    
    for(i=0;i<7;i++){
      *buf_frame++ = meter_addr[i];
    }
    *buf_frame++ = 0x01;
    for(i=0;i<4;i++){
      *buf_frame++ = meter_read[i];
    }
    *buf_frame++ = st_l;
    *buf_frame++ = st_h;
    
    *buf_frame++ = check_cs(buf_frame_+6,28);
    *buf_frame++ = FRAME_END;
    
    if(desc){
      //to m590e
      
      
      for(k = 0;k < 3;k++){
        send_server(buf_frame_,36);
        OSSemPend(&SEM_ACKData,
                  5000,
                  OS_OPT_PEND_BLOCKING,
                  &ts,
                  &err);
        if(err == OS_ERR_NONE){
          break;
        }
      }
    }else{
      //to 485
      Server_Write_485(buf_frame_,36);
    }
  }else{
    //ȫ����
    sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
    sFLASH_ReadBuffer((uint8_t *)&allmeter_count,sFLASH_METER_COUNT,2);
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
    
    times = allmeter_count/10;
    remain = allmeter_count%10;
    times_count = 0;
    times_ = times;
    if(remain > 0){
      times_ = times_ + 1;
    }
    
    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        sFLASH_ReadBuffer((uint8_t *)&meter_read,block_meter+14,4);
        sFLASH_ReadBuffer((uint8_t *)&st_l,block_meter+22,1);
        sFLASH_ReadBuffer((uint8_t *)&st_h,block_meter+23,1);
        
        if(header == 0){
          header = 1;
          times_count++;
          if(times_ == 1){
            //��֡
            meter_count = allmeter_count;
            meter_count_ = meter_count;
            
            len = ((9+14*meter_count_+5) << 2) | 0x03;    //+1  because of  *buf_frame++ = metertype;
            
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              meter_count = 10;
              meter_count_ = meter_count;
              
              len = ((9+14*meter_count_+5) << 2) | 0x03;
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
                
                len = ((9+14*meter_count_+5) << 2) | 0x03;
              }else{
                //�м�֡
                meter_count = 10;
                meter_count_ = meter_count;
                
                len = ((9+14*meter_count_+5) << 2) | 0x03;
              }
            }
          }
          
          *buf_frame++ = FRAME_HEAD;
          buf_frame_16 = (uint16_t *)buf_frame;
          *buf_frame_16++ = len;    //+1  because of  *buf_frame++ = metertype;
          *buf_frame_16++ = len;
          buf_frame = (uint8_t *)buf_frame_16;
          *buf_frame++ = FRAME_HEAD;
              
          *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
          /**/
          *buf_frame++ = deviceaddr[0];
          *buf_frame++ = deviceaddr[1];
          *buf_frame++ = deviceaddr[2];
          *buf_frame++ = deviceaddr[3];
          *buf_frame++ = deviceaddr[4];
          
          *buf_frame++ = AFN_CURRENT;
          if(times_ == 1){
            //��֡
            *buf_frame++ = ZERO_BYTE |SINGLE | CONFIRM| local_seq;
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              *buf_frame++ = ZERO_BYTE |MUL_FIRST | CONFIRM| local_seq;
            }else{
              if(times_count == times_){
                //β֡
                *buf_frame++ = ZERO_BYTE |MUL_LAST | CONFIRM| local_seq;
              }else{
                //�м�֡
                *buf_frame++ = ZERO_BYTE |MUL_MIDDLE | CONFIRM| local_seq;
              }
            }
          }
          data_seq = local_seq;
          addSEQ();
          
          *buf_frame++ = FN_CURRENT_METER;
          
          buf_frame_16 = (uint16_t *)buf_frame;
          *buf_frame_16++ = times_;    //�ܹ�����֡
          *buf_frame_16++ = times_count;  //�ڼ�֡
          buf_frame = (uint8_t *)buf_frame_16;
          *buf_frame++ = meter_type;
        }
        
        
        
        for(k=0;k<7;k++){
          *buf_frame++ = meter_addr[k];
        }
        *buf_frame++ = 0x01;
        for(k=0;k<4;k++){
          *buf_frame++ = meter_read[k];
        }
        *buf_frame++ = st_l;
        *buf_frame++ = st_h;
        
        meter_count--;
        if(meter_count == 0){
          header = 0;   //Ϊ��һ֡��׼��
          //������һ֡
          
          *buf_frame++ = check_cs(buf_frame_+6,9+14*meter_count_+5);
          *buf_frame++ = FRAME_END;
          
          
          if(desc){
            //to m590e
            for(k = 0;k < 3;k++){
              send_server(buf_frame_,17+14*meter_count_+5);
              OSSemPend(&SEM_ACKData,
                        5000,
                        OS_OPT_PEND_BLOCKING,
                        &ts,
                        &err);
              if(err == OS_ERR_NONE){
                break;
              }
            }
          }else{
            //to 485
            Server_Write_485(buf_frame_,17+14*meter_count_+5);
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
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
}

void meter_control(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_meter = 0;  //meter block ��ַ
  
  uint16_t cjq_count = 0;
  uint16_t cjqmeter_count = 0;
  
  uint16_t i = 0;
  uint16_t j = 0;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type = 0;
  
  uint8_t meter_fount = 0;
  uint8_t server_seq_ = 0;  
  //��ѯ�Ƿ��������
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //û�вɼ���������
    //todo ��Ӧ NACK
    return;
  }
  
  server_seq_ = *(buf_frame + SEQ_POSITION) & 0x0F;
  if(*(buf_frame + FN_POSITION) == FN_CLEAN){
    //send ack;
    device_ack(desc,server_seq_);
    meter_clean();
  }else{
    Device_Read(ENABLE);
    for(i = 0;meter_fount == 0 && i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        if(Mem_Cmp(buf_frame + 16,meter_addr,7) == DEF_YES && meter_type == *(buf_frame + 15)){
          //�ҵ�������ˡ�����
          
          if(cjq_open(cjq_addr,block_cjq)==0){
            //û�д򿪲ɼ���
              OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
            
              if(err != OS_ERR_NONE){
                //��ȡMUTEX������ ������...
                //return 0xFFFFFF;
                return;
              }
              sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
              *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x80;
              
              //�����úõ�Flash������д�뵽Flash�С�
              sFLASH_EraseSector((block_meter/0x1000)*0x1000);
              sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
              
              OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
          }else{
            switch(*(buf_frame + FN_POSITION)){
            case FN_OPEN:
              meter_open(meter_addr,block_meter,meter_type,desc,server_seq_);
              break;
            case FN_CLOSE:
              meter_close(meter_addr,block_meter,meter_type,desc,server_seq_);
              break;
            }
            
            cjq_close(cjq_addr,block_cjq);
          }
          meter_fount = 1; 
          break;
        }
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
    Device_Read(DISABLE);
  }
}

uint8_t cjq_isopen = 0;

uint8_t cjq_open(uint8_t * cjq_addr,uint32_t block_cjq){
    uint8_t buf_frame_[17];
    uint8_t * buf_frame = 0;
    uint8_t i;
    uint16_t msg_size;
    uint8_t * buf_readdata;
    uint8_t success = 0;
    uint8_t st_l;
    OS_ERR err;
    CPU_TS ts;
    
    if(slave_mbus == 0xAA){
      //mbus ~~~
      if(cjq_isopen == cjq_addr[0]){
        OSTmrStart(&TMR_CJQTIMEOUT,&err);
      }else{
        relay_1(DISABLE);
        relay_2(DISABLE);
        relay_3(DISABLE);
        relay_4(DISABLE);
        switch (cjq_addr[0]){
          case 1:
            relay_1(ENABLE);
            cjq_isopen = 1;
            break;
          case 2:
            relay_2(ENABLE);
            cjq_isopen = 2;
            break;
          case 3:
            relay_3(ENABLE);
            cjq_isopen = 3;
            break;
          case 4:
            relay_4(ENABLE);
            cjq_isopen = 4;
            break;
        }
        OSTmrStart(&TMR_CJQTIMEOUT,&err);
      }
      return 1;
    }
    
    if(slave_mbus == 0xBB){
      //�ɼ��� ~~~
      buf_frame = buf_frame_;
      *buf_frame++ = FRAME_HEAD;
      *buf_frame++ = 0xA0;  //�ɼ�����־
      for(i=0;i<6;i++){
        *buf_frame++ = cjq_addr[i];
      }
      *buf_frame++ = 0x00;  //�ɼ������λ
      *buf_frame++ = 0x04; //C
      *buf_frame++ = 0x04; //len
      *buf_frame++ = DATAFLAG_WC_L;
      *buf_frame++ = DATAFLAG_WC_H;
      *buf_frame++ = 0x01;
      *buf_frame++ = OPEN_CJQ;
      *buf_frame++ = check_cs(buf_frame_,11+4);
      *buf_frame++ = FRAME_END;
      
      for(i = 0;success == 0 && i < 2;i++){
        Slave_Write(buf_frame_,13+4);
        buf_readdata = OSQPend(&Q_ReadData,5000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
        if(err != OS_ERR_NONE){
          continue;
        }
        //���յ���ȷ������
        //�жϱ��ַ
        if(Mem_Cmp(cjq_addr,buf_readdata+2,6) == DEF_YES){
          //��ȡST
          st_l = *(buf_readdata + 14);
          if((st_l & 0x03) == 0x00){
            //opened  return ack
            success = 1;
          }else{
            //���ɼ���ʧ��  ��flash  return nack
            success = 0;
          }
        }
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
      if(success == 0){
        //���ɼ���ʧ��  ��flash  return nack
        OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
      
        if(err != OS_ERR_NONE){
          //��ȡMUTEX������ ������...
          //return 0xFFFFFF;
          return success;
        }
        
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        //����ʧ��  ��flash  return nack
        *(config_flash+block_cjq%0x1000 + 23) = 0x01;
        
        //�����úõ�Flash������д�뵽Flash�С�
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        
        OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
      }
      return success;
    }
    return 1; //����ײ�ֱ�ӽ���485��188Э��ı�  ֱ�ӷ���1
}

uint8_t cjq_close(uint8_t * cjq_addr,uint32_t block_cjq){
    uint8_t buf_frame_[17];
    uint8_t * buf_frame = 0;
    uint8_t i;
    uint16_t msg_size;
    uint8_t * buf_readdata;
    uint8_t success = 0;
    uint8_t st_l;
    OS_ERR err;
    CPU_TS ts;
    if(slave_mbus == 0xAA){
      //mbus ~~~
      relay_1(DISABLE);
      relay_2(DISABLE);
      relay_3(DISABLE);
      relay_4(DISABLE);
      cjq_isopen = 0;
      OSTmrStop(&TMR_CJQTIMEOUT,OS_OPT_TMR_NONE,0,&err);
      return 1;
    }
    if(slave_mbus == 0xBB){
      //�ɼ��� ~~~
      buf_frame = buf_frame_;
      *buf_frame++ = FRAME_HEAD;
      *buf_frame++ = 0xA0;  //�ɼ�����־
      for(i=0;i<6;i++){
        *buf_frame++ = cjq_addr[i];
      }
      *buf_frame++ = 0x00;  //�ɼ������λ
      *buf_frame++ = 0x04; //C
      *buf_frame++ = 0x04; //len
      *buf_frame++ = DATAFLAG_WC_L;
      *buf_frame++ = DATAFLAG_WC_H;
      *buf_frame++ = 0x01;
      *buf_frame++ = CLOSE_CJQ;
      *buf_frame++ = check_cs(buf_frame_,11+4);
      *buf_frame++ = FRAME_END;
      
      for(i = 0;success == 0 && i < 2;i++){
        Slave_Write(buf_frame_,13+4);
        buf_readdata = OSQPend(&Q_ReadData,4000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
        if(err != OS_ERR_NONE){
          continue;
        }
        //���յ���ȷ������
        //�жϱ��ַ
        if(Mem_Cmp(cjq_addr,buf_readdata+2,6) == DEF_YES){
          //��ȡST
          st_l = *(buf_readdata + 14);
          if((st_l & 0x03) == 0x02){
            //closed  return ack
            success = 1;
          }else{
            //���ɼ���ʧ��  ��flash  return nack
            success = 0;
          }
        }
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
    }
    return success;
}

void cjq_timeout(void *p_tmr,void *p_arg){
  OS_ERR err;
  //�رյ�Դ
  power_cmd(DISABLE);
  //�ر�ͨ��
  cjq_isopen = 0;
  OSTmrStop(&TMR_CJQTIMEOUT,OS_OPT_TMR_NONE,0,&err);
}


void meter_open(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_){
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[17];
    uint8_t i = 0;
    uint16_t msg_size = 0;
    uint8_t * buf_readdata = 0;
    uint8_t success = 0;
    uint8_t st_l = 0;
    uint8_t st_h = 0;
    uint8_t * buf_frame = 0;
    
    buf_frame = buf_frame_;
    *buf_frame++ = FRAME_HEAD;
    *buf_frame++ = meter_type;
    for(i=0;i<7;i++){
      *buf_frame++ = *(meter_addr + i);
    }
    *buf_frame++ = 0x04; //C
    *buf_frame++ = 0x04; //len
    
    if(di_seq == 0xFF){
      //Ĭ�ϵ�λ��ǰ
      *buf_frame++ = DATAFLAG_WV_L;
      *buf_frame++ = DATAFLAG_WV_H;
    }else{
      // ���շ��ر�ʹ��  �е�
      *buf_frame++ = DATAFLAG_WV_H;
      *buf_frame++ = DATAFLAG_WV_L;
    }
    
    *buf_frame++ = 0x01;
    *buf_frame++ = OPEN_VALVE;
    *buf_frame++ = check_cs(buf_frame_,11+4);
    *buf_frame++ = FRAME_END;
    
    for(i = 0;success == 0 && i < 1;i++){
      Slave_Write(fe,4);
      Slave_Write(buf_frame_,13+4);
      buf_readdata = OSQPend(&Q_ReadData,15000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        continue;
      }
      //���յ���ȷ������
      if(ack_action != 0xff){
        OSMemPut(&MEM_Buf,buf_readdata,&err);
        OSTimeDly(12000,
                  OS_OPT_TIME_DLY,
                  &err);
        success = 1;
      }else{
        //�жϱ��ַ
        if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
          //��ȡST
          st_l = *(buf_readdata + 14);
          st_h = *(buf_readdata + 15);
          if((st_l & 0x03) == 0x00){
            //opened  return ack
            success = 1;
          }else{
            //����ʧ��  ��flash  return nack
            success = 0;
          }
        }
        
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    if(success == 0){
      //����ʧ��  ��flash  return nack
      //Mem_Copy(configflash + 22,"\x43",1);  //��ʱ
      if((st_l & 0x03) == 0x03){
        //���ŷ����쳣
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }else{
        //��ʱ
        *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x40;
      }
      //device_nack(desc,server_seq_);
    }else{
      //��
      if(ack_action != 0xff){
        *(config_flash+block_meter%0x1000 + 22) = 0x00;
        *(config_flash+block_meter%0x1000 + 23) = 0x00;
      }else{
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }
      device_ack(desc,server_seq_);   //return nack;
    }
    
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((block_meter/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
}

void meter_close(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_){
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[17];
    uint8_t i = 0;
    uint16_t msg_size = 0;
    uint8_t * buf_readdata = 0;
    uint8_t success = 0;
    uint8_t st_l = 0;
    uint8_t st_h = 0;
    uint8_t * buf_frame = 0;  
  
    buf_frame = buf_frame_;
    *buf_frame++ = FRAME_HEAD;
    *buf_frame++ = meter_type;
    for(i=0;i<7;i++){
      *buf_frame++ = meter_addr[i];
    }
    *buf_frame++ = 0x04; //C
    *buf_frame++ = 0x04; //len
    
    if(di_seq == 0xFF){
      //Ĭ�ϵ�λ��ǰ
      *buf_frame++ = DATAFLAG_WV_L;
      *buf_frame++ = DATAFLAG_WV_H;
    }else{
      // ���շ��ر�ʹ��  �е�
      *buf_frame++ = DATAFLAG_WV_H;
      *buf_frame++ = DATAFLAG_WV_L;
    }
    
    *buf_frame++ = 0x01;
    *buf_frame++ = CLOSE_VALVE;
    *buf_frame++ = check_cs(buf_frame_,11+4);
    *buf_frame++ = FRAME_END;
    
    for(i = 0;success == 0 && i < 1;i++){
      Slave_Write(fe,4);
      Slave_Write(buf_frame_,13+4);
      buf_readdata = OSQPend(&Q_ReadData,15000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        continue;
      }
      //���յ���ȷ������
      if(ack_action != 0xff){
        OSMemPut(&MEM_Buf,buf_readdata,&err);
        OSTimeDly(12000,
                  OS_OPT_TIME_DLY,
                  &err);
        success = 1;
      }else{
        //�жϱ��ַ
        if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
          //��ȡST
          st_l = *(buf_readdata + 14);
          st_h = *(buf_readdata + 15);
          //���ڹط���ST�е����   D0 D1��ֻҪ��һ��Ϊ1����  
          //me��Э����ָ����D1 = 1
          //���գ�D0 = 1 Ϊ��
          if((st_l & 0x03) == 0x03){
            //����ʧ��  ��flash  return nack
            success = 0;
          }else{
            if((st_l & 0x02) == 0x02 || (st_l & 0x01) == 0x01){
              //opened  return ack
              success = 1;
            }else{
              //����ʧ��  ��flash  return nack
              success = 0;
            }
          }
        }
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
    }
    
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    
    if(success == 0){
      //����ʧ��  ��flash  return nack
      //Mem_Copy(configflash + 22,"\x43",1);  //��ʱ
      if((st_l & 0x03) == 0x03){
        //����ʧ��  ��flash  return nack
        //���Ż�
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }else{
        //��ʱ
        *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x40;
      }
      //device_nack(desc,server_seq_);  //return nack;
    }else{
      //�ط�
      if(ack_action != 0xff){
        *(config_flash+block_meter%0x1000 + 22) = 0x01;
        *(config_flash+block_meter%0x1000 + 23) = 0x00;
      }else{
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }
      device_ack(desc,server_seq_);   //return ack;
    }
    
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((block_meter/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}

/*
���ȶ����������Flash����ȡ��  ��ȡ���ŵ�ǰ״̬

*/
void meter_clean(void){
    
  OS_ERR err;
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_meter = 0;  //meter block ��ַ
  
  uint16_t cjq_count = 0;
  uint16_t cjqmeter_count = 0;
  
  uint16_t i = 0;
  uint16_t j = 0;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type = 0;
  
  uint8_t buf_frame_[17];
  uint8_t z = 0;
  uint8_t * buf_frame = 0; 
  
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //û�вɼ���������
    return;
  }
  
  Device_Read(ENABLE);
  for(i = 0;i < cjq_count;i++){
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
    sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
    
    if(cjq_open(cjq_addr,block_cjq) == 0){
      //û�д򿪲ɼ���
      //do nothing;
    }else{
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        //meter_read_single(meter_addr,block_meter,meter_type,desc);
        buf_frame = buf_frame_;
        *buf_frame++ = FRAME_HEAD;
        *buf_frame++ = meter_type;
        for(z=0;z<7;z++){
          *buf_frame++ = meter_addr[z];
        }
        *buf_frame++ = 0x04; //C
        *buf_frame++ = 0x04; //len
        
        if(di_seq == 0xFF){
          //Ĭ�ϵ�λ��ǰ
          *buf_frame++ = DATAFLAG_WV_L;
          *buf_frame++ = DATAFLAG_WV_H;
        }else{
          // ���շ��ر�ʹ��  �е�
          *buf_frame++ = DATAFLAG_WV_H;
          *buf_frame++ = DATAFLAG_WV_L;
        }
        
        *buf_frame++ = 0x01;
        *buf_frame++ = CLEAN_VALVE;
        *buf_frame++ = check_cs(buf_frame_,11+4);
        *buf_frame++ = FRAME_END;
        
        Slave_Write(fe,4);
        Slave_Write(buf_frame_,13+4);
        
        //��ȡ��һ����
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
        
        //��ʱ12s֮��ִ����һ��������
        OSTimeDly(12000,
                  OS_OPT_TIME_DLY,
                  &err);
      }
      cjq_close(cjq_addr,block_cjq);
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
  }
  Device_Read(DISABLE);
  
}

//config and query the parameter
void Task_Config(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint16_t msg_size;
  uint8_t * buf_frame;
  
  while(DEF_TRUE){
    buf_frame = OSQPend(&Q_Config,
                        0,
                        OS_OPT_PEND_BLOCKING,
                        &msg_size,
                        &ts,
                        &err);
    switch(*(buf_frame+AFN_POSITION)){
      case AFN_CONFIG:
        param_config(buf_frame,*(buf_frame+msg_size));
        break;
      case AFN_QUERY:
        param_query(buf_frame,*(buf_frame+msg_size));
        break;
    }
    OSMemPut(&MEM_Buf,buf_frame,&err);
  }
}
