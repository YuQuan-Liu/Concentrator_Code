

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
#include "utils.h"
#include "configs.h"
#include "bsp.h"
//#include "stdlib.h"

extern OS_MEM MEM_Buf;
extern OS_MEM MEM_ISR;

extern OS_Q Q_485_2;            //�ɼ��������͹���������
extern OS_Q Q_LORA;
extern OS_Q Q_Read;            //��������Queue
extern OS_Q Q_ReadData;        //���ͳ���ָ���  �²㷵�س�������
extern OS_Q Q_Config;         //��������Queue
extern OS_Q Q_Deal;         //������յ��ķ��������͹���������

extern OS_SEM SEM_LORA_OK;
extern OS_SEM SEM_HeartBeat;    //���շ���������Task to HeartBeat Task  ���յ������Ļ�Ӧ
extern OS_SEM SEM_ACKData;     //�����������ݵ�ACK
extern OS_SEM SEM_Send;      //got the '>'  we can send the data now  ���Է�������

extern OS_TMR TMR_CJQTIMEOUT;    //�򿪲ɼ���֮�� 20���ӳ�ʱ �Զ��ر�ͨ��

extern volatile uint8_t reading;
extern volatile uint8_t connectstate; 
extern volatile uint8_t lora_send;
extern uint8_t deviceaddr[5];

extern uint8_t slave_mbus; //0xaa mbus   0xff  485   0xBB~�ɼ���

extern uint8_t config_flash[];  //���ô���Flashʹ�õ�����  Sector==4K  ��Ҫһ��4K������
extern uint8_t *meterdata;  //ʹ�ú���Э�鳭��ʱ��ŷ��ص���Ϣ  ʹ��config_flash
extern OS_MUTEX MUTEX_CONFIGFLASH;    //�Ƿ����ʹ�� config_flash  4K ��������FLASH
extern uint8_t di_seq; //DI0 DI1 ˳��   0xAA~DI1��ǰ(ǧ��ͨ)   0xFF~DI0��ǰ(default)  
extern uint8_t ack_action;  //��Ӧ������~0xaa    �Ȳ�����Ӧ��~0xff
extern uint8_t protocol;  //Э������ 0xFF~188(Default)  1~EG 



extern uint8_t * volatile server_ptr;      //�ж��б���GPRS ������������
extern uint8_t * volatile server_ptr_;     //��¼�жϵĿ�ʼָ��


uint8_t heart_seq = 0;  //��¼���������к� �ȴ�ack
uint8_t data_seq = 0;  //��¼���ݵ����к� �ȴ�ack


uint8_t server_seq = 0;  //�����������к�  ����ʱ  ��ͬ�������к�

uint8_t fe[4] = {0xFE,0xFE,0xFE,0xFE};  //����ʱǰ�淢�͵�4��0xFE




uint8_t readingall = 0;   //�Ƿ����ڳ�ȫ����
uint8_t readingall_progress = 0;  //���ڳ�ȫ����Ľ������

/**
 * ����485-2���յ���ָ��
 */
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
    
    if(reading){
      //it is the frame come from the meter
      switch(protocol){
      case 0xFF:
        //188
        /*
        if(start_recv == 0){
          if(data == 0x68){
            *buf++ = data;
            frame_len = 0;
            start_recv = 1;
          }
        }else{
          *buf++ = data;
          if((buf-buf_) == 11){
            frame_len = *(buf_+10)+13;
          }
          if(frame_len > 0 && (buf-buf_) >= frame_len){
            //if it is the end of the frame
            if(*(buf-1) == 0x16){
              //check the frame cs
              if(*(buf-2) == check_cs(buf_,frame_len-2)){
                //the frame is ok;
                OSQPost(&Q_ReadData,
                        buf_,
                        frame_len,
                        OS_OPT_POST_FIFO,
                        &err);
                if(err == OS_ERR_NONE){
                  buf_ = 0;
                  buf = 0;
                  start_recv = 0;
                  frame_len = 0;
                }else{
                  buf = buf_;
                  start_recv = 0;
                  frame_len = 0;
                }
              }else{
                buf = buf_;
                start_recv = 0;
                frame_len = 0;
              }
            }else{
              buf = buf_;
              start_recv = 0;
              frame_len = 0;
            }
          }
        }*/
        break;
      case 0x01:
        if(start_recv == 0){
          if(data == 0x0E){
            *buf++ = data;
            frame_len = 0;
            start_recv = 1;
          }
        }else{
          *buf++ = data;
          if((buf-buf_) > 8){
            if(*(buf_+2) == 0x0B){
              //the slave is meter
              //post to the reading_q
              frame_len = 9;
              if(check_eor(buf_,9) == 0x00){
                //the frame is ok;
                OSQPost(&Q_ReadData,
                        buf_,
                        frame_len,
                        OS_OPT_POST_FIFO,
                        &err);
                if(err == OS_ERR_NONE){
                  buf_ = 0;
                  buf = 0;
                  start_recv = 0;
                  frame_len = 0;
                }else{
                  buf = buf_;
                  start_recv = 0;
                  frame_len = 0;
                }
              }else{
                buf = buf_;
                start_recv = 0;
                frame_len = 0;
              }
            }
          }
        }
        break;
      }
      
      
    }else{
      //it is the frame come from programmer
      if(start_recv == 0){
        if(data == 0x68){
          *buf++ = data;
          frame_len = 0;
          header_count = 1;
          header_ok = 0;
          start_recv = 1;
        }
      }else{
        *buf++ = data;
        if(header_ok == 0){
          header_count++;
          if(header_count == 6){
            if(*(buf_+5) ==0x68){
              len1 = *(uint16_t *)(buf_+1);
              len2 = *(uint16_t *)(buf_+3);
              if(len1 == len2){
                header_ok = 1;
                frame_len = len1 >> 2;
                frame_len += 8;
              }else{
                //the frame is error
                start_recv = 0;
                frame_len = 0;
                header_count = 0;
                header_ok = 0;
                buf = buf_;
              }
            }else{
              //the frame is error
              start_recv = 0;
              frame_len = 0;
              header_count = 0;
              header_ok = 0;
              buf = buf_;
            }
          }
        }else{
          if(frame_len > 0 && (buf-buf_) >= frame_len){
            //if it is the end of the frame
            if(*(buf-1) == 0x16){
              //check the frame cs
              if(*(buf-2) == check_cs(buf_+6,frame_len-8)){
                //the frame is ok;
                switch(*(buf_+AFN_POSITION)){
                  case AFN_CONFIG:
                  case AFN_QUERY:
                    *buf = 0x00;//��ʶ��һ֡����������485��
                    OSQPost(&Q_Config,
                            buf_,frame_len,
                            OS_OPT_POST_FIFO,
                            &err);
                    break;
                  case AFN_CONTROL:
                  case AFN_CURRENT:
                    *buf = 0x00;//��ʶ��һ֡����������485��
                    OSQPost(&Q_Read,
                            buf_,frame_len,
                            OS_OPT_POST_FIFO,
                            &err);
                    break;
                }
                if(err == OS_ERR_NONE){
                  buf_ = 0;
                  buf = 0;
                  start_recv = 0;
                  frame_len = 0;
                  header_count = 0;
                  header_ok = 0;
                }else{
                  buf = buf_;
                  start_recv = 0;
                  frame_len = 0;
                  header_count = 0;
                  header_ok = 0;
                }
              }else{
                buf = buf_;
                start_recv = 0;
                frame_len = 0;
                header_count = 0;
                header_ok = 0;
              }
            }else{
              buf = buf_;
              start_recv = 0;
              frame_len = 0;
              header_count = 0;
              header_ok = 0;
            }
          }
        }
      }
    }
    
  }
}

/**
 * ����LORA���յ���ָ��
 */
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
  uint8_t lora_model;  //1~tran 2~api 3~ok
  
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
    
    
    //it is the frame come from LORA
    if(start_recv == 0){
      switch(data){
      case 0x68:
        lora_model = 1;
        *buf++ = data;
        frame_len = 0;
        header_count = 1;
        header_ok = 0;
        start_recv = 1;
        break;
      case 0xFE:
        lora_model = 3;
        *buf++ = data;
        frame_len = 0;
        header_count = 1;
        header_ok = 0;
        start_recv = 1;
        break;
      case 0x0D:
        lora_model = 2;
        *buf++ = data;
        frame_len = 0;
        header_count = 1;
        header_ok = 0;
        start_recv = 1;
        break;
      };
    }else{
      *buf++ = data;
      
      switch(lora_model){
      case 1:
        if(header_ok == 0){
          header_count++;
          if(header_count == 6){
            if(*(buf_+5) ==0x68){
              len1 = *(uint16_t *)(buf_+1);
              len2 = *(uint16_t *)(buf_+3);
              if(len1 == len2){
                header_ok = 1;
                frame_len = len1 >> 2;
                frame_len += 8;
              }else{
                //the frame is error
                start_recv = 0;
                frame_len = 0;
                header_count = 0;
                header_ok = 0;
                buf = buf_;
              }
            }else{
              //the frame is error
              start_recv = 0;
              frame_len = 0;
              header_count = 0;
              header_ok = 0;
              buf = buf_;
            }
          }
        }else{
          if(frame_len > 0 && (buf-buf_) >= frame_len){
            //if it is the end of the frame
            if(*(buf-1) == 0x16){
              //check the frame cs
              if(*(buf-2) == check_cs(buf_+6,frame_len-8)){
                //the frame is ok;
                switch(*(buf_+AFN_POSITION)){
                case AFN_CONTROL:
                case AFN_CURRENT:
                  *buf = 0x02;//��ʶ��һ֡����������LORA��
                  OSQPost(&Q_Read,
                          buf_,frame_len,
                          OS_OPT_POST_FIFO,
                          &err);
                  break;
                }
                if(err == OS_ERR_NONE){
                  buf_ = 0;
                  buf = 0;
                  start_recv = 0;
                  frame_len = 0;
                  header_count = 0;
                  header_ok = 0;
                }else{
                  buf = buf_;
                  start_recv = 0;
                  frame_len = 0;
                  header_count = 0;
                  header_ok = 0;
                }
              }else{
                buf = buf_;
                start_recv = 0;
                frame_len = 0;
                header_count = 0;
                header_ok = 0;
              }
            }else{
              buf = buf_;
              start_recv = 0;
              frame_len = 0;
              header_count = 0;
              header_ok = 0;
            }
          }
        }
        break;
      case 2:
        header_count++;
        if(header_count == 6){
          if(*(buf_+5) ==0x0A){
            if(*(buf_+2) ==0x4F && *(buf_+3) ==0x4B){
              //get the OK 
              OSSemPost(&SEM_LORA_OK,
                        OS_OPT_POST_1,
                        &err);
            }
            //���¿�ʼ����
            start_recv = 0;
            frame_len = 0;
            header_count = 0;
            header_ok = 0;
            buf = buf_;
            
          }else{
            //the frame is error
            start_recv = 0;
            frame_len = 0;
            header_count = 0;
            header_ok = 0;
            buf = buf_;
          }
        }
        break;
      case 3:
        if(header_ok == 0){
          header_count++;
          if(header_count == 4){
            if(*(buf_+3) ==0x7F){
              frame_len = *(buf_+1) + 5;
              header_ok = 1;
            }else{
              //the frame is error
              start_recv = 0;
              frame_len = 0;
              header_count = 0;
              header_ok = 0;
              buf = buf_;
            }
          }
        }else{
          if(frame_len > 0 && (buf-buf_) >= frame_len){
            //if it is the end of the frame
            if(*(buf-1) == check_cs(buf_+4,frame_len-5)){
              //check the frame cs
              //the frame is ok ,send the frame to 485_2
              Write_485_2(buf_,frame_len);
              buf = buf_;
              start_recv = 0;
              frame_len = 0;
              header_count = 0;
              header_ok = 0;
            }else{
              buf = buf_;
              start_recv = 0;
              frame_len = 0;
              header_count = 0;
              header_ok = 0;
            }
          }
        }
        break;
      }
      
    }
  }
}




/**
 * ���ӷ����� ����GPRSģ�鷢�͹�����ָ��
 */
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



/**
 * ������������͹�����ָ��
 */
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


/**
 * GPRS ��������
 */
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
    
    heart_seq = addSEQ();
    *buf_frame++ = ZERO_BYTE |SINGLE | CONFIRM | heart_seq;
    
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


/**
 * ��ʱ���LORAģ���Ƿ�OK
 */
void Task_LORA_Check(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t inat = 0;
  uint8_t outat = 0;
  uint8_t i;
  
  while(DEF_TRUE){
    //3min  ÿ3���Ӽ��һ��
    OSTimeDly(120000,
              OS_OPT_TIME_DLY,
              &err);
    
    //����3��
    for(i = 0; i < 3;i++){
      inat = 0;
      outat = 0;
      Write_LORA("+++",3); //����ATģʽ
      OSSemPend(&SEM_LORA_OK,
                1000,
                OS_OPT_PEND_BLOCKING,
                &ts,
                &err);
      if(err == OS_ERR_NONE){
        inat = 1;
      }
      
      if(inat){
        Write_LORA("AT+ESC\r\n",8);  //�뿪ATģʽ
        OSSemPend(&SEM_LORA_OK,
                  1000,
                  OS_OPT_PEND_BLOCKING,
                  &ts,
                  &err);
        if(err == OS_ERR_NONE){
          outat = 1;
        }
      }
      
      if(inat == 1 && outat == 1){
        break;
      }else{
        continue;
      }
    }
    //���ʧ��
    if(inat == 0 || outat == 0){
      //restart LORA
      PWR_LORA_OFF();
      OSTimeDly(1000,
                OS_OPT_TIME_DLY,
                &err);
      PWR_LORA_ON();
    }
  }
}

/**
 * ÿ��3s����һ������ΪTEST�Ĳ���ָ��ɼ��������ź�ʹ��
 * LED3
 */
void Task_LORA_Send(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  
  while(DEF_TRUE){
    
    if(lora_send){
      Write_LORA("TEST",4); //��ʱ���͹��ɼ��������ź�ʹ��
      OSTimeDly(3000,
                OS_OPT_TIME_DLY,
                &err);
      LED3_ON();
      OSTimeDly(500,
                OS_OPT_TIME_DLY,
                &err);
      LED3_OFF();
      OSTimeDly(500,
                OS_OPT_TIME_DLY,
                &err);
    }else{
      LED3_ON();
      OSTimeDly(1000,
                OS_OPT_TIME_DLY,
                &err);
      LED3_OFF();
      OSTimeDly(1000,
                OS_OPT_TIME_DLY,
                &err);
    }
  }
}

/**
 * ����ĳ�������
 */
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
      /*
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
      }*/
      break;
    case 0x01:
      //EG  TODO
      //power_cmd(ENABLE);
      //meter_read_eg(buf_frame,*(buf_frame+msg_size));
      //power_cmd(DISABLE);
      break;
    }
    
    
    
    OSMemPut(&MEM_Buf,buf_frame,&err);
  }
}


/**
 * ������Э���
 */
/*
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
*/

/**
 * config and query the parameter
 */
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


/**
 * LED2
 * ����ָʾ  188��ȫ����ʱ  ��ʱ������������Fake֡
 */
void Task_LED(void *p_arg){
  OS_ERR err;
  uint8_t cnt = 0;
  uint8_t readingbeat[17];  //��ȫ����ʱ������
  uint8_t *buf_frame = 0;
  
  while(DEF_TRUE){
    //LED2
    if(reading == 0){
      LED2_ON();
      OSTimeDly(1000,
                OS_OPT_TIME_DLY,
                &err);
      LED2_OFF();
      OSTimeDly(1000,
                OS_OPT_TIME_DLY,
                &err);
      cnt = 0;
    }else{
      LED2_ON();
      OSTimeDly(100,
                OS_OPT_TIME_DLY,
                &err);
      LED2_OFF();
      OSTimeDly(100,
                OS_OPT_TIME_DLY,
                &err);
      /*
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
      }*/
    }
  }
}
