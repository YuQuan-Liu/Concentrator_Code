

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

extern OS_Q Q_485_2;            //采集器、表发送过来的数据
extern OS_Q Q_LORA;
extern OS_Q Q_Read;            //抄表任务Queue
extern OS_Q Q_ReadData;        //发送抄表指令后  下层返回抄表数据
extern OS_Q Q_Config;         //配置任务Queue
extern OS_Q Q_Deal;         //处理接收到的服务器发送过来的数据

extern OS_SEM SEM_LORA_OK;
extern OS_SEM SEM_HeartBeat;    //接收服务器数据Task to HeartBeat Task  接收到心跳的回应
extern OS_SEM SEM_ACKData;     //服务器对数据的ACK
extern OS_SEM SEM_Send;      //got the '>'  we can send the data now  可以发送数据

extern OS_TMR TMR_CJQTIMEOUT;    //打开采集器之后 20分钟超时 自动关闭通道

extern volatile uint8_t reading;
extern volatile uint8_t connectstate; 
extern volatile uint8_t lora_send;
extern uint8_t deviceaddr[5];
extern uint8_t cjqaddr[5];

extern uint8_t slave_mbus; //0xaa mbus   0xff  485   0xBB~采集器

extern uint8_t config_flash[];  //配置处理Flash使用的数组  Sector==4K  需要一个4K的数组
extern uint8_t *meterdata;  //使用海大协议抄表时存放返回的信息  使用config_flash
extern OS_MUTEX MUTEX_CONFIGFLASH;    //是否可以使用 config_flash  4K 数组配置FLASH
extern uint8_t di_seq; //DI0 DI1 顺序   0xAA~DI1在前(千宝通)   0xFF~DI0在前(default)  
extern uint8_t ack_action;  //先应答后操作~0xaa    先操作后应答~0xff
extern uint8_t protocol;  //协议类型 0xFF~188(Default)  1~EG 

extern OS_MUTEX MUTEX_SENDLORA;

extern uint8_t * volatile server_ptr;      //中断中保存GPRS 返回来的数据
extern uint8_t * volatile server_ptr_;     //记录中断的开始指针


uint8_t heart_seq = 0;  //记录心跳的序列号 等待ack
uint8_t data_seq = 0;  //记录数据的序列号 等待ack


uint8_t server_seq = 0;  //服务器端序列号  抄表时  会同步此序列号

uint8_t fe[4] = {0xFE,0xFE,0xFE,0xFE};  //抄表时前面发送的4个0xFE




uint8_t readingall = 0;   //是否正在抄全部表
uint8_t readingall_progress = 0;  //正在抄全部表的进度情况

/**
 * 处理485-2接收到的指令
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
    //收到0x68之后  如果200ms 没有收到数据  就认为超时了
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
                    *buf = 0x00;//标识这一帧数据是来自485的
                    OSQPost(&Q_Config,
                            buf_,frame_len,
                            OS_OPT_POST_FIFO,
                            &err);
                    break;
                  case AFN_CONTROL:
                  case AFN_CURRENT:
                    *buf = 0x00;//标识这一帧数据是来自485的
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
 * 处理LORA接收到的指令
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
    //收到0x68之后  如果200ms 没有收到数据  就认为超时了
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
                lora_send=0;
                *buf = 0x02;//标识这一帧数据是来自LORA的
                OSQPost(&Q_Deal,
                        buf_,frame_len,
                        OS_OPT_POST_FIFO,
                        &err);
                
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
              lora_send=0;
              //get the OK 
              OSSemPost(&SEM_LORA_OK,
                        OS_OPT_POST_1,
                        &err);
            }
            //重新开始接收
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
            if(*(buf-1) == check_eor(buf_+1,frame_len-2)){
              //check the frame cs
              //the frame is ok ,send the frame to 485_2
              lora_send=1;
              ack_lora_rssi(*(buf-2));
              //Write_485_2(buf_,frame_len);
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
 * 处理集中器发送过来的指令
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
  uint8_t desc = 0;
  
  while(DEF_TRUE){
    /*
    0x68 L L 0x68 C A     \r\n
    */
    buf_ptr_ = OSQPend(&Q_Deal,
            0,
            OS_OPT_PEND_BLOCKING,
            &msg_size,
            &ts,
            &err);
    
    start = buf_ptr_;
    
    //check the frame
    len = check_frame(start);
    
    //首先判断是不是找自己的
    //如果是找自己 判断自己是否在抄表  
    //如果不在抄表  去抄表
    //如果在抄表   直接返回ACK
    if(len){
      desc = *(buf_ptr_ + len);
      //判断集中器地址
      if(Mem_Cmp(deviceaddr,buf_ptr_+ADDR_POSITION,5) == DEF_YES){
        //集中器地址正确
        //判断采集器地址
        
        server_seq_ = *(start+SEQ_POSITION) & 0x0F;  //获得该帧的序列号
        switch(*(start+AFN_POSITION)){
        case AFN_ACK:
          //the ack of the Concentrator
          if(cjqaddr[0] == *(buf_ptr_+DATA_POSITION+2) && cjqaddr[1] == *(buf_ptr_+DATA_POSITION+1)){
            if(server_seq_ == data_seq){
              OSSemPost(&SEM_ACKData,
                        OS_OPT_POST_1,
                        &err);
            }else{
              //抛弃此应答帧
            }
          }
          break;
        case AFN_CONTROL:
        case AFN_CURRENT:
          if(cjqaddr[0] == *(buf_ptr_+DATA_POSITION+2) && cjqaddr[1] == *(buf_ptr_+DATA_POSITION+1)){
            if(reading){
              //在抄表   直接返回ACK
              device_ack_lora(desc,server_seq_);
            }else{
              //不在抄表  去抄表
              buf_copy = OSMemGet(&MEM_Buf,&err);
              if(buf_copy != 0){
                Mem_Copy(buf_copy,start,len);
                *(buf_copy + len) = desc;  //标识这一帧来自LORA
                OSQPost(&Q_Read,buf_copy,len,OS_OPT_POST_1,&err);
                if(err != OS_ERR_NONE){
                  OSMemPut(&MEM_Buf,buf_copy,&err);
                }
                device_ack_lora(desc,server_seq_);
              }
            }
          }else{
            //不是找我的
          }
          break;
        }
        
      }else{
        //集中器地址不对  do nothing 
      }
    }
    OSMemPut(&MEM_Buf,buf_ptr_,&err);
  }
}



/**
 * 定时检测LORA模块是否OK
 */
void Task_LORA_Check(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t inat = 0;
  uint8_t outat = 0;
  uint8_t i;
  
  while(DEF_TRUE){
    //3min  每3分钟检测一次
    OSTimeDly(120000,
              OS_OPT_TIME_DLY,
              &err);
    if(reading){
      continue;
    }
    OSMutexPend(&MUTEX_SENDLORA,3000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      continue;
    }
    //尝试3次
    for(i = 0; i < 3;i++){
      inat = 0;
      outat = 0;
      Write_LORA("+++",3); //进入AT模式
      OSSemPend(&SEM_LORA_OK,
                1000,
                OS_OPT_PEND_BLOCKING,
                &ts,
                &err);
      if(err == OS_ERR_NONE){
        inat = 1;
      }
      
      if(inat){
        Write_LORA("AT+ESC\r\n",8);  //离开AT模式
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
    OSMutexPost(&MUTEX_SENDLORA,OS_OPT_POST_NONE,&err);
    //检测失败
    if(inat == 0 || outat == 0){
      //restart LORA
      PWR_LORA_OFF();
      OSTimeDly(10000,
                OS_OPT_TIME_DLY,
                &err);
      PWR_LORA_ON();
    }
  }
}

/**
 * 每隔3s发送一条内容为TEST的测试指令供采集器测试信号使用
 * LED3
 */
void Task_LORA_Send(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  
  while(DEF_TRUE){
    
    if(lora_send){
      LED3_ON();
      OSTimeDly(100,
                OS_OPT_TIME_DLY,
                &err);
      LED3_OFF();
      OSTimeDly(100,
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
 * 具体的抄表任务
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
    Device_Read(ENABLE);
    switch(protocol){
    case 0xFF:
      //188
      break;
    case 0x01:
      //EG  
      PWR_485_ON();
      OSTimeDly(1000,
                OS_OPT_TIME_DLY,
                &err);
      meter_read_eg(buf_frame,*(buf_frame+msg_size));
      PWR_485_OFF();
      break;
    }
    Device_Read(DISABLE);
    OSMemPut(&MEM_Buf,buf_frame,&err);
  }
}


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
 * 抄表指示  188抄全部表时  定时往服务器发送Fake帧
 */
void Task_LED(void *p_arg){
  OS_ERR err;
  uint8_t cnt = 0;
  uint8_t readingbeat[17];  //抄全部表时的心跳
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
    }
  }
}
