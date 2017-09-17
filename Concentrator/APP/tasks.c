

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


extern OS_Q Q_485_2;            //�ɼ��������͹���������
extern OS_Q Q_LORA;
extern OS_Q Q_Read;            //��������Queue
extern OS_Q Q_ReadData;        //���ͳ���ָ���  �²㷵�س�������
extern OS_Q Q_ReadData_LORA;   //ͨ��LORA���صĳ����� Ӧ��
extern OS_Q Q_Config;         //��������Queue
extern OS_Q Q_Deal;         //������յ��ķ��������͹���������

extern OS_SEM SEM_LORA_OK;
extern OS_SEM SEM_HeartBeat;    //���շ���������Task to HeartBeat Task  ���յ������Ļ�Ӧ
extern OS_SEM SEM_ACKData;     //�����������ݵ�ACK
extern OS_SEM SEM_Send;      //got the '>'  we can send the data now  ���Է�������
extern OS_SEM SEM_CJQLORAACK;

extern OS_TMR TMR_CJQTIMEOUT;    //�򿪲ɼ���֮�� 20���ӳ�ʱ �Զ��ر�ͨ��

extern uint8_t deviceaddr[5];
extern uint8_t cjqaddr[5];
extern uint8_t cjqaddr_eg[2];
extern uint8_t slave_mbus; //0xaa mbus   0xff  485   0xBB~�ɼ���

extern uint8_t * volatile p_server;      //�ж��б���GPRS ������������
extern uint8_t * volatile p_server_;     //��¼�жϵĿ�ʼָ��


uint8_t data_seq = 0;  //��¼���ݵ����к� �ȴ�ack
uint8_t server_seq = 0;  //�����������к�  ����ʱ  ��ͬ�������к�

uint8_t fe[4] = {0xFE,0xFE,0xFE,0xFE};  //����ʱǰ�淢�͵�4��0xFE




uint8_t readingall = 0;   //�Ƿ����ڳ�ȫ����
uint8_t readingall_progress = 0;  //���ڳ�ȫ����Ľ������

/**
 * ����meter usart���յ�������
 */
void task_meter_raw(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * p_mem;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t msg_data;         //the data get from the queue
  
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_;       //keep the buf's ptr  used to release the buf
  
  
  while(DEF_TRUE){
    //�յ�0x68֮��  ���200ms û���յ�����  ����Ϊ��ʱ��
    p_mem = OSQPend(&Q_METER_USART,200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    
    if(err == OS_ERR_TIMEOUT){
      p_buf = p_buf_;
      continue;
    }
    
    msg_data = *p_mem;
    put_memisr(p_mem);
    
    if(p_buf == 0){
      p_buf = get_membuf();
      if(p_buf > 0){
        //get the buf
        p_buf_ = p_buf;
      }else{
        //do not get the buf
        continue;
      }
    }
    *p_buf++ = msg_data;
    
    //�����յ�֡��ʽ
    switch(check_meter_frame(p_buf_,p_buf)){
    case -1:
      //��ǰ֡����
      p_buf = p_buf_;
      break;
    case 0:
      //��ǰ֡û�н�����
      break;
    case 1:
      //��ǰ֡�������
      if(get_readding()){
        
        switch(get_protocol()){
        case 0x11:
          //EG
          frame_len = 9;
          OSQPost(&Q_METER,
                  p_buf_,
                  frame_len,
                  OS_OPT_POST_FIFO,
                  &err);
          break;
        case 0xFF:
          //188
          frame_len = *(p_buf_+10)+13;
          OSQPost(&Q_METER,
                  p_buf_,
                  frame_len,
                  OS_OPT_POST_FIFO,
                  &err);
          break;
        }
        
        if(err == OS_ERR_NONE){
          p_buf_ = 0;
          p_buf = 0;
        }else{
          p_buf = p_buf_;
        }
      }else{
        p_buf = p_buf_;
      }
      break;
    }
  }
}

/**
 * ���߲ɼ����ӿڽ��յ���ԭʼ����
 */
void task_cjq_raw(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * p_mem;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t msg_data;         //the data get from the queue
  
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_;       //keep the buf's ptr  used to release the buf
  
  uint8_t forme = 0;  //�Ƿ�������
  
  while(DEF_TRUE){
    //�յ�0x68֮��  ���200ms û���յ�����  ����Ϊ��ʱ��
    p_mem = OSQPend(&Q_CJQ_USART,200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    
    if(err == OS_ERR_TIMEOUT){
      p_buf = p_buf_;
      continue;
    }
    
    msg_data = *p_mem;
    put_memisr(p_mem);
    
    if(p_buf == 0){
      p_buf = get_membuf();
      if(p_buf > 0){
        //get the buf
        p_buf_ = p_buf;
      }else{
        //do not get the buf
        continue;
      }
    }
    *p_buf++ = msg_data;
    
    //�����յ�֡��ʽ
    switch(check_xintian_frame(p_buf_,p_buf)){
    case -1:
      //��ǰ֡����
      p_buf = p_buf_;
      break;
    case 0:
      //��ǰ֡û�н�����
      break;
    case 1:
      //��ǰ֡�������
      //����״̬�� �ɼ������ع���������֡  ���� Ӧ��֡
      frame_len = check_frame(p_buf_);
      if(get_readding()){
        forme = 0;
        switch(get_protocol()){
        case 0x11:
          //EG
          if(cjqaddr_eg[0] == *(p_buf_ + DATA_POSITION + 1) && cjqaddr_eg[1] == *(p_buf_ + DATA_POSITION)){
            forme = 1;
          }
          break;
        case 0xFF:
          //188
          if(cjqaddr[0] == *(p_buf_ + DATA_POSITION + 1) && cjqaddr[1] == *(p_buf_ + DATA_POSITION)){
            forme = 1;
          }
          break;
        }
        if(forme){
          if(*(buf_+AFN_POSITION) == AFN_ACK){
            //ACK
            signal_cjqack();
            p_buf = p_buf_;
          }else{
            //DATA
            //��ǰ�ɼ���   ACK   DATA to Queue  TODO...
            device_ack_lora(0,*(p_buf_ + SEQ_POSITION));
            OSQPost(&Q_CJQ,
                    p_buf_,
                    frame_len,
                    OS_OPT_POST_FIFO,
                    &err);
            if(err == OS_ERR_NONE){
              p_buf_ = 0;
              p_buf = 0;
            }else{
              p_buf = p_buf_;
            }
          }
        }else{
          p_buf = p_buf_;  //�������ҵ�
        }
      }else{
        //�Ҳ��ڳ���ģʽ��  ����
        switch(*(buf_+AFN_POSITION)){
        case AFN_CONFIG:
        case AFN_QUERY:
          *p_buf = 0x00;//��ʶ��һ֡����������485��
          OSQPost(&Q_CONFIG,
                  p_buf,
                  frame_len,
                  OS_OPT_POST_FIFO,
                  &err);
          if(err == OS_ERR_NONE){
            p_buf_ = 0;
            p_buf = 0;
          }else{
            p_buf = p_buf_;
          }
          break;
        default:
          p_buf = p_buf_;
          break;
        }
      }
      break;
    }
  }
}

/**
 * ����LORA���յ���ָ��
 */
void task_lora_raw(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * p_mem;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t msg_data;         //the data get from the queue
  
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_;       //keep the buf's ptr  used to release the buf
  
  uint8_t forme = 0;  //�Ƿ�������
  
  while(DEF_TRUE){
    //�յ�0x68֮��  ���200ms û���յ�����  ����Ϊ��ʱ��
    p_mem = OSQPend(&Q_LORA_USART,200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    
    if(err == OS_ERR_TIMEOUT){
      p_buf = p_buf_;
      continue;
    }
    
    msg_data = *p_mem;
    put_memisr(p_mem);
    
    if(p_buf == 0){
      p_buf = get_membuf();
      if(p_buf > 0){
        //get the buf
        p_buf_ = p_buf;
      }else{
        //do not get the buf
        continue;
      }
    }
    *p_buf++ = msg_data;
    
    //�����յ�֡��ʽ
    switch(check_lora_data2frame(p_buf_,p_buf)){
    case -1:
      //��ǰ֡����
      p_buf = p_buf_;
      break;
    case 0:
      //��ǰ֡û�н�����
      break;
    case 1:
      //��ǰ֡�������
      if(*(p_buf_) == 0x68){
        //����״̬�� �ɼ������ع���������֡  ���� Ӧ��֡
        if(get_readding()){
          forme = 0;
          switch(get_protocol()){
          case 0x11:
            //EG
            if(cjqaddr_eg[0] == *(p_buf_ + DATA_POSITION + 1) && cjqaddr_eg[1] == *(p_buf_ + DATA_POSITION)){
              forme = 1;
            }
            break;
          case 0xFF:
            //188
            if(cjqaddr[0] == *(p_buf_ + DATA_POSITION + 1) && cjqaddr[1] == *(p_buf_ + DATA_POSITION)){
              forme = 1;
            }
            break;
          }
          if(forme){
            if(*(buf_+AFN_POSITION) == AFN_ACK){
              //ACK
              signal_cjqack();
              p_buf = p_buf_;
            }else{
              //DATA
              //��ǰ�ɼ���   ACK   DATA to Queue  TODO...
              device_ack_lora(0,*(p_buf_ + SEQ_POSITION));
              frame_len = check_frame(p_buf_);
              OSQPost(&Q_CJQ,
                      p_buf_,
                      frame_len,
                      OS_OPT_POST_FIFO,
                      &err);
              if(err == OS_ERR_NONE){
                p_buf_ = 0;
                p_buf = 0;
              }else{
                p_buf = p_buf_;
              }
            }
          }else{
            p_buf = p_buf_;  //�������ҵ�
          }
        }else{
          p_buf = p_buf_; //�Ҳ��ڳ���ģʽ��  
        }
      }
      if(*(p_buf_) == 0x0D){
        //+++/AT+ESC return
        signal_lora_ok();
      }
      break;
    }
  }
}




/**
 * ���ӷ����� ����GPRSģ�鷢�͹�����ָ��
 */
void task_server(void *p_arg){
  OS_ERR err;
  
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint8_t getbuffail = 0;
  uint8_t connectfail = 0;
  
  uint8_t frame_start = 0; //���յ���֡����ʼ��ַ
  uint8_t frame_len = 0;  //���յ�֡���ܳ���
  uint8_t server_seq_ = 0; //���������͹��������� �����к�
  
  uint8_t * p_buf_copy = 0;  //��������֡ ����֡ post ������
  
  while(DEF_TRUE){
    if(get_connect_state()){
      if(p_buf == 0){
        p_buf = get_membuf();
        if(p_buf > 0){
          //get the buf
          getbuffail = 0;
          p_buf_ = p_buf;
          server_2buf(p_buf);
        }else{
          //do not get the buf
          getbuffail ++;
          //if(getbuffail >= 20){
          //  //20��û�л�ȡ��buf  
          //  device_cmd(0);
          //  *((uint8_t *)0) = 0x00;  //��ʹϵͳ����
          //}
          delayms(200);
          continue;
        }
      }
      
      if(p_server - p_server_ > 0){
        delayms(4);
        
        p_buf = p_server;
        replace_str00(p_buf_,p_buf);  //���ε�����ǰ��0x00
        if(Str_Str(p_buf_,"\n>")){
          
          p_buf = p_buf_;
          Mem_Set(p_buf_,0x00,256); //clear the buf
          server_2buf(p_buf);
          
          signal_sendgprs();
          continue;
        }
        
        if(Str_Str(p_buf_,"RECEIVE")){
          //oh it's the data  TODO...  \r\n+TCPRECV:0,**,0x68 L L 0x68 C A     \r\n
          frame_start = Str_Str(p_buf_,"\r\n\x68") + 2;
          frame_len = check_frame(frame_start);  //check the frame get the length
          if(frame_len){
            //the frame is ok
            server_seq_ = *(frame_start + SEQ_POSITION) & 0x0F;  //��ø�֡�����к�
            switch(*(frame_start+AFN_POSITION)){
            case AFN_ACK:  //the ack of the server
              if(server_seq_ == data_seq){
                signal_serverack();
              }
              break;
            case AFN_CONFIG:
            case AFN_QUERY:
              p_buf_copy =  get_membuf();
              if(p_buf_copy > 0){
                Mem_Copy(p_buf_copy,frame_start,frame_len);
                *(p_buf_copy + frame_len) = 0x01;  //��ʶ��һ֡���Է�����
                OSQPost(&Q_CONFIG,p_buf_copy,frame_len,OS_OPT_POST_1,&err);
                if(err != OS_ERR_NONE){
                  put_membuf(p_buf_copy);
                }
              }
              break;
            case AFN_CONTROL:
            case AFN_CURRENT:
              device_ack(0x01,server_seq_);  //ACK
              if(*(frame_start+FN_POSITION) == 0x05){//ƥ�����к�
                server_seq = server_seq_;
              }else{
                if(server_seq != server_seq_){  //�µĳ���ָ��  read
                  p_buf_copy =  get_membuf();
                  if(p_buf_copy > 0){
                    Mem_Copy(p_buf_copy,frame_start,frame_len);
                    *(p_buf_copy + frame_len) = 0x01;  //��ʶ��һ֡���Է�����
                    server_seq = server_seq_;
                    OSQPost(&Q_READ,p_buf_copy,frame_len,OS_OPT_POST_1,&err);
                    if(err != OS_ERR_NONE){
                      put_membuf(p_buf_copy);
                    }
                  }
                }
              }
              break;
            case AFN_LINK_TEST: //never will come to here
            case AFN_HISTORY://don't support
              break;
            default:
              break;
            }
          }
          p_buf =p_buf_;
          Mem_Set(p_buf_,0x00,256); //clear the buf
          server_2buf(p_buf_);
          continue;
        }
        
        if(Str_Str(p_buf_,"CLOSE") || Str_Str(p_buf_,"DEACT") || Str_Str(p_buf_,"ERROR")){
          //CLOSED   +PDP: DEACT\r\n
          Mem_Set(p_buf_,0x00,256); //clear the buf
          put_membuf(p_buf_);
          p_buf = 0;
          p_buf_ = 0;
          server_2buf(0);
          set_connect_state(0);
          continue;
        }
        
        //do not know what's that
        p_buf = p_buf_;
        Mem_Set(p_buf_,0x00,256); //clear the buf
        server_2buf(p_buf_);
      }else{
        delayms(4);
      }
    }else{
      
      if(p_buf_ != 0){
        Mem_Set(p_buf_,0x00,256); //clear the buf
        put_membuf(p_buf_);
        p_buf = 0;
        p_buf_ = 0;
        server_2buf(0);
      }
      
      device_cmd(0);
      device_cmd(1);
      if(connect()){
        connectfail = 0;
      }else{
        connectfail++;
        //if(connectfail > 20){
        //  device_cmd(0);
        //  *((uint8_t *)0) = 0x00;  //��ʹϵͳ����
        //}
      }
    }
  }
}

/**
 * GPRS ��������
 */
void task_heartbeat(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t beat[17];
  uint8_t * p_beat;
  uint8_t heart_ack = 0;
  uint8_t i;
  uint8_t * p_deviceaddr = 0;
  
  while(DEF_TRUE){
    if(get_connect_state()){
      if(lock_gprs()){
        for(i = 0;get_connect_state() && i < 3;i++){
          heart_ack = 0;
          
          p_beat = beat;
          *p_beat++ = FRAME_HEAD;
          *p_beat++ = 0x27;//(9 << 2) | 0x03;
          *p_beat++ = 0x00;
          *p_beat++ = 0x27;//(9 << 2) | 0x03;
          *p_beat++ = 0x00;
          *p_beat++ = FRAME_HEAD;
          
          *p_beat++ = ZERO_BYTE | DIR_TO_SERVER | PRM_START | START_FUN_TEST;
          
          p_deviceaddr = get_device_addr();
          *p_beat++ = p_deviceaddr[0];
          *p_beat++ = p_deviceaddr[1];
          *p_beat++ = p_deviceaddr[2];
          *p_beat++ = p_deviceaddr[3];
          *p_beat++ = p_deviceaddr[4];
          
          *p_beat++ = AFN_LINK_TEST;
          
          data_seq = addSEQ();
          *p_beat++ = ZERO_BYTE |SINGLE | CONFIRM | heart_seq;
          
          *p_beat++ = FN_HEARTBEAT;
          
          *p_beat++ = check_cs(beat+6,9);
          *p_beat++ = FRAME_END;
          if(send_server(beat,17)){
            if(wait_serverack(5000)){
              heart_ack = 1;
              break;
            }
          }
        }
        unlock_gprs();
        if(heart_ack){  //���յ�����ACK
          delayms(120000);
        }else{   //3�ζ�û���յ�����ACK
          set_connect_state(0);
        }
      }else{
        delayms(1000);  //û�еõ�GPRS��  1s������
      }
    }else{
      delayms(1000);  //������ 1s������
    }
  }
}


/**
 * ��ʱ���LORAģ���Ƿ�OK  
 */
void task_lora_check(void *p_arg){
  uint8_t inat = 0;
  uint8_t outat = 0;
  uint8_t i;
  
  while(DEF_TRUE){
    //2min  ÿ2���Ӽ��һ��
    delayms(120000);
    if(get_readding()){
      continue;
    }
    
    if(lock_lora()){
      //����3��
      for(i = 0; i < 3;i++){
        inat = 0;
        outat = 0;
        write_lora("+++",3); //����ATģʽ
        if(wait_lora_ok()){
          inat = 1;
        }
        
        if(inat){
          write_lora("AT+ESC\r\n",8);  //�뿪ATģʽ
          if(wait_lora_ok()){
            outat = 1;
          }
        }
        
        if(inat == 1 && outat == 1){
          break;
        }
      }
    }
    unlock_lora();
    
    //���ʧ��
    if(inat == 0 || outat == 0){
      //restart LORA
      PWR_LORA_OFF();
      delayms(10000);
      PWR_LORA_ON();
    }
  }
}


/**
 * ����ĳ�������
 */
void task_read(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint16_t msg_size;
  uint8_t * buf_frame;
  
  while(DEF_TRUE){
    buf_frame = OSQPend(&Q_READ,
                        0,
                        OS_OPT_PEND_BLOCKING,
                        &msg_size,
                        &ts,
                        &err);
    Device_Read(ENABLE);
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
      //EG  
      //power_cmd(ENABLE);
      meter_read_eg(buf_frame,msg_size,*(buf_frame+msg_size));
      //power_cmd(DISABLE);
      break;
    }
    Device_Read(DISABLE);
    
    
    OSMemPut(&MEM_Buf,buf_frame,&err);
  }
}


/**
 * ������Э���
 */
void meter_read_eg(uint8_t * buf_frame,uint8_t frame_len,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint16_t msg_size;
  uint8_t * lora_data;
  uint8_t i;
  uint8_t cjq_ok;
  uint8_t recv_ok=0;
  uint8_t all_single = *(buf_frame + DATA_POSITION);  //0x00~ȫ��   0xAA~����
  uint8_t cjq_h = *(buf_frame + DATA_POSITION + 1);
  uint8_t cjq_l = *(buf_frame + DATA_POSITION + 2);
  uint8_t allmeters = *(buf_frame + DATA_POSITION + 3);  //ȫ����ʱ��ʾ�ܱ���  ������ʱ��ʾ��ĵ�ַ
  uint8_t lora_seq_ = 0;
  uint8_t lora_seq = 0;
  uint8_t tmr_count =0;
  uint8_t meter_recv = 0;
  uint16_t data_len = 0;
  uint8_t metercnt = 0;
  
  //��ʾ���ڳ��Ĳɼ���
  cjqaddr_eg[0] = cjq_l;
  cjqaddr_eg[1] = cjq_h;
  
  for(i = 0;i < 3;i++){
    cjq_ok = 0;
    Write_LORA(buf_frame,frame_len);
    
    OSSemPend(&SEM_CJQLORAACK,
              10000,
              OS_OPT_PEND_BLOCKING,
              &ts,
              &err);
    
    if(err != OS_ERR_NONE){
      continue;
    }
    cjq_ok = 1;
    break;
  }
  
  
  //��ȡconfig_flash��ʹ��Ȩ
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //��ȡMUTEX������ ������...
    //return 0xFFFFFF;
    return;
  }
  meterdata = config_flash; //�����ص�������Ϣ�����config_flash
  Mem_Set(config_flash,0x00,0x1000); //clear the buf
  
  if(cjq_ok){
    //cjq is ok wait the data
    //when recv the data send the ack
    
    recv_ok=0;
    switch(all_single){
    case 0xAA:
      for(i = 0;i < 3;i++){
        lora_data = OSQPend(&Q_ReadData_LORA,
                            5000,
                            OS_OPT_PEND_BLOCKING,
                            &msg_size,
                            &ts,
                            &err);
        if(err != OS_ERR_NONE){
          continue;
        }
        
        lora_seq_ = *(lora_data + SEQ_POSITION);
        //device_ack_lora(desc,lora_seq_);
        //get the data
        recv_ok=1;
        //�жϲɼ�����ַ  �жϱ��ַ
        if(cjq_h == *(lora_data + DATA_POSITION) && cjq_l == *(lora_data + DATA_POSITION + 1) && allmeters == *(lora_data + DATA_POSITION + 3)){
          meterdata[0] = cjq_h;
          meterdata[1] = cjq_l;
          meterdata[2] = 0x00;
          meterdata[3] = *(lora_data + DATA_POSITION + 3);
          meterdata[4] = *(lora_data + DATA_POSITION + 4);
          meterdata[5] = *(lora_data + DATA_POSITION + 5);
        }else{
          meterdata[0] = cjq_h;
          meterdata[1] = cjq_l;
          meterdata[2] = 0xFF;
        }
        OSMemPut(&MEM_Buf,lora_data,&err);
        break;
      }
      if(!recv_ok){
        meterdata[0] = cjq_h;
        meterdata[1] = cjq_l;
        meterdata[2] = 0xFF;
      }
      break;
    case 0x00:
      while(tmr_count < 60){
        lora_data = OSQPend(&Q_ReadData_LORA,
                            1500,
                            OS_OPT_PEND_BLOCKING,
                            &msg_size,
                            &ts,
                            &err);
        if(err != OS_ERR_NONE){
          tmr_count++;
          continue;
        }
        
        
        lora_seq_ = *(lora_data + SEQ_POSITION);
        if(meter_recv == 0){
          lora_seq = lora_seq_-1; //ȷ����һ֡������
          meterdata[0] = cjq_h;
          meterdata[1] = cjq_l;
          meterdata[2] = 0x00;  
        }
        //device_ack_lora(desc,lora_seq_);
        
        data_len = (lora_data[1]&0xFF) | ((lora_data[2]&0xFF)<<8);
        data_len = data_len >> 2;
        
        if(lora_seq != lora_seq_){
          lora_seq = lora_seq_;
          metercnt = (data_len-12)/3;
          
          if(0xFF == lora_data[17]){
            //�ɼ�����ʱ~~~~����ָ��0xFF
            tmr_count=60;
          }else{
            //����Frame 
            //����������֡  
            for(i = 0;i < metercnt;i++){
              meterdata[meter_recv*3+i*3+3] = lora_data[18+3*i];
              meterdata[meter_recv*3+i*3+1+3] = lora_data[18+3*i+1];
              meterdata[meter_recv*3+i*3+2+3] = lora_data[18+3*i+2];
            }
          }
          //������meterdata copyǰ������  ���������λ
          meter_recv += metercnt;
        }
        
        OSMemPut(&MEM_Buf,lora_data,&err);
        if(meter_recv == allmeters){
          recv_ok=1;
          break;
        }
      }
      if(!recv_ok){
        //sorry �ɼ�������
        meterdata[0] = cjq_h;
        meterdata[1] = cjq_l;
        meterdata[2] = 0xFF;
      }
      break;
    }
  }else{
    //cjq is error send the overtime
    meterdata[0] = cjq_h;
    meterdata[1] = cjq_l;
    meterdata[2] = 0xFF;
  }
  
  
  //send the data to Server TODO...
  switch (all_single){
    case 0xAA:
      send_data_eg(1,desc);
    break;
    case 0x00:
      if(meterdata[2] != 0xFF){
        send_data_eg(allmeters,desc);
      }else{
        send_cjqtimeout_eg(desc);
      }
    break;
  }
  
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}

/**
 * config and query the parameter
 */
void task_config(void *p_arg){
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
 * ����ָʾ  
 * ��װʱ����LORA�ź�
 */
void task_led(void *p_arg){
  
  while(DEF_TRUE){
    while(get_lora_test()){
      LED2_OFF();
      LED3_ON();
      write_lora("TEST",4); //��ʱ���͹��ɼ��������ź�ʹ��
      delayms(3500);
      LED3_OFF();
      delayms(500);
    }
    
    //LED2
    if(get_readding()){
      LED2_ON();
      delayms(100);
      LED2_OFF();
      delayms(100);
    }else{
      LED2_ON();
      delayms(1000);
      LED2_OFF();
      delayms(1000);
    }
  }
}
