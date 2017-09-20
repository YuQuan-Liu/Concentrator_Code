#include "utils.h"
#include "device_params.h"
#include "stm32f10x_conf.h"
#include "serial.h"
#include "os.h"
#include "lib_str.h"
#include "gprs.h"
#include "spi_flash.h"
#include "stdlib.h"


extern uint8_t * volatile p_server;      //�ж��б���GPRS ������������
extern uint8_t * volatile p_server_;     //��¼�жϵĿ�ʼָ��


u8 *ats[20]={
	"AT\r",
	"ATE0\r",   //�رջ���
	"AT+CPIN?\r",  //��ȡSIM����ʶ
	"AT+CSQ\r",   //��ѯ�ź�ǿ��
	"AT+CREG?\r",  //�������ע��״̬
	"AT+CGATT?\r",  //���GPRS����״̬
	"AT+CIPMUX=1\r",  //���óɶ���·ģʽ
	"AT+CSTT=\"3GWAP\"\r",  //����APN   ��ͨ
        "AT+CSTT=\"CMNET\"\r",  //����APN   �ƶ�
	"AT+CIICR\r",   //����PPP����
	"AT+CIFSR\r",    //��ȡ����IP��ַ
	"AT+CIPSTART=0,\"TCP\",",   //+ip+port ����TCP����
	"AT+CIPSEND=0,",    //����·0 �Ϸ�������
};


uint8_t * send_at_recv(uint8_t *at,uint8_t *p_buf_start,uint32_t timeout){
  uint8_t * p_buf_end = 0;
  
  write_serverstr(at);
  server_2buf(p_buf_start);  //�������ݵ�buf_server
  delayms(timeout);
  p_buf_end = p_server;
  
  replace_str00(p_buf_start,p_buf_end);  //����start~end 0x00
  server_2buf(0);   //ֹͣ��������
  return p_buf_end;
}


uint8_t ate_(void){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint8_t result = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    p_buf = send_at_recv(ats[1],p_buf_,200);
    
    if(Str_Str(p_buf_,"OK")){
      result = 1;
    }
    
    put_membuf(p_buf_);
  }
  return result;
}


/*
���SIM��״̬
*/
uint8_t at_cpin(void){
  uint8_t i;
  uint8_t result = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    
    for(i = 0;i < 100;i++){
      if(i != 0){
        delayms(200);
      }
      p_buf = send_at_recv(ats[2],p_buf_,100);
      if(Str_Str(p_buf_,"READY")){
        result = 1;
        break;
      }
    }
    
    put_membuf(p_buf_);
  }
  return result;
}


uint8_t at_csq(void){
  uint8_t i;
  uint8_t result = 0;
  uint8_t good = 0;
  
  uint8_t *rssi_ptr = 0;
  uint8_t rssi = 0;
  uint8_t *ber_ptr = 0;
  uint8_t ber = 0;
  
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    
    for(i = 0;i < 100;i++){
      if(i != 0){
        delayms(200);
      }
      p_buf = send_at_recv(ats[3],p_buf_,100);
      if(Str_Str(p_buf_,"CSQ")){
        rssi_ptr = Str_Char_N(p_buf_,256,':')+2;
        rssi = atoi(rssi_ptr);
        ber_ptr = Str_Char_N(p_buf_,256,',')+1;
        ber = atoi(ber_ptr);
        if(rssi > 5){
          good++;
          if(good > 6){
            result = 1;
            break;
          }
        }else{
          good = 0;
        }
      }else{
        good = 0;
      }
    }
    
    put_membuf(p_buf_);
  }
  return result;
}

uint8_t at_creg(void){
  uint8_t i;
  uint8_t result = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    
    for(i = 0;i < 100;i++){
      if(i != 0){
        delayms(200);
      }
      p_buf = send_at_recv(ats[4],p_buf_,100);
      if(Str_Str(p_buf_,"+CREG: 0,1") || Str_Str(p_buf_,"+CREG: 0,5")){
        result = 1;
        break;
      }
    }
    
    put_membuf(p_buf_);
  }
  return result;
}


uint8_t check_cgatt(void){
  uint8_t i;
  uint8_t result = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    
    for(i = 0;i < 100;i++){
      if(i != 0){
        delayms(200);
      }
      p_buf = send_at_recv(ats[5],p_buf_,100);
      if(Str_Str(p_buf_,"+CGATT: 1")){
        result = 1;
        break;
      }
    }
    
    put_membuf(p_buf_);
  }
  return result;
}

//����·ģʽ
uint8_t at_cipmux(void){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint8_t result = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    p_buf = send_at_recv(ats[6],p_buf_,100);
    
    if(Str_Str(p_buf_,"OK")){
      result = 1;
    }
    
    put_membuf(p_buf_);
  }
  return result;
}

uint8_t at_apn(void){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint8_t result = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    if(get_simcard() == 0xAA){
      p_buf = send_at_recv(ats[7],p_buf_,100);  //��ͨ
    }else{
      p_buf = send_at_recv(ats[8],p_buf_,100);  //�ƶ�
    }
    
    if(Str_Str(p_buf_,"OK")){
      result = 1;
    }
    
    put_membuf(p_buf_);
  }
  return result;
}



uint8_t at_xiic(void){
  uint8_t i;
  uint8_t result = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    
    for(i = 0;i < 100;i++){
      if(i != 0){
        delayms(200);
      }
      p_buf = send_at_recv(ats[9],p_buf_,2000);
      if(Str_Str(p_buf,"OK")){
        result = 1;
        break;
      }
    }
    
    put_membuf(p_buf_);
  }
  return result;
}


uint8_t check_xiic(void){
  uint8_t i;
  uint8_t result = 0;
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    
    for(i = 0;i < 100;i++){
      if(i != 0){
        delayms(200);
      }
      p_buf = send_at_recv(ats[10],p_buf_,100);
      if(Str_Str(p_buf,".")){
        result = 1;
        break;
      }
    }
    
    put_membuf(p_buf_);
  }
  return result;
}

uint8_t at_tcpsetup(void){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint8_t result = 0;
  uint8_t * p_ip = 0;
  
  p_buf = get_membuf();
  if(p_buf){
    p_buf_ = p_buf;
    p_ip = get_ip();
    //ƴ��ATָ�p_buf_ 
    sprintf(p_buf_,"%s%d.%d.%d.%d,%d\r",ats[11],p_ip[0],p_ip[1],p_ip[2],p_ip[3],get_port());
    write_serverstr(p_buf_);
    Mem_Set(p_buf_,0x00,256);
    server_2buf(p_buf_);  //�������ݵ�buf_server
    delayms(3000);
    p_buf = p_server;
    
    replace_str00(p_buf_,p_buf);  //����start~end 0x00
    server_2buf(0);   //ֹͣ��������
    
    if(Str_Str(p_buf_,"CONNECT OK")){
      result = 1;
    }
    
    put_membuf(p_buf_);
  }
  return result;
}

/*
the data is the pointer to the data 
the count is the number of the data
*/
uint8_t send_server(uint8_t * data,uint16_t count){
  uint8_t result = 0;
  uint8_t sendcount[5];
  //send the data
  sprintf(sendcount,"%d",count);
  write_serverstr(ats[12]);
  write_serverstr(sendcount);
  write_serverstr("\r");
  
  if(wait_sendgprs(1000)){
    write_server(data,count);
    result=1;
  }
  return result;
}



uint8_t connect(void){
    if(ate_() == 0){
        return 0;
    }
    if(at_cpin() == 0){
        return 0;
    }
    if(at_csq() == 0){
        return 0;
    }
    if(at_creg() == 0){
        return 0;
    }
    
    if(check_cgatt() == 0){
        return 0;
    }
	
    if(at_cipmux() == 0){
        return 0;
    }
    
    //apn
    if(at_apn() == 0){
        return 0;
    }
    if(at_xiic() == 0){
        return 0;
    }
    if(check_xiic() == 0){
        return 0;
    }
    
    if(at_tcpsetup() == 0){
        return 0;
    }
    
    set_connect_state(1);
    return 1;
}


uint8_t device_cmd(uint8_t cmd){
  uint8_t cnt = 0;
  if(cmd){
    PWR_GPRS_ON();   //enable the power
    GPRS_PWRKEY_L();  //low the on_off
    delayms(1200);
    GPRS_PWRKEY_H();  //high the on_off
    
    delayms(3000);
    
    while(cnt < 100){
      
      if(ate_() == 1){
        return 1;
      }
      delayms(100);
      cnt++;
    }
    return 0;
  }else{
    PWR_GPRS_OFF();
    delayms(5000);
    return 1;
  }
}


