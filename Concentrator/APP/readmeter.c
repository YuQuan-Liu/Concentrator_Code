
#include "readmeter.h"
#include "utils.h"
#include "device_params.h"
#include "serial.h"
#include "frame.h"
#include "frame_188.h"
#include "spi_flash.h"
#include "configs.h"
#include "bsp.h"
#include "lib_str.h"
#include "lib_mem.h"
#include "gprs.h"

void meter_control(uint8_t * p_frame,uint16_t frame_len){
  switch(get_slave()){
  case 0xAA:
  case 0xFF:
    meter_control_meter(p_frame,frame_len);
    break;
  case 0xBB:
    meter_control_cjq(p_frame,frame_len);
    break;
  }
}

void meter_control_meter(uint8_t * p_frame,uint16_t frame_len){

}

void meter_control_cjq(uint8_t * p_frame,uint16_t frame_len){

}



void meter_read(uint8_t * p_frame,uint16_t frame_len){
  switch(get_slave()){
  case 0xAA:
  case 0xFF://�ײ��豸 ��  TODO...
    switch(*(p_frame+DATA_POSITION)){  //������  �����ɼ���(ͨ��)  ȫ����(ȫ��ͨ��)
    case 0xFF: //ȫ����
      meter_read_m_all(p_frame,frame_len);
      break;
    case 0xAA: //�����ɼ���ͨ��
      meter_read_m_channel(p_frame,frame_len);
      break;
    case 0x11: //������
      meter_read_m_meter(p_frame,frame_len);
      break;
    }
    break;
  case 0xBB://�ײ��豸CJQ
    switch(*(p_frame+DATA_POSITION)){  //������  �����ɼ���(ͨ��)  ȫ����(ȫ��ͨ��)
    case 0xFF: //ȫ����
      meter_read_c_all(p_frame,frame_len);
      break;
    case 0xAA: //�����ɼ���ͨ��
      meter_read_c_channel(p_frame,frame_len);
      break;
    case 0x11: //������
      meter_read_c_meter(p_frame,frame_len);
      break;
    }
    break;
  }
}


uint8_t search_cjq_in_membuf(uint8_t * p_buf_cjq,uint8_t * p_cjqaddr){
  uint8_t cjq_count_inbuf = 0;
  uint8_t i = 0;
  uint8_t * p_cjqaddr_inbuf = 0;

  cjq_count_inbuf = p_buf_cjq[255];

  for(i = 0;i < cjq_count_inbuf;i++){
    p_cjqaddr_inbuf = p_buf_cjq + i*10 + 1;
    if(Mem_Cmp(p_cjqaddr_inbuf,p_cjqaddr+1,4)){
      return 1;
    }
  }
  return 0;
}

void meter_read_c_all(uint8_t * p_frame,uint16_t frame_len){
  uint16_t cjq_count = 0;  //FLASH һ���ж��ٲɼ���
  uint32_t block_cjq = 0;
  uint32_t block_meter = 0;
  uint16_t c = 0;
  uint8_t * p_all_cjq = 0;

  uint8_t cjq_addr[5];
  uint8_t cjq_count_inbuf = 0;  //MEMBUF��һ���ж��ٸ��ɼ���

  uint8_t frame_data[2];
  uint8_t cjq_seq = 0;
  uint8_t ack = 0;
  uint8_t i = 0;

  uint8_t timeout_count = 30;
  uint8_t read_over_cjq = 0;   //������ɵĲɼ���(������ʱ)
  uint8_t all_read_over = 0;  //���еĲɼ����Ƿ񶼳��������
  uint8_t cjq_state = 0;   //��ǰ�ɼ����ĳ���״̬
  uint8_t read_over = 2;  //0~������  1~�������  2~��ʱ  ��ѯ�ɼ����Ƿ��ڳ��� ��Ӧ��״̬

  uint8_t * p_response = 0;
  uint16_t msg_size = 0;

  uint16_t all_frames = 0;
  uint16_t this_frame = 0;
  uint8_t frame_metercount = 0;


  uint8_t * p_meteraddr = 0;
  uint8_t * p_meter_read = 0;
  uint8_t * p_meter_status = 0;

  uint8_t m = 0;
  uint8_t z = 0;
  uint8_t * p_cjqaddr = 0;
  
  uint32_t send_ts = 0;  //����ָ���ʱ��
  
  p_all_cjq = get_membuf();
  if(p_all_cjq > 0){
    /*
    p_all_cjq   256�洢һ���ж��ٸ��ɼ���  255�洢�Ѿ������˼����ɼ���
    ÿ���ɼ���ռ10���ֽ�  12345~�ɼ�����ַ   6~�Ƿ����~[00~δ���  11~���  22~��ʱ]
    7~�Ƿ���յ��ɼ�������  00~û�н��յ�   11~���յ�
    */
    Mem_Set(p_all_cjq,0x00,256);

    sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //�ɼ�������
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //�ɼ�������ͷ

    for(c = 0;c < cjq_count;c++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);

      if(!search_cjq_in_membuf(p_all_cjq,cjq_addr)){
        cjq_count_inbuf = p_all_cjq[255];
        Mem_Copy(p_all_cjq + cjq_count_inbuf*10 + 1, cjq_addr + 1,4);
        p_all_cjq[255] = cjq_count_inbuf + 1;
      }

      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
    }

    //�������¹ҵĲɼ����豸�ĸ���
    cjq_count_inbuf = p_all_cjq[255];

    read_over_cjq = 0;   //������ɵĲɼ���(������ʱ)
    //���θ��ɼ������ͳ���ָ��
    for(c = 0; c < cjq_count_inbuf;c++){
      frame_data[0] = 0xFF;  //���ɼ���ȫ����
      cjq_seq = add_cjq_seq();
      set_cjq_data_seq(cjq_seq);

      ack = 0;
      for(i = 0;i < 3;i++){  //���ɼ�����ָ�� ����3��
        set_cjq_addr(p_all_cjq+10*c);
        if(write_frame_cjq(p_all_cjq+10*c, frame_data, 1,AFN_CURRENT,FN_METER,cjq_seq)){ //���ɼ������ͳ���ָ��
          if(wait_cjqack(10000)){  //�ȴ�10s�ɼ����Գ���ָ��ACK
            ack = 1;
            break;
          }
        }
      }
      if(ack){
        p_all_cjq[10*c+5] = 0x00;
      }else{
        p_all_cjq[10*c+5] = 0x22;
        read_over_cjq++;
      }
    }

    //ÿ10s����һ�����еĲɼ���  ���Ƿ񳭱����  �ȴ�5min
    timeout_count = 30;
    all_read_over = 0;  //���еĲɼ����Ƿ񶼳��������
    while(timeout_count > 0){
      timeout_count++;
      delayms(10000);

      for(c = 0; c < cjq_count_inbuf;c++){
        cjq_state = p_all_cjq[10*c+5];
        if(cjq_state == 0x00){   //�жϲɼ���״̬  ���ڳ���...
          cjq_seq = add_cjq_seq();
          set_cjq_data_seq(cjq_seq);

          read_over = 2;  //0~������  1~�������  2~��ʱ
          for(i = 0;i < 3;i++){  //���ɼ�����ָ�� ����3��
            set_cjq_addr(p_all_cjq+10*c);
            if(write_frame_cjq(p_all_cjq+10*c, (uint8_t *)0, 0,AFN_QUERY,FN_READING,cjq_seq)){ //���ɼ�������ѯ���Ƿ��ڳ���
              send_ts = get_timestamp();
              if(wait_q_cjq_ts(&p_response,&msg_size,10000,send_ts)){
                //�жϲɼ����Ƿ񳭱����
                if(*(p_response + DATA_POSITION)){  //���ڳ���
                  read_over = 0;
                }else{  //�������
                  read_over = 1;
                }
                put_membuf(p_response);
                break;
              }
            }
          }
          switch(read_over){
          case 0:
            p_all_cjq[10*c+5] = 0x00;
            break;
          case 1:
            p_all_cjq[10*c+5] = 0x11;
            read_over_cjq++;
            break;
          case 2:
            read_over_cjq++;
            p_all_cjq[10*c+5] = 0x22;
            break;
          }
        }
      }
      
      //���ϲ㷢�ͳ������֡
      frame_data[0] = read_over_cjq;
      frame_data[1] = cjq_count_inbuf;
      device_ack(*(p_frame+frame_len),add_server_seq(),frame_data,2,AFN_FAKE,FN_ACK);
      
      if(read_over_cjq == cjq_count_inbuf){
        all_read_over = 1;
        break;
      }
    }

    if(all_read_over){//ȫ���������
      //���α������г�����ɵĲɼ���  ��ȡ���µ�����
      for(c = 0; c < cjq_count_inbuf;c++){
        cjq_state = p_all_cjq[10*c+5];
        if(cjq_state == 0x11){   //�жϲɼ���״̬Ϊ�������
          cjq_seq = add_cjq_seq();
          set_cjq_data_seq(cjq_seq);

          for(i = 0;i < 3;i++){  //���ɼ�����ָ�� ����3��
            set_cjq_addr(p_all_cjq+10*c);
            if(write_frame_cjq(p_all_cjq+10*c, (uint8_t *)0, 0,AFN_QUERY,FN_ALL_READDATA,cjq_seq)){ //���ɼ������Ͱ�ȫ����ĳ�����������
              send_ts = get_timestamp();
              timeout_count = 10;
              while(timeout_count > 0){
                if(wait_q_cjq_ts(&p_response,&msg_size,10000,send_ts)){
                  //֡�ĳ���msg_size  �����֡��һ���ж��ٱ� frame_metercount
                  //һ���ж���֡all_frames  ���ǵڼ�֡this_frame
                  all_frames = *(p_response + DATA_POSITION) | *(p_response +DATA_POSITION+1)<<8;
                  this_frame = *(p_response + DATA_POSITION+2) | *(p_response + DATA_POSITION+3)<<8;
                  frame_metercount = (msg_size-8-9-5)/14;

                  for(m = 0;m < frame_metercount;m++){
                    //��֡�л�ȡ  ���ַ ��Ķ���  ���״̬
                    p_meteraddr = p_response + DATA_POSITION + 5 + 14 * m;
                    p_meter_read = p_response + DATA_POSITION + 5 + 8 + 14 * m;
                    p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4 + 14 * m;

                    //���ݵ�ǰ��ѯ�Ĳɼ�����ַ  �����ɼ����� ���
                    p_cjqaddr = get_cjq_addr();
                    for(z = 1;z <= 4;z++){
                      *(p_cjqaddr) = z;
                      block_cjq = search_cjq(p_cjqaddr);
                      if(block_cjq){
                        block_meter = search_meter(block_cjq,p_meteraddr);
                        if(block_meter){  //��������
                          meter_read_save(block_meter,p_meter_read,p_meter_status);
                          break;
                        }
                      }
                    }
                    *(p_cjqaddr) = 0;
                  }
                  put_membuf(p_response);
                  if(all_frames == this_frame){
                    break;
                  }
                }else{
                  timeout_count--;
                }
              }

              if(timeout_count > 0){ //�������е�֡OK
                p_all_cjq[10*c+6] = 0x11;
                break;
              }
            }
          }
        }
        //���ϲ㷢�Ͷ��ɼ�������������֡
        frame_data[0] = c+1;
        frame_data[1] = cjq_count_inbuf;
        device_ack(*(p_frame+frame_len),add_server_seq(),frame_data,2,AFN_FAKE,FN_ACK);
      }

      //�������µĲɼ���������  ȫ�����յ��˼�������
      //�����ݷ��ͳ�ȥ
      send_meter_data_all(*(p_frame + frame_len),p_all_cjq);
      
    }else{  //����ʱ  ����Ŀ����Բ���  ���Ҳ���

    }
    put_membuf(p_all_cjq);
  }
}

void meter_read_c_channel(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;

  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;

  uint8_t frame_data[6];
  uint8_t * p_meter_read = 0;
  uint8_t * p_meter_status = 0;

  uint8_t * p_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t cjq_seq = 0;

  uint8_t timeout_count = 10; //�������10�γ�ʱ���ж�Ϊʧ��
  uint16_t all_frames = 10;  //��ʾһ���᷵�ض���֡
  uint16_t this_frame = 0;   //��ʾ��������֡�еĵڼ�֡
  uint8_t frame_metercount = 0;  //���յ���֡���м���������
  uint8_t meter_type = 0;

  uint32_t send_ts = 0;  //����ָ���ʱ��
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  block_cjq = search_cjq(p_cjqaddr);

  if(block_cjq){  //find the cjq
    //���˳���ָ��͸��ɼ���
    frame_data[0] = *(p_frame+DATA_POSITION);
    for(i = 0;i < 5;i++){
      frame_data[i+1] = p_cjqaddr[i];
    }

    cjq_seq = add_cjq_seq();
    set_cjq_data_seq(cjq_seq);
    set_cjq_addr(p_cjqaddr);
    if(write_frame_cjq(p_cjqaddr, frame_data, 6,AFN_CURRENT,FN_METER,cjq_seq)){ //���ɼ������ͳ���ָ��
      send_ts = get_timestamp();
      if(wait_cjqack(10000)){  //�ȴ�10s�ɼ����Գ���ָ��ACK
        while(timeout_count > 0){
          if(wait_q_cjq_ts(&p_response,&msg_size,10000,send_ts)){
            //֡�ĳ���msg_size  �����֡��һ���ж��ٱ� frame_metercount
            //һ���ж���֡all_frames  ���ǵڼ�֡this_frame
            all_frames = *(p_response + DATA_POSITION) + *(p_response +DATA_POSITION+1)<<8;
            this_frame = *(p_response + DATA_POSITION+2) + *(p_response + DATA_POSITION+3)<<8;
            frame_metercount = (msg_size-8-9-5)/14;
            
            meter_type = *(p_response + DATA_POSITION + 4);
            for(i = 0;i < frame_metercount;i++){
              //��֡�л�ȡ  ���ַ ��Ķ���  ���״̬
              p_meteraddr = p_response + DATA_POSITION + 5 + 14 * i;
              p_meter_read = p_response + DATA_POSITION + 5 + 8 + 14 * i;
              p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4 + 14 * i;

              block_meter = search_meter(block_cjq,p_meteraddr);
              if(block_meter){  //��������
                meter_read_save(block_meter,p_meter_read,p_meter_status);
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

        //send the data out  
        if(timeout_count > 0){ //�������е�֡OK
          send_meter_data_channel(block_cjq,0,0,meter_type,*(p_frame+frame_len),0);
        }else{ //�������еı����ݳ�ʱ   ���Ҳ��� TODO...
          
        }
      }
    }
  }
}

void meter_read_c_meter(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;

  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;

  uint8_t frame_data[13];
  uint8_t * p_meter_read = 0;
  uint8_t * p_meter_status = 0;

  uint8_t * p_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t cjq_seq = 0;
  uint8_t meter_type = 0;

  uint32_t send_ts = 0;  //����ָ���ʱ��
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  p_meteraddr = p_cjqaddr + 5;

  block_cjq = search_cjq(p_cjqaddr);

  if(block_cjq){
    block_meter = search_meter(block_cjq,p_meteraddr);
    if(block_meter){
      //find the meter under the cjq
      //���˳���ָ��͸��ɼ���
      frame_data[0] = *(p_frame+DATA_POSITION);
      for(i = 0;i < 5;i++){
        frame_data[i+1] = p_cjqaddr[i];
      }
      for(i = 0;i < 7;i++){
        frame_data[i+6] = p_meteraddr[i];
      }

      cjq_seq = add_cjq_seq();
      set_cjq_data_seq(cjq_seq);
      set_cjq_addr(p_cjqaddr);
      if(write_frame_cjq(p_cjqaddr, frame_data, 13,AFN_CURRENT,FN_METER,cjq_seq)){ //���ɼ������ͳ���ָ��
        send_ts = get_timestamp();
        if(wait_cjqack(10000)){  //�ȴ�10s�ɼ����Գ���ָ��ACK
          if(wait_q_cjq_ts(&p_response,&msg_size,10000,send_ts)){
            //��֡�л�ȡ ��Ķ���  ���״̬
            meter_type = *(p_response + DATA_POSITION + 4);
            p_meter_read = p_response + DATA_POSITION + 5 + 8;
            p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4;
            //��������
            meter_read_save(block_meter,p_meter_read,p_meter_status);
            //send the data out  
            send_meter_data_single(p_meteraddr,p_meter_read,p_meter_status,meter_type,*(p_frame+frame_len));
            put_membuf(p_response);
          }
        }
      }
    }
  }
}


uint8_t write_frame_cjq(uint8_t * p_cjqaddr,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn,uint8_t server_seq_){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;
  uint8_t i = 0;
  uint8_t result = 0;

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    p_buf = (uint8_t *)p_buf_16;
    *p_buf++ = FRAME_HEAD;

    *p_buf++ = ZERO_BYTE | DIR_TO_DEVICE | PRM_START | SLAVE_FUN_DATA;
    /**/
    *p_buf++ = p_cjqaddr[0];
    *p_buf++ = p_cjqaddr[1];
    *p_buf++ = p_cjqaddr[2];
    *p_buf++ = p_cjqaddr[3];
    *p_buf++ = p_cjqaddr[4];

    *p_buf++ = afn;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = fn;

    for(i = 0;i < data_len;i++){
      *p_buf++ = p_data[i];
    }

    *p_buf++ = check_cs(p_buf_+6,9+data_len);
    *p_buf++ = FRAME_END;

    switch(get_device_mode()){
    case 0xFF:
      if(lock_lora()){
        write_lora(p_buf_,17+data_len);
        unlock_lora();
        result = 1;
      }else{
        result = 0;
      }
      break;
    case 0xAA:
      if(lock_cjq()){
        write_cjq(p_buf_,17+data_len);
        unlock_cjq();
        result = 1;
      }else{
        result = 0;
      }
      break;
    }
    put_membuf(p_buf_);
  }else{
    result = 0;
  }
  return result;
}



uint8_t meter_read_save(uint32_t block_meter,uint8_t * meter_read,uint8_t * meter_status){
  uint8_t * mem4k = 0;

  mem4k = get_mem4k();
  switch(get_protocol()){
  case 0xFF:
  case 0xEE:
    sFLASH_ReadBuffer(mem4k,(block_meter/0x1000)*0x1000,0x1000);  //��ȡ����Sector
    Mem_Copy(mem4k+block_meter%0x1000 + METER_FLASH_INDEX_METERSTATE,meter_status,2);  //��¼st��Ϣ
    Mem_Copy(mem4k+block_meter%0x1000 + METER_FLASH_INDEX_READ,meter_read,4);        //����

    sFLASH_EraseSector((block_meter/0x1000)*0x1000);  //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_WriteBuffer(mem4k,(block_meter/0x1000)*0x1000,0x1000);
    break;
  }
  return 1;
}



//��������ʱ  ���͵���������
void send_meter_data_single(uint8_t * p_meteraddr,uint8_t * meter_read ,uint8_t * meter_status,uint8_t meter_type,uint8_t desc){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;
  uint8_t i = 0;
  uint8_t frame_seq = 0;
  uint8_t frame_data_len = 0;

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;

    frame_seq = add_server_seq();
    set_server_data_seq(frame_seq);
    frame_data_len = 9+14+5;
    p_buf = ack_mulit_header(p_buf,get_device_addr(),0,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);


    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = 1;    //�ܹ�����֡
    *p_buf_16++ = 1;  //�ڼ�֡
    p_buf = (uint8_t *)p_buf_16;
    *p_buf++ = meter_type;

    for(i=0;i<7;i++){
      *p_buf++ = p_meteraddr[i];
    }
    *p_buf++ = 0x01;
    for(i=0;i<4;i++){
      *p_buf++ = meter_read[i];
    }
    *p_buf++ = meter_status[0];
    *p_buf++ = meter_status[1];

    *p_buf++ = check_cs(p_buf_+6,frame_data_len);
    *p_buf++ = FRAME_END;

    for(i = 0;i < 3;i++){
      switch(desc){
      case 0x01:
        if(lock_gprs()){
          send_server(p_buf_,p_buf-p_buf_);
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
        break;
      }
    }
    put_membuf(p_buf_);
  }
}

//���ɼ����������Ƿ���յ���
uint8_t check_cjq_timeout(uint8_t * p_all_cjq,uint8_t * cjq_addr){
  uint8_t i = 0;
  uint8_t cjq_result = 0;
  uint8_t cjq_count_inbuf = p_all_cjq[255];
  
  //���θ��ɼ������ͳ���ָ��
  for(i = 0; i < cjq_count_inbuf;i++){
    if(Mem_Cmp(p_all_cjq+10*i+1, cjq_addr+1, 4)){
      cjq_result = *(p_all_cjq+10*i+6);
      if(cjq_result == 0x00){
        cjq_result = 1;
      }
      if(cjq_result == 0x11){
        cjq_result = 0;
      }
      break;
    }
  }
  
  return cjq_result;
}

void send_meter_data_all(uint8_t desc,uint8_t * p_all_cjq){
  uint16_t cjq_count = 0;
  uint32_t block_cjq = 0;
  uint16_t meter_count = 0;

  uint16_t send_times = 0;  //���б� �����ܴ���
  uint16_t sended = 0;  //�Ѿ����͵�
  uint8_t remains = 0;
  uint16_t c = 0;
  uint8_t cjq_addr[5];

  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //�ɼ�������
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //�ɼ�������ͷ

  //��ȡ����ͨ�����еı������һ��Ҫ���Ͷ��ٴ�
  for(c = 0;c < cjq_count;c++){
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%10;
    send_times = send_times + meter_count/10;
    if(remains){
      send_times = send_times + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }

  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //�ɼ�������ͷ
  sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);  //�ɼ�����ַ
  
  for(c = 0;c < cjq_count;c++){
    //�жϲɼ����Ƿ�ʱ
    if(p_all_cjq == 0){
      send_meter_data_channel(block_cjq,send_times,sended,0x10,desc,0);
    }else{
      if(check_cjq_timeout(p_all_cjq,cjq_addr)){//�ɼ�����ʱ
        send_meter_data_channel(block_cjq,send_times,sended,0x10,desc,1);
      }else{  //û�г�ʱ
        send_meter_data_channel(block_cjq,send_times,sended,0x10,desc,0);
      }
    }
    
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%10;
    sended = sended + meter_count/10;
    if(remains){
      sended = sended + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);  //�ɼ�����ַ
  }
}


//���Ͳɼ�������ͨ��������     ֡�ĵ�ַ��Ϊ��ǰ�ɼ�����ַ
void send_meter_data_channel(uint32_t block_cjq_,uint16_t frame_times,uint16_t frame_times_start,uint8_t meter_type,uint8_t desc,uint8_t cjq_timeout){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;
  uint16_t i = 0;
  uint8_t j = 0;
  uint8_t k = 0;
  uint8_t frame_meter_count = 0;
  uint8_t meter_addr[7];
  uint8_t meter_read[4];
  uint8_t meter_status[2];

  uint32_t block_cjq = block_cjq_;
  uint32_t block_meter = 0;
  uint16_t cjqmeter_count = 0;
  uint8_t frame_data_len = 0;
  uint8_t frame_seq = 0;

  uint16_t times = 0;
  uint8_t remain = 0;
  uint16_t times_ = 0;      //һ��Ҫ���Ͷ���֡

  uint8_t ack = 0;  //�������ݺ��Ƿ�õ�ack

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;

    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
    sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);

    if(cjqmeter_count > 0){
      times = cjqmeter_count/10;
      remain = cjqmeter_count%10;
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
        if(times_==1){//��֡ 0
          frame_meter_count = cjqmeter_count;
          frame_data_len = 9+14*frame_meter_count+5;
          p_buf = ack_mulit_header(p_buf,get_device_addr(),0,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
        }else{
          if(i==0){//��֡ 1
            frame_meter_count = 10;
            frame_data_len = 9+14*frame_meter_count+5;
            p_buf = ack_mulit_header(p_buf,get_device_addr(),1,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
          }else{
            if(i==times_-1){//β֡ 3
              if(remain == 0){
                frame_meter_count = 10;
              }else{
                frame_meter_count = remain;
              }
              frame_data_len = 9+14*frame_meter_count+5;
              p_buf = ack_mulit_header(p_buf,get_device_addr(),3,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
            }else{//�м�֡ 2
              frame_meter_count = 10;
              frame_data_len = 9+14*frame_meter_count+5;
              p_buf = ack_mulit_header(p_buf,get_device_addr(),2,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
            }
          }
        }

        //֡��������
        p_buf_16 = (uint16_t *)p_buf;
        if(frame_times){
          *p_buf_16++ = frame_times;  //�ܴ���
        }else{
          *p_buf_16++ = times_;
        }
        *p_buf_16++ = i+1+frame_times_start;  //��ǰ����
        p_buf = (uint8_t *)p_buf_16;
        *p_buf++ = meter_type;

        //������
        for(k=0;k<frame_meter_count;k++){
          sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
          sFLASH_ReadBuffer((uint8_t *)&meter_read,block_meter+METER_FLASH_INDEX_READ,4);
          sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+METER_FLASH_INDEX_METERSTATE,2);
          for(j=0;j<7;j++){
            *p_buf++ = meter_addr[j];
          }
          *p_buf++ = 0x01;
          for(j=0;j<4;j++){
            *p_buf++ = meter_read[j];
          }
          if(cjq_timeout){  //�ɼ�����ʱ
            *p_buf++ = 0x40;
            *p_buf++ = 0x40;
          }else{
            *p_buf++ = meter_status[0];
            *p_buf++ = meter_status[1];
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
              send_server(p_buf_,p_buf-p_buf_);
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

/*
 * ����Ϊ������ֱ�ӳ���
 */

//���ɼ�������ͨ����
void meter_read_m_all(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0;

  uint8_t i = 0;
  uint8_t c = 1;  //�ɼ���ͨ������
  uint16_t meter_count = 0;
  uint16_t cjq_count = 0;

  uint8_t meter_read[4];
  uint8_t meter_status[2];
  uint8_t meter_addr[7];
  uint8_t cjq_addr[5];

  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //�ɼ�������
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //�ɼ�������ͷ

  for(c = 0;c < cjq_count;c++){

    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);

    cjq_relay_control(1,*(cjq_addr));  //���ɼ���ͨ��
    for(i = 0;i < meter_count;i++){   //�������ɼ���ͨ���µ����еı�
      sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
      sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);

      meter_read_single(block_meter, meter_addr, meter_type, meter_read, meter_status);

      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
    }
    cjq_relay_control(0,*(cjq_addr)); //�زɼ���ͨ��

    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }

  send_meter_data_all(*(p_frame + frame_len),0);
}

//���ɼ�������ͨ��
void meter_read_m_channel(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0;

  uint8_t * p_cjqaddr = 0;
  uint8_t i = 0;
  uint16_t meter_count = 0;

  uint8_t meter_read[4];
  uint8_t meter_status[2];
  uint8_t meter_addr[7];

  p_cjqaddr = p_frame+DATA_POSITION+1;

  block_cjq = search_cjq(p_cjqaddr);

  if(block_cjq){  //find the cjq
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);

    cjq_relay_control(1,*(p_cjqaddr));  //���ɼ���ͨ��
    for(i = 0;i < meter_count;i++){   //�������ɼ���ͨ���µ����еı�
      sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
      sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);

      meter_read_single(block_meter, meter_addr, meter_type, meter_read, meter_status);

      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
    }
    cjq_relay_control(0,*(p_cjqaddr)); //�زɼ���ͨ��

    //send the data out
    send_meter_data_channel(block_cjq,0,0,meter_type,*(p_frame+frame_len),0);
  }
}

//��������
void meter_read_m_meter(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0;

  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;

  uint8_t meter_read[4];
  uint8_t meter_status[2];


  p_cjqaddr = p_frame+DATA_POSITION+1;
  p_meteraddr = p_cjqaddr + 5;

  block_cjq = search_cjq(p_cjqaddr);

  if(block_cjq){
    block_meter = search_meter(block_cjq,p_meteraddr);
    if(block_meter){
      //find the meter under the cjq
      cjq_relay_control(1,*(p_cjqaddr));
      sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);
      if(meter_read_single(block_meter, p_meteraddr, meter_type, meter_read, meter_status)){
        //send the data out
        send_meter_data_single(p_meteraddr,meter_read,meter_status,meter_type,*(p_frame+frame_len));
      }
      cjq_relay_control(0,*(p_cjqaddr));
    }
  }
}

//ֻ�ܳ���
uint8_t meter_read_single(uint32_t block_meter, uint8_t *p_meteraddr,uint8_t meter_type,uint8_t * meter_read,uint8_t * meter_status){
  uint8_t success = 0;
  uint8_t * p_meter_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t j = 0;
  uint32_t send_ts = 0;  //����ָ���ʱ��
  uint32_t recv_ts = 0;  //����ӳ��ʱ��ʱ��
  uint8_t wait_cnt = 0;
  
  for(i =0;i<4;i++){
    meter_read[i] = 0x00;
  }
  meter_status[0] = 0x00;
  meter_status[1] = 0x00;

  for(i = 0;i < 3;i++){
    success = 0;
    if(meter_read_frame_send(p_meteraddr,meter_type)){
      send_ts = get_timestamp();
      if(wait_q_meter_ts(&p_meter_response,&msg_size,1200,send_ts)){
        switch(get_protocol()){ //����Э��  �жϵ�ַ
        case 0xFF:
        case 0xEE:
          if(Mem_Cmp(p_meteraddr,p_meter_response+2,7)){
            success = 1;
            meter_status[0] = *(p_meter_response + 31);//��ȡST L
            meter_status[1] = *(p_meter_response + 32);//��ȡST H
            for(j = 0;j < 4;j++){
              meter_read[j] = *(p_meter_response + 14 + j);
            }
          }
          break;
        }
        put_membuf(p_meter_response);
        if(success){
          break;
        }
      }else{  //�������ݳ�ʱ
        delayms(100);
      }
    }else{   //��������ʧ��
      delayms(100);
    }
  }

  if(!success){
    meter_status[0] = 0x40;
  }

  meter_read_save(block_meter,meter_read,meter_status);   //�����188Э���  �������Ϣ
  if(success){
    return 1;
  }else{
    return 0;
  }
}


//����Э�鷢�ͳ���֡
uint8_t meter_read_frame_send(uint8_t * p_meteraddr,uint8_t meter_type){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint8_t i = 0;

  p_buf = get_membuf();
  if(p_buf > 0){  //get the buf
    p_buf_ = p_buf;

    switch(get_protocol()){
    case 0xFF:
    case 0xEE:
      *p_buf++ = 0xFE;
      *p_buf++ = 0xFE;
      *p_buf++ = 0xFE;
      *p_buf++ = 0xFE;
      *p_buf++ = FRAME_HEAD;
      *p_buf++ = meter_type;
      for(i=0;i<7;i++){
        *p_buf++ = *(p_meteraddr + i);
      }
      *p_buf++ = 0x01; //C
      *p_buf++ = 0x03; //len
      switch(get_di_seq()){
      case 0xFF: //Ĭ�ϵ�λ��ǰ
        *p_buf++ = DATAFLAG_RD_L;
        *p_buf++ = DATAFLAG_RD_H;
        break;
      case 0xAA:
        *p_buf++ = DATAFLAG_RD_H;
        *p_buf++ = DATAFLAG_RD_L;
        break;
      }
      *p_buf++ = 0x01;
      *p_buf++ = check_cs(p_buf_+4,11+3);
      *p_buf++ = FRAME_END;
      break;
    }
    write_meter(p_buf_,p_buf-p_buf_);
    put_membuf(p_buf_);
    return 1;
  }else{
    return 0;
  }
}

//�ɼ���ͨ�����ؿ���
uint8_t cjq_relay_control(uint8_t cmd,uint8_t cjq){
  switch(cmd){
  case 1: //���̵���
    switch(cjq){
    case 1:
      RELAY1_ON();
      break;
    case 2:
      RELAY2_ON();
      break;
    case 3:
      RELAY3_ON();
      break;
    case 4:
      RELAY4_ON();
      break;
    }
    delayms(2000);
    break;
  case 0: //�ؼ̵���
    switch(cjq){
    case 1:
      RELAY1_OFF();
      break;
    case 2:
      RELAY2_OFF();
      break;
    case 3:
      RELAY3_OFF();
      break;
    case 4:
      RELAY4_OFF();
      break;
    }
    break;
  }
  return 1;
}






