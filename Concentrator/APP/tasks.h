
#ifndef TASK_H
#define TASK_H






void meter_read_188(uint8_t * buf_frame,uint8_t desc);  //����
void meter_read_eg(uint8_t * buf_frame,uint8_t desc);  //����
void meter_control(uint8_t * buf_frame,uint8_t desc);   //���ط�
void meter_read_single(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc);  //ֻ�ܶ���
void meter_send(uint8_t all,uint32_t block_meter_,uint8_t desc);  //all = 1 ����ȫ����  all = 0 ���ͱ���Ӧ�ı�
void meter_open(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_);
void meter_close(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_);
void meter_clean(void); //ִ��һ�ο��ط�����

void device_ack(uint8_t desc,uint8_t server_seq_);  //����ȷ��֡  1 ���͸�M590E������  0 ���͸�485

/*
����ײ��вɼ���  �ײ�ɼ���ʹ��͸��ģʽ
������ֱ�ӷ��ͳ���ָ��
cjq_open   cjq_close  ֻ��4·MBUS�̵������п���
*/
uint8_t cjq_open(uint8_t * cjq_addr,uint32_t block_cjq);
uint8_t cjq_close(uint8_t * cjq_addr,uint32_t block_cjq);

void cjq_timeout(void *p_tmr,void *p_arg);   //20min ��ʱ �ر�ͨ��

uint8_t relay_485(FunctionalState NewState);
uint8_t mbus_power(FunctionalState NewState);


//tasks
void Task_485_2(void *p_arg);
void Task_LORA(void *p_arg);
void Task_Server(void *p_arg);
void Task_DealServer(void *p_arg);
void Task_Connect(void *p_arg);
void Task_Read(void *p_arg);
void Task_HeartBeat(void *p_arg);
void Task_Config(void *p_arg);
void Task_LED(void *p_arg);

void addSEQ(void);  //����local_seq



#endif