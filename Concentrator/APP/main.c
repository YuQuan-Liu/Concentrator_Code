
#include "includes.h"
#include "tasks.h"

/*
* ��������TCB STK
*/

//OS_TCBs
OS_TCB  TCB_Start;
CPU_STK STK_Start[APP_START_TASK_STK_SIZE];


//����ɼ��������͹���������
OS_TCB TCB_METER_RAW;
CPU_STK STK_METER_RAW[APP_START_TASK_STK_SIZE];

OS_TCB TCB_CJQ_RAW;
CPU_STK STK_CJQ_RAW[APP_START_TASK_STK_SIZE];

OS_TCB TCB_LORA_RAW;
CPU_STK STK_LORA_RAW[APP_START_TASK_STK_SIZE];

//������������͹���������  
OS_TCB TCB_SERVER;
CPU_STK STK_SERVER[APP_START_TASK_STK_SIZE];

//���� "+TCPRECV"
OS_TCB TCB_SERVER_ACTION;
CPU_STK STK_SERVER_ACTION[APP_START_TASK_STK_SIZE];

//����������
OS_TCB TCB_HEARTBEAT;
CPU_STK STK_HEARTBEAT[APP_START_TASK_STK_SIZE];

// ���LORA�Ƿ�����  ��װʱ����LORA�ź�
OS_TCB TCB_LORA_CHECK;
CPU_STK STK_LORA_CHECK[APP_START_TASK_STK_SIZE];

//��������
OS_TCB TCB_READ;
CPU_STK STK_READ[APP_START_TASK_STK_SIZE*3];

//��������
OS_TCB TCB_CONFIG;
CPU_STK STK_CONFIG[APP_START_TASK_STK_SIZE*3];

OS_TCB TCB_SYN;
CPU_STK STK_SYN[APP_START_TASK_STK_SIZE];

OS_TCB TCB_LED;
CPU_STK STK_LED[APP_START_TASK_STK_SIZE];

//MBUS ����
OS_TCB TCB_OVERLORD;
CPU_STK STK_OVERLORD[APP_START_TASK_STK_SIZE];

/*
* ������Ҫ��ȫ��MEM
*/
//OS_MEMs
OS_MEM MEM_Buf;
uint8_t mem_buf[8][256];

OS_MEM MEM_ISR;  //be used in the ISR   get the data from the usart*  post it to the deal task
uint8_t mem_isr[30][4];

//OS_MUTEXs;  ��Ҫдgprs lora cjq meter ʱ��Ҫ��ȡ��
OS_MUTEX MUTEX_GPRS;
OS_MUTEX MUTEX_LORA;
OS_MUTEX MUTEX_CJQ;
OS_MUTEX MUTEX_METER;
OS_MUTEX MUTEX_MEM_4K;

//OS_SEMs ;
OS_SEM SEM_LORA_OK;      //���յ�LORA���ص�  0D 0A 4F 4B 0D 0A 
OS_SEM SEM_HEART_BEAT;   //��������������ACK
OS_SEM SEM_SERVER_ACK;   //�����������ݵ�ACK
OS_SEM SEM_SEND_GPRS;    //got the '>'  we can send the data now  ���Է�������
OS_SEM SEM_CJQ_ACK;      //�ɼ����Գ���ָ���ACK
OS_SEM SEM_SYN;      //ȥͬ��cjqaddr ��Ӧ�Ĳɼ���������

//OS_Qs
OS_Q Q_CJQ_USART;  //CJQ USART��������
OS_Q Q_METER_USART; //METER USART��������
OS_Q Q_LORA_USART;  //LORA USART��������
OS_Q Q_CJQ;  //LORA 485-CJQ ���շ��͵�����  Q_CJQ_USART  Q_LORA_USART  ������֡
OS_Q Q_METER;   //Q_METER_USART������֡
OS_Q Q_READ;    //Q_SERVER  Q_CJQ �����ȥ�����֡
OS_Q Q_CONFIG;  //Q_SERVER  Q_CJQ �����ȥ���õ�֡
OS_Q Q_SERVER_ACTION;  //���������͹�����ָ������    �������Ҫ�˰� ���յ�֮֡��ֱ��post��Ӧ��������

//OS_TMR
OS_TMR TMR_CJQTIMEOUT;    //�򿪲ɼ���ͨ��֮�� 20���ӳ�ʱ �Զ��ر�ͨ��

//OS_FLAG
OS_FLAG_GRP FLAG_Event;

void TaskStart(void *p_arg);
void TaskCreate(void);
void ObjCreate(void);


int main (void){
  OS_ERR err;
  CPU_IntDis();
  BSP_Init();
  OSInit(&err);
  
  OSTaskCreate((OS_TCB  *)&TCB_Start,
               (CPU_CHAR *)"START",
               (OS_TASK_PTR )TaskStart,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO,
               (CPU_STK *)&STK_Start[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  OSStart(&err);
}

void TaskStart(void *p_arg){
  CPU_INT32U cnts;
  CPU_INT32U cpu_clk_freq;
  
  CPU_Init();
  
  cpu_clk_freq = BSP_CPU_ClkFreq();
  cnts = cpu_clk_freq / (CPU_INT32U)OS_CFG_TICK_RATE_HZ;
  OS_CPU_SysTickInit(cnts);
  
  ObjCreate();
  //��ʼ��FLASH  w25x16
  sFLASH_Init();
  BSP_USART_Init();
  /**/
  TaskCreate();
  
  //Open the IWDG;
  //BSP_IWDG_Init();
  
  while(DEF_TRUE){
    IWDG_ReloadCounter();  //Reload IWDG counter
    
    LED1_ON();
    delayms(1000);
    LED1_OFF();
    delayms(1000);
  }
}

void TaskCreate(void){
  OS_ERR err;
  
  //the data come from meter 
  OSTaskCreate((OS_TCB  *)&TCB_METER_RAW,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_meter_raw,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 1,
               (CPU_STK *)&STK_METER_RAW[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  //MBUS ����
  OSTaskCreate((OS_TCB  *)&TCB_OVERLORD,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_overload,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 2,
               (CPU_STK *)&STK_OVERLORD[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  //OS_CFG_TICK_TASK_PRIO == 6 = APP_START_TASK_PRIO + 3
  
  //the data come from the server 
  OSTaskCreate((OS_TCB  *)&TCB_SERVER,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_server,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 4,
               (CPU_STK *)&STK_SERVER[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  //deal the server data 
  OSTaskCreate((OS_TCB  *)&TCB_SERVER_ACTION,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_server_action,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 5,
               (CPU_STK *)&STK_SERVER_ACTION[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  
  //deal the lora data 
  OSTaskCreate((OS_TCB  *)&TCB_LORA_RAW,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_lora_raw,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 6,
               (CPU_STK *)&STK_LORA_RAW[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  //deal the cjq data 
  OSTaskCreate((OS_TCB  *)&TCB_CJQ_RAW,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_cjq_raw,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 7,
               (CPU_STK *)&STK_CJQ_RAW[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  //read meter 
  OSTaskCreate((OS_TCB  *)&TCB_READ,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_read,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 8,
               (CPU_STK *)&STK_READ[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  //heart beat 
  OSTaskCreate((OS_TCB  *)&TCB_HEARTBEAT,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_heartbeat,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 9,
               (CPU_STK *)&STK_HEARTBEAT[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  //LORA Check
  OSTaskCreate((OS_TCB  *)&TCB_LORA_CHECK,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_lora_check,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 10,
               (CPU_STK *)&STK_LORA_CHECK[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  //config
  OSTaskCreate((OS_TCB  *)&TCB_CONFIG,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_config,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 11,
               (CPU_STK *)&STK_CONFIG[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  //syn
  OSTaskCreate((OS_TCB  *)&TCB_SYN,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_syn,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 12,
               (CPU_STK *)&STK_SYN[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  //blink the led 
  OSTaskCreate((OS_TCB  *)&TCB_LED,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_led,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 13,
               (CPU_STK *)&STK_LED[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);

}

void ObjCreate(void){
  OS_ERR err;
  
  //OS_MEM
  OSMemCreate((OS_MEM *)&MEM_Buf,
              (CPU_CHAR *)"",
              (void *)&mem_buf[0][0],
              (OS_MEM_QTY)8,
              (OS_MEM_SIZE)256,
              (OS_ERR *)&err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSMemCreate((OS_MEM *)&MEM_ISR,
              (CPU_CHAR *)"",
              (void *)&mem_isr[0][0],
              (OS_MEM_QTY)30,
              (OS_MEM_SIZE)4,
              (OS_ERR *)&err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  //OS_MUTEX;
  OSMutexCreate(&MUTEX_GPRS,"",&err);
  OSMutexCreate(&MUTEX_LORA,"",&err);
  OSMutexCreate(&MUTEX_CJQ,"",&err);
  OSMutexCreate(&MUTEX_METER,"",&err);
  OSMutexCreate(&MUTEX_MEM_4K,"",&err);
  

  //OS_SEM
  OSSemCreate(&SEM_LORA_OK,
              "",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_HEART_BEAT,
              "",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_SERVER_ACK,
              "",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_SEND_GPRS,
              "",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
    
  OSSemCreate(&SEM_CJQ_ACK,
              "",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_SYN,
              "",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  //OS_Q
  //data from slave
  OSQCreate(&Q_CJQ_USART,
            "",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_METER_USART,
            "",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_LORA_USART,
            "",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_CJQ,
            "",
            2,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_METER,
            "",
            2,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_READ,
            "",
            2,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_CONFIG,
            "",
            2,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_SERVER_ACTION,
            "",
            2,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  //OS_FLAGS
  OSFlagCreate(&FLAG_Event,
               "",
               (OS_FLAGS)0,
               &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  //OS_TMRs
  /*
  OSTmrCreate(&TMR_CJQTIMEOUT,
              "",
              12000,
              0,
              OS_OPT_TMR_ONE_SHOT,
              cjq_timeout,
              0,
              &err);
  */
}



