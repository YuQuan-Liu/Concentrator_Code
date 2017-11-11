
#include "includes.h"
#include "tasks.h"

/*
* 所有任务TCB STK
*/

//OS_TCBs
OS_TCB  TCB_Start;
CPU_STK STK_Start[APP_START_TASK_STK_SIZE];


//处理采集器、表发送过来的数据
OS_TCB TCB_METER_RAW;
CPU_STK STK_METER_RAW[APP_START_TASK_STK_SIZE];

OS_TCB TCB_CJQ_RAW;
CPU_STK STK_CJQ_RAW[APP_START_TASK_STK_SIZE];

OS_TCB TCB_LORA_RAW;
CPU_STK STK_LORA_RAW[APP_START_TASK_STK_SIZE];

// 检查LORA是否正常  安装时测试LORA信号
OS_TCB TCB_LORA_CHECK;
CPU_STK STK_LORA_CHECK[APP_START_TASK_STK_SIZE];

//抄表任务
OS_TCB TCB_READ;
CPU_STK STK_READ[APP_START_TASK_STK_SIZE*3];

//设置任务
OS_TCB TCB_CONFIG;
CPU_STK STK_CONFIG[APP_START_TASK_STK_SIZE*3];

OS_TCB TCB_LED;
CPU_STK STK_LED[APP_START_TASK_STK_SIZE];

//MBUS 超载
OS_TCB TCB_OVERLORD;
CPU_STK STK_OVERLORD[APP_START_TASK_STK_SIZE];

/*
* 所有需要的全局MEM
*/
//OS_MEMs
OS_MEM MEM_Buf;
uint8_t mem_buf[6][256];

OS_MEM MEM_ISR;  //be used in the ISR   get the data from the usart*  post it to the deal task
uint8_t mem_isr[30][4];

//OS_MUTEXs;  需要写lora cjq meter 时需要获取锁
OS_MUTEX MUTEX_LORA;
OS_MUTEX MUTEX_CJQ;
OS_MUTEX MUTEX_METER;
OS_MUTEX MUTEX_MEM_4K;

//OS_SEMs ;
OS_SEM SEM_LORA_OK;      //接收到LORA返回的  0D 0A 4F 4B 0D 0A 
OS_SEM SEM_JZQ_ACK;      //集中器对采集器的ACK

//OS_Qs
OS_Q Q_CJQ_USART;  //CJQ USART接收数据
OS_Q Q_METER_USART; //METER USART接收数据
OS_Q Q_LORA_USART;  //LORA USART接收数据
OS_Q Q_METER;   //Q_METER_USART处理后的帧
OS_Q Q_READ;    //去抄表的帧
OS_Q Q_CONFIG;  //去设置的帧

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
  
  uint8_t test = 0;////
  uint8_t testwrite = 0;////
  uint32_t sectionaddr = 0x000000;////
  uint8_t * mem4k = 0;////
  uint32_t block_new = 0x1C00;////
  uint32_t block_last = 0x1C00;////
  
  CPU_Init();
  
  cpu_clk_freq = BSP_CPU_ClkFreq();
  cnts = cpu_clk_freq / (CPU_INT32U)OS_CFG_TICK_RATE_HZ;
  OS_CPU_SysTickInit(cnts);
  
  ObjCreate();
  //初始化FLASH  w25x16
  sFLASH_Init();
  BSP_USART_Init();
  /**/
  TaskCreate();
  
  if(test){
    mem4k = get_mem4k();
    while(1){
      asm("NOP");
      sFLASH_ReadBuffer(mem4k,sectionaddr,0x1000);
      if(testwrite){
        sFLASH_WritePage((uint8_t *)&block_new,block_last + FLASH_POOL_NEXT_INDEX,3);  
      }
    }
  }
  
  //Open the IWDG;
  BSP_IWDG_Init();
  
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
  
  //MBUS 过载
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
  
  //deal the lora data 
  OSTaskCreate((OS_TCB  *)&TCB_LORA_RAW,
               (CPU_CHAR *)"",
               (OS_TASK_PTR )task_lora_raw,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 4,
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
               (OS_PRIO )APP_START_TASK_PRIO + 5,
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
               (OS_PRIO )APP_START_TASK_PRIO + 6,
               (CPU_STK *)&STK_READ[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3,
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
               (OS_PRIO )APP_START_TASK_PRIO + 8,
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
               (OS_PRIO )APP_START_TASK_PRIO + 9,
               (CPU_STK *)&STK_CONFIG[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3,
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
               (OS_PRIO )APP_START_TASK_PRIO + 10,
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
              (OS_MEM_QTY)6,
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
    
  OSSemCreate(&SEM_JZQ_ACK,
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
  
  //OS_FLAGS
  OSFlagCreate(&FLAG_Event,
               "",
               (OS_FLAGS)0,
               &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
}



