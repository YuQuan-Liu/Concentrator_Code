


#include "includes.h"
#include "tasks.h"



//OS_TCBs
OS_TCB  TCB_Start;
CPU_STK STK_Start[APP_START_TASK_STK_SIZE];


//处理采集器、表发送过来的数据
OS_TCB TCB_485_2;
CPU_STK STK_485_2[APP_START_TASK_STK_SIZE];

OS_TCB TCB_LORA;
CPU_STK STK_LORA[APP_START_TASK_STK_SIZE];

//处理服务器发送过来的数据
OS_TCB TCB_Server;
CPU_STK STK_Server[APP_START_TASK_STK_SIZE];

//task deal the data "+TCPRECV"
OS_TCB TCB_DealServer;
CPU_STK STK_DealServer[APP_START_TASK_STK_SIZE];

//the heartbeat task
OS_TCB TCB_HeartBeat;
CPU_STK STK_HeartBeat[APP_START_TASK_STK_SIZE];

//task deal the overload
OS_TCB TCB_LORA_Check;
CPU_STK STK_LORA_Check[APP_START_TASK_STK_SIZE];

//抄表任务
OS_TCB TCB_Read;
CPU_STK STK_Read[APP_START_TASK_STK_SIZE*3];

//设置任务
OS_TCB TCB_Config;
CPU_STK STK_Config[APP_START_TASK_STK_SIZE*3];

OS_TCB TCB_LORA_Send;
CPU_STK STK_LORA_Send[APP_START_TASK_STK_SIZE];

OS_TCB TCB_LED;
CPU_STK STK_LED[APP_START_TASK_STK_SIZE];

//OS_MEMs
//receive the data from the ISR    put the data in the mem to deal buf
OS_MEM MEM_Buf;
uint8_t mem_buf[6][256];

//be used in the ISR   get the data from the usart*  post it to the deal task
OS_MEM MEM_ISR;
uint8_t mem_isr[30][4];

//配置处理Flash使用的数组  Sector==4K  需要一个4K的数组
uint8_t config_flash[0x1000];
uint8_t *meterdata;  //使用海大协议抄表时存放返回的信息  使用config_flash

//OS_MUTEXs;
OS_MUTEX MUTEX_CONFIGFLASH;    //是否可以使用 config_flash  4K 数组配置FLASH
OS_MUTEX MUTEX_SENDSERVER;    //是否可以发送数据到服务器
OS_MUTEX MUTEX_SENDLORA;    //是否可以发送数据到LORA

//OS_SEMs ;

OS_SEM SEM_USART1_TX;    //往服务器发送数据
OS_SEM SEM_USART2_TX;     //往采集器、表发送数据
OS_SEM SEM_UART4_TX;

OS_SEM SEM_LORA_OK;   //接收到LORA返回的  0D 0A 4F 4B 0D 0A 
OS_SEM SEM_HeartBeat;    //接收服务器数据Task to HeartBeat Task  接收到心跳的回应
OS_SEM SEM_ACKData;    //服务器对数据的ACK
OS_SEM SEM_Send;      //got the '>'  we can send the data now  可以发送数据

//OS_Qs
OS_Q Q_485_2;            //采集器、表发送过来的数据
OS_Q Q_LORA;
OS_Q Q_Read;             //抄表任务Queue
OS_Q Q_ReadData;        //发送抄表指令后  下层返回抄表数据
OS_Q Q_ReadData_LORA;   //通过LORA返回的抄表结果 应答
OS_Q Q_Config;         //配置任务Queue
OS_Q Q_Deal;         //处理接收到的服务器发送过来的数据

//OS_TMR
OS_TMR TMR_CJQTIMEOUT;    //打开采集器通道之后 20分钟超时 自动关闭通道


uint8_t * volatile server_ptr = 0;      //中断中保存GPRS 返回来的数据
uint8_t * volatile server_ptr_ = 0;     //记录中断的开始指针


volatile uint8_t connectstate = 0;       //0 didn't connect to the server   1 connect to the server
volatile uint8_t reading = 0;   //0 didn't reading meters    1  reading meters

volatile uint8_t lora_send = 0;  //每3s发送TEST到LORA

uint8_t ack_action = 0xff;  //先应答后操作~0xaa    先操作后应答~0xff
uint8_t slave_mbus = 0xbb; //0xaa~mbus   0xff~485   0xbb~采集器
uint8_t di_seq; //DI0 DI1 顺序   0xAA~DI1在前(千宝通)   0xFF~DI0在前(default)  
uint8_t protocol = 0x01;  //协议类型 0xFF~188(Default)  1~EG 

uint8_t deviceaddr[5] = {0x99,0x09,0x00,0x00,0x57};      //集中器地址
uint8_t cjqaddr[5] = {0x01,0x00,0x00,0x00,0x00};     //正在抄表的采集器地址

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
  OS_ERR err;
  uint32_t flashid;
  
  CPU_Init();
  
  cpu_clk_freq = BSP_CPU_ClkFreq();
  cnts = cpu_clk_freq / (CPU_INT32U)OS_CFG_TICK_RATE_HZ;
  OS_CPU_SysTickInit(cnts);
  
  while(DEF_TRUE){
    //check the w25x16 是否存在
    
    flashid = sFLASH_ReadID();
    if(FLASH_ID == flashid){
      break;
    }
    OSTimeDly(100,
              OS_OPT_TIME_DLY,
              &err);
  }
  
  sFLASH_PoolInit();
  /**/
  TaskCreate();
  ObjCreate();
  
  
  //Open the IWDG;
  //BSP_IWDG_Init();
  
  while(DEF_TRUE){
    /* Reload IWDG counter */
    IWDG_ReloadCounter();
    
    //LED1
    LED1_ON();
    OSTimeDly(1000,
              OS_OPT_TIME_DLY,
              &err);
    LED1_OFF();
    OSTimeDly(1000,
              OS_OPT_TIME_DLY,
              &err);
  }
}

void TaskCreate(void){
  OS_ERR err;
  
  /*the data come from slave */
  OSTaskCreate((OS_TCB  *)&TCB_485_2,
               (CPU_CHAR *)"U2",
               (OS_TASK_PTR )Task_485_2,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 1,
               (CPU_STK *)&STK_485_2[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  /*the data come from the server */
  OSTaskCreate((OS_TCB  *)&TCB_Server,
               (CPU_CHAR *)"U1",
               (OS_TASK_PTR )Task_Server,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 2,
               (CPU_STK *)&STK_Server[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  //OS_CFG_TICK_TASK_PRIO == 6 = APP_START_TASK_PRIO + 3
  
  OSTaskCreate((OS_TCB  *)&TCB_LORA,
               (CPU_CHAR *)"U4",
               (OS_TASK_PTR )Task_LORA,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 4,
               (CPU_STK *)&STK_LORA[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  /*deal the server data */
  OSTaskCreate((OS_TCB  *)&TCB_DealServer,
               (CPU_CHAR *)"deal",
               (OS_TASK_PTR )Task_DealServer,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 5,
               (CPU_STK *)&STK_DealServer[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  /*read meter */
  OSTaskCreate((OS_TCB  *)&TCB_Read,
               (CPU_CHAR *)"read",
               (OS_TASK_PTR )Task_Read,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 6,
               (CPU_STK *)&STK_Read[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  /*heart beat */
  OSTaskCreate((OS_TCB  *)&TCB_HeartBeat,
               (CPU_CHAR *)"heart",
               (OS_TASK_PTR )Task_HeartBeat,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 7,
               (CPU_STK *)&STK_HeartBeat[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  /*LORA Check */
  OSTaskCreate((OS_TCB  *)&TCB_LORA_Check,
               (CPU_CHAR *)"lora_check",
               (OS_TASK_PTR )Task_LORA_Check,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 8,
               (CPU_STK *)&STK_LORA_Check[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  /*config */
  OSTaskCreate((OS_TCB  *)&TCB_Config,
               (CPU_CHAR *)"config",
               (OS_TASK_PTR )Task_Config,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 9,
               (CPU_STK *)&STK_Config[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  /*blink the led 3*/
  OSTaskCreate((OS_TCB  *)&TCB_LORA_Send,
               (CPU_CHAR *)"lora_send",
               (OS_TASK_PTR )Task_LORA_Send,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 10,
               (CPU_STK *)&STK_LORA_Send[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  //blink the led 2
  /**/
  OSTaskCreate((OS_TCB  *)&TCB_LED,
               (CPU_CHAR *)"LED",
               (OS_TASK_PTR )Task_LED,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 11,
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
              (CPU_CHAR *)"frame",
              (void *)&mem_buf[0][0],
              (OS_MEM_QTY)6,
              (OS_MEM_SIZE)256,
              (OS_ERR *)&err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSMemCreate((OS_MEM *)&MEM_ISR,
              (CPU_CHAR *)"isr",
              (void *)&mem_isr[0][0],
              (OS_MEM_QTY)30,
              (OS_MEM_SIZE)4,
              (OS_ERR *)&err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  //OS_MUTEX;
  //OS_MUTEX MUTEX_CONFIGFLASH;    //是否可以使用 config_flash  4K 数组配置FLASH
  OSMutexCreate(&MUTEX_CONFIGFLASH,"",&err);
  OSMutexCreate(&MUTEX_SENDSERVER,"",&err);
  OSMutexCreate(&MUTEX_SENDLORA,"",&err);
  //OS_SEM
  OSSemCreate(&SEM_USART1_TX,
              "u1_tx",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_USART2_TX,
              "u2_tx",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_UART4_TX,
              "u4_tx",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_LORA_OK,
              "lora_ok",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
    
  OSSemCreate(&SEM_HeartBeat,
              "heart",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_ACKData,
              "ackdata",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_Send,
              "tcpsend",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  //OS_Q
  //data from slave
  OSQCreate(&Q_485_2,
            "485_2",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_LORA,
            "lora",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_Read,
            "read",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_ReadData,
            "readdata",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_ReadData_LORA,
            "loradata",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_Config,
            "config",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_Deal,
            "deal",
            4,
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



