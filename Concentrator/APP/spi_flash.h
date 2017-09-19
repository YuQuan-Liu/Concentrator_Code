

#ifndef SPI_FLASH_H

#define SPI_FLASH_H

#include "stm32f10x_conf.h"

/*
  W25X16
*/

#define FLASH_ID 0XEF14
//ָ���
#define W25X_WRITE_EN		        0x06 
#define W25X_WRITE_DIS		        0x04 
#define W25X_RDSR		        0x05 
#define W25X_WRSR		        0x01 
#define W25X_READ		        0x03 
#define W25X_FAST_READ	        	0x0B 
#define W25X_FAST_READ_DUAL		0x3B 
#define W25X_PAGE_PROGRAM		0x02 
#define W25X_BLOCK_ERASE		0xD8 
#define W25X_SECTOR_ERASE		0x20 
#define W25X_CHIP_ERASE			0xC7 
#define W25X_POWER_DOWN			0xB9 
#define W25X_RELEASE_POWER_DOWN	        0xAB 
#define W25X_DEVICEID			0xAB 
#define W25X_RD_DEVICEID        	0x90 
#define W25X_JEDECID    		0x9F

/**
  * @brief  Deselect sFLASH: Chip Select pin low
  */
#define sFLASH_CS_LOW()         GPIO_ResetBits(GPIOA, GPIO_Pin_4)
/**
  * @brief  Deselect sFLASH: Chip Select pin high
  */
#define sFLASH_CS_HIGH()        GPIO_SetBits(GPIOA, GPIO_Pin_4)

#define sFLASH_SPI              SPI1    /*!<the spi flash use>*/
#define sFLASH_DUMMY_BYTE       0xA5
#define sFLASH_WIP_FLAG         0x01    /*!< Write In Progress (WIP) flag */
#define sFLASH_SPI_PAGESIZE     0x100

#define sFLASH_START_ADDR       0x000000
#define sFLASH_END_ADDR         0x1FFFFF
#define sFLASH_POOL_START_ADDR       sFLASH_START_ADDR+sFLASH_SECTOR_SIZE


#define sFLASH_CON_START_ADDR         sFLASH_START_ADDR   //��һ��section������Ҫ��������Ϣ
#define sFLASH_CON_APN          sFLASH_CON_START_ADDR
#define sFLASH_CON_USER         sFLASH_CON_START_ADDR + 0x10
#define sFLASH_CON_PASSWORD     sFLASH_CON_START_ADDR + 0x20
#define sFLASH_CON_WEB          sFLASH_CON_START_ADDR + 0x30    //ʹ��0x20 ֻʹ���˵�һ���ֽ�  ��ʾ���ӵ��Ǹ����� //0x00~���Թ���~IP   0xFF~δ����~������avenger0422.vicp.cc��
#define sFLASH_CON_IP           sFLASH_CON_START_ADDR + 0x50    //ʹ��0x20
#define sFLASH_CON_IP1           sFLASH_CON_START_ADDR + 0x70-1    
#define sFLASH_CON_IP2           sFLASH_CON_START_ADDR + 0x70-2    
#define sFLASH_CON_IP3           sFLASH_CON_START_ADDR + 0x70-3    
#define sFLASH_CON_IP4           sFLASH_CON_START_ADDR + 0x70-4    
#define sFLASH_CON_PORT         sFLASH_CON_START_ADDR + 0x70
#define sFLASH_CON_PORT_         sFLASH_CON_START_ADDR + 0x80 - 2
#define sFLASH_DEVICE_ADDR      sFLASH_CON_START_ADDR + 0x80
    
#define sFLASH_POOL             sFLASH_CON_START_ADDR + 0x90
#define sFLASH_POOL_FREE        sFLASH_POOL + 3
#define sFLASH_POOL_USED        sFLASH_POOL_FREE + 2    
#define sFLASH_POOL_ALL         sFLASH_POOL_USED + 2   
#define sFLASH_SECTOR_SIZE        0x1000  //4K
#define sFLASH_SECTOR_NUM         (sFLASH_END_ADDR - sFLASH_START_ADDR + 1)/sFLASH_SECTOR_SIZE  //512

#define sFLASH_POOL_SIZE        0x400  //1K    //��511��Sector �ֳ�2044��1k��С�Ĵ洢��
#define sFLASH_POOL_NUM         (sFLASH_CON_START_ADDR - sFLASH_START_ADDR)/sFLASH_POOL_SIZE  //2044
    
#define sFLASH_PAGE_SIZE        0x100  //256
    
#define sFLASH_CJQ_Q_START      sFLASH_CON_START_ADDR + 0xA0
#define sFLASH_CJQ_COUNT        sFLASH_CJQ_Q_START + 3        //�ɼ���������
#define sFLASH_CJQ_Q_LAST       sFLASH_CJQ_COUNT + 2        //���һ���ɼ����ĵ�ַ


#define sFLASH_METER_Q_START    sFLASH_CON_START_ADDR + 0xB0
#define sFLASH_METER_COUNT      sFLASH_METER_Q_START + 3      //�������
#define sFLASH_METER_Q_LAST     sFLASH_METER_COUNT + 2        //���һ���ɼ����ĵ�ַ
    
#define sFLASH_POOL_INIT        sFLASH_METER_Q_LAST + 3          //FLASH ��ʼ��û��  0xAA ��ʼ����    0xFF û�г�ʼ��

#define sFLASH_METER_MBUS    sFLASH_CON_START_ADDR + 0xC0     //�Ƿ����MBUS   0xAA MBUS��   0xFF  û��MBUS��(default)   
#define sFLASH_READMETER_DI_SEQ    sFLASH_METER_MBUS + 0x01     //DI0 DI1 ˳��   0xAA~DI1��ǰ(ǧ��ͨ)   0xFF~DI0��ǰ(default)   
#define sFLASH_ACK_ACTION    sFLASH_READMETER_DI_SEQ + 0x01     //��Ӧ������~0xaa    �Ȳ�����Ӧ��~0xff   
#define sFLASH_PROTOCOL    sFLASH_ACK_ACTION + 0x01     //Э������ 0xFF~188(Default)  1~EG  
#define sFLASH_SIMCARD    sFLASH_PROTOCOL + 0x01     //0xff~�ƶ�  0xaa~��ͨ 
#define sFLASH_METER_BAUD    sFLASH_SIMCARD + 0x01     ////96H����9600 bps; 48H����4800 bps; 24H����2400 bps; 12H����1200 bps


//*************************************************************
#define FLASH_POOL_NEXT_INDEX   3

#define CJQ_FLASH_INDEX_ADDR    6   //5byte  �ɼ�����ַ
#define CJQ_FLASH_INDEX_FIRSTMETER       CJQ_FLASH_INDEX_ADDR+5   //3byte  ��һ�����ַ
#define CJQ_FLASH_INDEX_LASTMETER        CJQ_FLASH_INDEX_FIRSTMETER+3  //3byte  ���һ�����ַ
#define CJQ_FLASH_INDEX_METERCOUNT      CJQ_FLASH_INDEX_LASTMETER+3   //2byte ������
#define CJQ_FLASH_INDEX_PREVCJQ         CJQ_FLASH_INDEX_METERCOUNT+2   //3byte ��һ���ɼ���
#define CJQ_FLASH_INDEX_CJQSTATE        CJQ_FLASH_INDEX_PREVCJQ+3   //1byte  �ɼ���״̬
    

#define METER_FLASH_INDEX_ADDR 6   //7byte  ���ַ
#define METER_FLASH_INDEX_TYPE          METER_FLASH_INDEX_ADDR+7   //1byte  ������
#define METER_FLASH_INDEX_READ          METER_FLASH_INDEX_TYPE+1   //4byte  �����
#define METER_FLASH_INDEX_PREVMETER          METER_FLASH_INDEX_READ+4   //3byte  ��һ�����ַ
#define METER_FLASH_INDEX_DATA          METER_FLASH_INDEX_PREVMETER+3   //1byte  ������
#define METER_FLASH_INDEX_METERSTATE          METER_FLASH_INDEX_DATA+1   //2byte  ��״̬
#define METER_FLASH_INDEX_HALFREAD          METER_FLASH_INDEX_METERSTATE+2   //4byte  ��λ



    
void sFLASH_DeInit(void);
void sFLASH_Init(void);
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_EraseBulk(void);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_EraseWritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sFLASH_ReadID(void);
void sFLASH_StartReadSequence(uint32_t ReadAddr);
void sFLASH_PoolInit(void);   //��ʼ��flash���
void sFLASH_Init(void);   //��ʼ��flashģ��

/*
  ����sFLASH_POOL�ĵ�ַ��ȡ���п�
  �����ؿ��п�ĵ�ַ
*/
uint32_t GetFlash(void);
/*
  ��Flash��Żص�sFLASH_POOL��
*/
uint8_t PutFlash(uint32_t put);

void param_conf(void);  //��Flash�ж���������������Ϣ

/**
  * @brief  Low layer functions
  */
uint8_t sFLASH_ReadByte(void);
uint8_t sFLASH_SendByte(uint8_t byte);
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);

#endif