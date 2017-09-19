

#ifndef SPI_FLASH_H

#define SPI_FLASH_H

#include "stm32f10x_conf.h"

/*
  W25X16
*/

#define FLASH_ID 0XEF14
//指令表
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


#define sFLASH_CON_START_ADDR         sFLASH_START_ADDR   //第一个section保存需要的配置信息
#define sFLASH_CON_APN          sFLASH_CON_START_ADDR
#define sFLASH_CON_USER         sFLASH_CON_START_ADDR + 0x10
#define sFLASH_CON_PASSWORD     sFLASH_CON_START_ADDR + 0x20
#define sFLASH_CON_WEB          sFLASH_CON_START_ADDR + 0x30    //使用0x20 只使用了第一个字节  表示连接到那个域名 //0x00~测试过了~IP   0xFF~未测试~域名（avenger0422.vicp.cc）
#define sFLASH_CON_IP           sFLASH_CON_START_ADDR + 0x50    //使用0x20
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

#define sFLASH_POOL_SIZE        0x400  //1K    //将511个Sector 分成2044个1k大小的存储块
#define sFLASH_POOL_NUM         (sFLASH_CON_START_ADDR - sFLASH_START_ADDR)/sFLASH_POOL_SIZE  //2044
    
#define sFLASH_PAGE_SIZE        0x100  //256
    
#define sFLASH_CJQ_Q_START      sFLASH_CON_START_ADDR + 0xA0
#define sFLASH_CJQ_COUNT        sFLASH_CJQ_Q_START + 3        //采集器的数量
#define sFLASH_CJQ_Q_LAST       sFLASH_CJQ_COUNT + 2        //最后一个采集器的地址


#define sFLASH_METER_Q_START    sFLASH_CON_START_ADDR + 0xB0
#define sFLASH_METER_COUNT      sFLASH_METER_Q_START + 3      //表的数量
#define sFLASH_METER_Q_LAST     sFLASH_METER_COUNT + 2        //最后一个采集器的地址
    
#define sFLASH_POOL_INIT        sFLASH_METER_Q_LAST + 3          //FLASH 初始化没有  0xAA 初始化了    0xFF 没有初始化

#define sFLASH_METER_MBUS    sFLASH_CON_START_ADDR + 0xC0     //是否采用MBUS   0xAA MBUS表   0xFF  没有MBUS表(default)   
#define sFLASH_READMETER_DI_SEQ    sFLASH_METER_MBUS + 0x01     //DI0 DI1 顺序   0xAA~DI1在前(千宝通)   0xFF~DI0在前(default)   
#define sFLASH_ACK_ACTION    sFLASH_READMETER_DI_SEQ + 0x01     //先应答后操作~0xaa    先操作后应答~0xff   
#define sFLASH_PROTOCOL    sFLASH_ACK_ACTION + 0x01     //协议类型 0xFF~188(Default)  1~EG  
#define sFLASH_SIMCARD    sFLASH_PROTOCOL + 0x01     //0xff~移动  0xaa~联通 
#define sFLASH_METER_BAUD    sFLASH_SIMCARD + 0x01     ////96H——9600 bps; 48H——4800 bps; 24H——2400 bps; 12H——1200 bps


//*************************************************************
#define FLASH_POOL_NEXT_INDEX   3

#define CJQ_FLASH_INDEX_ADDR    6   //5byte  采集器地址
#define CJQ_FLASH_INDEX_FIRSTMETER       CJQ_FLASH_INDEX_ADDR+5   //3byte  第一个表地址
#define CJQ_FLASH_INDEX_LASTMETER        CJQ_FLASH_INDEX_FIRSTMETER+3  //3byte  最后一个表地址
#define CJQ_FLASH_INDEX_METERCOUNT      CJQ_FLASH_INDEX_LASTMETER+3   //2byte 表数量
#define CJQ_FLASH_INDEX_PREVCJQ         CJQ_FLASH_INDEX_METERCOUNT+2   //3byte 上一个采集器
#define CJQ_FLASH_INDEX_CJQSTATE        CJQ_FLASH_INDEX_PREVCJQ+3   //1byte  采集器状态
    

#define METER_FLASH_INDEX_ADDR 6   //7byte  表地址
#define METER_FLASH_INDEX_TYPE          METER_FLASH_INDEX_ADDR+7   //1byte  表类型
#define METER_FLASH_INDEX_READ          METER_FLASH_INDEX_TYPE+1   //4byte  表读数
#define METER_FLASH_INDEX_PREVMETER          METER_FLASH_INDEX_READ+4   //3byte  上一个表地址
#define METER_FLASH_INDEX_DATA          METER_FLASH_INDEX_PREVMETER+3   //1byte  数据项
#define METER_FLASH_INDEX_METERSTATE          METER_FLASH_INDEX_DATA+1   //2byte  表状态
#define METER_FLASH_INDEX_HALFREAD          METER_FLASH_INDEX_METERSTATE+2   //4byte  半位



    
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
void sFLASH_PoolInit(void);   //初始化flash块池
void sFLASH_Init(void);   //初始化flash模块

/*
  根据sFLASH_POOL的地址获取空闲块
  并返回空闲块的地址
*/
uint32_t GetFlash(void);
/*
  将Flash块放回到sFLASH_POOL中
*/
uint8_t PutFlash(uint32_t put);

void param_conf(void);  //从Flash中读数参数的配置信息

/**
  * @brief  Low layer functions
  */
uint8_t sFLASH_ReadByte(void);
uint8_t sFLASH_SendByte(uint8_t byte);
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);

#endif