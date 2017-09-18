


#include "spi_flash.h"
#include "stm32f10x_conf.h"
#include "os.h"
#include "utils.h"
#include "device_params.h"

/**
  * @brief  Erases the specified FLASH sector.
  * @param  SectorAddr: address of the sector to erase.
  * @retval None
  */
void sFLASH_EraseSector(uint32_t SectorAddr)
{
  /*!< Send write enable instruction */
  sFLASH_WriteEnable();

  /*!< Sector Erase */
  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();
  /*!< Send Sector Erase instruction */
  sFLASH_SendByte(W25X_SECTOR_ERASE);
  /*!< Send SectorAddr high nibble address byte */
  sFLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  /*!< Send SectorAddr medium nibble address byte */
  sFLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  /*!< Send SectorAddr low nibble address byte */
  sFLASH_SendByte(SectorAddr & 0xFF);
  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();

  /*!< Wait the end of Flash writing */
  sFLASH_WaitForWriteEnd();
}

/**
  * @brief  Erases the entire FLASH.
  * @param  None
  * @retval None
  */
void sFLASH_EraseBulk(void)
{
  OS_ERR err;
  /*!< Send write enable instruction */
  sFLASH_WriteEnable();
  sFLASH_WaitForWriteEnd();
  /*!< Bulk Erase */
  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();
  /*!< Send Bulk Erase instruction  */
  sFLASH_SendByte(W25X_CHIP_ERASE);
  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();
  
  /*
  OSTimeDlyHMSM(0,0,0,500,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  */

  /*!< Wait the end of Flash writing */
  sFLASH_WaitForWriteEnd();
}

/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE cycle 
  *         (Page WRITE sequence).
  * @note   The number of byte can't exceed the FLASH page size.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
  *         or less than "sFLASH_PAGESIZE" value.
  * @retval None
  */
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  /*!< Enable the write access to the FLASH */
  sFLASH_WriteEnable();

  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();
  /*!< Send "Write to Memory " instruction */
  sFLASH_SendByte(W25X_PAGE_PROGRAM);
  /*!< Send WriteAddr high nibble address byte to write to */
  sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /*!< Send WriteAddr medium nibble address byte to write to */
  sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /*!< Send WriteAddr low nibble address byte to write to */
  sFLASH_SendByte(WriteAddr & 0xFF);

  /*!< while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /*!< Send the current byte */
    sFLASH_SendByte(*pBuffer);
    /*!< Point on the next byte to be written */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();

  /*!< Wait the end of Flash writing */
  sFLASH_WaitForWriteEnd();
}

/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE cycle 
  *         (Page WRITE sequence).
  * @note   The number of byte can't exceed the FLASH page size.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
  *         or less than "sFLASH_PAGESIZE" value.
  * @retval None
  */
void sFLASH_EraseWritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  /*!< Erase the sector */
  sFLASH_EraseSector(WriteAddr);
  /*!< Enable the write access to the FLASH */
  sFLASH_WriteEnable();
  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();
  /*!< Send "Write to Memory " instruction */
  sFLASH_SendByte(W25X_PAGE_PROGRAM);
  /*!< Send WriteAddr high nibble address byte to write to */
  sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /*!< Send WriteAddr medium nibble address byte to write to */
  sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /*!< Send WriteAddr low nibble address byte to write to */
  sFLASH_SendByte(WriteAddr & 0xFF);

  /*!< while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /*!< Send the current byte */
    sFLASH_SendByte(*pBuffer);
    /*!< Point on the next byte to be written */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();

  /*!< Wait the end of Flash writing */
  sFLASH_WaitForWriteEnd();
}

/**
  * @brief  Writes block of data to the FLASH. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the FLASH.
  * @retval None
  */
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % sFLASH_SPI_PAGESIZE;
  count = sFLASH_SPI_PAGESIZE - Addr;
  NumOfPage =  NumByteToWrite / sFLASH_SPI_PAGESIZE;
  NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

  if (Addr == 0) /*!< WriteAddr is sFLASH_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
    {
      sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /*!< NumByteToWrite > sFLASH_PAGESIZE */
    {
      while (NumOfPage--)
      {
        sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
        WriteAddr +=  sFLASH_SPI_PAGESIZE;
        pBuffer += sFLASH_SPI_PAGESIZE;
      }

      sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /*!< WriteAddr is not sFLASH_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
    {
      if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > sFLASH_PAGESIZE */
      {
        temp = NumOfSingle - count;

        sFLASH_WritePage(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        sFLASH_WritePage(pBuffer, WriteAddr, temp);
      }
      else
      {
        sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /*!< NumByteToWrite > sFLASH_PAGESIZE */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / sFLASH_SPI_PAGESIZE;
      NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

      sFLASH_WritePage(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
        WriteAddr +=  sFLASH_SPI_PAGESIZE;
        pBuffer += sFLASH_SPI_PAGESIZE;
      }

      if (NumOfSingle != 0)
      {
        sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

/**
  * @brief  Reads a block of data from the FLASH.
  * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
  * @param  ReadAddr: FLASH's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the FLASH.
  * @retval None
  */
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "Read from Memory " instruction */
  sFLASH_SendByte(W25X_READ);

  /*!< Send ReadAddr high nibble address byte to read from */
  sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /*!< Send ReadAddr medium nibble address byte to read from */
  sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /*!< Send ReadAddr low nibble address byte to read from */
  sFLASH_SendByte(ReadAddr & 0xFF);
  
  while (NumByteToRead--) /*!< while there is data to be read */
  {
    /*!< Read a byte from the FLASH */
    *pBuffer = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
    /*!< Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the FLASH.
  * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
  * @param  ReadAddr: FLASH's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the FLASH.
  * @retval None
  */
void sFLASH_ReadStr(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "Read from Memory " instruction */
  sFLASH_SendByte(W25X_READ);

  /*!< Send ReadAddr high nibble address byte to read from */
  sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /*!< Send ReadAddr medium nibble address byte to read from */
  sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /*!< Send ReadAddr low nibble address byte to read from */
  sFLASH_SendByte(ReadAddr & 0xFF);
  
  while (NumByteToRead--) /*!< while there is data to be read */
  {
    /*!< Read a byte from the FLASH */
    *pBuffer = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
    
    //if it is the end of the str then break
    if(*pBuffer == '\0'){
      break;
    }
    
    /*!< Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();
}

/**
  * @brief  Reads FLASH identification.
  * @param  None
  * @retval FLASH identification
  */
uint32_t sFLASH_ReadID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "RDID " instruction */
  sFLASH_SendByte(W25X_RD_DEVICEID);

  
  sFLASH_SendByte(sFLASH_DUMMY_BYTE);
  sFLASH_SendByte(sFLASH_DUMMY_BYTE);
  sFLASH_SendByte(0x00);
  
  /*!< Read a byte from the FLASH */
  Temp0 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

  /*!< Read a byte from the FLASH */
  Temp1 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();

  Temp = (Temp0 << 8) | Temp1;

  return Temp;
}

/**
  * @brief  Initiates a read data byte (READ) sequence from the Flash.
  *   This is done by driving the /CS line low to select the device, then the READ
  *   instruction is transmitted followed by 3 bytes address. This function exit
  *   and keep the /CS line low, so the Flash still being selected. With this
  *   technique the whole content of the Flash is read with a single READ instruction.
  * @param  ReadAddr: FLASH's internal address to read from.
  * @retval None
  */
void sFLASH_StartReadSequence(uint32_t ReadAddr)
{
  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "Read from Memory " instruction */
  sFLASH_SendByte(W25X_READ);

  /*!< Send the 24-bit address of the address to read from -------------------*/
  /*!< Send ReadAddr high nibble address byte */
  sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /*!< Send ReadAddr medium nibble address byte */
  sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /*!< Send ReadAddr low nibble address byte */
  sFLASH_SendByte(ReadAddr & 0xFF);
}

/**
  * @brief  Reads a byte from the SPI Flash.
  * @note   This function must be used only if the Start_Read_Sequence function
  *         has been previously called.
  * @param  None
  * @retval Byte Read from the SPI Flash.
  */
uint8_t sFLASH_ReadByte(void)
{
  return (sFLASH_SendByte(sFLASH_DUMMY_BYTE));
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t sFLASH_SendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(sFLASH_SPI, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(sFLASH_SPI);
}

/**
  * @brief  Sends a Half Word through the SPI interface and return the Half Word
  *         received from the SPI bus.
  * @param  HalfWord: Half Word to send.
  * @retval The value of the received Half Word.
  */
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send Half Word through the sFLASH peripheral */
  SPI_I2S_SendData(sFLASH_SPI, HalfWord);

  /*!< Wait to receive a Half Word */
  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the Half Word read from the SPI bus */
  return SPI_I2S_ReceiveData(sFLASH_SPI);
}

/**
  * @brief  Enables the write access to the FLASH.
  * @param  None
  * @retval None
  */
void sFLASH_WriteEnable(void)
{
  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "Write Enable" instruction */
  sFLASH_SendByte(W25X_WRITE_EN);

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();
}

/**
  * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
  *         status register and loop until write opertaion has completed.
  * @param  None
  * @retval None
  */
void sFLASH_WaitForWriteEnd(void)
{
  uint8_t flashstatus = 0;

  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "Read Status Register" instruction */
  sFLASH_SendByte(W25X_RDSR);

  /*!< Loop as long as the memory is busy with a write cycle */
  do
  {
    /*!< Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    flashstatus = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

  }
  while ((flashstatus & sFLASH_WIP_FLAG) == SET); /* Write in progress */

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();
}

void sFLASH_Init(void){
  uint8_t flashinit = 0x00;
  
  //��һ�ζ�ȡ��ʱ��������ݲ��ԡ�  
  //�������Ϊ�ϵ�� ��һ��ָ�����Ǯ /CS �����ɸ߱�Ϊ��  ���μ��ĵ�/CS �ܽŽ��ܣ�
  //After power-up, /CS must transition from high to low before a new instruction will be accepted.
  sFLASH_ReadBuffer(&flashinit,sFLASH_POOL_INIT,1);  
  sFLASH_ReadBuffer(&flashinit,sFLASH_POOL_INIT,1);
  //sFLASH_EraseBulk();    //erase the chip
  sFLASH_ReadBuffer(&flashinit,sFLASH_POOL_INIT,1);
  if(flashinit == 0xFF){
    sFLASH_PoolInit();
  }
  //the flash is inited
  param_conf();  //read the param from the flash
  
}

uint8_t sFLASH_PoolInit(){
  uint16_t i;
  uint32_t sectionaddr;
  uint32_t nextaddr;
  
  //the flash need init
  sFLASH_EraseBulk();    //erase the chip
  //ֻ��ʼ��ǰ��511��  ���һ���������������Ϣ
  //sFLASH_POOL_NUM == 2044       sFLASH_POOL_SIZE == 1K 
  for(i = 0;i < sFLASH_POOL_NUM;i++){
    sectionaddr = sFLASH_POOL_START_ADDR + sFLASH_POOL_SIZE * i;
    nextaddr = sectionaddr + sFLASH_POOL_SIZE;
    if(i == sFLASH_POOL_NUM - 1){
      nextaddr = 0x000000;
    }
    sFLASH_WritePage((uint8_t *)&sectionaddr,sectionaddr,3);
    sFLASH_WritePage((uint8_t *)&nextaddr,sectionaddr+3,3);
  }
  //the pool start
  nextaddr = sFLASH_POOL_START_ADDR;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_POOL,3);
  //pool free
  nextaddr = 2044;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_POOL_FREE,2);
  //pool used
  nextaddr = 0x000000;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_POOL_USED,2);
  //pool all
  nextaddr = 2044;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_POOL_ALL,2);
  
  //CJQ Q
  nextaddr = 0x000000;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_CJQ_Q_START,3);
  nextaddr = 0x000000;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_CJQ_COUNT,2);
  nextaddr = 0x000000;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_CJQ_Q_LAST,3);
  
  //Meter Q
  nextaddr = 0x000000;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_METER_Q_START,3);
  nextaddr = 0x000000;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_METER_COUNT,2);
  
  nextaddr = 0xABCDAA;
  sFLASH_WritePage((uint8_t *)&nextaddr,sFLASH_POOL_INIT,1);
  
}

uint32_t GetFlash(void){
  uint32_t first = 0;//pool �еĵ�һ�����п�ĵ�ַ
  uint32_t next = 0;//pool �еڶ������п�ĵ�ַ
  uint32_t free = 0;
  uint32_t used = 0;
  uint8_t * mem4k = 0;
  
  sFLASH_ReadBuffer((uint8_t *)&first,sFLASH_POOL,3);
  
  //û�п��п���
  if(first == 0x000000){
    return 0x000000;
  }
  
  if(lock_mem4k()){
    mem4k = get_mem4k();
    sFLASH_ReadBuffer((uint8_t *)&next,first+3,3);
    sFLASH_ReadBuffer((uint8_t *)&free,sFLASH_POOL_FREE,2);
    sFLASH_ReadBuffer((uint8_t *)&used,sFLASH_POOL_USED,2);
    
    free--;
    used++;
    
    //����Config Flash ��
    //��next �ĵ�ַ�浽  sFLASH_POOL
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);   //�����ֶ�����ֻʹ����1��page ����ʹ��256
    Mem_Copy(mem4k + (sFLASH_POOL - sFLASH_CON_START_ADDR),(uint8_t *)&next,3);
    Mem_Copy(mem4k + (sFLASH_POOL_FREE - sFLASH_CON_START_ADDR),(uint8_t *)&free,2);
    Mem_Copy(mem4k + (sFLASH_POOL_USED - sFLASH_CON_START_ADDR),(uint8_t *)&used,2);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    
    //����õ���Flash��
    sFLASH_ReadBuffer(mem4k,(first/0x1000)*0x1000,0x1000);  //��ȡ����Sector
    
    //����õ�Flash�����Ϊ 0xFF  ǰ������byte ����Ϊ��ǰFlash��ĵ�ַ
    Mem_Set(mem4k+first%0x1000,0xFF,0x100);    
    Mem_Copy(mem4k+first%0x1000,(uint8_t *)&first,3);
    
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((first/0x1000)*0x1000);
    sFLASH_WriteBuffer(mem4k,(first/0x1000)*0x1000,0x1000);
    
    unlock_mem4k();
    return first;
  }else{
    return 0x000000;
  }
}

uint8_t PutFlash(uint32_t put){
  uint32_t first = 0;//pool �еĵ�һ�����п�ĵ�ַ
  uint32_t next = 0;//pool �еڶ������п�ĵ�ַ
  uint32_t free = 0;
  uint32_t used = 0;
  uint8_t * mem4k = 0;
  
  sFLASH_ReadBuffer((uint8_t *)&first,sFLASH_POOL,3);
  sFLASH_ReadBuffer((uint8_t *)&next,first+3,3);
  sFLASH_ReadBuffer((uint8_t *)&free,sFLASH_POOL_FREE,2);
  sFLASH_ReadBuffer((uint8_t *)&used,sFLASH_POOL_USED,2);
  
  if(lock_mem4k()){
    mem4k = get_mem4k();
    
    //����Ҫ�Ż�ȥ��flash��
    sFLASH_ReadBuffer(mem4k,(put/0x1000)*0x1000,0x1000);  //��ȡ����Sector
    //���Żص�Flash�����Ϊ 0xFF  ǰ������byte ����Ϊ��ǰFlash��ĵ�ַ ������byteΪ��һ������Flash���ַ
    Mem_Set(mem4k+put%0x1000,0xFF,0x100);    
    Mem_Copy(mem4k+put%0x1000,(uint8_t *)&put,3);
    Mem_Copy(mem4k+put%0x1000+3,(uint8_t *)&first,3);
    
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((put/0x1000)*0x1000);
    sFLASH_WriteBuffer(mem4k,(put/0x1000)*0x1000,0x1000);
    
    free++;
    used--;
    //����Config Flash ��
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_POOL - sFLASH_CON_START_ADDR),(uint8_t *)&put,3);
    Mem_Copy(mem4k + (sFLASH_POOL_FREE - sFLASH_CON_START_ADDR),(uint8_t *)&free,2);
    Mem_Copy(mem4k + (sFLASH_POOL_USED - sFLASH_CON_START_ADDR),(uint8_t *)&used,2);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    
    unlock_mem4k();
    return 1;
  }else{
    return 0;
  }
  
}


void param_conf(void){
  uint8_t * mem4k = 0;
  
  if(lock_mem4k()){
    mem4k = get_mem4k();
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    if(*(mem4k + sFLASH_CON_IP1) != 0xFF){
      set_ip(mem4k + sFLASH_CON_IP1);
    }
    
    if(*(mem4k + sFLASH_CON_IP1) != 0xFF){
      set_port(*(uint16_t *)(mem4k+sFLASH_CON_PORT_));
    }
    
    if(*(mem4k + sFLASH_DEVICE_ADDR) != 0xFF){
      set_addr(mem4k+sFLASH_DEVICE_ADDR);
    }
    
    set_slave(*(mem4k+sFLASH_METER_MBUS));
    set_di_seq(*(mem4k+sFLASH_READMETER_DI_SEQ));
    set_ack_valve(*(mem4k+sFLASH_ACK_ACTION));
    set_protocol(*(mem4k+sFLASH_PROTOCOL));
    set_simcard(*(mem4k+sFLASH_SIMCARD));
    
    unlock_mem4k();
  }
}