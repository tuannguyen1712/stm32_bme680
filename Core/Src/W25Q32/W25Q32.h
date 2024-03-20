#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

//#include "stm32f1xx_hal.h"
#include "stm32f4xx_hal.h"

#define WRITE_EN				0x06
#define WRITE_DIS				0x04
#define READ_DATA				0x03
#define PAGE_PROGRAM			0x02
#define ERASE_SECTOR			0x20
#define ERASE_BLOCK_32			0x52
#define ERASE_BLOCK_64			0xD8
#define ERASE_CHIP				0x60
#define RDSR1					0x05

#define SECTOR_SIZE				4096

extern bool rx_spi_flg;				// need interrupt handle function in main.c

void W25Q32_Init(SPI_HandleTypeDef *hspix, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void W25Q32_CS_LOW();
void W25Q32_CS_HIGH();
void WriteEnable();
void WriteDisable();
void W25Q32_WaitEndCycle();
void W25Q32_Send_Receive(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t tx_num);
void W25Q32_ReadData(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t len);
void W25Q32_WriteData(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t len);
void W25Q32_erase4k(uint32_t add);
void W25Q32_erase32k(uint32_t add);
void W25Q32_erase64k(uint32_t add);
void W25Q32_eraseChip();
