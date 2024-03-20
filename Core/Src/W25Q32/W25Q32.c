#include "W25Q32.h"

GPIO_TypeDef *CS_PORT;
uint16_t CS_Pin;
SPI_HandleTypeDef *hspi;
uint8_t dum_byte = 0xff;

void W25Q32_Init(SPI_HandleTypeDef *hspix, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	hspi = hspix;
	CS_PORT = GPIOx;
	CS_Pin = GPIO_Pin;
}

void W25Q32_CS_LOW() {
	HAL_GPIO_WritePin(CS_PORT, CS_Pin, 0);
}

void W25Q32_CS_HIGH() {
	HAL_GPIO_WritePin(CS_PORT, CS_Pin, 1);
}

void WriteEnable() {
	uint8_t *tsm = malloc(sizeof(uint8_t));
	uint8_t *rev = malloc(sizeof(uint8_t));
	tsm[0] = WRITE_EN;

	W25Q32_CS_LOW();
	W25Q32_Send_Receive(tsm, rev, 1);
	W25Q32_CS_HIGH();

	free(tsm);
	free(rev);
}

void WriteDisable() {
	uint8_t *tsm = (uint8_t*) malloc(sizeof(uint8_t));
	uint8_t *rev = (uint8_t*) malloc(sizeof(uint8_t));
	tsm[0] = WRITE_DIS;

	W25Q32_CS_LOW();
	W25Q32_Send_Receive(tsm, rev, 1);
	W25Q32_CS_HIGH();

	free(tsm);
	free(rev);
}

void W25Q32_WaitEndCycle() {
	W25Q32_CS_LOW();
	uint8_t check;
	uint8_t *tx = (uint8_t*) malloc(sizeof(uint8_t));
	uint8_t *rx = (uint8_t*) malloc(sizeof(uint8_t));
	*tx = RDSR1;

	W25Q32_CS_LOW();
	W25Q32_Send_Receive(tx, rx, 1);
	do {
		*tx = dum_byte;
		W25Q32_Send_Receive(tx, rx, 1);
		check = *rx;
	} while (check & 0x01);
	W25Q32_CS_HIGH();
	free(tx);
	free(rx);
}

void W25Q32_Send_Receive(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t tx_num) {
	rx_spi_flg = 0;
	HAL_SPI_TransmitReceive_IT(hspi, tx_buf, rx_buf, tx_num);
	while (!rx_spi_flg) {
	}
}

void W25Q32_ReadData(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t len) {
	W25Q32_CS_LOW();

	uint8_t *cmd = malloc(sizeof(uint8_t) * (4 + len));
	uint8_t *ret = malloc(sizeof(uint8_t) * len);
	cmd[0] = READ_DATA;
	cmd[1] = (ReadAddr & 0x00FF0000) >> 16;
	cmd[2] = (ReadAddr & 0x0000FF00) >> 8;
	cmd[3] = ReadAddr & 0x000000FF;
	W25Q32_Send_Receive(cmd, ret, 4);
	W25Q32_Send_Receive(cmd + 4, pBuffer , len);
	W25Q32_CS_HIGH();
	free(cmd);
	free(ret);
}

void W25Q32_WriteData(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t len) {
	uint8_t *cmd = malloc(sizeof(uint8_t) * (4 + len));
	uint8_t *data = malloc(sizeof(uint8_t) * len);
	uint8_t *ret = malloc(sizeof(uint8_t) * len);

	WriteEnable();
	W25Q32_CS_LOW();
	cmd[0] = PAGE_PROGRAM;
	cmd[1] = (WriteAddr & 0x00FF0000) >> 16;
	cmd[2] = (WriteAddr & 0x0000FF00) >> 8;
	cmd[3] = WriteAddr & 0x000000FF;
	W25Q32_Send_Receive(cmd, data, 4);
	for (uint8_t i = 0; i < len; i++) {
		*(data + i) = *(pBuffer + i);
	}
	W25Q32_Send_Receive(data, ret, len);
	W25Q32_CS_HIGH();
	W25Q32_WaitEndCycle();
	WriteDisable();
	free(cmd);
	free(data);
}

void W25Q32_erase4k(uint32_t add) {
	uint8_t *cmd = (uint8_t*) malloc(sizeof(uint8_t) * 4);
	uint8_t *ret = (uint8_t*) malloc(sizeof(uint8_t) * 4);
	cmd[0] = ERASE_SECTOR;
	cmd[1] = (add & 0x00FF0000) >> 16;
	cmd[2] = (add & 0x0000FF00) >> 8;
	cmd[3] = add & 0x0000000FF;

	WriteEnable();
	W25Q32_CS_LOW();
	W25Q32_Send_Receive(cmd, ret, 4);
	W25Q32_CS_HIGH();
	W25Q32_WaitEndCycle();
	WriteDisable();
	free(cmd);
	free(ret);
}

void W25Q32_erase32k(uint32_t add) {
	uint8_t *cmd = (uint8_t*) malloc(sizeof(uint8_t) * 4);
	uint8_t *ret = (uint8_t*) malloc(sizeof(uint8_t) * 4);
	cmd[0] = ERASE_BLOCK_32;
	cmd[1] = (add & 0x00FF0000) >> 16;
	cmd[2] = (add & 0x0000FF00) >> 8;
	cmd[3] = add & 0x0000000FF;

	WriteEnable();
	W25Q32_CS_LOW();
	W25Q32_Send_Receive(cmd, ret, 4);
	W25Q32_CS_HIGH();
	W25Q32_WaitEndCycle();
	WriteDisable();
	free(cmd);
	free(ret);
}

void W25Q32_erase64k(uint32_t add) {
	uint8_t *cmd = (uint8_t*) malloc(sizeof(uint8_t) * 4);
	uint8_t *ret = (uint8_t*) malloc(sizeof(uint8_t) * 4);
	cmd[0] = ERASE_BLOCK_64;
	cmd[1] = (add & 0x00FF0000) >> 16;
	cmd[2] = (add & 0x0000FF00) >> 8;
	cmd[3] = add & 0x0000000FF;

	WriteEnable();
	W25Q32_CS_LOW();
	W25Q32_Send_Receive(cmd, ret, 4);
	W25Q32_CS_HIGH();
	W25Q32_WaitEndCycle();
	WriteDisable();
	free(cmd);
	free(ret);
}

void W25Q32_eraseChip() {
	uint8_t *cmd = (uint8_t*) malloc(sizeof(uint8_t) * 4);
	uint8_t *ret = (uint8_t*) malloc(sizeof(uint8_t) * 4);
	cmd[0] = ERASE_SECTOR;

	WriteEnable();
	W25Q32_CS_LOW();
	W25Q32_Send_Receive(cmd, ret, 1);
	W25Q32_CS_HIGH();
	W25Q32_WaitEndCycle();
	WriteDisable();
	free(cmd);
	free(ret);
}
