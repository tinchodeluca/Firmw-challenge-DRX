/*
 * W25Q80.c
 *
 *  Created on: 2 abr. 2024
 *      Author: mdelu
 */

#include "W25Q80.h"

/*
 *  (ID + X + Y + Z + TEMP )* 2 BYTES = 10bytes
 *  W25Q_Get_data will get 10Bytes with FASTREAD method
 *  total 1,048,576 >> 10bytes/data --> 104.857 data entries ->32bits to select each
 */ //87,381
void W25Q_Init(SPI_HandleTypeDef* spi, GPIO_TypeDef *cs_port, uint16_t cs_pin){
	hspi_flash    = *spi;
	flash_CH_PORT = *cs_port;
	flash_CH_PIN  = cs_pin;
}

HAL_StatusTypeDef W25Q_Read_data(uint8_t *ReadData, uint32_t ADD , int16_t SIZE) {
	HAL_StatusTypeDef _spi_state;
	int64_t txData = 0;

/*	|Byte4 |  Byte3  |  Byte2  |  Byte1  | from address 32bits
 *  xxxxxxx|b23...b16|b15...b08|b07...b00|
 *
 	 	 	 |		LSB byte1	  		   |byte2( ADD byte3)	  |byte3( ADD byte2)	    |byte4( ADD byte1)	 |*/
	txData = ((int64_t) W25Q80_FAST_READ ) | ((ADD >> 8)& 0xFF00) | ((ADD << 16)& 0x00FF00) | ((ADD << 24)& 0xFF0000);

	HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_RESET); //low

	//Send only the 5 LSB of txData:
	// 1 byte ->command
	// 3 byte ->start address to read
	// 1 byte ->dummy "0" fast read
	_spi_state = HAL_SPI_Transmit(&hspi_flash, (uint8_t *)&txData, 5, 2000);
	_spi_state = HAL_SPI_Receive(&hspi_flash, ReadData, SIZE, 5000);//Receive size data
//	if ( HAL_OK != _spi_state )
//		return HAL_ERROR;
	HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_SET); // stop receiving

	return _spi_state;
}

HAL_StatusTypeDef W25Q_Write_data( uint8_t *WriteData, uint32_t ADD , int16_t SIZE) {
	HAL_StatusTypeDef _spi_state;

	uint32_t PAGE   = ADD /256; // Witch PAGE
	uint32_t OFFSET = ADD %256; // In the PAGE

	uint8_t txData [260]; //256 max + 4 of COMMAND and address

	//If the data to write is bigger than the remaining size of the page
	// we will need to make two loops to write
	// see from datasheet section 8.5.13 Page Program (02h)
	uint8_t ITERATIONS = (SIZE + OFFSET -1)/256;

	if (SIZE >= (256 - OFFSET)){
		uint32_t Address      = ADD;
		uint32_t Pointer_DATA = 0;
		uint16_t bytes_rem;

		for (int _page =0; _page < ITERATIONS ; _page+=256){ //We add 256 each page
			W25Q80_Set_Write_State(1);//Enable
//			Address += _page;
			Address = PAGE + OFFSET + _page;

			txData[0] = W25Q80_PAGE_PROGRAM;
			txData[1] = ((Address >> 16)& 0xFF);
			txData[2] = ((Address >>  8)& 0xFF);
			txData[3] = ((Address      )& 0xFF);

			if (256 > (SIZE + OFFSET))
				bytes_rem =  SIZE;
			else
				bytes_rem = (256 - OFFSET);

			for (uint8_t i=4; i<bytes_rem; i++)
				txData[i] = WriteData[i -4 + Pointer_DATA]; //data to be written and keeping a pointer to the remaining

			Pointer_DATA += bytes_rem; //Now the pointer is at the end of the ones in queue

			HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_RESET); //low
			_spi_state = HAL_SPI_Transmit(&hspi_flash, txData, (bytes_rem + 4) , 2000);
			HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_SET);
			W25Q80_Is_Busy();
			W25Q80_Set_Write_State(0);//Disable
//			if ( HAL_OK != _spi_state )
//				return _spi_state;

			OFFSET = 0; //End of PAGE another begins
			SIZE  -= bytes_rem; //the previews size remove the recent added
		}
	}
	else { // If the data fits in the remaining of the page
		W25Q80_Set_Write_State(1);//Enable

		txData[0] = W25Q80_PAGE_PROGRAM;
		txData[1] = ((ADD >> 16)& 0xFF);
		txData[2] = ((ADD >>  8)& 0xFF);
		txData[3] = ((ADD      )& 0xFF);

		for (uint8_t i=4; i<SIZE; i++)
			txData[i] = WriteData[i -4];

		HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_RESET); //low
		_spi_state = HAL_SPI_Transmit(&hspi_flash, txData, SIZE, 2000);
		HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_SET);
//		if ( HAL_OK != _spi_state )
//			return _spi_state;
		W25Q80_Is_Busy();
		W25Q80_Set_Write_State(0);//Disable
	}
	UNUSED(_spi_state);
	return HAL_OK;
}
void W25Q80_Set_Write_State(uint8_t STATE){
	uint8_t txData;
	HAL_StatusTypeDef _spi_state;

	if (0 == STATE)//False
		txData = W25Q80_WRITE_DISABLE; // DISABLE write
	else
		txData = W25Q80_WRITE_ENABLE;// ENABLE write

	HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_RESET); //low
	_spi_state = HAL_SPI_Transmit(&hspi_flash, &txData, 1, 2000);
	HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_SET);

	HAL_Delay(5); //5ms
	UNUSED(_spi_state);
}
void W25Q80_Is_Busy(void){
	HAL_StatusTypeDef _spi_state;
	uint8_t txData = W25Q80_READ_SR1, ReadStatus;

	HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_RESET); //low
	_spi_state = HAL_SPI_Transmit(&hspi_flash, &txData, 1, 2000);
//	if ( HAL_OK != _spi_state )
//		return _spi_state;
	_spi_state = HAL_SPI_Receive(&hspi_flash, &ReadStatus, 1, 5000);//Receive size data
	while (ReadStatus & 0x01){
		_spi_state = HAL_SPI_Receive(&hspi_flash, &ReadStatus, 1, 5000);//Receive size data
		//	if ( HAL_OK != _spi_state )
		//		return _spi_state;
	}
	HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_SET);
	UNUSED(_spi_state);
}
void W25Q80_Full_Erase(void){
	HAL_StatusTypeDef _spi_state;
	uint8_t txData = W25Q80_ERASE;

	W25Q80_Set_Write_State(1);
	HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_RESET); //low
	_spi_state = HAL_SPI_Transmit(&hspi_flash, &txData, 1, 2000);
	HAL_GPIO_WritePin(&flash_CH_PORT, flash_CH_PIN, GPIO_PIN_SET);

	W25Q80_Is_Busy();
	W25Q80_Set_Write_State(0);
	UNUSED(_spi_state);
}
