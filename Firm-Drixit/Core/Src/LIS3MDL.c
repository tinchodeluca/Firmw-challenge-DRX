/*
 * LIS3MDL.c
 *
 *  Created on: 29 mar. 2024
 *      Author: mdelu
 */
#include "LIS3MDL.h"

void LIS3MDL_init(SPI_HandleTypeDef* spi, GPIO_TypeDef *cs_port, uint16_t cs_pin){
	hspi_magnet	   = *spi;
	magnet_CH_PORT = *cs_port;
	magnet_CH_PIN  = cs_pin;
}

void LIS3MDL_config(){
	uint8_t Reg1,
			Reg2,
			Reg3,
			Reg4,
			Reg5,
			IntReg;

	Reg1 = 0x80 | 0x60 | 0x10 ; // 1111 0000 //UHP
	Reg2 = 0x40;				// 0100 0000 //+-12gaus
	Reg3 =(0x00 | 0x00 )& 0x37; // 0000 0000 //4wire continuous mode
	Reg4 =(0x0C | 0x00 )& 0xF1; // 0000 1100 //UHP
	Reg5 =(0x00 | 0x00 )& 0x3F; // 0100 0000 //continuous update

	IntReg = 0x00; 				// 0000 0000

	HAL_GPIO_WritePin(&magnet_CH_PORT, magnet_CH_PIN, GPIO_PIN_RESET);

	LIS3MDL_Write_Reg(CTRL_REG1, Reg1);
	LIS3MDL_Write_Reg(CTRL_REG2, Reg2);
	LIS3MDL_Write_Reg(CTRL_REG3, Reg3);
	LIS3MDL_Write_Reg(CTRL_REG4, Reg4);
	LIS3MDL_Write_Reg(CTRL_REG5, Reg5);
	LIS3MDL_Write_Reg(INT_CFG, IntReg);

	HAL_GPIO_WritePin(&magnet_CH_PORT, magnet_CH_PIN, GPIO_PIN_SET);
}

void LIS3MDL_Write_Reg(uint8_t reg, uint8_t value){
	uint8_t Data[2];

	Data[0] = reg;
	Data[1] = value;

	HAL_SPI_Transmit(&hspi_magnet, Data, 2, HAL_MAX_DELAY);
}

LIS3_DATA LIS3MDL_Get_XYZT(){
	uint8_t rxData[8], STATUS;
	LIS3_DATA outData;
	int16_t ax_x, ax_y, ax_z, temp;

	uint8_t txData = STATUS_REG | 0x80; //MULTIPLE READS -From status reg

	HAL_GPIO_WritePin(&magnet_CH_PORT, magnet_CH_PIN, GPIO_PIN_RESET); //low

	HAL_SPI_Transmit(&hspi_magnet, &txData, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi_magnet, STATUS, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi_magnet, rxData, 8, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(&magnet_CH_PORT, magnet_CH_PIN, GPIO_PIN_SET); //high

	if(STATUS & STATUS_ZYXDA){ // If xyz data available
		ax_x = (int16_t)(rxData[1] << 8 | rxData[0]);
		ax_y = (int16_t)(rxData[3] << 8 | rxData[2]);
		ax_z = (int16_t)(rxData[5] << 8 | rxData[4]);
		temp = (int16_t)(rxData[7] << 8 | rxData[6]);
	}
	else{
		ax_x = 0;
		ax_y = 0;
		ax_z = 0;
		temp = 0;
	}
	outData.axis_X = ax_x;
	outData.axis_Y = ax_y;
	outData.axis_Z = ax_z;
	outData.temp   = temp;

	return outData;
}
