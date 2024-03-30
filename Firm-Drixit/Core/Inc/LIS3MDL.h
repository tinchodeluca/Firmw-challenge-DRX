/********************************
 * Variables
 ********************************/
SPI_HandleTypeDef hspi_magnet;
GPIO_TypeDef magnet_CH_PORT;
uint16_t magnet_CH_PIN;

typedef struct{
	int16_t axis_X;
	int16_t axis_Y;
	int16_t axis_Z;
	int16_t temp;
}LIS3_DATA;


/********************************
 * Addresses
 ********************************/
//Reserved 00 - 0E -- -- Reserved
//Reserved 10 - 1F -- -- Reserved
//#define Reserved 0x25 - 26 -- -- Reserved

#define WHO_AM_I   0x0F 0000 //1111 00111101 Dummy register

#define CTRL_REG1  0x20 //0010 0000 00010000
#define CTRL_REG2  0x21 //0010 0001 00000000
#define CTRL_REG3  0x22 //0010 0010 00000011
#define CTRL_REG4  0x23 //0010 0011 00000000
#define CTRL_REG5  0x24 //0010 0100 00000000

#define STATUS_REG 0x27 //0010 0111 Output
#define OUT_X_L    0x28 //0010 1000 Output
#define OUT_X_H    0x29 //0010 1001 Output
#define OUT_Y_L    0x2A //0010 1010 Output
#define OUT_Y_H    0x2B //0010 1011 Output
#define OUT_Z_L    0x2C //0010 1100 Output
#define OUT_Z_H    0x2D //0010 1101 Output
#define TEMP_OUT_L 0x2E //0010 1110 Output
#define TEMP_OUT_H 0x2F //0010 1111 Output

#define INT_CFG    0x30 //00110000 00000000
#define INT_SRC    0x31 //00110001 00000000
#define INT_THS_L  0x32 //00110010 00000000
#define INT_THS_H  0x33 //00110011 00000000

#define STATUS_ZYXDA  0b00001000

/********************************
 * Definitions
 ********************************/
void LIS3MDL_config();
void LIS3MDL_init(SPI_HandleTypeDef* spi, uint8_t add, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void LIS3MDL_Write_Reg(uint8_t reg, uint8_t value);
LIS3_DATA LIS3MDL_Get_XYZT();
