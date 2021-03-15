/*!
* @file		tft.c - st7789v2 TFTLCD driver
* @brief	TFT display resolution 240*320, color 262K RGB(666)
* @note		#1 use 4lines SPI bus to communicate and st HAL spi driver lib
*			#2 use DMA transmit data
*			#3 IO pin: SCL(SPI clock) SDA(data out/in) CSX(SPI chip select) WRX(data/command) RST(reset) PWR(power)
* @author	AirMaxSys
* @date		2021/03/13	- first edit
*/

#include "tft.h"

#define TFT_RST_LO()	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define TFT_RST_HI()	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)
#define TFT_CS_LO()		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define TFT_CS_HI()		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define	TFT_WR_DAT()	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET)
#define	TFT_WR_CMD()	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET)

static int8_t tft_write_cmd()
{
	TFT_WR_CMD();
	TFT_CS_LO();

	TFT_CS_HI();
}

static int8_t tft_write_dat()
{
	TFT_WR_DAT();
	TFT_CS_LO();

	TFT_CS_HI();
}

void tft_hw_reset(void)
{
	TFT_RST_LO();
	// hold more than 10us
	HAL_Delay(1);
	TFT_RST_HI();
	// wait for chip initial register value
	HAL_Delay(5);
}

void tft_soft_reset(void)
{

}

int8_t tft_send_datas()
{

}

int8_t tft_recv_datas()
{

}
