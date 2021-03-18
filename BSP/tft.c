/*!
* @file		tft.c - st7789v2 TFTLCD driver
* @brief	TFT display resolution 240*320, color 262K RGB(666)
* @note		#1 use 4lines SPI bus to communicate and st HAL spi driver lib
*			#2 use DMA transmit data
*			#3 IO pin: SCL(SPI clock) SDA(data out/in) CSX(SPI chip select) WRX(data/command) RST(reset) PWR(power)	
*			#4 when multiple byte read, stm32 SPI can't shift dummy bit(dummy cycle), so that CMD 0x4h(read display id)
*				can not succeed!!!
* @author	AirMaxSys
* @date		2021/03/13	- first edit
*/

#include "tft.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define TFT_PWR_ON()	HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_SET)
#define TFT_PWR_OFF()	HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET)
#define TFT_RST_LO()	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define TFT_RST_HI()	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)
#define TFT_CS_LO()		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define TFT_CS_HI()		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define	TFT_WR_DAT()	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET)
#define	TFT_WR_CMD()	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET)

extern SPI_HandleTypeDef hspi3;

static int8_t tft_write_cmd(uint8_t cmd)
{
	TFT_WR_CMD();
	TFT_CS_LO();
	if (HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
		return -1;
	TFT_CS_HI();
	return 0;
}

static int8_t tft_write_dat()
{
	TFT_WR_DAT();
	TFT_CS_LO();

	TFT_CS_HI();
	return 0;
}

void tft_init(void)
{

}

void tft_power_on(void)
{
	TFT_PWR_ON();
	tft_hw_reset();
}

void tft_hw_reset(void)
{
	TFT_RST_LO();
	// hold more than 10us
	HAL_Delay(120);
	TFT_RST_HI();
	// wait for chip initial register value
	HAL_Delay(5);
}

int8_t tft_send_datas()
{
	return 0;
}

int8_t tft_recv_datas(uint8_t *rxbuf, uint16_t len)
{
	TFT_WR_DAT();
	TFT_CS_LO();
	if (HAL_SPI_Receive(&hspi3, rxbuf, len, HAL_MAX_DELAY) != HAL_OK)
		return -1;
	TFT_CS_HI();
	return 0;
}

/* command functions*/

void tft_sw_reset(void)
{

}

void tft_read_display_id(void)
{
	uint8_t cmd = 0x0;
	uint8_t rxbuf[3] = {0x0};

	// write command
	cmd = 0xDA;
	TFT_CS_LO();
	TFT_WR_CMD();
	HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
	
	TFT_WR_DAT();
	HAL_SPI_Receive(&hspi3, rxbuf, 1, HAL_MAX_DELAY);
	TFT_CS_HI();
	
	cmd = 0xDB;
	TFT_CS_LO();
	TFT_WR_CMD();
	HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
	
	TFT_WR_DAT();
	HAL_SPI_Receive(&hspi3, &rxbuf[1], 1, HAL_MAX_DELAY);
	TFT_CS_HI();
	
	cmd = 0xDC;
	TFT_CS_LO();
	TFT_WR_CMD();
	HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
	
	TFT_WR_DAT();
	HAL_SPI_Receive(&hspi3, &rxbuf[2], 1, HAL_MAX_DELAY);
	TFT_CS_HI();

	/* output*/
	printf("display id row data:");
	for (uint8_t i = 0; i < 3; ++i)
		printf("0x%02x ", rxbuf[i]);
	puts("");
}
