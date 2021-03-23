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
	if (HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
		TFT_CS_HI();
		return -1;
	}
	TFT_CS_HI();
	return 0;
}

static int8_t tft_wr_cmd_rd_dat(uint8_t cmd, uint8_t *dat)
{

	TFT_CS_LO();
	TFT_WR_CMD();
	if (HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
		TFT_CS_HI();
		return -1;
	}
	
	TFT_WR_DAT();
	if (HAL_SPI_Receive(&hspi3, dat, 1, HAL_MAX_DELAY) != HAL_OK) {
		TFT_CS_HI();
		return -1;
	}
	TFT_CS_HI();
	return 0;
}

int8_t tft_write_dats(uint8_t cmd, uint8_t *txbuf, uint16_t len)
{
	TFT_CS_LO();
	TFT_WR_CMD();
	if (HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
		TFT_CS_HI();
		return -1;
	TFT_WR_DAT();
	if (HAL_SPI_Transmit(&hspi3, &txbuf, len, HAL_MAX_DELAY) != HAL_OK) {
		TFT_CS_HI();
		return -1;
	}
	TFT_CS_HI();
	return 0;
}

int8_t tft_read_dats(uint8_t *rxbuf, uint16_t len)
{
	TFT_WR_DAT();
	TFT_CS_LO();
	if (HAL_SPI_Receive(&hspi3, rxbuf, len, HAL_MAX_DELAY) != HAL_OK) {
		TFT_CS_HI();
		return -1;
	}
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

/* command functions*/

void tft_sw_reset(void)
{
	tft_write_cmd(TFT_CMD_SW_RESET);
	// in normal mode wait at least 5ms
	HAL_Delay(5);
	// in sleep in mode wait at least 120ms
}

void tft_read_display_id(void)
{
	uint8_t cmd = 0x0;
	uint8_t rxbuf[3] = {0x0};

	/* get LCD IDs*/
	tft_wr_cmd_rd_dat(TFT_CMD_RDID1, &rxbuf[0]);
	tft_wr_cmd_rd_dat(TFT_CMD_RDID2, &rxbuf[1]);
	tft_wr_cmd_rd_dat(TFT_CMD_RDID3, &rxbuf[2]);

	/* debug output*/
	printf("display id row data:");
	for (uint8_t i = 0; i < 3; ++i)
		printf("0x%02x ", rxbuf[i]);
	puts("");
}
