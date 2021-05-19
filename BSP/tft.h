#ifndef _TFT_H_
#define _TFT_H_

#ifdef  _cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* tft LCD cmd definations */
#define	TFT_CMD_SW_RESET    0x1u    // software reset
#define TFT_CMD_RDDPM       0xAu    // read display power mode
#define TFT_CMD_RAMWR       0x2Cu   // memory write
#define TFT_CMD_RAWRD       0x2Eu   // memory read
#define	TFT_CMD_RDID1       0xDAu   // read ID1
#define	TFT_CMD_RDID2       0xDBu   // read ID2
#define	TFT_CMD_RDID3       0xDCu   // read ID3

#define TFT_PWR_ON()	HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_SET)
#define TFT_PWR_OFF()	HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET)
#define TFT_RST_LO()	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define TFT_RST_HI()	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)
#define TFT_CS_LO()		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define TFT_CS_HI()		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define	TFT_WR_DAT()	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET)
#define	TFT_WR_CMD()	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET)

int8_t tft_write_dats(uint8_t cmd, uint8_t *txbuf, uint16_t len);

void tft_init(void);
void tft_power_on(void);
void tft_hw_reset(void);
void tft_set_display_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void tft_read_display_id(void);

#ifdef  _cplusplus
}
#endif

#endif
