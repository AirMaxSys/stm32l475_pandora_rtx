/*!
* @file		st7789v2.c - st7789v2 LCD driver
* @brief	TFT display resolution 240*320, color 262K RGB(666)
* @note		#1 use 4lines SPI bus to communicate and st HAL spi driver lib
*			#2 use DMA transmit data
*			#3 IO pin: SCL(SPI clock) SDA(data out/in) CSX(SPI chip select) WRX(data/command) RST(reset) PWR(power)	
*			#4 when multiple byte read, stm32 SPI can't shift dummy bit(dummy cycle), so that CMD 0x4h(read display id)
*				can not succeed!!!
* @author	AirMaxSys
* @date		2021/03/13	- first edit
*/

#include "st7789v2.h"
#include "common.h"
#include "config.h"

extern SPI_HandleTypeDef hspi3;

static inline void st7789_delay(const uint16_t ms)
{
#if USE_RTOS == 1
    osDelay(ms);
#else
    HAL_Delay(ms);
#endif
}

static inline void st7789_pin_write(const GPIO_TypeDef *port, uint16_t pin, uint32_t state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

static void st7789_write_cmd(const uint8_t cmd)
{
    st7789_pin_write(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi3, &cmd, 1);
}

static void st7789_write_data(const uint8_t data)
{
    st7789_pin_write(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit_IT(&hspi3, &data, 1);
}

void st7789_power_on(void)
{
    st7789_pin_write(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_SET);
}

void st7789_power_off(void)
{
    st7789_pin_write(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);
}

void st7789_enter_sleep(void)
{
    st7789_pin_write(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);
    st7789_delay(5);
    st7789_write_cmd(0x10);
}

void st7789_exit_sleep(void)
{
    st7789_pin_write(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_SET);
    st7789_delay(5);
    st7789_write_cmd(0x11);
    st7789_delay(120);
}

void st7789_init(void)
{
}

void tft_set_display_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
}
