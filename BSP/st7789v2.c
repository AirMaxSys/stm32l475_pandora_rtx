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
#if USE_RTX_RTOS == 1
    osDelay(ms);
#elif USING_UCOS_RTOS == 1
#elif USING_FREE_RTOS == 1
#elif USING_ST_HAL_LIB == 1
    HAL_Delay(ms);
#endif
}

static inline void st7789_pin_write(const GPIO_TypeDef *port, uint16_t pin, uint32_t state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

static void st7789_write_cmd(const uint8_t cmd)
{
    st7789_pin_write(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi3, &cmd, 1);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void st7789_write_data(const uint8_t data)
{
    st7789_pin_write(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi3, &data, 1);
    st7789_pin_write(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
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

void st7789_reset(void)
{
    st7789_pin_write(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    st7789_delay(100);
    st7789_pin_write(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
}

void st7789_init(void)
{
    st7789_reset();
    /* Memory Data Access Control */
    st7789_write_cmd(0x36);
    st7789_write_data(0x00);
    /* Seting RGB565 */
    st7789_write_cmd(0x3A);
    st7789_write_data(0x65);
    /* Porch Setting */
    st7789_write_cmd(0xB2);
    st7789_write_data(0x0C);
    st7789_write_data(0x0C);
    st7789_write_data(0x00);
    st7789_write_data(0x33);
    st7789_write_data(0x33);
    /*  Gate Control */
    st7789_write_cmd(0xB7);
    st7789_write_data(0x35);
    /* VCOM Setting */
    st7789_write_cmd(0xBB);
    st7789_write_data(0x19);
    /* LCM Control */
    st7789_write_cmd(0xC0);
    st7789_write_data(0x2C);
    /* VDV and VRH Command Enable */
    st7789_write_cmd(0xC2);
    st7789_write_data(0x01);
    /* VRH Set */
    st7789_write_cmd(0xC3);
    st7789_write_data(0x12);
    /* VDV Set */
    st7789_write_cmd(0xC4);
    st7789_write_data(0x20);
    /* Frame Rate Control in Normal Mode */
    st7789_write_cmd(0xC6);
    st7789_write_data(0x0F);
    /* Power Control 1 */
    st7789_write_cmd(0xD0);
    st7789_write_data(0xA4);
    st7789_write_data(0xA1);
    /* Positive Voltage Gamma Control */
    st7789_write_cmd(0xE0);
    st7789_write_data(0xD0);
    st7789_write_data(0x04);
    st7789_write_data(0x0D);
    st7789_write_data(0x11);
    st7789_write_data(0x13);
    st7789_write_data(0x2B);
    st7789_write_data(0x3F);
    st7789_write_data(0x54);
    st7789_write_data(0x4C);
    st7789_write_data(0x18);
    st7789_write_data(0x0D);
    st7789_write_data(0x0B);
    st7789_write_data(0x1F);
    st7789_write_data(0x23);
    /* Negative Voltage Gamma Control */
    st7789_write_cmd(0xE1);
    st7789_write_data(0xD0);
    st7789_write_data(0x04);
    st7789_write_data(0x0C);
    st7789_write_data(0x11);
    st7789_write_data(0x13);
    st7789_write_data(0x2C);
    st7789_write_data(0x3F);
    st7789_write_data(0x44);
    st7789_write_data(0x51);
    st7789_write_data(0x2F);
    st7789_write_data(0x1F);
    st7789_write_data(0x1F);
    st7789_write_data(0x20);
    st7789_write_data(0x23);
    /* Display Inversion On */
    st7789_write_cmd(0x21);
    /* Sleep Out */
    st7789_write_cmd(0x11);
    /* wait for power stability */
    st7789_delay(100);

    /* Display on*/
    st7789_power_on();
    st7789_write_cmd(0x29);
}