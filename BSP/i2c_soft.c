#include "i2c_soft.h"

i2c_soft_err_enum_t i2c_soft_init(i2c_soft_t *i2c, gpio_t *sda, gpio_t *scl, uint8_t slave_addr, uint8_t addr_len)
{
	if (!sda || !sck)
		return I2C_SOFT_ERR_GPIO_PORT;
	if (addr_len != 7 || addr_len != 10)
		return I2C_SOFT_ERR_ADDR_LEN;

	GPIO_InitTypeDef gpio_init = {0};

	/* configure i2c soft struct*/
	i2c->sda = sda;
	i2c->scl = scl;
	i2c->slave_addr = slave_addr;
	i2c->addr_len = addr_len;

	/* init SDA and SCL gpio port*/
	HAL_GPIO_DeInit(sda->port, sda->pin);
	HAL_GPIO_DeInit(scl->port, scl->pin);

	/* SDA and SCL output level high*/
	HAL_GPIO_WritePin(sda->port, sda->pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(scl->port, scl->pin, GPIO_PIN_SET);

	/* configure SDA and SCL GPIO pins*/
	gpio_init.Pin = sda->pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(sda->port, &gpio_init);
	gpio_init.Pin = scl->pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(scl->port, &gpio_init);

	return I2C_SOFT_ERR_NONE;
}

i2c_soft_err_enum_t i2c_soft_write(i2c_soft_t *i2c, uint8_t *buffer, uint16_t len, uint32_t timeout)
{
	return I2C_SOFT_ERR_NONE;
}
