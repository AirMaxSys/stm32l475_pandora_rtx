#include "i2c_soft.h"
#include <stdio.h>

i2c_soft_t *p_i2c_sf;

/* pin output level*/
#define PIN_HI(port, pin)		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)
#define PIN_LO(port, pin)		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
// read pin
#define PIN_RD(port, pin)		(HAL_GPIO_ReadPin((port), (pin)) == GPIO_PIN_SET) ? 1 : 0

static void delay()
{
	for (int i = 0; i < 10; ++i)
		for (int k = 0; k < 1000; ++k)
			__NOP();
}

static void set_sda_write_mode(void)
{
#if 0
	GPIO_InitTypeDef gpio_init = {0};

	HAL_GPIO_DeInit(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	gpio_init.Pin = p_i2c_sf->sda->pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(p_i2c_sf->sda->port, &gpio_init);
#endif
	GPIOC->MODER&=~(3<<(1*2));GPIOC->MODER|=1<<(1*2);
}

static void set_sda_read_mode(void)
{
#if 0
	GPIO_InitTypeDef gpio_init = {0};

	HAL_GPIO_DeInit(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	gpio_init.Pin = p_i2c_sf->sda->pin;
	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(p_i2c_sf->sda->port, &gpio_init);
#endif
	GPIOC->MODER&=~(3<<(1*2));GPIOC->MODER|=0<<(1*2);
}

i2c_soft_err_enum_t i2c_soft_init(i2c_soft_t *i2c)
{
//	if (!i2c)
//		return I2C_SOFT_ERR_INIT_INSTENCE;
//	if (!i2c->sda || !i2c->scl)
//		return I2C_SOFT_ERR_GPIO_PORT;
//	if (i2c->addr_len != 7 || i2c->addr_len != 10)
//		return I2C_SOFT_ERR_ADDR_LEN;

    __HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitTypeDef gpio_init = {0};

	/* globle i2c point to soft instence*/
	p_i2c_sf = i2c;

	/* init SDA and SCL gpio port*/
	// HAL_GPIO_DeInit(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	// HAL_GPIO_DeInit(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

	/* configure SDA and SCL GPIO pins must set in open drain mode*/
	gpio_init.Pin = p_i2c_sf->sda->pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(p_i2c_sf->sda->port, &gpio_init);
	gpio_init.Pin = p_i2c_sf->scl->pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(p_i2c_sf->scl->port, &gpio_init);

	PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

	return I2C_SOFT_ERR_NONE;
}

static void send_start(void)
{
	set_sda_write_mode();
	PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	delay();
	PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	delay();
	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
}

static void send_stop(void)
{
	set_sda_write_mode();
	PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	delay();
	PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	delay();
	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
}


static void send_ack(void)
{
	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	set_sda_write_mode();
	PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
}

static void send_noack(void)
{
	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	set_sda_write_mode();
	PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
}

static uint8_t wait_ack(void)
{
	uint8_t res = 0;

	set_sda_read_mode();
	PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	res = PIN_RD(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

	return res;
}

static uint8_t send_byte_with_ack(uint8_t byte)
{
	set_sda_write_mode();
	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

	for (int8_t i = 7; i >= 0; --i) {
		// if ((byte >> i) & 0x1)
		if ((byte & 0x80) >> 7)
			PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		else 
			PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		byte <<= 1;
		PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	}
	if (wait_ack() != 0) {
		p_i2c_sf->err_code = I2C_SOFT_ERR_TX_ABORT_NOACK;
		return 1;
	}
	return 0;
}

static uint8_t recv_byte_with_ack(void)
{
	uint8_t ret = 0;

	set_sda_read_mode();
	for (int8_t i = 7; i >= 0; --i) {
		PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		ret <<= 1;
		if (PIN_RD(p_i2c_sf->sda->port, p_i2c_sf->sda->pin))
			ret++;
			// ret |= (1 << i);
	}
	send_ack();

	return ret;
}

static uint8_t recv_byte_with_noack(void)
{
	uint8_t ret = 0;

	set_sda_read_mode();
	for (int8_t i = 7; i >= 0; --i) {
		PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		ret <<= 1;
		if (PIN_RD(p_i2c_sf->sda->port, p_i2c_sf->sda->pin))
			ret++;
			// ret |= (1 << i);
	}
	send_noack();

	return ret;
}

static i2c_soft_err_enum_t send_addr(i2c_soft_mode_enum_t wr_rd_mode)
{
	uint8_t addr = (p_i2c_sf->slave_addr << 1) + (uint8_t)wr_rd_mode;

#if 0
	set_sda_write_mode();
	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

	for (int16_t i = p_i2c_sf->addr_len; i >= 0; --i) {
		if ((addr >> i) & 0x1)
			PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		else 
			PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		// need delay ?
	}
	if (wait_ack() != 0)
		return I2C_SOFT_ERR_SLAVE_ADDR_NOACK;
#endif
	send_byte_with_ack(addr);
	return I2C_SOFT_ERR_NONE;
}

i2c_soft_err_enum_t i2c_soft_write(uint8_t *buffer, uint16_t len)
{
	uint8_t res =  I2C_SOFT_ERR_NONE;

	// send start condition
	send_start();
	// send slave address
	if (send_addr(I2C_SOFT_WRITE) != I2C_SOFT_ERR_NONE) {
		send_stop();
		printf("i2c write send addr err\r\n");
		return res;
	}
	// send bytes with ack
	for (uint16_t i = 0; i < len; ++i)
		if (send_byte_with_ack(buffer[i]) != 0) {
			printf("i2c write with ack err\r\n");
		}
	// send stop
	send_stop();

	return res;
}

i2c_soft_err_enum_t i2c_soft_read(uint8_t *buffer, uint16_t len)
{
	send_start();
	if (send_addr(I2C_SOFT_READ) != I2C_SOFT_ERR_NONE) {
		send_stop();
		printf("i2c read send addr err\r\n");
		return I2C_SOFT_ERR_SLAVE_ADDR_NOACK;
	}
	for (uint16_t i = 0; i < len-1; ++i)
		buffer[i] = recv_byte_with_ack();
	buffer[len-1] = recv_byte_with_noack();
	send_stop();

	return I2C_SOFT_ERR_NONE;
}
