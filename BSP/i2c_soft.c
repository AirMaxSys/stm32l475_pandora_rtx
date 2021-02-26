#include "i2c_soft.h"

/* pin output level*/
#define PIN_HI(port, pin)		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)
#define PIN_LO(port, pin)		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
// read pin
#define PIN_RD(port, pin)		(HAL_GPIO_ReadPin((port), (PIN)) == GPIO_PIN_SET) ? 1 : 0

static void set_sda_write_mode(void)
{
	if (!p_i2c_sf)	return;

	GPIO_InitTypeDef gpio_init = {0};

	HAL_GPIO_DeInit(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	HAL_GPIO_WritePin(p_i2c_sf->sda->port, p_i2c_sf->sda->pin, GPIO_PIN_SET);
	gpio_init.Pin = p_i2c_sf->sda->pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(p_i2c_sf->sda->port, &gpio_init);
}

static void set_sda_read_mode(void)
{
	if (!p_i2c_sf)	return;

	GPIO_InitTypeDef gpio_init = {0};

	HAL_GPIO_DeInit(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	gpio_init.Pin = p_i2c_sf->sda->pin;
	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(p_i2c_sf->sda->port, &gpio_init);
}

i2c_soft_err_enum_t i2c_soft_init(i2c_soft_t *i2c)
{
	if (!i2c)
		return I2C_SOFT_ERR_INIT_INSTENCE;
	if (!i2c->sda || !i2c->scl)
		return I2C_SOFT_ERR_GPIO_PORT;
	if (i2c->addr_len != 7 || i2c->addr_len != 10)
		return I2C_SOFT_ERR_ADDR_LEN;

	GPIO_InitTypeDef gpio_init = {0};

	/* globle i2c point to soft instence*/
	p_i2c_sf = i2c;

	/* init SDA and SCL gpio port*/
	HAL_GPIO_DeInit(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	HAL_GPIO_DeInit(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);

	/* SDA and SCL output level high*/
	HAL_GPIO_WritePin(p_i2c_sf->sda->port, p_i2c_sf->sda->pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(p_i2c_sf->scl->port, p_i2c_sf->scl->pin, GPIO_PIN_SET);

	/* configure SDA and SCL GPIO pins must set in open drain mode*/
	gpio_init.Pin = p_i2c_sf->sda->pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(p_i2c_sf->sda->port, &gpio_init);
	gpio_init.Pin = p_i2c_sf->scl->pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(p_i2c_sf->scl->port, &gpio_init);

	return I2C_SOFT_ERR_NONE;
}

static void send_start_sig(void)
{
	PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
}

static uint8_t wait_ack(void)
{
	uint8_t res = 0;

	PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	set_sda_read_mode();
	res = PIN_RD(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
	set_sda_write_mode();

	return res;
}

static uint8_t send_byte_with_ack(uint8_t byte)
{
	for (uint8_t i = 7; i >= 0; --i) {
		PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		if ((addr >> i) & 0x1)
			PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		else 
			PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	}
	if (wait_ack() != 0) {
		p_i2c_sf->err_code = I2C_SOFT_ERR_TX_ABORT_NOACK;
		return 1;
	}
	return 0;
}

static uint8_t send_byte_with_noack(uint8_t byte)
{
	for (uint8_t i = 7; i >= 0; --i) {
		PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		if ((addr >> i) & 0x1)
			PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		else 
			PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	}
	if (wait_ack() != 1) {
		return 1;
	}
	return 0;
}

static void send_stop(void)
{
	PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
	PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
}

static i2c_soft_err_enum_t send_addr(i2c_soft_mode_enum_t wr_rd_mode)
{
	uint8_t addr = (p_i2c_sf->addr << 1) + wr_rd_mode;

	for (uint16_t i = p_i2c_sf->addr_len; i >= 0; --i) {
		PIN_LO(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		if ((addr >> i) & 0x1)
			PIN_HI(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		else 
			PIN_LO(p_i2c_sf->sda->port, p_i2c_sf->sda->pin);
		PIN_HI(p_i2c_sf->scl->port, p_i2c_sf->scl->pin);
		// need delay ?
	}
	if (wait_ack() != 0)
		return I2C_SOFT_ERR_SLAVE_ADDR_NOACK;
	return I2C_SOFT_ERR_NONE;
}

i2c_soft_err_enum_t i2c_soft_write(i2c_soft_t *i2c, uint8_t *buffer, uint16_t len)
{
	uint8_t res =  I2C_SOFT_ERR_NONE;

	// send start condition
	send_start_sig();
	// send slave address
	if (send_addr(I2C_SOFT_WRITE) != I2C_SOFT_ERR_NONE)
		return res;

	for (uint16_t i = 0; i < len - 2; ++i)
		if (send_byte_with_ack(buffer[len]) != 0)
			return res;

	// send

	return res;
}

i2c_soft_err_t i2c_soft_read(i2c_soft_t *i2c, uint8_t *buffer, uint16_t len)
{
	return I2C_SOFT_ERR_NONE;
}
