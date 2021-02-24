#ifndef _I2C_SOFT_
#define	_I2C_SOFT

#include "main.h"	// GPIO port define
#include "common.h"
#include <stdint>

typedef enum {
	I2C_SOFT_ERR_NONE = 0,
	I2C_SOFT_ERR_GPIO_PORT,
	I2C_SOFT_ERR_ADDR_LEN
} i2c_soft_err_enum_t;

typedef struct i2c_soft {
	gpio_t *sda;
	gpio_t *scl;
	uint8_t slave_addr;
	uint8_t addr_len;
} i2c_soft_t;

#endif
