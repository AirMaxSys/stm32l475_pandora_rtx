#ifndef _I2C_SOFT_H_
#define	_I2C_SOFT_H_

#include "main.h"	// GPIO port define
#include "common.h"
#include <stdint.h>

typedef enum {
	I2C_SOFT_WRITE = 0,
	I2C_SOFT_READ = 1
} i2c_soft_mode_enum_t;

typedef enum {
	I2C_SOFT_ERR_NONE = 0,
	I2C_SOFT_ERR_INIT_INSTENCE,
	I2C_SOFT_ERR_GPIO_PORT,
	I2C_SOFT_ERR_ADDR_LEN,
	I2C_SOFT_ERR_SLAVE_ADDR_NOACK,
	I2C_SOFT_ERR_TX_ABORT_NOACK,
} i2c_soft_err_enum_t;

typedef struct i2c_soft {
	gpio_t *sda;
	gpio_t *scl;
	uint16_t slave_addr;
	uint8_t addr_len;
	i2c_soft_err_enum_t err_code;
} i2c_soft_t, *p_i2c_sf;

#endif
