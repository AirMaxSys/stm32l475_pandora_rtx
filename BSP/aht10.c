#include "aht10.h"
#include <stdio.h>

#define AHT10_CMD_CALIBRATE	0xE1u
#define AHT10_CMD_FETCH_DAT	0xACu

gpio_t aht10_i2c_sda = {GPIOC, GPIO_PIN_1};
gpio_t aht10_i2c_scl = {AHT10_IIC_CLK_GPIO_Port, AHT10_IIC_CLK_Pin};

i2c_soft_t aht10_i2c_dev = {
	.sda = &aht10_i2c_sda,
	.scl = &aht10_i2c_scl,
	.addr_len = 7,
	.slave_addr = 0x38,
};

uint8_t aht10_status_reg(void)
{
	// default status regster value is 0x19
	uint8_t status_reg = 0;
	// read aht10 STATUS value
	i2c_soft_read(&status_reg, 1);
	return status_reg;
}

static void aht10_calibrate(void)
{
	uint8_t cmd[] = {AHT10_CMD_CALIBRATE, 0x8, 0};
	i2c_soft_write(cmd, 3);
}

void aht10_init(aht10_t *paht)
{
	if (!paht)	return;

	i2c_soft_init(&aht10_i2c_dev);
	printf("AHT10 init ok, status value is:0x%02x\r\n", aht10_status_reg());
	paht->temp = paht->humi = 0;
	aht10_calibrate();
}

void aht10_get_value(void)
{
	uint8_t data[6];
	uint8_t cmd[] = {AHT10_CMD_FETCH_DAT, 0, 0};
	i2c_soft_write(cmd, 3);
	i2c_soft_read(data, 6);
	printf("AHT10 fetch row data:");
	for (uint8_t i = 0; i < 6; ++i)
		printf("0x%02x ", data[i]);
	puts("\r");
	float humi = ((data[1]) << 12 | data[2] << 4 | (data[3] & 0xF0)) * 100.0 / (1 << 20);
	float temp = ((data[3] & 0xf) << 16 | data[4] << 8 | data[5]) * 200.0 / (1 << 20) - 50;
	printf("AHT10 temp:%.1f humi:%.1f%%\r\n", temp, humi);
}

float aht10_temp(void)
{

}

float ath10_humi(void)
{

}
