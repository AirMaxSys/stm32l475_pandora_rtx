#include "aht10.h"

#define AHT10_DBG	1

#define AHT10_I2C_ADDR		0x38u
/* AHT10 commands*/
#define AHT10_CMD_CALIBRATE	0xE1u
#define AHT10_CMD_FETCH_DAT	0xACu

gpio_t aht10_i2c_sda = {I2C_SOFT_SDA_Port, I2C_SOFT_SDA_Pin};
gpio_t aht10_i2c_scl = {AHT10_IIC_CLK_GPIO_Port, AHT10_IIC_CLK_Pin};

i2c_soft_t aht10_i2c_dev = {
	.sda = &aht10_i2c_sda,
	.scl = &aht10_i2c_scl,
	.addr_len = 7,
	.slave_addr = AHT10_I2C_ADDR,
};

uint8_t aht10_status_reg(void)
{
	// default status regster value is 0x19
	uint8_t status_reg = 0;
	
	// read aht10 STATUS value
	i2c_soft_recv_datas(&status_reg, 1);
	return status_reg;
}

static void aht10_calibrate(void)
{
	uint8_t cmd[] = {AHT10_CMD_CALIBRATE, 0x8, 0};
	
	if (i2c_soft_send_datas(cmd, 3, 1) != I2C_SOFT_ERR_NONE) {
		printf("aht10 calibrate I2C error\r\n");
	}
}

void aht10_init(aht10_t *paht)
{
	if (!paht)	return;

	i2c_soft_init(&aht10_i2c_dev);
	printf("AHT10 init ok, status value is:0x%02x\r\n", aht10_status_reg());
	paht->temp = paht->humi = 0;
	memset(paht->buf, 0x0, AHT10_BUFFER_SIZE);
	aht10_calibrate();
}

void aht10_get_value(aht10_t *paht)
{
	uint8_t ret = 0;
	uint8_t cmd[] = {AHT10_CMD_FETCH_DAT, 0, 0};
	
	if ((ret = i2c_soft_send_datas(cmd, 3, 1)) != I2C_SOFT_ERR_NONE) {
		printf("aht10 get datas err send cmd err=%d\r\n", ret);
		return;
	}
	if (i2c_soft_recv_datas(paht->buf, 6) != I2C_SOFT_ERR_NONE) {
		printf("aht10 get datas err fetch datas\r\n");
		return;
	}
#ifdef AHT10_DBG
	printf("AHT10 fetch row data:");
	for (uint8_t i = 0; i < AHT10_BUFFER_SIZE; ++i)
		printf("0x%02x ", paht->buf[i]);
	puts("\r");
#endif
	paht->humi = ((paht->buf[1]) << 12 | paht->buf[2] << 4 | (paht->buf[3] & 0xF0)) * 1000.0 / (1 << 20);
	paht->temp = ((paht->buf[3] & 0xf) << 16 | paht->buf[4] << 8 | paht->buf[5]) * 2000.0 / (1 << 20);
#ifdef AHT10_DBG
	printf("AHT10 temp:%.1f humi:%.1f%%\r\n", aht10_temp(paht), aht10_humi(paht));
#endif
}

float aht10_temp(aht10_t *paht)
{
	return (paht->temp/10.0-50);
}

float aht10_humi(aht10_t *paht)
{
	return (paht->humi/10.0);
}
