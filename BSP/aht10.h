#ifndef	_AHT10_H_
#define	_AHT10_H_

#ifdef	__cplusplus
extern "C" {
#endif

#include "i2c_soft.h"
#include "main.h"
#include <stdint.h>

typedef struct aht10 {
	uint16_t temp;
	uint16_t humi;
} aht10_t;

void aht10_init(aht10_t *paht);
void aht10_get_value(void);
float aht10_temp(void);
float ath10_humi(void);

uint8_t aht10_status_reg(void);

#ifdef __cplusplus
}
#endif

#endif
