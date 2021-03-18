#ifndef	_TFT_H_
#define	_TFT_H_

#ifdef _cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void tft_power_on(void);
void tft_hw_reset(void);

void tft_read_display_id(void);

#ifdef _cplusplus
}
#endif

#endif
