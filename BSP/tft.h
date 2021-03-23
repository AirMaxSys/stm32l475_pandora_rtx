#ifndef	_TFT_H_
#define	_TFT_H_

#ifdef _cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* tft LCD cmd definations */
#define	TFT_CMD_SW_RESET	0x1u	// software reset
#define TFT_CMD_RDDPM		0xAu	// read display power mode
#define TFT_CMD_RAMWR		0x2Cu	// memory write
#define TFT_CMD_RAWRD		0x2Eu	// memory read
#define	TFT_CMD_RDID1		0xDAu	// read ID1
#define	TFT_CMD_RDID2		0xDBu	// read ID2
#define	TFT_CMD_RDID3		0xDCu	// read ID3

void tft_power_on(void);
void tft_hw_reset(void);

void tft_read_display_id(void);

#ifdef _cplusplus
}
#endif

#endif
