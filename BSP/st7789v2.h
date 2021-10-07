#ifndef _ST7789V2_H_
#define _ST7789V2_H_

#ifdef  _cplusplus
extern "C" {
#endif

/* tft LCD cmd definations */
#define	TFT_CMD_SW_RESET    0x1u    // software reset
#define TFT_CMD_RDDPM       0xAu    // read display power mode
#define TFT_CMD_RAMWR       0x2Cu   // memory write
#define TFT_CMD_RAWRD       0x2Eu   // memory read
#define	TFT_CMD_RDID1       0xDAu   // read ID1
#define	TFT_CMD_RDID2       0xDBu   // read ID2
#define	TFT_CMD_RDID3       0xDCu   // read ID3

int8_t tft_write_dats(uint8_t cmd, uint8_t *txbuf, uint16_t len);

void tft_init(void);
void tft_power_on(void);
void tft_hw_reset(void);
void tft_set_display_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void tft_read_display_id(void);

#ifdef  _cplusplus
}
#endif

#endif
