#ifndef I2S_SETUP_H
#define I2S_SETUP_H 

#include <stdint.h> 
#include <stddef.h> 

#define CODEC_DMA_BUF_LEN 192


extern int16_t codec_data[CODEC_DMA_BUF_LEN];

extern int codec_ready;

#ifdef __cplusplus
extern "C" {
#endif
	
void i2s_setup();
int codec_pos();
int codec_pop(void *p, int bytes);

#ifdef __cplusplus
}
#endif
	
#endif /* I2S_SETUP_H */
