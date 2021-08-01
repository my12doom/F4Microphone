#ifndef I2S_SETUP_H
#define I2S_SETUP_H 

#include <stdint.h> 
#include <stddef.h> 

#define AUDIO_SAMPLE_RATE 48000
#define AUDIO_RES 24
#define AUDIO_CHANNEL 2
#define CODEC_DMA_BUF_LEN (AUDIO_SAMPLE_RATE*AUDIO_RES/8*AUDIO_CHANNEL/1000/2)


#ifdef __cplusplus
extern "C" {
#endif

void i2s_setup(void);
int i2s_check(void);

int codec_pos(void);
int codec_pop(void *p, int bytes);

#ifdef __cplusplus
}
#endif
	
#endif /* I2S_SETUP_H */
