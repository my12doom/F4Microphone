#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_dma.h>
#include <misc.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "i2s.h"
#include "fifo2.h"
#include "F4SysTimer.h"

FIFO<CODEC_DMA_BUF_LEN*8> fifo;

int16_t codec_data[CODEC_DMA_BUF_LEN];
int codec_ready = 0;

int32_t tx_data[6000];
int tx_data_count = 0;

typedef __packed struct
{
	uint8_t d[6];
} sample24;

typedef __packed struct
{
	uint8_t d[8];
} sample32;

int16_t data16;
int i2s_configure_tx(float tx_freq = 1000)
{
	float epsilon = 1e-2;
	float PI = acos(-1.0f);

	float phase = 0;
	memset(tx_data, 0, sizeof(tx_data));
	for(int i=0; i<sizeof(tx_data)/sizeof(tx_data[0]); i++)
	{
		if (AUDIO_RES == 24)
		{
			int32_t data24 = cos(phase) * ((1<<31)-1);
			uint8_t msb = data24 >> 24;
			uint8_t midsb = data24 >> 16;
			uint8_t lsb = data24 >> 8;
			
			sample32 *p32 = (sample32*)&tx_data[i*2+0];
			sample32 *p32r = (sample32*)&tx_data[i*2+1];
			
			p32->d[0] = msb;
			p32->d[1] = 0;
			p32->d[2] = lsb;
			p32->d[3] = midsb;
			
			*p32r = *p32;
		}
		else
		{
			data16 = cos(phase) * 32767;
			tx_data[i] = ((uint32_t)data16 << 16) | (((uint32_t)data16)&0xffff);
		}

		phase += tx_freq * 2 * PI / 48000;
		if (phase >= 2*PI)
			phase -= 2*PI;

		if (i>0 && (fabs(phase) < epsilon || fabs(phase-2*PI) < epsilon))
		{
			tx_data_count = i+1;
			break;
		}

		if (i == sizeof(tx_data)/sizeof(tx_data[0])/2-1)
			return -1;
	}
	
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(I2S3ext->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&tx_data[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = (AUDIO_RES == 24) ? (tx_data_count*4) : (tx_data_count*2);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_MemoryBurst_Single;
	DMA_Cmd(DMA1_Stream5, DISABLE);

	DMA_DeInit(DMA1_Stream5);
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_HTIF0);
	DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF0);
	DMA_ITConfig(DMA1_Stream5, DMA_IT_HT, ENABLE);
	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);

	I2S_InitTypeDef i2s_init_str;
	i2s_init_str.I2S_Mode = I2S_Mode_SlaveRx;		// yes RX, strange behavior in StdPeriph driver
	i2s_init_str.I2S_Standard = I2S_Standard_LSB;
	i2s_init_str.I2S_DataFormat = (AUDIO_RES == 16) ? I2S_DataFormat_16bextended : I2S_DataFormat_24b;
	i2s_init_str.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	i2s_init_str.I2S_AudioFreq = 48000;
	i2s_init_str.I2S_CPOL = I2S_CPOL_High;
	I2S_Cmd(I2S3ext, DISABLE);
	SPI_I2S_DeInit(I2S3ext);
	I2S_FullDuplexConfig(I2S3ext, &i2s_init_str);
	
	SPI_I2S_DMACmd(I2S3ext, SPI_I2S_DMAReq_Tx, ENABLE);

	return 0;
}

void i2s_setup()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	// GPIO & AF configure
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);			// CK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_I2S3ext);		// SD_ext
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);			// SD
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3);		// WS
	
	/*
	i2s_configure_tx();
	//I2S_Cmd(SPI3, ENABLE);
	I2S_Cmd(I2S3ext, ENABLE);
	return;
	*/

	// DMA configure
	NVIC_InitTypeDef   NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(SPI3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&codec_data[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = CODEC_DMA_BUF_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Cmd(DMA1_Stream0, DISABLE);
	fifo.reset();

	DMA_DeInit(DMA1_Stream0);
	DMA_Init(DMA1_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream0, ENABLE);
	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_HTIF0);
	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
	DMA_ITConfig(DMA1_Stream0, DMA_IT_HT, ENABLE);
	DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
	
	// I2S configure
	I2S_InitTypeDef i2s_init_str;
	i2s_init_str.I2S_Mode = I2S_Mode_SlaveRx;
	i2s_init_str.I2S_Standard = I2S_Standard_LSB;
	i2s_init_str.I2S_DataFormat = (AUDIO_RES == 16) ? I2S_DataFormat_16bextended : I2S_DataFormat_24b;
	i2s_init_str.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	i2s_init_str.I2S_AudioFreq = AUDIO_SAMPLE_RATE;
	i2s_init_str.I2S_CPOL = I2S_CPOL_High;
	I2S_Cmd(SPI3, DISABLE);
	SPI_I2S_DeInit(SPI3);
	I2S_Init(SPI3, &i2s_init_str);	
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);

	// configure tx
	i2s_configure_tx(500);
	
	// go!
	I2S_Cmd(SPI3, ENABLE);
	I2S_Cmd(I2S3ext, ENABLE);
}

int i2s_check()
{	
	// wait for data
	while(DMA1_Stream0->NDTR > CODEC_DMA_BUF_LEN/2)
		;

	// check for framing error
	// TODO: check and fix in realtime
	if (SPI3->SR & 0x100)
		return -1;
	
	return 0;
}

extern "C" int codec_pos()
{
	return CODEC_DMA_BUF_LEN - DMA1_Stream0->NDTR;
}

extern "C" int codec_pop(void *p, int bytes)
{
	return fifo.pop(p, bytes);
}

int dt = 0;
bool rise = true;
extern "C" void DMA1_Stream5_IRQHandler(void)
{
	int t = systimer->gettime();

	int32_t *data;
	static float phase = 0;
	int count = (AUDIO_RES == 24 ? tx_data_count : tx_data_count/2);
	static float tx_freq = 5;
	static float PI = acos(-1.0f);
	
	float scale = 3e-4;
	//tx_freq *= rise ? (1+scale) : (1-scale);
	tx_freq = 1200;
	if (tx_freq > 20000 || tx_freq < 5)
		rise = !rise;

	// first half buffer
    if (DMA1->HISR & DMA_HISR_HTIF5) {
        DMA1->HIFCR = DMA_HIFCR_CHTIF5;
		data = tx_data;
    }

	// second half buffer
    if (DMA1->HISR & DMA_HISR_TCIF5) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;
		data = tx_data + count;
    }

	count /= 2;
	for(int i=0; i<count; i++)
	{
		if (AUDIO_RES == 24)
		{
			int32_t data24 = cos(phase) * ((1<<29)-1) + (rand() & 1);
			uint8_t msb = data24 >> 24;
			uint8_t midsb = data24 >> 16;
			uint8_t lsb = data24 >> 8;
			
			sample32 *p32 = (sample32*)&data[i*2+0];
			sample32 *p32r = (sample32*)&data[i*2+1];
			
			p32->d[0] = msb;
			p32->d[1] = 0;
			p32->d[2] = lsb;
			p32->d[3] = midsb;
			
			*p32r = *p32;
		}
		else
		{
			int16_t data16 = cos(phase) * 32767;
			data[i] = ((uint32_t)data16 << 16) | (((uint32_t)data16)&0xffff);
		}

		phase += tx_freq * 2 * PI / 48000;
		if (phase >= 2*PI)
			phase -= 2*PI;
	}

	dt = systimer->gettime() - t;

}

extern "C" void DMA1_Stream0_IRQHandler(void)
{
	int16_t *data;


	// first half buffer
    if (DMA1->LISR & DMA_LISR_HTIF0) {
        /* clear flag */
        DMA1->LIFCR = DMA_LIFCR_CHTIF0;
        codec_ready |= 1;
		data = codec_data;
    }

	// second half buffer
    if (DMA1->LISR & DMA_LISR_TCIF0) {
        /* clear flag */
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;
        codec_ready |= 2;
		data = codec_data + CODEC_DMA_BUF_LEN/2;
		
    }

	if (AUDIO_RES == 16)
	{	
		int count = CODEC_DMA_BUF_LEN/2;
		extern int channel;
		if (channel == 0)
		{
			// stereo, do nothing
		}
		else if (channel == 1)
		{
			// left channel
			for(int i=0; i<count; i+=2)
				data[i+1] = data[i];
		}
		else if (channel == 2)
		{
			// right channel
			for(int i=0; i<count; i+=2)
				data[i] = data[i+1];
		}
		else
		{
			// mute
			memset(data, 0, CODEC_DMA_BUF_LEN);
		}
		
		fifo.put(data, CODEC_DMA_BUF_LEN);
	}

	else if (AUDIO_RES == 24)
	{
		// strange i2s byte mapping
		int sample_count = CODEC_DMA_BUF_LEN / sizeof(sample32);
		sample24 *p24 = (sample24*)data;
		sample32 *p32 = (sample32*)data;
		for(int i=0; i<sample_count; i++)
		{
			sample24 s;
			s.d[0] = p32[i].d[2];
			s.d[1] = p32[i].d[3];
			s.d[2] = p32[i].d[0];

			s.d[3] = p32[i].d[2+4];
			s.d[4] = p32[i].d[3+4];
			s.d[5] = p32[i].d[0+4];
			
			p24[i] = s;
		}

		fifo.put(p24, sample_count * sizeof(sample24));
	}

}
