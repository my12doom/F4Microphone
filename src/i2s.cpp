#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_dma.h>
#include <misc.h>

#include <math.h>
#include <string.h>
#include "i2s.h"
#include "fifo2.h"

FIFO<CODEC_DMA_BUF_LEN*4> fifo;

int16_t codec_data[CODEC_DMA_BUF_LEN];
int codec_ready = 0;

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

	// DMA configure
	NVIC_InitTypeDef   NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_Init(&NVIC_InitStructure);

restart:
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
	i2s_init_str.I2S_DataFormat = I2S_DataFormat_16bextended;
	i2s_init_str.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	i2s_init_str.I2S_AudioFreq = 48000;
	i2s_init_str.I2S_CPOL = I2S_CPOL_High;
	I2S_Cmd(SPI3, DISABLE);
	SPI_I2S_DeInit(SPI3);
	I2S_Init(SPI3, &i2s_init_str);	
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
	//SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
	
	// go!
	I2S_Cmd(SPI3, ENABLE);

	// wait for data
	while(DMA1_Stream0->NDTR == CODEC_DMA_BUF_LEN)
		;

	// check for framing error
	// TODO: check and fix in realtime
	if (SPI3->SR & 0x100)
		goto restart;

}

extern "C" int codec_pos()
{
	return CODEC_DMA_BUF_LEN - DMA1_Stream0->NDTR;
}

extern "C" int codec_pop(void *p, int bytes)
{
	return fifo.pop(p, bytes);
}


extern "C" void DMA1_Stream0_IRQHandler(void)
{
	//NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
	int16_t *data;
	int count = CODEC_DMA_BUF_LEN/2;


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

/*
extern "C" void DMA1_Stream5_IRQHandler(void)
{
	//NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn);

	// first half buffer	
	if (DMA1->HISR & DMA_HISR_HTIF5) {
		DMA1->HIFCR = DMA_HIFCR_CHTIF5;
		codec_ready |= 1;
	}

	// second half buffer
	if (DMA1->HISR & DMA_HISR_TCIF5) {
		DMA1->HIFCR = DMA_HIFCR_CTCIF5;
		codec_ready |= 2;
	}
}
*/