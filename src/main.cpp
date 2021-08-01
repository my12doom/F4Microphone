extern "C" {
#include "usbd_audio_core.h"
#include "usbd_usr.h"
}
#include "i2s.h"
#include "F4GPIO.h"
#include "F4SysTimer.h"
#include "II2C.h"
#include <math.h>


using namespace HAL;
using namespace STM32F4;

extern "C" void SetSysClock();
int default_download_IC_1(bool _24bit);
int set_volume(uint8_t gain_code, bool muted = false);	// 0.75db/LSB


volatile int16_t volume = 0x3f00;
volatile uint8_t muted = 0;
volatile int volume_changed = 0;
volatile int channel = 0;
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;
F4GPIO scl(GPIOC, GPIO_Pin_13);
F4GPIO sda(GPIOC, GPIO_Pin_14);
I2C_SW i2c(&scl, &sda);

	int t;
	int32_t v[48];

int main(void)
{	
#ifdef USE_ULPI_PHY
	// use clock from USB3320
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));		// switch to HSI
	
	F4GPIO usb_reset(GPIOC, GPIO_Pin_14);
	usb_reset.set_mode(MODE_OUT_PushPull);
	usb_reset.write(0);
	systimer->delayms(10);
	usb_reset.write(1);
	systimer->delayms(5);	
	SetSysClock();
	SystemCoreClockUpdate();
	
	// check clock
	F4GPIO led0(GPIOC, GPIO_Pin_6);
	F4GPIO led1(GPIOC, GPIO_Pin_7);

	led0.write(1);
	led1.write(1);
	led0.set_mode(MODE_OUT_OpenDrain);
	led1.set_mode(MODE_OUT_OpenDrain);
	if (SystemCoreClock != 84000000)
		led0.write(0);
	else
		led1.write(0);
#endif
	systimer->delayms(10);

	F4GPIO led2(GPIOB, GPIO_Pin_15);
	led2.write(1);
	led2.set_mode(MODE_OUT_PushPull);

	systimer->delayms(100);
	i2s_setup();
	i2c.set_speed(20);
	for(int i=0; i<3; i++)
	if (default_download_IC_1(AUDIO_RES==24)<0)
		NVIC_SystemReset();
	
	
	if (i2s_check()<0)
		NVIC_SystemReset();
		


	led2.write(0);

	F4GPIO button(GPIOB, GPIO_Pin_14);
	button.set_mode(MODE_IN);
	int last_button = button.read();

	USBD_Init(&USB_OTG_dev,
			USB_OTG_HS_CORE_ID,
			&USR_desc, 
			&AUDIO_cb, 
			&USR_cb);

	
	float PI = acos(-1.0);
	t = systimer->gettime();
	for(int i=0; i<48; i++)
	{
		v[i] = 32767 * sin(i*2*PI*0.01f);
	}
	
	t = systimer->gettime() - t;


	while(1)
	{
		bool but = button.read();
		if (but != last_button)
		{
			systimer->delayus(200);		// debounce
			last_button = but;
			if (but)
				channel = (channel+1)%4;
		}

		if (volume_changed)
		{
			set_volume(volume/0x10 + 0x3f, muted);
			volume_changed = 0;
		}
	}
}

int SIGMA_WRITE_REGISTER_BLOCK( uint8_t devAddress, uint16_t address, int length, uint8_t* pData )
{
    if (i2c.start()<0) {
        return -1;
    }

    i2c.tx(devAddress&0xFE);

    if (i2c.wait_ack()<0) {
		i2c.stop();
		return -1;
    }

    i2c.tx(address>>8);
    if (i2c.wait_ack()<0)
	{
		i2c.stop();
		return -1;
	}

	i2c.tx(address);
    if (i2c.wait_ack()<0)
	{
		i2c.stop();
		return -1;
	}

    for(int i=0; i<length; i++)
    {
        i2c.tx(pData[i]);
        
        if (i2c.wait_ack()<0)
        {
            i2c.stop();
            return -1;
        }
    }

    i2c.stop();

    return 0;
}

int SIGMA_WRITE_DELAY( uint8_t devAddress, int length, uint8_t *pData )
{
	systimer->delayms(100);
	return 0;
}

int set_volume(uint8_t gain_code, bool muted/* = false */)	// 0.75db/LSB
{
	gain_code &= 0x3f;
	gain_code <<= 2;
	gain_code |= 1;
	if (!muted)
		gain_code |= 0x2;
	uint8_t v[2] = {gain_code, gain_code};

	return SIGMA_WRITE_REGISTER_BLOCK(0x70, 0x400E, 2, v);
}
