/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: rtc low level

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hal-rtc.h"

#define HAL_RTC_TICK			(1000)

uint32_t hal_rtc_cnt;
uint32_t hal_rtc_reload_val;

static rtc_call_back_t rtc_call_back = NULL;

#define HAL_RTC_DEBUG			(0)

void hal_rtc_init(rtc_call_back_t rtc_cb)
{
	if(rtc_cb == NULL){
		return;
	}

	rtc_call_back = rtc_cb;
	hal_rtc_cnt = 0;

#if (HAL_RTC_DEBUG == 0)
	/** Enable LSI as RTC clock, LSI: 38KHz, RTCCLK: 38KHz, RTCCLK = LSI */
	CLK->CRTCR = (uint8_t)CLK_RTCCLKSource_LSI | CLK_RTCCLKDiv_1;
	while(CLK->CRTCR & CLK_CRTCR_RTCSWBSY);
#else
	/** Enable HSI as RTC clock, LSI: 16MHz, RTCCLK: 16MHz/4, RTCCLK = 4MHz */
	CLK->CRTCR = (uint8_t)CLK_RTCCLKSource_HSI | CLK_RTCCLKDiv_4;
	while(CLK->CRTCR & CLK_CRTCR_RTCSWBSY);
#endif

	/** Enable RTC peripheral */
	CLK->PCKENR2 |= CLK_PCKENR2_RTC;

	/** Disable RTC protection */
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	/** Disable the Wakeup timer in RTC_CR2 register */
	RTC->CR2 &= (uint8_t)~RTC_CR2_WUTE;
	RTC->ISR2 &= ~RTC_ISR2_WUTF;

	/** Enter initialization mode */
	RTC->ISR1 |= (RTC_ISR1_INIT);
	while( (RTC->ISR1 & RTC_ISR1_INITF) != RTC_ISR1_INITF );

#if (HAL_RTC_DEBUG == 0)
	/** Prescaler 38 = (1 + 1)x(18 + 1) */
	RTC->APRER = 1;
	RTC->SPRERH = 0;
	RTC->SPRERL= 19;
#else
	/** Prescaler 38 = (1 + 1)x(18 + 1) */
	RTC->APRER = 1;
	RTC->SPRERH = 0;
	RTC->SPRERL= 1;
#endif

	/** Exit init mode */
	RTC->ISR1 &= (uint8_t)~RTC_ISR1_INIT;

	/** Choose SPRE as clock */
	RTC->CR1 = (RTC->CR1 & ~RTC_CR1_WUCKSEL) | RTC_WakeUpClock_CK_SPRE_16bits;

	/** Enable write to Wakeup timer, wait until valid */
	RTC->ISR1 |= RTC_ISR1_WUTWF;
	while( (RTC->ISR1 & RTC_ISR1_WUTWF) != RTC_ISR1_WUTWF );

	/** Init wakeup register */
	RTC->WUTRH = (uint8_t)((HAL_RTC_TICK) >> 8);
	RTC->WUTRL = (uint8_t)(HAL_RTC_TICK);

	/** Disable write to Wakeup timer */
	RTC->ISR1 &= (uint8_t)~RTC_ISR1_WUTWF;

	/** Enable wakeup timer */
	RTC->CR2 |= (uint8_t)RTC_CR2_WUTE;

	/** Enable wakeup timer interrupt */
	RTC->CR2 |= (uint8_t)RTC_CR2_WUTIE;

	/** Enable RTC protection */
	RTC->WPR = 0xFF;
}

void hal_rtc_start()
{
	/** Disable RTC protection */
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	/** Enable wakeup timer */
	RTC->CR2 |= (uint8_t)RTC_CR2_WUTE;

	/** Enable wakeup timer interrupt */
	RTC->CR2 |= (uint8_t)RTC_CR2_WUTIE;

	/** Enable RTC protection */
	RTC->WPR = 0xFF;
}

void hal_rtc_stop()
{
	hal_rtc_reload_val = 0;
	hal_rtc_cnt = 0;

	/** Disable RTC protection */
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	/** Disable wakeup timer */
	RTC->CR2 &= (uint8_t)~RTC_CR2_WUTE;

	/** Disable wakeup timer interrupt */
	RTC->CR2 &= (uint8_t)~RTC_CR2_WUTIE;

	/** Enable RTC protection */
	RTC->WPR = 0xFF;
}

/** return RTC tick unit in Hz */
uint32_t hal_rtc_tick(void)
{
	return (HAL_RTC_TICK);
}

static void hal_rtc_set_reg(uint16_t val)
{
	/** Disable RTC protection */
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	/** Disable wakeup timer */
	RTC->CR2 &= (uint8_t)~RTC_CR2_WUTE;

	/** Enable write to Wakeup timer, wait until valid */
	RTC->ISR1 |= RTC_ISR1_WUTWF;
	while( (RTC->ISR1 & RTC_ISR1_WUTWF) != RTC_ISR1_WUTWF );

	/** Disable RTC protection */
	RTC->WUTRH = (uint8_t)((val) >> 8);
	RTC->WUTRL = (uint8_t)(val);

	/** Disable write to Wakeup timer */
	RTC->ISR1 &= (uint8_t)~RTC_ISR1_WUTWF;

	/** Enable RTC protection */
	RTC->CR2 |= (uint8_t)RTC_CR2_WUTE;

	/** Enable RTC protection */
	RTC->WPR = 0xFF;
}

void hal_rtc_set_timeout(uint32_t val)
{
	if(val == 0){
		return;
	}

	hal_rtc_reload_val = val;
	hal_rtc_cnt = 0;

	hal_rtc_set_reg(val);
}


uint32_t hal_rtc_get_elapsed(void)
{
	uint32_t ret = 0, wutr;

	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	/** Enable write to Wakeup timer, wait until valid */
//	RTC->ISR1 |= RTC_ISR1_WUTWF;
//	while( (RTC->ISR1 & RTC_ISR1_WUTWF) != RTC_ISR1_WUTWF );

	wutr = RTC->WUTRH;
	wutr <<= 8;
	wutr |= RTC->WUTRL;

	/** Enable RTC protection */
	RTC->WPR = 0xFF;

	if(hal_rtc_cnt == 0){
		ret = (hal_rtc_reload_val&0x0000FFFF) - wutr;
	}else{
		ret = hal_rtc_cnt + (0xFFFF - wutr);
	}

	return ret;
}

INTERRUPT_HANDLER( HAL_RTC, 4 )
{
	if(RTC->ISR2 & RTC_ISR2_WUTF){
		RTC->ISR2 &= ~RTC_ISR2_WUTF;
		if( hal_rtc_reload_val == 0 || rtc_call_back == NULL ){
			hal_rtc_stop();
			return;
		}
		if( (hal_rtc_reload_val & 0xFFFF0000) == 0 ){
			/** time to go */
			hal_rtc_cnt = 0;
			hal_rtc_set_reg(hal_rtc_reload_val);
			rtc_call_back();
		}else{
			if( hal_rtc_cnt == 0){
				hal_rtc_cnt = hal_rtc_reload_val & 0x0000FFFF;
				hal_rtc_set_reg(0xFFFF);
			}else{
				hal_rtc_cnt += 0x00010000;
				if(hal_rtc_cnt == hal_rtc_reload_val){
					/** time to go */
					hal_rtc_cnt = 0;
					hal_rtc_set_reg(hal_rtc_reload_val);
					rtc_call_back();
				}else if(hal_rtc_cnt > hal_rtc_reload_val){
					/** hal_rtc_cnt should never be larger than hal_rtc_reload_val */
					while(1);
				}else{
					/**do nothing.*/
				}
			}
		}
	}
}




