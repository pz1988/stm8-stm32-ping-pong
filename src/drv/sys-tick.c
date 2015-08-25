/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: system tick to drive delay and supply application software timer.

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hal-sys-tick.h"
#include "sys-tick.h"

#ifdef STM32_SENSERNODE_DELAY
#include "timer-board.h"
#endif

#define SYS_TICK_NUM            16
#define SYS_TICK_STA_MSB        0x8000
#define SYS_TICK_MAX_VALUE      0xFFFF

#define SYS_TICK_MS             (HAL_SYS_TICK_PERIOD)

/**

*/
static volatile uint16_t sys_tick_ctl=0;
static volatile uint16_t sys_tick_use_sta=0;
static volatile uint16_t sys_tick[SYS_TICK_NUM]={0};

static volatile uint32_t delay_counter = 0;

/**
	@brief avr timer0 compare A interrupt. when system clock is 8MHz
		interrupt cycle 100us.
*/
static void sys_tick_handler(void)
{
	uint8_t i;

	delay_counter++;

	/** check every sys_tick flag */
	for(i=0;i<SYS_TICK_NUM;i++){
		/** see if sys tick is open */
		if(sys_tick_ctl&(SYS_TICK_STA_MSB>>i)){
			/** opened sys_tick flag plus 1 */
			if(sys_tick[i] != SYS_TICK_MAX_VALUE){
				sys_tick[i]++;
			}
		}
	}
}

/**
	@brief timer0 initialize function. timer base 100us.
*/
void sys_tick_init(void)
{
	hal_sys_tick_init(sys_tick_handler);
	delay_counter = 0;
}

/**
	@brief system tick flag set ON/OFF
	@param num: specifies the tick flag number to set
		@arg 0~8
	@param status: new status for the specified tick flag
		parameter value: ON, OFF.
	@retval 0-successful, 1-failed
*/
uint8_t sys_tick_set(uint8_t num, sys_tick_sta_type status)
{
	if(num>SYS_TICK_NUM){
		return 1;
	}
	if(status == ON){
		sys_tick_ctl |= (SYS_TICK_STA_MSB>>num);
	}else{
		sys_tick_ctl &= ~(SYS_TICK_STA_MSB>>num);
//		sys_tick[num]=0;
	}
	return 0;
}

/**
	@brief clear specified system tick flag
	@para num: specifies the tick flag number to set
		@arg 0~8
	@retval 0-successful, 1-failed
*/
uint8_t sys_tick_clear(uint8_t num)
{
	if(num>SYS_TICK_NUM){
		return 1;
	}
	sys_tick[num]=0;
	return 0;
}

/**
	@brief get specified system tick flag value
	@para num: specifies the tick flag number to set
		@arg 0~8
	@retval system tick flag value
*/
uint16_t sys_tick_get(uint8_t num)
{
	if(num>SYS_TICK_NUM){
		return 0;
	}
	return sys_tick[num];
}

/**
	@brief apply system tick flag
	@retval applied system tick flag number
*/
uint8_t sys_tick_apply(void)
{
	uint8_t i;
	for(i=0;i<SYS_TICK_NUM;i++){
		if( (sys_tick_use_sta&(SYS_TICK_STA_MSB>>i)) == 0){
			sys_tick_use_sta |= (SYS_TICK_STA_MSB>>i);
			return i;
		}
	}
	/** no idle tick */
	return 0xFF;
}

/**
	@brief set a specified system tick flag specified value
	@para num: specifies the tick flag number to set
		@arg 0~8
	@retval 0-successful, 1-failed
*/
uint8_t sys_tick_set_value(uint8_t num, uint16_t value)
{
	if(num>SYS_TICK_NUM){
		return 1;
	}else{
		sys_tick[num] = value;
	}
	return 0;
}

void delay_ms(uint32_t ms)
{
	#ifndef STM32_SENSERNODE_DELAY
	uint32_t end_value;

    /** Make sure IRQ is enabled to do the delay */
    irq_state_t irq_sta = irq_enable();

	end_value = delay_counter + ms;
	while(delay_counter < end_value);

    /** restore previous IRQ status */
    irq_restore(irq_sta);
	
	/*uint32_t temp;  

 SysTick->LOAD = 32000 * ms;  

 SysTick->VAL=0x00;

 SysTick->CTRL=0x01;
 do 
 {  
    temp=SysTick->CTRL;

 }while((temp&0x01)&&(!(temp&(1<<16))));
 
  SysTick->CTRL=0x00;
  SysTick->VAL =0x00;*/
  #else
	
  TimerHwDelayMs( ms );
	#endif
}

#if (HAL_SYS_TICK_PERIOD < 1000)
void delay_us(uint32_t us)
{
	uint32_t end_value;
	end_value = delay_counter + us/100;
	while(delay_counter < end_value);
}
#endif

uint32_t millis( void )
{
    uint32_t tmp;
	
	#ifdef STM32_SENSERNODE_DELAY
    /** disable IRQ, and get current IRQ status */
    irq_state_t irq_sta = irq_disable();

	tmp = TimerHwGetDelayValue();
	//tmp = delay_counter;

    /** restore previous IRQ status */
    irq_restore(irq_sta);
	#else
	/** disable IRQ, and get current IRQ status */
    irq_state_t irq_sta = irq_disable();

	tmp = delay_counter;

    /** restore previous IRQ status */
    irq_restore(irq_sta);
	#endif
	

	return (tmp);
}

