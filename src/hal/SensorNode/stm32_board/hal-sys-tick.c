/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: system tick HAL driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hardware.h"
#include "hal-sys-tick.h"
#include "timer-board.h"

static hal_sys_tick_handle_t sys_tick_handler = NULL;

void hal_sys_tick_init(hal_sys_tick_handle_t handler)
{
	sys_tick_handler = handler;

	if( sys_tick_handler == NULL){
	  	/** sys_tick_handler never be NULL*/
		while(1);
	}

   TimerHwInit();
}


//Need to confirmed 
//INTERRUPT_HANDLER( TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler, 19 )
//{
//	if(TIM2->SR & 0x01){
//	    TIM2->SR &= ~0x01;
//		sys_tick_handler();
//	}else{
//        /** clear all interrupt flag */
//	}
//    return;
//}


