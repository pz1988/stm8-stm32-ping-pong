/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: STM8 clock initial

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hardware.h"
#include "hal-sys-clk.h"

void hal_clk_init(void)
{
 	CLK->CKDIVR = CLK_SYSCLKDiv_1;				// System clock source /1
	CLK->ICKCR = (uint8_t)CLK_ICKCR_FHWU    |	// Fast wakeup from Halt/Active-halt mode
	                     CLK_ICKCR_FHWU     |	// MVR regulator OFF in Active-halt mode
						 CLK_ICKCR_LSION	|   // Low-speed internal RC oscillator ON
	                     CLK_ICKCR_HSION;       // High-speed internal RC oscillator ON
	CLK->SWR = (uint8_t)CLK_SYSCLKSource_HSI; 	// HSI selected as system clock source
	CLK->SWCR |= (uint8_t)CLK_SWCR_SWEN;		// Enable clock switch execution

	while(CLK->SCSR != CLK_CSSR_CSSEN);			// Enable the clock security system
}
