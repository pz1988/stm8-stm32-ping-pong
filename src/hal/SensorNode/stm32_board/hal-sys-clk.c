/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: STM32 clock initial

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hardware.h"
#include "hal-sys-clk.h"

void hal_clk_init(void)
{
 	RCC_HCLKConfig( RCC_SYSCLK_Div1 );          //CLK->CKDIVR = CLK_SYSCLKDiv_1;				// System clock source /1
	#if defined( USE_BOOTLOADER )
        // Set the Vector Table base location at 0x3000
        NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x3000 );
#endif
        // We use IRQ priority group 4 for the entire project
        // When setting the IRQ, only the preemption priority is used
        NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

        // Disable Systick
        SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk;    // Systick IRQ off 
        SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;            // Clear SysTick Exception pending flag
}
