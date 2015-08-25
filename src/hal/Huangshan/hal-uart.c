/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: UART HAL driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hardware.h"
#include "hal-uart.h"

#define UPE 						0
#define FE 							1
#define DOR 						3

#define FRAMING_ERROR 				(1<<FE)
#define PARITY_ERROR 				(1<<UPE)
#define DATA_OVERRUN 				(1<<DOR)

static uart_tx_call_back_t tx_call_back = NULL;
static uart_rx_call_back_t rx_call_back = NULL;

// TODO: make config parameter valid, now baud rate is fixed 9600bps
void hal_uart_init(uart_config_t *config, uart_tx_call_back_t tx_hanlder,
				   uart_rx_call_back_t rx_handler)
{
	tx_call_back = tx_hanlder;
	rx_call_back = rx_handler;

	if( tx_call_back == NULL || rx_call_back == NULL ){
		while(1);
	}

	if( config == NULL ){
		while(1);
	}

	// PC3 output
	GPIOC->CR1 |= (1<<3);	// Set PC2 and PC3 as alternate function push-pull (software pull up)
	GPIOC->DDR |= (1<<3);	// Set Tx and Rx as output
	GPIOC->ODR |= (1<<3);

    // PC2 input
	GPIOC->CR1 = (1<<2);	// Set PC2 and PC3 as alternate function push-pull (software pull up)
	GPIOC->DDR &= ~(1<<2);	// Set Tx and Rx as output

	SYSCFG->RMPCR1 &= ~0x30;
	SYSCFG->RMPCR1 |= 0x00;	// USART1_TX on PC3 and USART1_RX on PC2

	CLK->PCKENR1 |= 0x20;	// SYSCLK to USART1 enable

	USART1->BRR1 = 0x68;		// BaudRate = 9600
	USART1->BRR2 = 0x03;

	USART1->CR1 = 0x00;		// 1 start bit, 8 data bits
	USART1->CR3 = 0x00;		// 1 stop bit
	USART1->CR2 = 0x6c;		// Transmitter and Receive enable
}

uint8_t hal_uart_tx_is_empty(void)
{
  	if( USART1->SR&0x80 ){
		return 1;
	}

	return 0;
}

void hal_uart_write_tx_reg(uint8_t val)
{
	USART1->DR = val;
}

/** uart rx interrupt */
INTERRUPT_HANDLER( USART1_RX_TIM5_CC_IRQHandler, 28 )
{
    uint8_t status;
    uint8_t data;

    /** get recieve status,
        read recieve buffer(this can also clear interrupt) */
    status = USART1->SR;
    data = USART1->DR;

    /** check recieve status 0x1C*/
    if((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0) {
		rx_call_back(data);
    } else {
        /** hardware fault */
    }
}

/** uart tx interrupt */
INTERRUPT_HANDLER( USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler, 27 )
{
    if(USART1->SR & 0x40) {
        if( 0 == tx_call_back() ) {
            /** no packet need to transmit, clear interrupt */
            USART1->SR &= ~ 0x40;
        }
    }
	return;
}
