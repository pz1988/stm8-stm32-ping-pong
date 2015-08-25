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

Gpio_t Uart_tx;
Gpio_t Uart_rx;
//for debugger
//uart_tx_rx_flag uart_txrx_running;
/*debugger end*/


// TODO: make config parameter valid, now baud rate is fixed 9600bps
void hal_uart_init(uart_config_t *config, uart_tx_call_back_t tx_hanlder,
				   uart_rx_call_back_t rx_handler)
{
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	tx_call_back = tx_hanlder;
	rx_call_back = rx_handler;

	if( tx_call_back == NULL || rx_call_back == NULL ){
		while(1);
	}

	if( config == NULL ){
		while(1);
	}

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );
	//CLK->PCKENR1 |= 0x20;	// SYSCLK to USART1 enable
	USART_DeInit( USART1 );
	
    //UART_TX PA9 output alternate function push-pull
	GpioInit( &Uart_tx, UART_TX, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
	/*// PC3 output
	GPIOA->CR1 |= (1<<3);	// Set PC2 and PC3 as alternate function push-pull (software pull up)
	GPIOA->DDR |= (1<<3);	// Set Tx and Rx as output
	GPIOA->ODR |= (1<<3);*/

    //UART_RX PA10 output alternate function push-pull
    GpioInit( &Uart_tx, UART_RX, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
	/*// PC2 input
	GPIOA->CR1 = (1<<2);	// Set PC2 and PC3 as alternate function push-pull (software pull up)
	GPIOA->DDR &= ~(1<<2);	// Set Tx and Rx as output*/

	GPIO_PinAFConfig( Uart_tx.port, ( Uart_tx.pin & 0x0F ), GPIO_AF_USART1 );
    GPIO_PinAFConfig( Uart_rx.port, ( Uart_rx.pin & 0x0F ), GPIO_AF_USART1 );

	//Need to confirmed 
	/*SYSCFG->RMPCR1 &= ~0x30;
	SYSCFG->RMPCR1 |= 0x00;	// USART1_TX on PC3 and USART1_RX on PC2*/

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );

    USART_InitStructure.USART_BaudRate = config->baud; 
	//USART1->BRR1 = 0x68;		// BaudRate = 9600
	//USART1->BRR2 = 0x03;

	USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
    USART_InitStructure.USART_Mode = 0x000C;
	USART_InitStructure.USART_WordLength = 0x0000;
	USART_InitStructure.USART_StopBits = 0x0000;
	USART_InitStructure.USART_Parity = 0x0000;
	USART_InitStructure.USART_HardwareFlowControl = 0x0000;
	
    USART_Init( USART1, &USART_InitStructure );

	USART_Cmd( USART1, ENABLE );
	
	/*USART1->CR1 = 0x00;		// 1 start bit, 8 data bits
	USART1->CR3 = 0x00;		// 1 stop bit
	USART1->CR2 = 0x6c;		// Transmitter and Receive enable*/
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

//Need to confirmed 
///** uart rx interrupt */
//INTERRUPT_HANDLER( USART1_RX_TIM5_CC_IRQHandler, 28 )
//{
//    uint16_t status;
//    uint16_t data;

//    /** get recieve status,
//        read recieve buffer(this can also clear interrupt) */
//    status = USART1->SR;
//    data = USART1->DR;

//    /** check recieve status 0x1C*/
//    if((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0) {
//		rx_call_back(data);
//    } else {
//        /** hardware fault */
//    }
//}


//Need to confirmed 
///* uart tx interrupt */
//INTERRUPT_HANDLER( USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler, 27 )
//{
//    if(USART1->SR & 0x40) {
//        if( 0 == tx_call_back() ) {
//            /** no packet need to transmit, clear interrupt */
//            USART1->SR &= ~ 0x40;
//        }
//    }
//	return;
//}*/

/** uart rx interrupt */
void USART1_RX_IRQHandler()
{
    uint16_t status;
    uint16_t data;

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

/* uart tx interrupt */
void USART1_TX_IRQHandler()
{
    if(USART1->SR & 0x40) {
        if( 0 == tx_call_back() ) {
            /** no packet need to transmit, clear interrupt */
            USART1->SR &= ~ 0x40;
        }
    }
	return;
}

void USART1_IRQHandler( void )
{
//  if( uart_txrx_running == UART_TX_RUNNING )
//  	{
//  	   if(USART1->SR & 0x40) {
//        if( 0 == tx_call_back() ) {
//            /** no packet need to transmit, clear interrupt */
//            USART1->SR &= ~ 0x40;
//        }
//       }
//	   return; 
//  	}
//  else
//  	{
//  	   uint16_t status;
//       uint16_t data;

//        /** get recieve status,
//            read recieve buffer(this can also clear interrupt) */
//        status = USART1->SR;
//        data = USART1->DR;

//        /** check recieve status 0x1C*/
//        if((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0) {
//		    rx_call_back(data);
//        } else {
//        /** hardware fault */
//        }
//  	}

    uint16_t status;
	  uint8_t data;

    if( USART_GetITStatus( USART1, USART_IT_TXE ) != RESET )
    {    
        if( 0 == tx_call_back() )
        {
            // Disable the USART Transmit interrupt
            USART_ITConfig( USART1, USART_IT_TXE, DISABLE );
        }
    }

    if( USART_GetITStatus( USART1, USART_IT_ORE_RX ) != RESET )
    {
        USART_ReceiveData( USART1 );
    }

    if( USART_GetITStatus( USART1, USART_IT_RXNE ) != RESET )
    {    
        /** get recieve status,
              read recieve buffer(this can also clear interrupt) */
        status = USART1->SR;
        data = USART1->DR;
        /** check recieve status 0x1C*/
        if((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
				{
		        rx_call_back(data);
        } 

		}
}

