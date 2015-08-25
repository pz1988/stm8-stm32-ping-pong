/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: high level uart driver/API

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "uart.h"

static uint8_t uart_rx_buf[UART_RX_BUF_LEN]= {0};
static uint8_t uart_tx_buf[UART_TX_BUF_LEN]= {0};

static uint8_t uart_rx_rd_index, uart_rx_cnt, uart_rx_wr_index;
static uint8_t uart_tx_rd_index, uart_tx_cnt, uart_tx_wr_index;

//static uint8_t uart_tick_num;

uint8_t uart_tx_handler(void)
{
	uint8_t ret;

	if( uart_tx_cnt == 0 ){
		ret = 0;
	}else{
		ret = uart_tx_cnt;

		/** TX buffer is not empty, transmit next byte */
		hal_uart_write_tx_reg(uart_tx_buf[uart_tx_rd_index++]);
		--uart_tx_cnt;
		if (uart_tx_rd_index == UART_TX_BUF_LEN) {
			uart_tx_rd_index=0;
		}
    }

	return ret;
}

void uart_rx_handler(uint8_t data)
{
	if(uart_rx_cnt == UART_RX_BUF_LEN){
		/** No more space for receive, discard data */
		return;
	}
	/** data to buffer */
	uart_rx_buf[uart_rx_wr_index++]=data;
	/** TX write pointer point to head */
	if (uart_rx_wr_index == UART_RX_BUF_LEN) {
		uart_rx_wr_index=0;
	}
	/** remain data in TX buffer plus one */
	++uart_rx_cnt;
}

void uart_init(uart_config_t *config)
{
  	uart_rx_rd_index=0, uart_rx_cnt=0, uart_rx_wr_index=0;
    uart_tx_rd_index=0, uart_tx_cnt=0, uart_tx_wr_index=0;
	hal_uart_init(config, uart_tx_handler, uart_rx_handler);
}

void uart_putchar(uint8_t c)
{
	uint8_t flag;
	
	#ifdef STM32_SENSERNODE_DELAY
	irq_state_t irq_sta;
	#endif
	
    /** wait until TX buffer is not full */
    while(uart_tx_cnt ==  UART_TX_BUF_LEN );

	flag = hal_uart_tx_is_empty(); /** 0: tx register is not empty */

    /** disable IRQ, and get current IRQ status */
    irq_sta = irq_disable();

    /** TX buffer is not empty or TX modoule is not busy */
    if(uart_tx_cnt || (flag == 0)) {
        /** data to buffer */
        uart_tx_buf[uart_tx_wr_index++]=c;
        /** TX write pointer point to head */
        if (uart_tx_wr_index == UART_TX_BUF_LEN) {
            uart_tx_wr_index=0;
        }
        /** remain data in TX buffer plus one */
        ++uart_tx_cnt;	
		
    } else {
        /** TX modoule idle, data to USART data register */
        hal_uart_write_tx_reg(c);
    }

	#ifdef STM32_SENSERNODE_DELAY
	// Enable the USART Transmit interrupt
    USART_ITConfig( USART1, USART_IT_TXE, ENABLE );
	#endif

    /** restore previous IRQ status */
    irq_restore(irq_sta);
}

int16_t uart_getchar(void)
{
	int16_t ret;

	if( uart_rx_cnt == 0 ){
		return -1;
	}

	ret = uart_rx_buf[uart_rx_rd_index++];
	--uart_rx_cnt;
	if (uart_rx_rd_index == UART_RX_BUF_LEN) {
		uart_rx_rd_index=0;
	}

	return ret;
}

int uart_readable()
{
	return uart_rx_cnt;
}

void uart_putbuf(uint8_t *buf, uint8_t len)
{
	while(len--){
		uart_putchar(*buf++);
	}
}

void uart_putstring(char *str)
{
	while(*str){
		uart_putchar(*str++);
	}
}

static const char hex_tab[] = "0123456789ABCDEF";
void uart_puthex(uint8_t data)
{
	uint8_t tmp;
	tmp = hex_tab[data>>4];
	uart_putchar(tmp);
	tmp = hex_tab[data&0x0F];
	uart_putchar(tmp);
}

void uart_putbuf_hex(uint8_t *buf, uint8_t len)
{
	uint8_t i;
	for(i=0; i<len; i++){
		uart_puthex(buf[i]);
		uart_putchar(' ');
	}
}
