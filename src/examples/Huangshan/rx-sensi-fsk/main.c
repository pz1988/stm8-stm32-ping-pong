/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: SX1276 FSK sensitivity test

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hardware.h"
#include "hal-sys-clk.h"
#include "sys-tick.h"
#include "uart.h"
#include "led.h"
#include "sx1276.h"
#include "sx1276-fsk-reg.h"

void sx1276_fsk_rx_test_mode(void)
{
	hal_sx1276_set_irq(SX1276_IRQ_DIO0, false, NULL);
	hal_sx1276_set_irq(SX1276_IRQ_DIO1, false, NULL);
	hal_sx1276_set_irq(SX1276_IRQ_DIO2, false, NULL);
	hal_sx1276_set_irq(SX1276_IRQ_DIO3, false, NULL);
	hal_sx1276_set_irq(SX1276_IRQ_DIO4, false, NULL);

	sx1276_set_mode(RFLR_OPMODE_STANDBY);
	delay_ms(20);

	sx1276_set_mode(RFLR_OPMODE_FSRX);
	delay_ms(20);

	sx1276_write(REG_PACKETCONFIG2,0x00);  // continuous mode

	delay_ms(10);

	sx1276_write(REG_DIOMAPPING1,0x00);
	delay_ms(10);
	sx1276_write(REG_DIOMAPPING2,0x00);
	delay_ms(10);
	sx1276_set_mode(RFLR_OPMODE_RX);
	delay_ms(20);
}

void sx1276_fsk_set_frf( uint32_t frf )
{
    frf = ( uint32_t )( ( double )frf / ( double )SX1276_FREQ_STEP );
	sx1276_write( REG_FRFMSB, ( uint8_t )( ( frf >> 16 ) & 0xFF ) );
	sx1276_write( REG_FRFMID, ( uint8_t )( ( frf >> 8 ) & 0xFF ) );
	sx1276_write( REG_FRFLSB, ( uint8_t )( frf & 0xFF ) );
}

void sx1276_fsk_set_fdev( uint32_t fdev )
{
	if(fdev>100000){
		fdev=100000;
	}

    fdev = ( uint32_t )( ( double )fdev / ( double )SX1276_FREQ_STEP );
	sx1276_write( REG_FDEVMSB, ( uint8_t )( ( fdev >> 8 ) & 0xFF ) );
	sx1276_write( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );
}

void sx1276_fsk_set_bitrate( uint32_t br )
{
	if(br>100000){
		br=100000;
	}
    br = ( uint32_t )( ( double )SX1276_FXOSC / ( double )br );
	sx1276_write( REG_BITRATEMSB, ( uint8_t )( ( br >> 8 ) & 0xFF ) );
	sx1276_write( REG_BITRATELSB, ( uint8_t )( br & 0xFF ) );
}

#define SX1276_FSK_RXBW_TAB_LENGTH				(21)
static const uint32_t sx1276_fsk_rxbw_tab[SX1276_FSK_RXBW_TAB_LENGTH]={
	 2600,  3100,  3900,  5200,  6300,  7800,  10400,  12500,  15600,  20800,
	25000, 31300, 41700, 50000, 62500, 83300, 100000, 125000, 166700, 200000,
	250000
};

void sx1276_set_rxbw(uint32_t fdev, uint32_t br)
{
	uint8_t reg, RxBwMant, RxBwExp, i;
	uint32_t sum;

	sum = fdev + br/2 + 10000;

	for(i=0; i<SX1276_FSK_RXBW_TAB_LENGTH; i++){
		if(sx1276_fsk_rxbw_tab[i] > sum){
			break;
		}
	}

	if(i == SX1276_FSK_RXBW_TAB_LENGTH){
		// invalid fdev and bitrate (Fdev + bitrate/2 > RxBw)
		// fdev and bitrate are out of range, use the largest RXBW
		i = (SX1276_FSK_RXBW_TAB_LENGTH-1);
	}

	RxBwExp = 7 - i/3;
	RxBwMant = 2 - i%3;

	reg = sx1276_read(REG_RXBW);
	reg = (reg&0xE0) | (RxBwMant<<3) | RxBwExp;
	sx1276_write(REG_RXBW, reg);
}

void main()
{
	uint32_t timestamp;

	DISABLE_IRQ();

	hal_clk_init();
	sys_tick_init();

	ENABLE_IRQ();

    sx1276_init(FSK, NULL);

    //Frequency 433.4MHz
    sx1276_fsk_set_frf(433400000);
    //Fdev 25KHz
    sx1276_fsk_set_fdev(25000);
    //Bitrate 10KHz
    sx1276_fsk_set_bitrate(10000);
    //Set RxBw depends on bitrate and fdev
    sx1276_set_rxbw(25000, 10000);

    sx1276_write(0x10,0xFF);

    //Disable AGC, set G1
    sx1276_write(0x0C,0x20);
    sx1276_write(0x0D,0x00);

    sx1276_fsk_rx_test_mode();

    timestamp = millis();

	while(1){
		if( millis() - timestamp > 1000){
			timestamp = millis();
            led_blink(LED0,100);
		}
        led_evt();
	}
}