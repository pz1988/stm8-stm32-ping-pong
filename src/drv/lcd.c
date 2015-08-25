/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: LCD ST7920 driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "lcd.h"
#include "hal-lcd.h"
#include "sys-tick.h"

#define LCD_TEST_STR0               "                "
#define LCD_TEST_STR1               LCD_TEST_STR0
#define LCD_TEST_STR2               LCD_TEST_STR0
#define LCD_TEST_STR3               LCD_TEST_STR0

void lcd_init(void)
{
    hal_lcd_init();

    lcd_write_cmd( 0x30 );	// 8-bit interface, basic instructions
	lcd_write_cmd( 0x06 );	// cursor shift right, AC + 1 automatic
	lcd_write_cmd( 0x0c );	// enable display, cursor OFF
	lcd_write_cmd( 0x01 );	// clear display
	delay_ms( 2 );

    lcd_putstring(0, 0, LCD_TEST_STR0);
    lcd_putstring(1, 0, LCD_TEST_STR1);
    lcd_putstring(2, 0, LCD_TEST_STR2);
    lcd_putstring(3, 0, LCD_TEST_STR3);
}

void lcd_write_cmd(uint8_t cmd)
{
    irq_state_t irq_sta = irq_disable();

    HAL_LCD_CS_H();
    hal_lcd_write(0xF8);
    hal_lcd_write(cmd&0xF0);
    hal_lcd_write((cmd<<4) & 0xF0);
    HAL_LCD_CS_L();

    irq_restore(irq_sta);
}

void lcd_write_data(uint8_t dt)
{
    irq_state_t irq_sta = irq_disable();

    HAL_LCD_CS_H();
    hal_lcd_write(0xFA);
    hal_lcd_write(dt&0xF0);
    hal_lcd_write((dt<<4) & 0xF0);
    HAL_LCD_CS_L();

    irq_restore(irq_sta);
}

// The length of the string should be even.
void lcd_putstring( uint8_t row, uint8_t col, uint8_t *str )
{
	if( col >= 8 ){
		col = 0;
		row++;
	}

	if( row >= 4 ){
        row = 0;
    }

	switch( row ){
    case 0:
        lcd_write_cmd( 0x80 + col );
        break;
    case 1:
        lcd_write_cmd( 0x90 + col );
        break;
    case 2:
        lcd_write_cmd( 0x88 + col );
        break;
    case 3:
        lcd_write_cmd( 0x98 + col );
        break;
    default:
        break;
	}

	while( *str != '\0' ){
		lcd_write_data( *str );
		str++;
		lcd_write_data( *str );
		str++;
	}
}

void lcd_putbuf( uint8_t row, uint8_t col, uint8_t *str, uint8_t len )
{
    int i;
	if( col >= 8 ){
		col = 0;
		row++;
	}

	if( row >= 4 ){
        row = 0;
    }

	switch( row ){
    case 0:
        lcd_write_cmd( 0x80 + col );
        break;
    case 1:
        lcd_write_cmd( 0x90 + col );
        break;
    case 2:
        lcd_write_cmd( 0x88 + col );
        break;
    case 3:
        lcd_write_cmd( 0x98 + col );
        break;
    default:
        break;
	}

    for(i=0; i<len; i++){
        lcd_write_data( str[i] );
    }
}
