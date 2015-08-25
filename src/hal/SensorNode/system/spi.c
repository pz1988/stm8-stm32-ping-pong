/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board SPI driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Ping Zheng
*/ 
#include "hal-spi.h"

SPI_InitTypeDef SPI_InitStructure;

void SpiDeInit( Spi_t *obj )
{
    SPI_Cmd( obj->Spi, DISABLE );
    SPI_I2S_DeInit( obj->Spi );

    GpioInit( &obj->Mosi, obj->Mosi.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Miso, obj->Miso.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );
    GpioInit( &obj->Sclk, obj->Sclk.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Nss, obj->Nss.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

}

void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
    SPI_Cmd( obj->Spi, DISABLE );
    
    if( ( ( ( bits == 8 ) || ( bits == 16 ) ) == false ) ||
        ( ( ( cpol >= 0 ) && ( cpol <= 1 ) ) == false ) ||
        ( ( ( cpha >= 0 ) && ( cpha <= 1 ) ) == false ) )
    {
        // SPI error
        while( 1 );
    }

    SPI_InitStructure.SPI_Mode = ( slave == 0x01 ) ? SPI_Mode_Slave : SPI_Mode_Master;
    SPI_InitStructure.SPI_CPOL = ( cpol == 0x01 ) ? SPI_CPOL_High : SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = ( cpha == 0x01 ) ? SPI_CPHA_2Edge : SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_DataSize = ( bits == 8 ) ? SPI_DataSize_8b : SPI_DataSize_16b;
    SPI_Init( obj->Spi, &SPI_InitStructure );

    SPI_Cmd( obj->Spi, ENABLE );
}

void SpiFrequency( Spi_t *obj, uint32_t hz )
{
    uint32_t divisor;

    SPI_Cmd( obj->Spi, DISABLE );

    divisor = SystemCoreClock / hz;
    
    // Find the nearest power-of-2
    divisor = divisor > 0 ? divisor-1 : 0;
    divisor |= divisor >> 1;
    divisor |= divisor >> 2;
    divisor |= divisor >> 4;
    divisor |= divisor >> 8;
    divisor |= divisor >> 16;
    divisor++;

    divisor = __ffs( divisor ) - 1;

    divisor = ( divisor > 0x07 ) ? 0x07 : divisor;

    SPI_InitStructure.SPI_BaudRatePrescaler = divisor << 3;
    SPI_Init( obj->Spi, &SPI_InitStructure );

    SPI_Cmd( obj->Spi, ENABLE );
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    if( ( obj == NULL ) || ( obj->Spi ) == NULL )
    {
        while( 1 );
    }
    
    while( SPI_I2S_GetFlagStatus( obj->Spi, SPI_I2S_FLAG_TXE ) == RESET );
    SPI_I2S_SendData( obj->Spi, outData );
    while( SPI_I2S_GetFlagStatus( obj->Spi, SPI_I2S_FLAG_RXNE ) == RESET );
    return SPI_I2S_ReceiveData( obj->Spi );
}

