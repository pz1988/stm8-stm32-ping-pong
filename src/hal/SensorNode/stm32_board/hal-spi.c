/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: spi low level driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#include "hardware.h"
#include "hal-spi.h"
#include "spi.h"

extern Spi_t Spi1;
extern SPI_InitTypeDef SPI_InitStructure;


/** SPI SCK -- PA5 */
#define HAL_SPI_SCK_BIT					//(1<<5)
/** Output push pull */
#define HAL_SPI_SCK_OUTPUT()		    GpioInit( &Spi1.Sclk, RADIO_SCLK, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 )


/** SPI MOSI -- PA7 */
#define HAL_SPI_MOSI_BIT				//S(1<<7)
/** Output push pull */
#define HAL_SPI_MOSI_OUTPUT()			GpioInit( &Spi1.Mosi, RADIO_MOSI, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 )


/** SPI MISO -- PA6 */
#define HAL_SPI_MISO_BIT				//(1<<6)
/** Input Pull Up */  /*PIN_ALTERNATE_FCT is prepared for the GPIO_PinAFConfig() func*/
#define HAL_SPI_MISO_INPUT()			GpioInit( &Spi1.Miso, RADIO_MISO, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 )


void hal_spi_init( uint32_t freq, spi_sck_polarity_t sck_pol, spi_sample_edge_t smpl_edge )
{
	//uint8_t spi_reg = 0;
	//uint8_t spi_presc = 0;
	//int8_t i;
	Spi_t *obj = &Spi1;

	obj->pinnet = NC;
	
	HAL_SPI_SCK_OUTPUT();
	HAL_SPI_MOSI_OUTPUT();
	HAL_SPI_MISO_INPUT();

	GPIO_PinAFConfig( obj->Mosi.port, ( obj->Mosi.pin & 0x0F ), GPIO_AF_SPI1 );
	GPIO_PinAFConfig( obj->Miso.port, ( obj->Miso.pin & 0x0F ), GPIO_AF_SPI1 );
	GPIO_PinAFConfig( obj->Sclk.port, ( obj->Sclk.pin & 0x0F ), GPIO_AF_SPI1 );
	//SYSCFG->RMPCR1 &= (uint8_t)( ~SYSCFG_RMPCR1_SPI1_REMAP );	// Remap SPI1 to PB4~7


    if( obj->pinnet!= NC )
	{
	  GpioInit( &obj->Nss, obj->pinnet, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
		GPIO_PinAFConfig( obj->Nss.port, ( obj->Nss.pin & 0x0F ), GPIO_AF_SPI1 );
	}
	else
	{
	    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	}
	//SYSCFG->RMPCR1 &= (uint8_t)( ~SYSCFG_RMPCR1_SPI1_REMAP );	// Remap SPI1 to PB4~7

	// Choose SPI interface according to the given pins
    obj->Spi = ( SPI_TypeDef* )SPI1_BASE;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );
	//CLK->PCKENR1 |= CLK_PCKENR1_SPI1;	// SYSCLK to SPI1 enable
	
	
    if( obj->pinnet == NC )
    {
        // 8 bits, CPOL = 0, CPHA = 0, MASTER
        SpiFormat( obj, 8, 0, 0, 0 );
    }
	else
	{
		// 8 bits, CPOL = 0, CPHA = 0, SLAVE
        SpiFormat( obj, 8, 0, 0, 1 );
	}
	
	SpiFrequency( obj, 10000000 );

  SPI_Cmd( obj->Spi, ENABLE );

}

uint8_t hal_spi_wr( uint8_t byte )
{
    /*uint16_t outData = byte;

	SPI_TypeDef *obj = ( SPI_TypeDef* ) SPI1_BASE;
	
	if( ( obj == NULL ) || ( obj->Spi ) == NULL )
    {
        while( 1 );
    }
    
    while( SPI_I2S_GetFlagStatus( obj, SPI_I2S_FLAG_TXE ) == RESET );
    SPI_I2S_SendData( obj, outData );
    while( SPI_I2S_GetFlagStatus( obj, SPI_I2S_FLAG_RXNE ) == RESET );
    return (uint_8)SPI_I2S_ReceiveData( obj );*/
	
	// Loop while DR register is not empty
	while ( ( SPI1->SR & 0x02 ) == 0x00 );

	// Send byte through the SPI peripheral
	SPI1->DR = (uint16_t)byte;

	// Wait to receive a byte
	while ( ( SPI1->SR & 0x01 ) == 0x00 );

	// Return the byte read from the SPI bus
	return (uint8_t)SPI1->DR;
	
}
