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

/** SPI SCK -- PB5 */
#define HAL_SPI_SCK_BIT					(1<<5)
/** Output push pull */
#define HAL_SPI_SCK_OUTPUT()			GPIOB->DDR |= HAL_SPI_SCK_BIT; \
										GPIOB->CR2 |= HAL_SPI_SCK_BIT; \
										GPIOB->CR1 |= HAL_SPI_SCK_BIT;
/** SPI MOSI -- PB6 */
#define HAL_SPI_MOSI_BIT				(1<<6)
/** Output push pull */
#define HAL_SPI_MOSI_OUTPUT()			GPIOB->DDR |= HAL_SPI_MOSI_BIT; \
										GPIOB->CR2 |= HAL_SPI_MOSI_BIT; \
										GPIOB->CR1 |= HAL_SPI_MOSI_BIT;
/** SPI MISO -- PB7 */
#define HAL_SPI_MISO_BIT				(1<<7)
/** Input Pull Up */
#define HAL_SPI_MISO_INPUT()			GPIOB->CR2 &= ~HAL_SPI_MISO_BIT; \
										GPIOB->CR1 |= HAL_SPI_MISO_BIT; \
										GPIOB->DDR &= ~HAL_SPI_MISO_BIT;

void hal_spi_init(uint32_t freq, spi_sck_polarity_t sck_pol, spi_sample_edge_t smpl_edge)
{
	uint8_t spi_reg = 0;
	uint8_t spi_presc = 0;
	int8_t i;

	CLK->PCKENR1 |= CLK_PCKENR1_SPI1;	// SYSCLK to SPI1 enable

	HAL_SPI_SCK_OUTPUT();
	HAL_SPI_MOSI_OUTPUT();
	HAL_SPI_MISO_INPUT();

	SYSCFG->RMPCR1 &= (uint8_t)( ~SYSCFG_RMPCR1_SPI1_REMAP );	// Remap SPI1 to PB4~7

	spi_presc = F_CPU/freq,
	spi_presc >>= 1;
	for(i=0; i<8; i++){
		spi_presc >>= 1;
		if(spi_presc == 0){
			break;
		}
	}
	spi_presc = i<<3;

	if( sck_pol == SPI_SCK_POLARITY_LOW ){
		if( smpl_edge == SPI_SAMPLE_ON_RISING_EDGE ){
			spi_reg = (uint8_t)SPI_CPOL_Low | SPI_CPHA_1Edge;
		}else{
			spi_reg = (uint8_t)SPI_CPOL_Low | SPI_CPHA_2Edge;
		}
	}else{
		if( smpl_edge == SPI_SAMPLE_ON_RISING_EDGE ){
			spi_reg = (uint8_t)SPI_CPOL_High | SPI_CPHA_2Edge;
		}else{
			spi_reg = (uint8_t)SPI_CPOL_High | SPI_CPHA_1Edge;
		}
	}

	SPI1->CR1 = ( uint8_t )SPI_FirstBit_MSB | spi_presc | SPI_Mode_Master | spi_reg;
	SPI1->CR2 = ( uint8_t )SPI_Direction_2Lines_FullDuplex |	// 2-line undirection data mode selected
        SPI_NSS_Soft                    |	// Software slave management enabled
            SPI_Direction_Tx;					// NSS value is ignored
	SPI1->CR1 |= (uint8_t)SPI_CR1_SPE;		// SPI1 enabled

}

uint8_t hal_spi_wr( uint8_t byte )
{
	// Loop while DR register is not empty
	while ( ( SPI1->SR & 0x02 ) == 0x00 );

	// Send byte through the SPI peripheral
	SPI1->DR = byte;

	// Wait to receive a byte
	while ( ( SPI1->SR & 0x01 ) == 0x00 );

	// Return the byte read from the SPI bus
	return SPI1->DR;
}
