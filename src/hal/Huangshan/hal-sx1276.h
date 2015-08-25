/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: SX1276 Low-Level driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Jiapeng Li
*/

#ifndef __HAL_SX1276_H
#define __HAL_SX1276_H

#include "hardware.h"
#include "sys-tick.h"

#define SX1276_FXOSC				(26000000ul)
#define SX1276_LF_FREQ_MAX			(525000000ul)

#define SX1276_RSSI_OFFSET_HF       (-157)
#define SX1276_RSSI_OFFSET_LF       (-164)

/** SX1276 SPI NSS Pin -- PB4 */
#define HAL_SX1276_NSS_BIT			(1<<4)
/** Output push pull */
#define HAL_SX1276_NSS_OUTPUT()			GPIOB->DDR |= HAL_SX1276_NSS_BIT; \
										GPIOB->CR2 |= HAL_SX1276_NSS_BIT; \
										GPIOB->CR1 |= HAL_SX1276_NSS_BIT;
/** Input floating */
#define HAL_SX1276_NSS_INPUT()			GPIOB->CR2 &= ~HAL_SX1276_NSS_BIT; \
										GPIOB->CR1 &= ~HAL_SX1276_NSS_BIT; \
										GPIOB->DDR &= ~HAL_SX1276_NSS_BIT;
#define HAL_SX1276_NSS_H()				GPIOB->ODR |= HAL_SX1276_NSS_BIT;
#define HAL_SX1276_NSS_L()				GPIOB->ODR &= ~HAL_SX1276_NSS_BIT;

/** SX1276 SPI Reset Pin -- PA2 */
#define HAL_SX1276_RST_BIT				(1<<2)
/** Output push pull */
#define HAL_SX1276_RST_OUTPUT()			GPIOA->DDR |= HAL_SX1276_RST_BIT; \
										GPIOA->CR2 |= HAL_SX1276_RST_BIT; \
										GPIOA->CR1 |= HAL_SX1276_RST_BIT;
/** Input floating */
#define HAL_SX1276_RST_INPUT()			GPIOA->CR2 &= ~HAL_SX1276_RST_BIT; \
										GPIOA->CR1 &= ~HAL_SX1276_RST_BIT; \
										GPIOA->DDR &= ~HAL_SX1276_RST_BIT;
#define HAL_SX1276_RST_H()				GPIOA->ODR |= HAL_SX1276_RST_BIT;
#define HAL_SX1276_RST_L()				GPIOA->ODR &= ~HAL_SX1276_RST_BIT;

typedef void (*hal_sx1276_irq_callback_t) (void);

typedef enum{
	SX1276_IRQ_DIO0 = 0,
	SX1276_IRQ_DIO1,
	SX1276_IRQ_DIO2,
	SX1276_IRQ_DIO3,
	SX1276_IRQ_DIO4,
	SX1276_IRQ_DIO5,
}sx1276_irq_t;

typedef enum{
	SX1276_RF_SWITCH_IDLE,		//LOW POWER MODE
	SX1276_RF_SWITCH_RX,
	SX1276_RF_SWITCH_TX,
}sx1276_rf_switch_mode_t;

typedef enum{
	/* SX1276 use RFO_HF/RFO_LF as output, and SX1276 has ability to
		switch between RFO_HF/RFO_LF automatically,
		depends on the frequency used. */
	SX1276_TX_PORT_RFO,
	SX1276_TX_PORT_PA_BOOST,
}sx1276_tx_port_t;

void hal_sx1276_init(void);
void hal_sx1276_set_irq(sx1276_irq_t irq, bool sta, hal_sx1276_irq_callback_t cb);
void hal_sx1276_set_rf_switch_mode(sx1276_rf_switch_mode_t rf_switch);
sx1276_tx_port_t hal_sx1276_get_tx_port(uint32_t freq);

#endif




