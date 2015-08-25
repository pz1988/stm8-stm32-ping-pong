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

#include "hardware.h"
#include "hal-sx1276.h"
#include "hal-spi.h"

/** SX1276 Switch Power -- PD7 */
#define HAL_SX1276_SW_PWR_BIT			(1<<7)
/** Output push pull */
#define HAL_SX1276_SW_PWR_OUTPUT()		GPIOD->CR2 &= ~HAL_SX1276_SW_PWR_BIT; \
										GPIOD->DDR |= HAL_SX1276_SW_PWR_BIT; \
										GPIOD->CR1 |= HAL_SX1276_SW_PWR_BIT; \
										GPIOD->CR2 |= HAL_SX1276_SW_PWR_BIT;
/** Input floating */
#define HAL_SX1276_SW_PWR_INPUT()		GPIOD->CR2 &= ~HAL_SX1276_SW_PWR_BIT; \
										GPIOD->DDR &= ~HAL_SX1276_SW_PWR_BIT; \
										GPIOD->CR1 &= ~HAL_SX1276_SW_PWR_BIT;
#define HAL_SX1276_SW_PWR_H()			GPIOD->ODR |= HAL_SX1276_SW_PWR_BIT;
#define HAL_SX1276_SW_PWR_L()			GPIOD->ODR &= ~HAL_SX1276_SW_PWR_BIT;
#define HAL_SX1276_SW_PWR_ON()			HAL_SX1276_SW_PWR_H()
#define HAL_SX1276_SW_PWR_OFF()			HAL_SX1276_SW_PWR_L()

/** SX1276 HF Switch control IO -- PA4 */
#define HAL_SX1276_SW_HF_BIT			(1<<4)
/** Output push pull */
#define HAL_SX1276_SW_HF_OUTPUT()		GPIOA->CR2 &= ~HAL_SX1276_SW_HF_BIT; \
										GPIOA->DDR |= HAL_SX1276_SW_HF_BIT; \
										GPIOA->CR1 |= HAL_SX1276_SW_HF_BIT; \
										GPIOA->CR2 |= HAL_SX1276_SW_HF_BIT;
/** Input floating */
#define HAL_SX1276_SW_HF_INPUT()		GPIOA->CR2 &= ~HAL_SX1276_SW_HF_BIT; \
										GPIOA->DDR &= ~HAL_SX1276_SW_HF_BIT; \
										GPIOA->CR1 &= ~HAL_SX1276_SW_HF_BIT;
#define HAL_SX1276_SW_HF_H()			GPIOA->ODR |= HAL_SX1276_SW_HF_BIT;
#define HAL_SX1276_SW_HF_L()			GPIOA->ODR &= ~HAL_SX1276_SW_HF_BIT;
#define HAL_SX1276_SW_HF_TX()			HAL_SX1276_SW_HF_H()
#define HAL_SX1276_SW_HF_RX()			HAL_SX1276_SW_HF_L()

/** SX1276 LF Switch control IO -- PC6  */
#define HAL_SX1276_SW_LF_BIT			(1<<6)
/** Output push pull */
#define HAL_SX1276_SW_LF_OUTPUT()		GPIOC->CR2 &= ~HAL_SX1276_SW_LF_BIT; \
										GPIOC->DDR |= HAL_SX1276_SW_LF_BIT; \
										GPIOC->CR1 |= HAL_SX1276_SW_LF_BIT; \
										GPIOC->CR2 |= HAL_SX1276_SW_LF_BIT;
/** Input floating */
#define HAL_SX1276_SW_LF_INPUT()		GPIOC->CR2 &= ~HAL_SX1276_SW_LF_BIT; \
										GPIOC->DDR &= ~HAL_SX1276_SW_LF_BIT; \
										GPIOC->CR1 &= ~HAL_SX1276_SW_LF_BIT;
#define HAL_SX1276_SW_LF_H()			GPIOC->ODR |= HAL_SX1276_SW_LF_BIT;
#define HAL_SX1276_SW_LF_L()			GPIOC->ODR &= ~HAL_SX1276_SW_LF_BIT;
#define HAL_SX1276_SW_LF_TX()			HAL_SX1276_SW_LF_L()
#define HAL_SX1276_SW_LF_RX()			HAL_SX1276_SW_LF_H()

/** SX1276 Digital IO DIO0 -- PD4 */
#define HAL_SX1276_DIO0_BIT				(1<<4)
/** Input floating */
#define HAL_SX1276_DIO0_INPUT()			GPIOD->CR2 &= ~HAL_SX1276_DIO0_BIT; \
										GPIOD->CR1 &= ~HAL_SX1276_DIO0_BIT; \
										GPIOD->DDR &= ~HAL_SX1276_DIO0_BIT;
#define HAL_SX1276_DIO0					(GPIOD->IDR & HAL_SX1276_DIO0_BIT)
#define HAL_SX1276_DIO0_INT_ENABLE()	GPIOD->CR2 |= HAL_SX1276_DIO0_BIT;
#define HAL_SX1276_DIO0_INT_DISABLE()	GPIOD->CR2 &= ~HAL_SX1276_DIO0_BIT;


/** SX1276 Digital IO DIO1 -- PD5 */
#define HAL_SX1276_DIO1_BIT				(1<<5)
#define HAL_SX1276_DIO1_INPUT()			GPIOD->CR2 &= ~HAL_SX1276_DIO1_BIT; \
										GPIOD->CR1 &= ~HAL_SX1276_DIO1_BIT; \
										GPIOD->DDR &= ~HAL_SX1276_DIO1_BIT;
#define HAL_SX1276_DIO1					(GPIOD->IDR & HAL_SX1276_DIO1_BIT)
#define HAL_SX1276_DIO1_INT_ENABLE()	GPIOD->CR2 |= HAL_SX1276_DIO1_BIT;
#define HAL_SX1276_DIO1_INT_DISABLE()	GPIOD->CR2 &= ~HAL_SX1276_DIO1_BIT;

/** SX1276 Digital IO DIO2 -- PD6 */
#define HAL_SX1276_DIO2_BIT				(1<<6)
#define HAL_SX1276_DIO2_INPUT()			GPIOD->CR2 &= ~HAL_SX1276_DIO2_BIT; \
										GPIOD->CR1 &= ~HAL_SX1276_DIO2_BIT; \
										GPIOD->DDR &= ~HAL_SX1276_DIO2_BIT;
#define HAL_SX1276_DIO2					(GPIOD->IDR & HAL_SX1276_DIO2_BIT)
#define HAL_SX1276_DIO2_INT_ENABLE()	GPIOD->CR2 |= HAL_SX1276_DIO2_BIT;
#define HAL_SX1276_DIO2_INT_DISABLE()	GPIOD->CR2 &= ~HAL_SX1276_DIO2_BIT;

/** PD7 is used for RF Switch Power control */
#if 0
/** SX1276 Digital IO DIO3 -- PD7 */
#define HAL_SX1276_DIO3_BIT
#define HAL_SX1276_DIO3_INPUT()
#define HAL_SX1276_DIO3
#define HAL_SX1276_DIO3_INT_ENABLE()
#define HAL_SX1276_DIO3_INT_DISABLE()
#else
#define HAL_SX1276_DIO3_BIT				(1<<7)
#define HAL_SX1276_DIO3_INPUT()			GPIOD->CR2 &= ~HAL_SX1276_DIO3_BIT; \
										GPIOD->CR1 &= ~HAL_SX1276_DIO3_BIT; \
										GPIOD->DDR &= ~HAL_SX1276_DIO3_BIT;
#define HAL_SX1276_DIO3					(GPIOD->IDR & HAL_SX1276_DIO3_BIT)
#define HAL_SX1276_DIO3_INT_ENABLE()	GPIOD->CR2 |= HAL_SX1276_DIO3_BIT;
#define HAL_SX1276_DIO3_INT_DISABLE()	GPIOD->CR2 &= ~HAL_SX1276_DIO3_BIT;
#endif

/** SX1276 Digital IO DIO4 -- PC4 (STM8L152 PC port doesn't support EXTI function) */
#define HAL_SX1276_DIO4_BIT				(1<<4)
#define HAL_SX1276_DIO4_INPUT()			GPIOC->CR2 &= ~HAL_SX1276_DIO4_BIT; \
										GPIOC->CR1 &= ~HAL_SX1276_DIO4_BIT; \
										GPIOC->DDR &= ~HAL_SX1276_DIO4_BIT;
#define HAL_SX1276_DIO4					(GPIOC->IDR & HAL_SX1276_DIO4_BIT)
#define HAL_SX1276_DIO4_INT_ENABLE()
#define HAL_SX1276_DIO4_INT_DISABLE()

#if 1
#define HAL_SX1276_DIO5_INPUT()
#define HAL_SX1276_DIO5_INT_ENABLE()
#define HAL_SX1276_DIO5_INT_DISABLE()
#else
/** SX1276 Digital IO DIO5 -- PC5 (STM8L152 PC port doesn't support EXTI function)*/
#define HAL_SX1276_DIO5_BIT				(1<<5)
#define HAL_SX1276_DIO5_INPUT()			GPIOC->CR2 &= ~HAL_SX1276_DIO5_BIT; \
										GPIOC->CR1 &= ~HAL_SX1276_DIO5_BIT; \
										GPIOC->DDR &= ~HAL_SX1276_DIO5_BIT;
#define HAL_SX1276_DIO5					(GPIOC->IDR & HAL_SX1276_DIO5_BIT)
#define HAL_SX1276_DIO5_INT_ENABLE()
#define HAL_SX1276_DIO5_INT_DISABLE()
#endif

hal_sx1276_irq_callback_t hal_sx1276_irq_callback[6] = { 0 };

void hal_sx1276_init(void)
{
	HAL_SX1276_SW_LF_INPUT();
	HAL_SX1276_SW_HF_INPUT();
	HAL_SX1276_SW_PWR_INPUT();

	HAL_SX1276_DIO0_INPUT();
	HAL_SX1276_DIO1_INPUT();
	HAL_SX1276_DIO2_INPUT();
	HAL_SX1276_DIO3_INPUT();
	HAL_SX1276_DIO4_INPUT();
	HAL_SX1276_DIO5_INPUT();

    DISABLE_IRQ();
	EXTI->CR2 = 0x55;		// Px4~7 rising edge interrupt

    ENABLE_IRQ();

	/** Clear interrupt flag */
	EXTI->SR1 |= (HAL_SX1276_DIO0_BIT | HAL_SX1276_DIO1_BIT | \
					HAL_SX1276_DIO2_BIT | HAL_SX1276_DIO3_BIT);

	HAL_SX1276_NSS_OUTPUT();
	HAL_SX1276_NSS_H();

	HAL_SX1276_RST_INPUT();

    /** SPI 10MHz, SCK Low when IDlE, sample on rising edge */
	hal_spi_init(10000000, SPI_SCK_POLARITY_LOW, SPI_SAMPLE_ON_RISING_EDGE);

    /** SPI 4MHz, SCK Low when IDlE, sample on rising edge */
	//hal_spi_init(4000000, SPI_SCK_POLARITY_LOW, SPI_SAMPLE_ON_RISING_EDGE);
}

void hal_sx1276_set_irq(sx1276_irq_t irq, bool sta, hal_sx1276_irq_callback_t cb)
{
	hal_sx1276_irq_callback[irq] = cb;
	switch(irq){
	case SX1276_IRQ_DIO0:
		if(sta){
			HAL_SX1276_DIO0_INT_ENABLE();
		}else{
			HAL_SX1276_DIO0_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO1:
		if(sta){
			HAL_SX1276_DIO1_INT_ENABLE();
		}else{
			HAL_SX1276_DIO1_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO2:
		if(sta){
			HAL_SX1276_DIO2_INT_ENABLE();
		}else{
			HAL_SX1276_DIO2_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO3:
		if(sta){
			HAL_SX1276_DIO3_INT_ENABLE();
		}else{
			HAL_SX1276_DIO3_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO4:
		if(sta){
			HAL_SX1276_DIO4_INT_ENABLE();
		}else{
			HAL_SX1276_DIO4_INT_DISABLE();
		}
		break;
	case SX1276_IRQ_DIO5:
		if(sta){
			HAL_SX1276_DIO5_INT_ENABLE();
		}else{
			HAL_SX1276_DIO5_INT_DISABLE();
		}
		break;
	}
}

void hal_sx1276_set_rf_switch_mode(sx1276_rf_switch_mode_t rf_switch)
{
	static uint8_t init_flag = 0;	// 0: IO uninitialized, 1: initialed.

	switch(rf_switch){
	case SX1276_RF_SWITCH_RX:
		if( init_flag == 0){
			init_flag = 1;
			/** RF Switch IO initial */
			HAL_SX1276_SW_HF_OUTPUT();
			HAL_SX1276_SW_LF_OUTPUT();
			HAL_SX1276_SW_PWR_OUTPUT();
			HAL_SX1276_SW_PWR_ON();
		}
		HAL_SX1276_SW_HF_RX();
		HAL_SX1276_SW_LF_RX();
		break;
	case SX1276_RF_SWITCH_TX:
		if( init_flag == 0){
			init_flag = 1;
			/** RF Switch IO initial */
			HAL_SX1276_SW_HF_OUTPUT();
			HAL_SX1276_SW_LF_OUTPUT();
			HAL_SX1276_SW_PWR_OUTPUT();
			HAL_SX1276_SW_PWR_ON();
		}
		HAL_SX1276_SW_HF_TX();
		HAL_SX1276_SW_LF_TX();
		break;
	case SX1276_RF_SWITCH_IDLE:
		init_flag = 0;
		/** Disable RF Switch IO to save power */
		HAL_SX1276_SW_HF_INPUT();
		HAL_SX1276_SW_LF_INPUT();
		HAL_SX1276_SW_PWR_INPUT();
		break;
	default:
		break;
	}
}

sx1276_tx_port_t hal_sx1276_get_tx_port(uint32_t freq)
{
	if( freq > SX1276_LF_FREQ_MAX ){
		/** RFO is used as HF OUTPUT*/
		return SX1276_TX_PORT_RFO;
	}

	/** PA_BOOST is used as LF OUTPUT */
	return SX1276_TX_PORT_PA_BOOST;
}

/** Interrupt routine */
INTERRUPT_HANDLER( DIO0, 12 )
{
	if( EXTI->SR1 | HAL_SX1276_DIO0_BIT ){
        /** Clear IRQ Flag */
		EXTI->SR1 |= HAL_SX1276_DIO0_BIT;

		if( hal_sx1276_irq_callback[0] != NULL ){
			hal_sx1276_irq_callback[0]();
		}
	}
}

INTERRUPT_HANDLER( DIO1, 13 )
{
	if( EXTI->SR1 | HAL_SX1276_DIO1_BIT ){
        /** Clear IRQ Flag */
		EXTI->SR1 |= HAL_SX1276_DIO1_BIT;

		if( hal_sx1276_irq_callback[1] != NULL ){
			hal_sx1276_irq_callback[1]();
		}
	}
}

INTERRUPT_HANDLER( DIO2, 14 )
{
	if( EXTI->SR1 | HAL_SX1276_DIO2_BIT ){
        /** Clear IRQ Flag */
		EXTI->SR1 |= HAL_SX1276_DIO2_BIT;

		if( hal_sx1276_irq_callback[2] != NULL ){
			hal_sx1276_irq_callback[2]();
		}
	}
}

INTERRUPT_HANDLER( DIO3, 15 )
{
	if( EXTI->SR1 | HAL_SX1276_DIO3_BIT ){
        /** Clear IRQ Flag */
		EXTI->SR1 |= HAL_SX1276_DIO3_BIT;

        if( hal_sx1276_irq_callback[3] != NULL ){
			hal_sx1276_irq_callback[3]();
		}
	}
}
