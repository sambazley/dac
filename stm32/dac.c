/* Copyright (C) 2021 Sam Bazley
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "dac.h"
#include "spdif.h"
#include "uart.h"
#include "usb.h"
#include <stm32f0xx.h>

//SPI1 TX = DMA_Channel3

/* GPIOA */
#define RST 1

#define LRCK 4
#define SCLK 5
#define DOUT 7


static void setup_pin(int pin)
{
	GPIOA->MODER &= ~(3 << (2 * pin));
	GPIOA->MODER |= 2 << (2 * pin);
	GPIOA->AFR[pin / 8] &= ~(0xF << ((pin % 8) * 4));
	GPIOA->AFR[pin / 8] |= 0 << ((pin % 8) * 4);
}

static void i2s_init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI1->I2SCFGR = 0;
	SPI1->CR2 = 0;

	SPI1->I2SCFGR |= SPI_I2SCFGR_I2SMOD;
	SPI1->I2SCFGR &= ~SPI_I2SCFGR_I2SSTD_Msk;
	SPI1->I2SCFGR |= SPI_I2SCFGR_I2SCFG_1; // master, transmit
	SPI1->I2SPR |= SPI_I2SPR_ODD;
	SPI1->I2SPR |= 12;
	SPI1->I2SPR &= ~2;

	SPI1->CR2 |= SPI_CR2_TXDMAEN;

	setup_pin(LRCK);
	setup_pin(SCLK);
	setup_pin(DOUT);

	SPI1->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

static void dma_init()
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	DMA1_Channel3->CPAR = (uint32_t) &(SPI1->DR);
	DMA1_Channel3->CMAR = (uint32_t) &usb_audio_data;
	DMA1_Channel3->CNDTR = USB_AUDIO_SAMPLE_COUNT;

	DMA1_Channel3->CCR |= DMA_CCR_DIR; // read from memory
	DMA1_Channel3->CCR |= 1 << DMA_CCR_PSIZE_Pos; // 16 bit peripheral
	DMA1_Channel3->CCR |= 1 << DMA_CCR_MSIZE_Pos; // 16 bit memory
	DMA1_Channel3->CCR |= DMA_CCR_CIRC;
	DMA1_Channel3->CCR |= DMA_CCR_MINC;
	DMA1_Channel3->CCR |= DMA_CCR_TCIE;

	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void dac_en(int en)
{
	if (en) {
		GPIOA->ODR |= 1 << RST;

		i2s_init();
		dma_init();
	} else {
		GPIOA->ODR &= ~(1 << RST);
	}
}

void dac_init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~(3 << (2 * RST));
	GPIOA->MODER |= (1 << (2 * RST));
	dac_en(0);
}

void dac_irq()
{
	if (DMA1->ISR & DMA_ISR_TCIF3) {
		usb_audio_complete_sync();
	}

	DMA1->IFCR |= (0xf << 8);
}
