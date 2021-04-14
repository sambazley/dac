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

#include "io.h"
#include "dac.h"
#include "spdif.h"
#include "uart.h"
#include "usb.h"
#include <stm32f0xx.h>

/* GPIOA */
#define OUTPUT_EN 6
#define I2S_SEL 8
#define INPUT_SEL 1

/* GPIOF */
#define DIR_ERROR 0

static void input_select_setup()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	GPIOB->MODER &= ~(3 << (2 * I2S_SEL));
	GPIOB->MODER |= 1 << (2 * 8);

	GPIOB->MODER &= ~(3 << (2 * INPUT_SEL));
}

void io_enable()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

	GPIOA->MODER &= ~(3 << (2 * OUTPUT_EN));
	GPIOA->MODER |= 1 << (2 * OUTPUT_EN);

	GPIOF->MODER &= ~(3 << (2 * DIR_ERROR));

	input_select_setup();

	spdif_en(1);
	dac_en(1);

	for (int i = 0; i < 10000000; i++) {
		__NOP();
	}

	GPIOA->ODR |= 1 << OUTPUT_EN;

	while (1) {
		if (GPIOB->IDR & (1 << INPUT_SEL)) {
			GPIOB->ODR &= ~(1 << I2S_SEL);
			usb_audio_en(1);
		} else {
			if (GPIOF->IDR & (1 << DIR_ERROR)) {
				GPIOB->ODR &= ~(1 << I2S_SEL);
				usb_audio_en(0);
			} else {
				GPIOB->ODR |= 1 << I2S_SEL;
			}
		}
	}
}

