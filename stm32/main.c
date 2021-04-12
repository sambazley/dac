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
#include "io.h"
#include "spdif.h"
#include "uart.h"
#include "usb.h"
#include <stm32f0xx.h>

static void rcc_init()
{
	FLASH->ACR |= FLASH_ACR_LATENCY;

	RCC->CR2 |= RCC_CR2_HSI48ON;

	while (!(RCC->CR2 & RCC_CR2_HSI48RDY)) {
		__NOP();
	}

	/*
	 * Set system clock to 38.4 MHz
	 * 48 / 15 * 12 = 38.4
	 */

	RCC->CFGR2 |= (15 - 1) << RCC_CFGR2_PREDIV_Pos; // PREDIV = 15
	RCC->CFGR |= (12 - 2) << RCC_CFGR_PLLMUL_Pos; // PLLMUL = 12

	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI48_PREDIV;

	RCC->CR |= RCC_CR_PLLON;

	while (!(RCC->CR & RCC_CR_PLLRDY)) {
		__NOP();
	}

	RCC->CFGR |= RCC_CFGR_SW_PLL;

	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {
		__NOP();
	}
}

void boot()
{
	rcc_init();
	uart_init();

	uart_send_str("[2J");
	uart_send_str("[H");

	spdif_init();
	dac_init();

	usb_init();

	int i = 3000000;
	while (1) {
		if (!usb_received_request && i-- == 0) {
			break;
		}

		if (usb_selected_config != 0) {
			break;
		}
	}

	io_enable();
}
