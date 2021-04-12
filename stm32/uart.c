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

#include "uart.h"
#include <stm32f0xx.h>

#define USART USART2
#define TX 2

void uart_init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	GPIOA->MODER |= (2 << 2 * TX);
	GPIOA->AFR[TX / 8] |= 1 << (4 * (TX % 8));

	USART->BRR = 38400000/115200;
	USART->CR1 |= USART_CR1_TE;
	USART->CR1 |= USART_CR1_UE;
}

void uart_send(char c)
{
	while (!(USART->ISR & USART_ISR_TC)) {
		__NOP();
	}
	USART->TDR = c;
}

void uart_send_str(const char *str)
{
	while (*str) {
		uart_send(*str++);
	}
}

void uart_send_int(uint32_t n)
{
	int sending = 0;
	for (int i = 10; i >= 0; i--) {
		uint32_t x = n;
		for (int j = 0; j < i; j++) {
			x /= 10;
		}

		x %= 10;

		if (x) {
			sending = 1;
		}

		if (sending) {
			uart_send('0' + x);
		}
	}

	if (!sending) {
		uart_send('0');
	}
}
