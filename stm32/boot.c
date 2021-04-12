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
#include <string.h>

static inline void copy_data()
{
	extern char _sdata, _edata, _sidata;

	memcpy(&_sdata, &_sidata, &_edata - &_sdata);
}

static inline void clear_bss()
{
	extern char _sbss, _ebss;

	memset(&_sbss, 0, &_ebss - &_sbss);
}

void Reset_Handler()
{
	extern int boot();
	extern void __libc_init_array();

	copy_data();
	clear_bss();

	__libc_init_array();

	boot();

	while (1) {
		__asm__ __volatile__("NOP");
	}
}

void Hard_Fault()
{
	uart_send_str("hard fault\n");
}
