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

#include <stm32f0xx.h>

#define OB_USER_nBOOT0    (1 << 3)
#define OB_USER_BOOT_SEL  (1 << 7)
#define OB_USER_DFU_MASK  (OB_USER_nBOOT0 | OB_USER_BOOT_SEL)

static void set_option_byte_user(uint8_t user)
{
	uint8_t rdp = OB->RDP;

	while (FLASH->SR & FLASH_SR_BSY) {
		__NOP();
	}

	if (FLASH->CR & FLASH_CR_LOCK) {
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	}

	if (!(FLASH->CR & FLASH_CR_OPTWRE)) {
		FLASH->OPTKEYR = FLASH_OPTKEY1;
		FLASH->OPTKEYR = FLASH_OPTKEY2;
	}

	FLASH->CR |= FLASH_CR_OPTER;
	FLASH->CR |= FLASH_CR_STRT;
	while (FLASH->SR & FLASH_SR_BSY) {
		__NOP();
	}
	FLASH->CR &= ~FLASH_CR_OPTER;

	FLASH->CR |= FLASH_CR_OPTPG;
	OB->RDP = rdp;
	while (FLASH->SR & FLASH_SR_BSY) {
		__NOP();
	}
	OB->USER = user;
	while (FLASH->SR & FLASH_SR_BSY) {
		__NOP();
	}
	FLASH->CR &= ~FLASH_CR_OPTPG;

	FLASH->CR &= ~FLASH_CR_OPTWRE;

	FLASH->CR |= FLASH_CR_OBL_LAUNCH;

	while (1) {
		__NOP();
	}
}

void dfu_enter()
{
	set_option_byte_user(OB->USER & ~OB_USER_DFU_MASK);
}

void dfu_reset_if_updated()
{
	if (!(FLASH->OBR & FLASH_OBR_BOOT_SEL) ||
			!(FLASH->OBR & FLASH_OBR_nBOOT0)) {
		set_option_byte_user(OB->USER | OB_USER_DFU_MASK);
	}
}
