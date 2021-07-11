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

#ifndef USB_IMPL_H
#define USB_IMPL_H

#include <stdint.h>

#define USB_AUDIO_SAMPLE_COUNT 960

extern volatile uint16_t usb_audio_data [USB_AUDIO_SAMPLE_COUNT];

void usb_audio_en(int en);

void usb_audio_half_sync();
void usb_audio_complete_sync();

void usb_impl_init();

#endif /* USB_IMPL_H */
