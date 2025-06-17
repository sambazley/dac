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

#include "usb.h"
#include "uart.h"
#include <usblib.h>
#include <stm32f0xx.h>

static struct usb_endpoint endpoints [] = {
	{64, 64, USB_EP_CONTROL, DIR_BIDIR},
	{256, 256, USB_EP_ISOCHRONOUS, DIR_OUT},
};

static const uint8_t device_descriptor [] = {
	18,         // bLength (constant)
	1,          // bDescriptorType (constant)
	0x10, 0x01, // bcdUSB - USB 1.1
	0,          // bDeviceClass
	0,          // bDeviceSubClass
	0,          // bDeviceProtocol
	64,         // bMaxPacketSize
	0x76, 0x98, // idVendor
	0x78, 0x56, // idProduct
	0x01, 0x01, // bcdDevice - device version number
	1,          // iManufacturer
	2,          // iProduct
	3,          // iSerialNumber
	1,          // bNumConfigurations
};

#define MSB(x) ((x) >> 8)
#define LSB(x) ((x) & 0xFF)

#define AUDIO_CONT_SIZE (9 + 12 + 9)
#define CFG_CONT_SIZE (9 + 9 + AUDIO_CONT_SIZE + 9 + 9 + 7 + 11 + 7 + 7)

static const uint8_t config1_descriptor [] = {
	9,                                          // bLength (constant)
	2,                                          // bDescriptorType (configuration)
	LSB(CFG_CONT_SIZE), MSB(CFG_CONT_SIZE),     // wTotalLength
	2,                                          // bNumInterfaces
	1,                                          // bConfigurationValue
	0,                                          // iConfiguration
	0x80,                                       // bmAttributes
	75,                                         // bMaxPower

	//interface 0
	9,                                          // bLength (constant)
	4,                                          // bDescriptorType (interface)
	0,                                          // bInterfaceNumber
	0,                                          // bAlternateSetting
	0,                                          // bNumEndpoints
	0x01,                                       // bInterfaceClass (audio)
	0x01,                                       // bInterfaceSubClass (audio control)
	0,                                          // bInterfaceProtocol
	0,                                          // iInterface

		//audio interface header
		9,                                          // bLength
		0x24,                                       // bDescriptorType (CS_INTERFACE)
		1,                                          // bDescriptorSubtype (HEADER)
		0x00, 0x01,                                 // bcdADC (1.0)
		LSB(AUDIO_CONT_SIZE), MSB(AUDIO_CONT_SIZE), // wTotalLen
		1,                                          // bInCollection
		1,                                          // baInterfaceNr

		//input terminal
		12,                                         // bLength
		0x24,                                       // bDescriptorType (CS_INTERFACE)
		2,                                          // bDescriptorSubype (input terminal)
		1,                                          // bTerminalID
		0x01, 0x01,                                 // wTerminalType (USB streaming)
		0,                                          // bAssocTerminal (none)
		2,                                          // bNrChannels
		0x03, 0x00,                                 // wChannelConfig
		0,                                          // iChannelNames
		0,                                          // iTerminal

		//output terminal
		9,                                          // bLength
		0x24,                                       // bDescriptorType (CS_INTERFACE)
		3,                                          // bDescriptorSubtype (output terminal)
		2,                                          // bTerminalID
		0x02, 0x03,                                 // wTerminalType (Headphones)
		0,                                          // bAssocTerminal
		1,                                          // bSourceID
		0,                                          // iTerminal

	//interface 1
	9,                                          // bLength (constant)
	4,                                          // bDescriptorType (interface)
	1,                                          // bInterfaceNumber
	0,                                          // bAlternateSetting
	0,                                          // bNumEndpoints
	0x01,                                       // bInterfaceClass (audio)
	0x02,                                       // bInterfaceSubClass (audio streaming)
	0,                                          // bInterfaceProtocol
	0,                                          // iInterface

	//alternate audio interface
	9,                                          // bLength (constant)
	4,                                          // bDescriptorType (interface)
	1,                                          // bInterfaceNumber
	1,                                          // bAlternateSetting
	1,                                          // bNumEndpoints
	0x01,                                       // bInterfaceClass (audio)
	0x02,                                       // bInterfaceSubClass (audio streaming)
	0,                                          // bInterfaceProtocol
	0,                                          // iInterface

		//audio stream descriptor
		7,                                          // bLength
		0x24,                                       // bDescriptorType (CS_INTERFACE)
		0x01,                                       // bDescriptorSubType (AS_GENERAL)
		1,                                          // bTerminalLink
		USB_AUDIO_FRAME_COUNT / 2,                  // bDelay
		0x01, 0x00,                                 // wFormatTag (PCM)

		//format type descriptor
		11,                                         // bLength
		0x24,                                       // bDescriptorType (CS_INTERFACE)
		0x02,                                       // bDescriptorSubtype (FORMAT_TYPE)
		1,                                          // bFormatType
		2,                                          // bNrChannels
		2,                                          // bSubFrameSize
		16,                                         // bBitResolution
		1,                                          // bSamFreqType
		0x80, 0xBB, 0x00,                           // 48 kHz

		//isochronous endpoint descriptor
		7,                                          // bLength
		5,                                          // bDescriptorType (endpoint)
		0x01,                                       // bEndpointAddress (EP1 out)
		0x09,                                       // bmAttributes isochronous, adaptive
		0x00, 0x01,                                 // wMaxPacketSize (256 bytes)
		1,                                          // bInterval (1ms)

		7,                                          // bLength
		0x25,                                       // bDescriptorType (CS_ENDPOINT)
		0x01,                                       // bDescriptorSubtype (EP_GENERAL)
		0,                                          // bmAttributes
		0,                                          // bLockDelayUnits (none)
		0, 0,                                       // wLockDelay (0 ms)
};

static const uint8_t lang_str [] = {
	6, 3,
	0x09, 0x04, 0x00, 0x00
};

static const uint8_t vendor_str [] = {
	22, 3,
	'S', 0,
	'a', 0,
	'm', 0,
	' ', 0,
	'B', 0,
	'a', 0,
	'z', 0,
	'l', 0,
	'e', 0,
	'y', 0
};

static const uint8_t product_str [] = {
	28, 3,
	'H', 0,
	'e', 0,
	'a', 0,
	'd', 0,
	'p', 0,
	'h', 0,
	'o', 0,
	'n', 0,
	'e', 0,
	' ', 0,
	'D', 0,
	'A', 0,
	'C', 0
};

static const uint8_t serial_no_str [] = {
	4, 3, '1', 0
};

enum DescriptorType {
	DESC_DEVICE = 1,
	DESC_CONFIG,
	DESC_STRING
};

#define DESCRIPTOR(type, index, wIndex, addr) \
{(type << 8) | index, wIndex, addr, sizeof(addr)}

static struct usb_descriptor descriptors [] = {
	DESCRIPTOR(DESC_DEVICE, 0, 0x0000, device_descriptor),
	DESCRIPTOR(DESC_CONFIG, 0, 0x0000, config1_descriptor),
	DESCRIPTOR(DESC_STRING, 0, 0x0000, lang_str),
	DESCRIPTOR(DESC_STRING, 1, 0x0409, vendor_str),
	DESCRIPTOR(DESC_STRING, 2, 0x0409, product_str),
	DESCRIPTOR(DESC_STRING, 3, 0x0409, serial_no_str),
};

static void on_control_out_interface0(struct usb_interface *iface,
		volatile struct usb_setup_packet *sp)
{
	(void) iface;

	switch (sp->bRequest) {
	default:
		uart_send_str("== UNHANDLED INTERFACE 0 REQUEST ");
		uart_send_int(sp->bRequest);
		uart_send_str(" ==\n");
		usb_ack(0);
	}
}

enum {
	REQ_SET_INTERFACE = 11
};

static void on_control_out_interface1(struct usb_interface *iface,
		volatile struct usb_setup_packet *sp)
{
	volatile uint16_t *epr = USB_EP(1);

	switch (sp->bRequest) {
	case REQ_SET_INTERFACE:
		if (iface->alternate) {
			if (*epr & USB_EP_DTOG_TX) {
				*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_TX;
			}

			if (*epr & USB_EP_DTOG_RX) {
				*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_RX;
			}

			usb_ep_set_rx_status(1, USB_EP_RX_VALID);
			usb_ep_set_tx_status(1, USB_EP_TX_VALID);
		} else {
			usb_ep_set_rx_status(1, USB_EP_RX_DIS);
			usb_ep_set_tx_status(1, USB_EP_TX_DIS);
		}

		usb_ack(0);

		break;
	default:
		uart_send_str("== UNHANDLED INTERFACE 1 REQUEST ");
		uart_send_int(sp->bRequest);
		uart_send_str(" ==\n");
		usb_ack(0);
	}
}

static struct usb_interface interfaces [] = {
	{0, 0, on_control_out_interface0},
	{1, 0, on_control_out_interface1},
};

static volatile int audio_en;

void usb_audio_en(int en)
{
	audio_en = en;
}

static volatile uint16_t usb_audio_ptr = USB_AUDIO_SAMPLE_COUNT / 2 + 1;
static volatile uint16_t stopped_count;

static volatile uint8_t i2s_count = 0;
static volatile uint8_t usb_count = 0;
static volatile float integral = 0.0f;
static volatile float last_error = 0.0f;
static volatile int8_t init_output = -1;

void usb_audio_complete_sync()
{
	int16_t measure;
	int16_t error;
	int8_t output;

	const float kp = 0.001f;
	const float ki = 0.0001f;
	const float kd = 0.0f;

	float p, i, d;

	if (stopped_count == 0) {
		usb_count = 0;
		i2s_count = 0;
		usb_audio_ptr = USB_AUDIO_SAMPLE_COUNT / 2 + 1;
		integral = 0.0f;
		last_error = 0.0f;
		return;
	}

	i2s_count++;
	stopped_count--;

	if (init_output == -1) {
		init_output = (CRS->CR >> CRS_CR_TRIM_Pos) & 0x3F;
	}

	measure = (int16_t) (usb_count * USB_AUDIO_SAMPLE_COUNT + usb_audio_ptr) -
		(int16_t) (i2s_count * USB_AUDIO_SAMPLE_COUNT);

	error = measure - USB_AUDIO_SAMPLE_COUNT / 2 - 1;

	integral += error;

	p = kp * error;
	i = ki * integral;
	d = kd * (last_error - error);

	last_error = error;

	output = p + i + d + init_output;

	if (output < 0) {
		output = 0;
	} else if (output > 0x3f) {
		output = 0x3f;
	}

	CRS->CR &= ~CRS_CR_AUTOTRIMEN;
	CRS->CR = (CRS->CR & (~CRS_CR_TRIM)) | (output << CRS_CR_TRIM_Pos);

	if (i2s_count < usb_count) {
		usb_count -= i2s_count;
		i2s_count -= i2s_count;
	} else {
		i2s_count -= usb_count;
		usb_count -= usb_count;
	}
}

volatile uint16_t usb_audio_data [USB_AUDIO_SAMPLE_COUNT] = {0};

static void on_audio_data_in()
{
	int16_t *din;
	uint16_t samples = usb_ep_get_rx_count(1) / 2;
	int r = 0, l = 0;

	if (!audio_en) {
		for (int i = 0; i < USB_AUDIO_SAMPLE_COUNT; i++) {
			usb_audio_data[i] = 0;
		}

		return;
	}

	if (USB->EP1R & USB_EP_DTOG_RX) {
		din = (int16_t *) usb_pma_addr(1, PMA_RX);
	} else {
		din = (int16_t *) usb_pma_addr(1, PMA_TX);
	}

	for (int i = 0; i < samples; i++) {
		usb_audio_data[(i + usb_audio_ptr) % USB_AUDIO_SAMPLE_COUNT] = din[i];
	}

	usb_audio_ptr += samples;

	if (usb_audio_ptr >= USB_AUDIO_SAMPLE_COUNT) {
		usb_audio_ptr -= USB_AUDIO_SAMPLE_COUNT;
		usb_count++;
	}

	for (int i = 0; i < samples; i += 2) {
		if (usb_audio_data[i]) {
			r = 1;
			break;
		}
	}

	for (int i = 1; i < samples; i += 2) {
		if (usb_audio_data[i]) {
			l = 1;
			break;
		}
	}

	if (r && !l) {
		usb_audio_data[1] = 1;
	} else if (l && !r) {
		usb_audio_data[0] = 1;
	}

	if (r || l) {
		stopped_count = 10;
	}
}

static void on_correct_transfer(uint8_t ep)
{
	if (ep == 0x81) {
		on_audio_data_in();
	}
}

static struct usb_configuration conf = {
	.endpoints = endpoints,
	.endpoint_count = sizeof(endpoints) / sizeof(struct usb_endpoint),

	.descriptors = descriptors,
	.descriptor_count = sizeof(descriptors) / sizeof(struct usb_descriptor),

	.interfaces = interfaces,
	.interface_count = sizeof(interfaces) / sizeof(struct usb_interface),

	.on_correct_transfer = on_correct_transfer,
	.log_str = uart_send_str,
	.log_int = uart_send_int,
};

void usb_impl_init()
{
	usb_init(&conf);
}
