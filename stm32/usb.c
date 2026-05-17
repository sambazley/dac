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
#include "dfu.h"
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
#define IFACE0_SIZE (9 + AUDIO_CONT_SIZE)
#define IFACE1_SIZE (9 + 9 + 7 + 11 + 7 + 7)
#define IFACE2_SIZE (9 + 9)

#define CFG_CONT_SIZE (9 + IFACE0_SIZE + IFACE1_SIZE + IFACE2_SIZE)

static const uint8_t config1_descriptor [] = {
	9,                                          // bLength (constant)
	2,                                          // bDescriptorType (configuration)
	LSB(CFG_CONT_SIZE), MSB(CFG_CONT_SIZE),     // wTotalLength
	3,                                          // bNumInterfaces
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

	//interface 2 (DFU)
	9,                                          // bLength
	4,                                          // bDescriptorType (interface)
	2,                                          // bInterfaceNumber
	0,                                          // bAlternateSetting
	0,                                          // bNumEndpoints
	0xFE,                                       // bInterfaceClass (application specific)
	0x01,                                       // bInterfaceSubClass (DFU)
	0x01,                                       // bInterfaceProtocol (runtime)
	0,                                          // iInterface

		9,                                          // bLength
		0x21,                                       // bDescriptorType (DFU)
		0x0B,                                       // bmAttributes (can download, can upload, will detach)
		0xFA, 0x00,                                 // wDetachTimeOut (250 ms)
		0x40, 0x00,                                 // wTransferSize (64)
		0x10, 0x01,                                 // bcdDFUVersion (1.10)
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

enum {
	REQ_DFU_DETACH    = 0x00,
	REQ_DFU_GETSTATUS = 0x03,
	REQ_SET_INTERFACE = 0x0B
};

static void on_control_out_interface0(struct usb_interface *iface,
		struct usb_setup_packet *sp)
{
	(void) iface;

	uart_send_str("== UNHANDLED INTERFACE 0 REQUEST ");
	uart_send_int(sp->bRequest);
	uart_send_str(" ==\n");
	usb_ack(0);
}

static void on_control_out_interface2(struct usb_interface *iface,
		struct usb_setup_packet *sp)
{
	(void) iface;
	static const uint8_t status[6] = {0, 0, 0, 0, 0, 0};

	switch (sp->bRequest) {
	case REQ_DFU_DETACH:
		usb_ack(0);
		dfu_enter();
		break;
	case REQ_DFU_GETSTATUS:
		usb_send_data(0, status, sizeof(status), sp->wLength);
		break;
	default:
		uart_send_str("== UNHANDLED INTERFACE 2 REQUEST ");
		uart_send_int(sp->bRequest);
		uart_send_str(" ==\n");
		usb_ack(0);
	}
}

static void on_control_out_interface1(struct usb_interface *iface,
		struct usb_setup_packet *sp)
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
	{2, 0, on_control_out_interface2},
};

static volatile int audio_en;

void usb_audio_en(int en)
{
	audio_en = en;
}

#define BUFFER_TARGET (USB_AUDIO_SAMPLE_COUNT / 2)

static volatile uint16_t usb_audio_ptr = BUFFER_TARGET + 1;
static volatile uint16_t stopped_count;
static volatile float integral = 0.0f;
static volatile int8_t init_output = -1;

static uint16_t dma_position()
{
	uint16_t pos = USB_AUDIO_SAMPLE_COUNT - DMA1_Channel3->CNDTR;
	if (pos >= USB_AUDIO_SAMPLE_COUNT) {
		pos = 0;
	}
	return pos;
}

static void usb_audio_sync()
{
	const float kp = 0.005f;
	const float ki = 0.001f;
	const float integral_limit = 50000.0f;
	float p, i;

	int32_t measure, error, output;

	if (init_output == -1) {
		init_output = (CRS->CR >> CRS_CR_TRIM_Pos) & 0x3F;
		CRS->CR &= ~CRS_CR_AUTOTRIMEN;
	}

	measure = (int32_t) usb_audio_ptr - (int32_t) dma_position();
	if (measure < 0) {
		measure += USB_AUDIO_SAMPLE_COUNT;
	}

	error = measure - BUFFER_TARGET;
	integral += error;

	if (integral > integral_limit) {
		integral = integral_limit;
	} else if (integral < -integral_limit) {
		integral = -integral_limit;
	}

	p = kp * (float) error;
	i = ki * integral;

	output = p + i + init_output;

	if (output < 0) {
		output = 0;
	} else if (output > 0x3f) {
		output = 0x3f;
	}

	CRS->CR = (CRS->CR & ~CRS_CR_TRIM) | (output << CRS_CR_TRIM_Pos);
}

void usb_audio_complete_sync()
{
	if (stopped_count > 0) {
		stopped_count--;
	}
}

volatile uint16_t usb_audio_data [USB_AUDIO_SAMPLE_COUNT] = {0};

static void on_audio_data_in(uint8_t *data, uint8_t len)
{
	int16_t *din = (int16_t *) data;
	uint16_t samples = len / 2;
	int r = 0, l = 0;

	if (!audio_en) {
		for (int i = 0; i < USB_AUDIO_SAMPLE_COUNT; i++) {
			usb_audio_data[i] = 0;
		}

		return;
	}

	if (stopped_count == 0) {
		usb_audio_ptr = dma_position() + BUFFER_TARGET;
		if (!(usb_audio_ptr & 1)) {
			usb_audio_ptr++;
		}
		if (usb_audio_ptr >= USB_AUDIO_SAMPLE_COUNT) {
			usb_audio_ptr -= USB_AUDIO_SAMPLE_COUNT;
		}
		integral = 0.0f;
	}

	for (int i = 0; i < samples; i++) {
		usb_audio_data[(i + usb_audio_ptr) % USB_AUDIO_SAMPLE_COUNT] = din[i];
	}

	usb_audio_ptr += samples;

	if (usb_audio_ptr >= USB_AUDIO_SAMPLE_COUNT) {
		usb_audio_ptr -= USB_AUDIO_SAMPLE_COUNT;
	}

	for (int i = 0; i < samples; i += 2) {
		if (din[i]) {
			r = 1;
			break;
		}
	}

	for (int i = 1; i < samples; i += 2) {
		if (din[i]) {
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
		usb_audio_sync();
	}
}

static void on_correct_transfer(uint8_t ep, uint8_t *data, uint8_t len)
{
	if (ep == 0x01) {
		on_audio_data_in(data, len);
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
