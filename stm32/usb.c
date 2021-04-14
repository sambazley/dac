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
#include <stm32f0xx.h>
#include "io.h"
#include "uart.h"

struct BufDescEntry {
	volatile uint16_t addr, count;
} __attribute__((packed));

struct BufDesc {
	volatile struct BufDescEntry tx, rx;
} __attribute__((packed));

#define btable ((volatile struct BufDesc *) USB_PMAADDR)

#define PMA_TX 0
#define PMA_RX 1

#define USB_EP(x) (&USB->EP0R + x * 2)

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static struct Endpoint {
	uint16_t rx_size, tx_size;
	uint16_t type;
} endpoints [] = {
	{64, 64, USB_EP_CONTROL},
	{256, 256, USB_EP_ISOCHRONOUS},
};

static uint16_t *pma_addr(uint8_t ep_addr, uint8_t isrx)
{
	int nep = sizeof(endpoints) / sizeof(struct Endpoint);

	uint16_t addr = nep * sizeof(struct BufDesc);

	for (int i = 0; i < ep_addr; i++) {
		addr += endpoints[i].tx_size;
		addr += endpoints[i].rx_size;
	}

	if (isrx) {
		addr += endpoints[ep_addr].tx_size;
	}

	return (uint16_t *) (uint32_t) (USB_PMAADDR + addr);
}

static void usb_set_irq()
{
	USB->CNTR = USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM |
		USB_CNTR_RESETM;
}

static void set_ep_rx_count(volatile struct BufDescEntry *e, int length)
{
	if (length > 62) {
		e->count = 0x8000 | (((length >> 5) - 1) << 10);
	} else {
		e->count = (length >> 1) << 10;
	}
}

static void set_ep_tx_status(volatile uint16_t *epreg, uint16_t status)
{
	*epreg = (*epreg & USB_EPTX_DTOGMASK) ^ status;
}

static void set_ep_rx_status(volatile uint16_t *epreg, uint16_t status)
{
	*epreg = (*epreg & USB_EPRX_DTOGMASK) ^ status;
}

static volatile struct USBTransmitData {
	uint16_t length, wLength;
	uint16_t sent;
	const uint8_t *data;
} usb_tx_data [1];

static void usb_continue_send_data(int ep)
{
	volatile struct USBTransmitData *tx = &usb_tx_data[ep];
	uint16_t *dest = pma_addr(ep, PMA_TX);
	uint8_t tx_length = tx->length - tx->sent;

	if (tx_length > tx->wLength) {
		tx_length = tx->wLength;
	} else if (tx_length > endpoints[ep].tx_size) {
		tx_length = endpoints[ep].tx_size;
	}

	for (int i = 0; i < tx_length; i += 2) {
		*dest++ = tx->data[i] | ((uint16_t) tx->data[i + 1] << 8);
	}

	tx->data += tx_length;
	tx->sent += tx_length;

	btable[ep].tx.count = tx_length;

	set_ep_tx_status(&USB->EP0R, USB_EP_TX_VALID);
}

static void usb_send_data(uint8_t endpoint,
		const uint8_t *data,
		uint16_t length,
		uint16_t wLength)
{
	usb_tx_data[endpoint].data = data;
	usb_tx_data[endpoint].length = length;
	usb_tx_data[endpoint].wLength = wLength;
	usb_tx_data[endpoint].sent = 0;

	usb_continue_send_data(endpoint);
}

struct USBSetupPacket {
	volatile uint8_t bmRequestType;
	volatile uint8_t bRequest;
	volatile uint16_t wValue;
	volatile uint16_t wIndex;
	volatile uint16_t wLength;
} __attribute__((packed));

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
		1,                                          // bDelay
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

static const struct DescriptorEntry {
	const uint16_t wValue;
	const uint16_t wIndex;
	const uint8_t *addr;
	const uint8_t length;
} descriptor_table [] = {
	DESCRIPTOR(DESC_DEVICE, 0, 0x0000, device_descriptor),
	DESCRIPTOR(DESC_CONFIG, 0, 0x0000, config1_descriptor),
	DESCRIPTOR(DESC_STRING, 0, 0x0000, lang_str),
	DESCRIPTOR(DESC_STRING, 1, 0x0409, vendor_str),
	DESCRIPTOR(DESC_STRING, 2, 0x0409, product_str),
	DESCRIPTOR(DESC_STRING, 3, 0x0409, serial_no_str),
};

static void find_descriptor(const uint16_t wValue,
		const uint16_t wIndex,
		const uint8_t **addr,
		uint8_t *length)
{
	int ndesc = sizeof(descriptor_table) / sizeof(struct DescriptorEntry);

	for (int i = 0; i < ndesc; i++) {
		const struct DescriptorEntry *entry = descriptor_table + i;
		if (entry->wValue == wValue && entry->wIndex == wIndex) {
			*addr = entry->addr;
			*length = entry->length;
			return;
		}
	}

	uart_send_str("No descriptor found, wValue: ");
	uart_send_int(wValue);
	uart_send_str(", wIndex: ");
	uart_send_int(wIndex);
	uart_send_str("\n");

	*addr = 0;
	*length = 0;
}

enum Request {
	REQ_CLEAR_FEATURE = 1,
	REQ_SET_ADDRESS = 5,
	REQ_GET_DESCRIPTOR = 6,
	REQ_SET_CONFIGURATION = 9,
	REQ_SET_INTERFACE = 11,
	REQ_AUDIO_GET_CUR = 129,
	REQ_AUDIO_GET_MIN = 130,
	REQ_AUDIO_GET_MAX = 131,
	REQ_AUDIO_GET_RES = 132,
};

enum Recipient {
	RECIP_DEVICE,
	RECIP_INTERFACE,
	RECIP_ENDPOINT,
	RECIP_OTHER
};

struct Interface {
	uint16_t index;
	uint16_t alternate;
	void (*on_ctrl)(struct Interface *, volatile struct USBSetupPacket *);
};

volatile uint8_t usb_selected_config;
volatile uint8_t usb_received_request;
volatile uint16_t usb_audio_data [USB_AUDIO_SAMPLE_COUNT] = {0};

static volatile int address = 0;

static void ack(int ep)
{
	btable[ep].tx.count = 0;
	set_ep_tx_status(USB_EP(ep), USB_EP_TX_VALID);
}

static void on_control_out_interface0(struct Interface *iface,
		volatile struct USBSetupPacket *sp)
{
	(void) iface;

	switch (sp->bRequest) {
	default:
		uart_send_str("== UNHANDLED INTERFACE 0 REQUEST ");
		uart_send_int(sp->bRequest);
		uart_send_str(" ==\n");
		ack(0);
	}
}

static void on_control_out_interface1(struct Interface *iface,
		volatile struct USBSetupPacket *sp)
{
	switch (sp->bRequest) {
	case REQ_SET_INTERFACE:
		uart_send_str("SET_IFACE ");
		uart_send_int(sp->wValue);
		uart_send_str("\n");
		iface->alternate = sp->wValue;
		volatile uint16_t *epr = USB_EP(1);

		if (iface->alternate) {
			if (*epr & USB_EP_DTOG_TX) {
				*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_TX;
			}

			if (*epr & USB_EP_DTOG_RX) {
				*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_RX;
			}

			set_ep_rx_status(epr, USB_EP_RX_VALID);
			set_ep_tx_status(epr, USB_EP_TX_VALID);
		} else {
			set_ep_rx_status(epr, USB_EP_RX_DIS);
			set_ep_tx_status(epr, USB_EP_TX_DIS);
		}

		ack(0);

		break;
	default:
		uart_send_str("== UNHANDLED INTERFACE 1 REQUEST ");
		uart_send_int(sp->bRequest);
		uart_send_str(" ==\n");
		ack(0);
	}
}

static struct Interface interfaces [] = {
	{0, 0, on_control_out_interface0},
	{1, 0, on_control_out_interface1},
};

static void on_control_out_interface(volatile struct USBSetupPacket *sp)
{
	int niface = sizeof(interfaces) / sizeof(struct Interface);
	for (int i = 0; i < niface; i++) {
		struct Interface *iface = interfaces + i;
		if ((sp->wIndex & 0xFF) == iface->index) {
			iface->on_ctrl(iface, sp);
			return;
		}
	}

	uart_send_str("Interface ");
	uart_send_int(sp->wIndex);
	uart_send_str(" not found\n");
}

static void on_control_out_device(volatile struct USBSetupPacket *sp)
{
	switch (sp->bRequest) {
		case REQ_GET_DESCRIPTOR:
		{
			uart_send_str("GET_DESC ");
			uart_send_int(sp->wValue);
			uart_send_str(", ");
			uart_send_int(sp->wIndex);
			uart_send_str("\n");

			const uint8_t *addr;
			uint8_t length;

			find_descriptor(sp->wValue, sp->wIndex, &addr, &length);

			if (!addr) {
				uart_send_str("Failed to find descriptor\n");
				set_ep_tx_status(&USB->EP0R, USB_EP_TX_STALL);
				break;
			}

			usb_send_data(0, addr, length, sp->wLength);
			break;
		}
		case REQ_SET_ADDRESS:
			uart_send_str("SET_ADDR");
			address = sp->wValue & 0x7F;
			uart_send_int(address);
			uart_send_str("\n");
			ack(0);
			break;
		case REQ_SET_CONFIGURATION:
			uart_send_str("SET_CONF");
			uart_send_int(sp->wValue);
			uart_send_str("\n");
			usb_selected_config = sp->wValue;

			ack(0);

			break;
		default:
			uart_send_str("== UNHANDLED REQUEST ");
			uart_send_int(sp->bRequest);
			uart_send_str(" ==\n");
			ack(0);
	}
}

static void on_control_out()
{
	if (USB->EP0R & USB_EP_SETUP) {
		volatile struct USBSetupPacket *sp =
			(volatile struct USBSetupPacket *) pma_addr(0, PMA_RX);

		uint8_t recipient = sp->bmRequestType & 0x1f;

		switch (recipient) {
		case RECIP_DEVICE:
			on_control_out_device(sp);
			break;
		case RECIP_INTERFACE:
			on_control_out_interface(sp);
			return;
		default:
			uart_send_str("Recipient ");
			uart_send_int(recipient);
			uart_send_str(" not implemented for control out");
			return;
		}
	}
}

static void on_control_in()
{
	uint16_t remaining = usb_tx_data[0].length - usb_tx_data[0].sent;
	if (remaining) {
		usb_continue_send_data(0);
	}

	set_ep_rx_status(&USB->EP0R,USB_EP_RX_VALID);

	if (address) {
		USB->DADDR = address | USB_DADDR_EF;
		address = 0;
	}
}

static volatile int audio_en;
static volatile int16_t usb_audio_ptr = 1;
static volatile int16_t drop = 0, insert = 0;

#define MARGIN 192
#define DROP_COUNT 24

static volatile uint16_t stopped_count;

void usb_audio_en(int en)
{
	audio_en = en;
}

void usb_audio_half_sync()
{
	if (!stopped_count) {
		usb_audio_ptr = MARGIN + 1;
		drop = 0;
		insert = 0;
		return;
	}

	stopped_count--;

	if (usb_audio_ptr > USB_AUDIO_SAMPLE_COUNT / 2 - MARGIN &&
			usb_audio_ptr <= USB_AUDIO_SAMPLE_COUNT / 2) {
		drop = DROP_COUNT;
		insert = 0;
	} else if (usb_audio_ptr > USB_AUDIO_SAMPLE_COUNT / 2 &&
			usb_audio_ptr < USB_AUDIO_SAMPLE_COUNT / 2 + MARGIN) {
		drop = 0;
		insert = DROP_COUNT;
	}
}

void usb_audio_complete_sync()
{
	if (!stopped_count) {
		usb_audio_ptr = USB_AUDIO_SAMPLE_COUNT / 2 + MARGIN + 1;
		drop = 0;
		insert = 0;
		return;
	}

	stopped_count--;

	if (usb_audio_ptr > USB_AUDIO_SAMPLE_COUNT - MARGIN) {
		drop = DROP_COUNT;
		insert = 0;
	} else if (usb_audio_ptr < MARGIN) {
		drop = 0;
		insert = DROP_COUNT;
	}
}

static void on_audio_data_in()
{
	int16_t *din;
	uint16_t samples;
	int r = 0, l = 0;

	stopped_count = 10;

	if (USB->EP1R & USB_EP_DTOG_RX) {
		din = (int16_t *) pma_addr(1, PMA_RX);
		samples = (btable[1].rx.count & 0x3ff) / 2;
	} else {
		din = (int16_t *) pma_addr(1, PMA_TX);
		samples = (btable[1].tx.count & 0x3ff) / 2;
	}

	for (int i = 0; i < samples; i++) {
#define dest usb_audio_data[((i + usb_audio_ptr) % USB_AUDIO_SAMPLE_COUNT)]
		if (drop && i == 0) {
			usb_audio_ptr -= 2;
			if (usb_audio_ptr < 0) {
				usb_audio_ptr += USB_AUDIO_SAMPLE_COUNT;
			}
			drop--;
		}

		dest = *(uint16_t *) &din[i];

		if (insert && i == 1) {
			usb_audio_ptr++;
			int16_t interpolation = (din[i - 1] + din[i + 1]) / 2;
			dest = *(uint16_t *) &interpolation;

			usb_audio_ptr++;
			int16_t interpolation2 = (din[i] + din[i + 2]) / 2;
			dest = *(uint16_t *) &interpolation2;

			insert--;
		}
#undef dest
	}

	usb_audio_ptr += samples;
	usb_audio_ptr %= USB_AUDIO_SAMPLE_COUNT;

	if (!audio_en) {
		for (int i = 0; i < USB_AUDIO_SAMPLE_COUNT; i++) {
			usb_audio_data[i] = 0;
		}
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
}

static void on_correct_transfer()
{
	uint8_t ep = USB->ISTR & USB_ISTR_EP_ID;

	usb_received_request = 1;

	if (ep == 0) {
		if ((USB->ISTR & USB_ISTR_DIR) != 0) {
			on_control_out();
			USB->EP0R = USB->EP0R & ~USB_EP_CTR_RX & USB_EPREG_MASK;
		} else {
			on_control_in();
			USB->EP0R = USB->EP0R & ~USB_EP_CTR_TX & USB_EPREG_MASK;
		}
	} else if (ep == 1 && (USB->ISTR & USB_ISTR_DIR) != 0) {
		USB->EP1R=USB->EP1R & ~USB_EP_CTR_RX & USB_EPREG_MASK;

		on_audio_data_in();
	}
}

static void open_endpoints()
{
	int nep = sizeof(endpoints) / sizeof(struct Endpoint);

	for (int i = 0; i < nep; i++) {
		volatile uint16_t *epr = USB_EP(i);
		struct Endpoint *ep = &endpoints[i];

		set_ep_rx_status(epr, USB_EP_RX_DIS);
		set_ep_tx_status(epr, USB_EP_TX_DIS);

		*epr = (*epr & USB_EPREG_MASK) | i;
		*epr = (*epr & USB_EP_T_MASK) | ep->type;

		if (*epr & USB_EP_DTOG_TX) {
			*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_TX;
		}

		if (*epr & USB_EP_DTOG_RX) {
			*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_RX;
		}

		switch (ep->type) {
		case USB_EP_CONTROL:
			set_ep_rx_count(&btable[i].rx, ep->rx_size);
			set_ep_rx_status(epr,USB_EP_RX_VALID);
			set_ep_tx_status(epr,USB_EP_TX_NAK);
			break;
		case USB_EP_ISOCHRONOUS:
			set_ep_rx_count(&btable[i].rx, ep->rx_size);
			set_ep_rx_count(&btable[i].tx, ep->tx_size);

			set_ep_rx_status(epr, USB_EP_RX_VALID);
			break;
		default:
			uart_send_str("Endpoint type ");
			uart_send_int(ep->type);
			uart_send_str(" not implemented\n");
		}
	}
}

void usb_irq()
{
	while (USB->ISTR & USB_ISTR_CTR) {
		on_correct_transfer();
	}

	if (USB->ISTR & USB_ISTR_WKUP) {
		USB->CNTR &= ~USB_CNTR_FSUSP;
		USB->CNTR &= ~USB_CNTR_LPMODE;
		USB->ISTR &= ~USB_ISTR_WKUP;
	}

	if (USB->ISTR & USB_ISTR_SUSP) {
		USB->CNTR |= USB_CNTR_FSUSP;
		USB->CNTR |= USB_CNTR_LPMODE;
		USB->ISTR &= ~USB_ISTR_SUSP;
	}

	if (USB->ISTR & USB_ISTR_RESET) {
		usb_selected_config = 0;

		USB->DADDR = USB_DADDR_EF;
		open_endpoints();
		USB->ISTR &= ~USB_ISTR_RESET;
	}
}

void usb_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_CRSEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;

	USB->BTABLE = 0;

	USB->BCDR |= USB_BCDR_DPPU;

	int nep = sizeof(endpoints) / sizeof(struct Endpoint);

	for (int i = 0; i < nep; i++) {
		uint32_t rx = (uint32_t) pma_addr(i, PMA_RX);
		uint32_t tx = (uint32_t) pma_addr(i, PMA_TX);
		btable[i].rx.addr = rx - USB_PMAADDR;
		btable[i].tx.addr = tx - USB_PMAADDR;
	}

	usb_selected_config = 0;
	usb_received_request = 0;

	USB->ISTR = 0;
	NVIC_EnableIRQ(USB_IRQn);
	usb_set_irq();
}
