/* Name: main.c
 * Project: hid-gamepad
 * Author: André Luiz de Amorim, based on hid-mouse example by Christian Starkjohann
 * Creation Date: 2016-08-07
 * Copyright: (C) 2016 By André Luiz de Amorim
 * Copyright: (C) 2008 By Objective Development Software Gmbh
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdbool.h>

#include "usbdrv.h"

/* ------------------------------------------------------------------------- */

/*
 * Descriptor created using HID Descriptor Tool from usb.org.
 * Gamepad with two axes (X and Y) and 10 buttons.
 */
PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
		0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
		0x09, 0x05,                    // USAGE (Game Pad)
		0xa1, 0x01,                    // COLLECTION (Application)
		0xa1, 0x00,                    //   COLLECTION (Physical)
		0x05, 0x09,                    //     USAGE_PAGE (Button)
		0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
		0x29, 0x0a,                    //     USAGE_MAXIMUM (Button 10)
		0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
		0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
		0x75, 0x01,                    //     REPORT_SIZE (1)
		0x95, 0x0a,                    //     REPORT_COUNT (10)
		0x81, 0x02,                    //     INPUT (Data,Var,Abs)
		0x75, 0x06,                    //     REPORT_SIZE (6)
		0x95, 0x01,                    //     REPORT_COUNT (1)
		0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
		0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
		0x09, 0x30,                    //     USAGE (X)
		0x09, 0x31,                    //     USAGE (Y)
		0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
		0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
		0x75, 0x08,                    //     REPORT_SIZE (8)
		0x95, 0x02,                    //     REPORT_COUNT (2)
		0x81, 0x02,                    //     INPUT (Data,Var,Abs)
		0xc0,                          //     END_COLLECTION
		0xc0                           // END_COLLECTION
		};

/* Type Defines: */
/** Type define for the joystick HID report structure, for creating and sending HID reports to the host PC.
 *  This mirrors the layout described to the host in the HID report descriptor, in Descriptors.c.
 *
 *     B08 B07 B06 B05 B04 B03 B02 B01 .... Two bytes with buttons plus padding.
 *       .   .   .   .   .   . B10 B09
 *      X7  X6  X5  X4  X3  X2  X1  X0 .... 8 bit signed relative coordinate x.
 *      Y7  Y6  Y5  Y4  Y3  Y2  Y1  Y0 .... 8 bit signed relative coordinate y.
 *
 **/
typedef struct
{
	uint8_t ButtonL; /**< Bit mask of the currently pressed joystick buttons */
	uint8_t ButtonH; /**< Bit mask of the currently pressed joystick buttons */
	uint8_t  X; /**< Current absolute joystick X position, as a signed 8-bit integer */
	uint8_t  Y; /**< Current absolute joystick Y position, as a signed 8-bit integer */
} USB_JoystickReport_Data_t;

/* Button mapping structures: */
typedef enum {
	MAP_PORTB = 0,
	MAP_PORTC,
	MAP_PORTD
} MapPort_t;

typedef struct {
	MapPort_t port;
	uint8_t mask;
} InputMap_t;

#define NUM_BUTTONS 10
#define NUM_INPUT 14

/* Buttons are mapped in the first 10 entries. Axes are stored at the end. */
typedef enum {
	MAP_AXIS_UP = 10,
	MAP_AXIS_DOWN,
	MAP_AXIS_LEFT,
	MAP_AXIS_RIGHT
} MapAxes_t;

/* The input map is held in an array, the 10 buttons first,
 * then the 4 diretions.
 *
 * The pins are mapped as follows:
 *     Pin      --- --- PB5 PB4 PB3 PB2 PB1 PB0
 *     Function             B04 B03 B02 B01   R
 *
 *     Pin      --- --- PC5 PC4 PC3 PC2 PC1 PC0
 *     Function         B10 B09 B08 B07 B06 B05
 *
 *     Pin      PD7 PD6 PD5 --- PD3 --- PD1 PD0
 *     Function   L   U   D
 */
const InputMap_t inputMap[NUM_INPUT] = { { MAP_PORTB, _BV(1) }, { MAP_PORTB, _BV(2) },
		{ MAP_PORTB, _BV(3) }, { MAP_PORTB, _BV(4) }, { MAP_PORTC, _BV(0) },
		{ MAP_PORTC, _BV(1) }, { MAP_PORTC, _BV(2) }, { MAP_PORTC, _BV(3) },
		{ MAP_PORTC, _BV(4) }, { MAP_PORTC, _BV(5) }, { MAP_PORTD, _BV(6) }, { MAP_PORTD, _BV(5) },
		{ MAP_PORTD, _BV(7) }, { MAP_PORTB, _BV(0) } };

uint8_t buttonOrder[NUM_BUTTONS];
uint8_t eeButtonOrder[NUM_BUTTONS] EEMEM = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
uint8_t newButtonOrder[NUM_BUTTONS];

static USB_JoystickReport_Data_t jsRep;
static USB_JoystickReport_Data_t tmpJsRep;


/* ------------------------------------------------------------------------- */

static inline uint8_t EEPROM_read(const void *eep_addr) {
	/* Wait for completion of previous write */
	while (EECR & (1 << EEWE));
	/* Set up address register */
	EEAR = (uint16_t) eep_addr;
	/* Start eeprom read by writing EERE */
	EECR |= (1 << EERE);
	/* Return data from data register */
	return EEDR;
}

/* ------------------------------------------------------------------------- */

static void EEPROM_read_block(uint8_t sram_dest[], uint8_t eep_orig[], uint8_t count) {
	uint8_t i;
	for (i = 0; i < count; ++i) {
		sram_dest[i] = EEPROM_read(&eep_orig[i]);
	};
}

/* ------------------------------------------------------------------------- */

static inline void EEPROM_write(uint8_t *eep_addr, uint8_t data) {
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE));
	/* Set up address and data registers */
	EEAR = (uint16_t) eep_addr;
	EEDR = data;
	/* Write logical one to EEMWE */
	EECR |= (1<<EEMWE);
	/* Start eeprom write by setting EEWE */
	EECR |= (1<<EEWE);
}

/* ------------------------------------------------------------------------- */

static void EEPROM_write_block(uint8_t sram_orig[], uint8_t eep_dest[], uint8_t count) {
	uint8_t i;
	for (i = 0; i < count; ++i) {
		EEPROM_write(&eep_dest[i], sram_orig[i]);
	};
}

/* ------------------------------------------------------------------------- */

static inline void LED_on(void)
{
	PORTB &= ~(1 << PB5);
}

/* ------------------------------------------------------------------------- */

static inline void LED_off(void)
{
	PORTB |= (1 << PB5);
}

/* ------------------------------------------------------------------------- */

static inline void LED_toggle(void)
{
	if (~PORTB & (1 << PB5)) {
		LED_off();
	} else {
		LED_on();
	}
}

/* ------------------------------------------------------------------------- */

static bool InputPressed(uint8_t input) {
	InputMap_t map;
	if (input < NUM_BUTTONS) {
		input = buttonOrder[input];
	}
	map = inputMap[input];
	switch (map.port) {
	case MAP_PORTB:
		return ~PINB & map.mask;
	case MAP_PORTC:
		return ~PINC & map.mask;
	case MAP_PORTD:
		return ~PIND & map.mask;
	}
	return false;

}

/* ------------------------------------------------------------------------- */

static bool IsMapped(uint8_t button, uint8_t orderMap[])
{
	uint8_t i;

	for (i = 0; i < NUM_BUTTONS; ++i) {
		if (orderMap[i] == button) {
			return true;
		}
	}
	return false;
}

static void MapInput(void)
{
	uint8_t input, button, count;

	/* Default button mapping. */
	for (input = 0; input < NUM_BUTTONS; ++input) {
		buttonOrder[input] = input;
	}
	if (!InputPressed(8) || !InputPressed(9)) {
		/* No remapping, load previous map. */
		EEPROM_read_block(buttonOrder, eeButtonOrder, NUM_BUTTONS);
		return;
	}

	/* Will remap buttons. Initialize blank new map. */
	for (input = 0; input < NUM_BUTTONS; ++input) {
		newButtonOrder[input] = 0xFF;
	}
	/* Blink slowly to inform the user we are remapping. */
	for (count = 0; count < 20; ++count) {
		LED_toggle();
		_delay_ms(100);
	}
	input = 0;
	while (input < NUM_BUTTONS) {
		button = 0;
		count = 0;
		/*
		 * Cycle through inputs and see if it keeps pressed for
		 * one second. Can't map the same input to another button.
		 */
		while (count < 20) {
			++count;
			_delay_ms(50);
			if (InputPressed(button) && !IsMapped(button, newButtonOrder)) {
				LED_on();
			} else {
				LED_toggle();
				count = 0;
				++button;
				if (button >= NUM_BUTTONS) {
					button = 0;
				}
			}
		}
		/* Found the new input for this button. */
		LED_off();
		_delay_ms(1000);
		newButtonOrder[input] = button;
		++input;
	}
	/* Done remapping. Use the new map and save to eeprom. */
	LED_off();
	for (input = 0; input < NUM_BUTTONS; ++input) {
		buttonOrder[input] = newButtonOrder[input];
	}
	EEPROM_write_block(buttonOrder, eeButtonOrder, NUM_BUTTONS);
}

/* ------------------------------------------------------------------------- */
/** Configure joystick and button pins. */
static void InputInit(void)
{
	InputMap_t map;
	uint8_t i;

	DDRB |= (1 << PB5); // Output: LED attached to PB5

	/* Inputs with pullup. */
	for(i = 0; i < NUM_INPUT; ++i) {
		map = inputMap[i];
		switch (map.port) {
		case MAP_PORTB:
			DDRB &= ~map.mask;
			PORTB |= map.mask;
			break;
		case MAP_PORTC:
			DDRC &= ~map.mask;
			PORTC |= map.mask;
			break;
		case MAP_PORTD:
			DDRD &= ~map.mask;
			PORTD |= map.mask;
			break;
		}
	}
}

/* ------------------------------------------------------------------------- */

static uchar idleRate = 0;

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
	usbRequest_t *rq = (void *) data;

	/* The following requests are never used. But since they are required by
	 * the specification, we implement them in this example.
	 */
	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) { /* class request type */
		if (rq->bRequest == USBRQ_HID_GET_REPORT) { /* wValue: ReportType (highbyte), ReportID (lowbyte) */
			/* we only have one report type, so don't look at wValue */
			usbMsgPtr = (usbMsgPtr_t) &jsRep;
			return sizeof(jsRep);
		} else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
			usbMsgPtr = (usbMsgPtr_t) &idleRate;
			return 1;
		} else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
			idleRate = rq->wValue.bytes[1];
		}
	} else {
		/* no vendor specific requests implemented */
	}
	return 0; /* default for not implemented requests: return no data back to host */
}

/* ------------------------------------------------------------------------- */

static void buildReport() {
	if (InputPressed(MAP_AXIS_LEFT)) {
		jsRep.X = 0;
	}
	else if (InputPressed(MAP_AXIS_RIGHT)) {
		jsRep.X = 255;
	}
	else {
		jsRep.X = 128;
	}

	if (InputPressed(MAP_AXIS_DOWN)) {
		jsRep.Y = 0;
	}
	else if (InputPressed(MAP_AXIS_UP)) {
		jsRep.Y = 255;
	}
	else {
		jsRep.Y = 128;
	}

	jsRep.ButtonL = 0x00;
	jsRep.ButtonL |= InputPressed(0) << 0;
	jsRep.ButtonL |= InputPressed(1) << 1;
	jsRep.ButtonL |= InputPressed(2) << 2;
	jsRep.ButtonL |= InputPressed(3) << 3;
	jsRep.ButtonL |= InputPressed(4) << 4;
	jsRep.ButtonL |= InputPressed(5) << 5;
	jsRep.ButtonL |= InputPressed(6) << 6;
	jsRep.ButtonL |= InputPressed(7) << 7;

	jsRep.ButtonH = 0x00;
	jsRep.ButtonH |= InputPressed(8) << 0;
	jsRep.ButtonH |= InputPressed(9) << 1;

	if (jsRep.ButtonL || jsRep.ButtonH) {
		LED_on();
	} else {
		LED_off();
	}

}

/* ------------------------------------------------------------------------- */

static char updateReport() {
	buildReport();
	if ((jsRep.ButtonL != tmpJsRep.ButtonL)
			|| (jsRep.ButtonH != tmpJsRep.ButtonH)
			|| (jsRep.X != tmpJsRep.X)
			|| (jsRep.Y != tmpJsRep.Y)) {
		tmpJsRep = jsRep;
		return 1;
	} else {
		return 0;
	}
}

/* ------------------------------------------------------------------------- */

int __attribute__((OS_main)) main(void) {
	uchar i;

	InputInit();
	MapInput();
	wdt_enable(WDTO_1S);
	usbInit();
	usbDeviceDisconnect(); /* enforce re-enumeration, do this while interrupts are disabled! */
	i = 0;
	while (--i) { /* fake USB disconnect for > 250 ms */
		wdt_reset();
		_delay_ms(1);
	}
	usbDeviceConnect();
	sei();

	for (;;) {
		wdt_reset();
		usbPoll();
		if (usbInterruptIsReady() && updateReport()) {
			usbSetInterrupt((void *) &jsRep, sizeof(jsRep));
		}
	}
}

/* ------------------------------------------------------------------------- */
