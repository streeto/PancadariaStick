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
#include <util/delay.h>

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

/*
 * The data described by this descriptor consists of 4 bytes:
 *     B08 B07 B06 B05 B04 B03 B02 B01 .... Two bytes with buttons plus padding.
 *       .   .   .   .   .   . B10 B09
 *      X7  X6  X5  X4  X3  X2  X1  X0 .... 8 bit signed relative coordinate x.
 *      Y7  Y6  Y5  Y4  Y3  Y2  Y1  Y0 .... 8 bit signed relative coordinate y.
 */
#define REPORT_LEN 4
#define B_0 0
#define B_1 1
#define X_AX 2
#define Y_AX 3
static uchar reportBuffer[REPORT_LEN];
static uchar tmpReportBuffer[REPORT_LEN];


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
			usbMsgPtr = (usbMsgPtr_t) &reportBuffer;
			return sizeof(reportBuffer);
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
	/*
	 * The pins are mapped as follows:
	 *     Pin      --- --- PB5 PB4 PB3 PB2 PB1 PB0
	 *     Function         B10 B09 B08 B07 ---   R
	 *
	 *     Pin      --- --- PC5 PC4 PC3 PC2 PC1 PC0
	 *     Function         B06 B05 B04 B03 B02 B01
	 *
	 *     Pin      PD7 PD6 PD5 --- PD3 --- PD1 PD0
	 *     Function   L   U   D
	 */

	// Buttons
	tmpReportBuffer[B_0] = ~PINC & 0b00111111;
	tmpReportBuffer[B_0] |= (~PINB & 0b00001100) << 4;
	tmpReportBuffer[B_1] = (~PINB & 0b00110000) >> 4;

	// X axis
	if (~PINB & _BV(0)) {
		// Right
		tmpReportBuffer[X_AX] = 255;
	} else if (~PIND & _BV(7)) {
		// Left
		tmpReportBuffer[X_AX] = 0;
	} else {
		// Center
		tmpReportBuffer[X_AX] = 128;
	}

	// Y axis
	if (~PIND & _BV(6)) {
		// Up
		tmpReportBuffer[Y_AX] = 255;
	} else if (~PIND & _BV(5)) {
		// Down
		tmpReportBuffer[Y_AX] = 0;
	} else {
		//Center
		tmpReportBuffer[Y_AX] = 128;
	}
}

/* ------------------------------------------------------------------------- */

static char updateReport() {
	uchar i;
	uchar changed = 0;

	buildReport();
	for (i = 0; i < REPORT_LEN; ++i) {
		if (reportBuffer[i] != tmpReportBuffer[i]) {
			changed = 1;
		}
		reportBuffer[i] = tmpReportBuffer[i];
	}
	return changed;
}

/* ------------------------------------------------------------------------- */

static void portInit() {
	// Reset status: port bits are inputs without pull-up.
	// Enable pull-up on all pins, except D+/D- (PD2 and PD4).
	PORTB = 0xFF;
	PORTC = 0xFF;
	PORTD = 0xFA;
}

/* ------------------------------------------------------------------------- */

int __attribute__((noreturn)) main(void) {
	uchar i;

	wdt_enable(WDTO_1S);
	portInit();
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
			usbSetInterrupt((void *) &reportBuffer, sizeof(reportBuffer));
		}
	}
}

/* ------------------------------------------------------------------------- */
