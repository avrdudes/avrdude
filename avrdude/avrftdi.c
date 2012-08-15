/*
 * avrftdi - extension for avrdude, Wolfgang Moser, Ville Voipio
 * Copyright (C) 2011 Hannes Weisbach, Doug Springer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* $Id$ */
/*
 * Interface to the MPSSE Engine of FTDI Chips using libftdi.
 */
#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <stdarg.h>

#include "avrdude.h"
#include "avr.h"
#include "pgm.h"
#include "avrftdi.h"
#include "avrpart.h"
#include "tpi.h"
#include "usbasp.h"

#ifdef HAVE_LIBUSB
#ifdef HAVE_LIBFTDI

#include <ftdi.h>
#if defined(HAVE_USB_H)
#  include <usb.h>
#elif defined(HAVE_LUSB0_USB_H)
#  include <lusb0_usb.h>
#else
#  error "libusb needs either <usb.h> or <lusb0_usb.h>"
#endif

enum { FTDI_SCK = 1, FTDI_MOSI, FTDI_MISO, FTDI_RESET };
#define FTDI_DEFAULT_MASK ( (1 << (FTDI_SCK - 1)) | (1 << (FTDI_MOSI - 1)) )

#define E(x, ftdi)                                                  \
	do {                                                              \
		if ((x))                                                        \
		{                                                               \
			fprintf(stderr, "%s:%d %s() %s: %s (%d)\n\t%s\n",             \
					__FILE__, __LINE__, __FUNCTION__,                         \
					#x, strerror(errno), errno, ftdi_get_error_string(ftdi)); \
			return -1;                                                    \
		}                                                               \
	} while(0);

#define E_VOID(x, ftdi)                                             \
	do {                                                              \
		if ((x))                                                        \
		{                                                               \
			fprintf(stderr, "%s:%d %s() %s: %s (%d)\n\t%s\n",             \
					__FILE__, __LINE__, __FUNCTION__,                         \
	 			 #x, strerror(errno), errno, ftdi_get_error_string(ftdi));  \
		}                                                               \
	} while(0);


#define to_pdata(pgm) \
	((avrftdi_t *)((pgm)->cookie))

/* This is for running the code without having a FTDI-device.
 * The generated code is useless! For debugging purposes only.
 * This should never be defined, unless you know what you are
 * doing.
 * If you think you know what you are doing: YOU DONT!
 */
//#define DRYRUN

typedef struct avrftdi_s {
	/* pointer to struct maintained by libftdi to identify the device */
	struct ftdi_context* ftdic; 
	/* bitmask of values for pins. bit 0 represents pin 0 ([A|B]DBUS0) */
	uint16_t pin_value;
	/* bitmask of pin direction. a '1' make a pin an output.
	 * bit 0 corresponds to pin 0. */
	uint16_t pin_direction;
	/* don't know. not useful. someone put it in. */
	uint16_t led_mask;
	/* total number of pins supported by a programmer. varies with FTDI chips */
	int pin_limit;
} avrftdi_t;

static int write_flush(avrftdi_t *);

/*
 * returns a human-readable name for a pin number. the name should match with
 * the pin names used in FTDI datasheets.
 */
static char*
ftdi_pin_name(avrftdi_t* pdata, int pin)
{
	static char pin_name[16];

	char interface = '@';
	char port;

	/* INTERFACE_ANY is zero, so @ is used
	 * INTERFACE_A is one, so '@' + 1 = 'A'
	 * and so forth ...
	 * be aware, there is an 'interface' member in ftdi_context,
	 * however, we really want the 'index' member here.
	 */
	interface += pdata->ftdic->index;

	/* This is FTDI's naming scheme.
	 * probably 'D' is for data and 'C' for control
	 */
	if(pin < 8)
		port = 'D';
	else
		port = 'C';

	snprintf(pin_name, sizeof(pin_name), "%c%cBUS%d", interface, port, pin-1);

	return pin_name;
}

/*
 * output function, to save if(vebose>level)-constructs. also prefixes output
 * with "avrftdi" to identify were messages came from.
 * TODO: make this a macro, so that __LINE_ and __func__ macros can be used.
 */
static void
avrftdi_print(int level, const char * fmt, ...)
{
	va_list ap;
	if(verbose > level)
	{
		fprintf(stderr, "avrftdi: ");
		va_start(ap, fmt);
		vfprintf(stderr, fmt, ap);
		va_end(ap);
	}
}

/*
 * helper function to print a binary buffer *buf of size len. begin and end of
 * the dump are enclosed in the string contained in *desc. offset denotes the
 * number of bytes which are printed on the first line (may be 0). after that
 * width bytes are printed on each line
 */
static void buf_dump(unsigned char *buf, int len, char *desc,
		     int offset, int width)
{
	int i;
	fprintf(stderr, "%s begin:\n", desc);
	for (i = 0; i < offset; i++)
		fprintf(stderr, "%02x ", buf[i]);
	fprintf(stderr, "\n");
	for (i++; i <= len; i++) {
		fprintf(stderr, "%02x ", buf[i-1]);
		if((i-offset) != 0 && (i-offset)%width == 0)
		    fprintf(stderr, "\n");
	}
	fprintf(stderr, "%s end\n", desc);
}

/*
 * calculates the so-called 'divisor'-value from a given frequency.
 * the divisor is sent to the chip.
 */
static int set_frequency(avrftdi_t* ftdi, uint32_t freq)
{
	uint32_t divisor;
	uint8_t buf[3];

	/* divisor on 6000000 / freq - 1 */
	divisor = (6000000 / freq) - 1;
	if (divisor < 0) {
		fprintf(stderr,
			"%s failure: Frequency too high (%u > 6 MHz)\n",
			progname, freq);
		fprintf(stderr,
			"resetting Frequency to 6MHz\n");
		divisor = 0;
	}

	if (divisor > 65535) {
		fprintf(stderr,
			"%s failure: Frequency too low (%u < 91.553 Hz)\n",
			progname, freq);
		fprintf(stderr,
			"resetting Frequency to 91.553Hz\n");
		divisor = 65535;
	}

	avrftdi_print(0, "frequency: %d\n", 6000000/(divisor+1));
	avrftdi_print(1, "clock divisor: 0x%04x\n", divisor);

	buf[0] = TCK_DIVISOR;
	buf[1] = (uint8_t)(divisor & 0xff);
	buf[2] = (uint8_t)((divisor >> 8) & 0xff);

#ifndef DRYRUN
	E(ftdi_write_data(ftdi->ftdic, buf, 3) < 0, ftdi->ftdic);
#endif

	return 0;
}

/*
 * Adds a single pin to the direction mask and sets the pin state inactive in
 * the value mask. the value of 'inactive' is chosen according to the pin
 * configuration (high or low active).
 */
static int add_pin(PROGRAMMER *pgm, int pinfunc)
{
	int pin, pin_mask, inverted, fail;
	avrftdi_t* pdata = to_pdata(pgm);
	
	fail = 0;
	pin = pgm->pinno[pinfunc] & PIN_MASK;
	inverted = pgm->pinno[pinfunc] & PIN_INVERSE;
	pin_mask = (1 << (pin - 1));

	/* not configured */
	if(!pin)
	{
		avrftdi_print(0, "Pin %s not configured\n", avr_pin_name(pinfunc));
		return 0;
	}

	/* check that the pin number is in range */
	if (pin > pdata->pin_limit)
	{
		fprintf(stderr,
			"%s invalid pin definition for pin %s. Configured as pin %d, but highest pin is %d.\n",
			progname, avr_pin_name(pinfunc), pin, pdata->pin_limit);
		fail = 1;
	}

	/* check if the pin is still available */
	if (pdata->pin_direction & pin_mask)
	{
		fprintf(stderr,
			"%s failure: pin %d (%s) is used twice. The second use is %s.\n",
			progname, pin, ftdi_pin_name(pdata, pin), avr_pin_name(pinfunc));
		fail = 1;
	}

	/* 
	 * No need to fail for a wrongly configured led.
	 * MISO, MOSI and SCK are fixed and correctly set during setup.
	 * Maybe we should fail for wrongly configured VCC or BUFF pins?
	 */
	if(fail)
	{
		if(pinfunc == PIN_AVR_RESET)
		{
			fprintf(stderr, "Aborting, since the reset pin is wrongly configured\n");
			return -1;
		}
		else
		{
			fprintf(stdout, "Ignoring wrongly configured pin.\n");
			return 0;
		}
	}

	/* all checks passed - do actual work */
	avrftdi_print(0, "Configure pin %d (%s) as %s (%s active)\n",
			pin, ftdi_pin_name(pdata, pin),
			avr_pin_name(pinfunc), (inverted) ? "low": "high");

	{
		/* create mask */
		pdata->pin_direction |= pin_mask;
		/* and set default value */
		if(inverted)
			pdata->pin_value |= pin_mask;
		else
			pdata->pin_value &= ~(pin_mask);
	}

	if(PIN_LED_ERR == pinfunc ||
		 PIN_LED_VFY == pinfunc ||
		 PIN_LED_RDY == pinfunc ||
		 PIN_LED_PGM == pinfunc) {
		pdata->led_mask |= pin_mask;
	}
	
	return 0;
}

/*
 * Add pins by pin mask
 * Check an entire mask for correctness and plausibility. Performed checks are
 * the pin number is lower that the total number of pins and the pin is not
 * configured yet.
 * If at least one test fails, the entire mask is discarded.
 * These basic tests could possibly moved to avrdude core, since it does not
 * contain any tests (as far as I can tell).
 */
static int add_pins(PROGRAMMER *pgm, int pinfunc)
{
	int pin, inverted, fail;
	uint32_t pin_mask, pin_bit;
	avrftdi_t* pdata = to_pdata(pgm);

	pin_mask = (pgm->pinno[pinfunc] & PIN_MASK) >> 1;
	/* FIXME: I think you cannot inverse these multi-pin options */
	inverted = pgm->pinno[pinfunc] & PIN_INVERSE;

	if(!pin_mask)
	{
		avrftdi_print(0, "Pins for %s not configured.\n", avr_pin_name(pinfunc));
		return 0;
	}

	fail = 0;
	/* check every configured pin */
	for(pin = 0; (1 << pin) & (PIN_MASK); pin++)
	{
		pin_bit = 1 << pin;
		
		/* skip, if this pin is not in the mask to be configured */
		if(!(pin_bit & pin_mask))
			continue;
		

		/* 0 is not a valid pin, see above, we use 1 << (pin - 1) to create pin_bit */
		if(pin + 1 > pdata->pin_limit)
		{
			fprintf(stderr,
				"%s invalid pin definition for pin %s. Configured as pin %d, but highest pin is %d.\n",
				progname, avr_pin_name(pinfunc), pin + 1, pdata->pin_limit);
			fail = 1;
		}

		if(pin_bit & pdata->pin_direction)
		{
			fprintf(stderr,
				"%s failure: pin %d (%s) is used twice. The second use is %s.\n",
				progname, pin, ftdi_pin_name(pdata, pin), avr_pin_name(pinfunc));
			fail = 1;
		}

	}

	/* we can ignore those, because only VCC and BUFF pins, can have multiples.
	 * VCC and BUFF are not essential
	 */
	if(fail)
	{
			fprintf(stdout, "Ignoring wrongly configured pins.\n");
			return 0;
	}

	/* conditional output */
	for(pin = 0; (1 << pin) & (PIN_MASK); pin++)
	{
		pin_bit = 1 << pin;

		/* skip if pin is not set */
		if(!(pin_bit & pin_mask))
			continue;

		/* remember, we count from 1, not 0 */
		avrftdi_print(0, "Configured pin %d (%s) as %s (%s active)\n",
			pin+1, ftdi_pin_name(pdata, pin+1),
			avr_pin_name(pinfunc), (inverted) ? "low": "high");
	}

	/* do the work */
	pdata->pin_direction |= (uint16_t)pin_mask;
	if(inverted)
		pdata->pin_value |= pin_mask;
	else
		pdata->pin_value &= ~pin_mask;

	return 0;
}

/*
 * This function sets or clears any pin, except SCK, MISO and MOSI. Depending
 * on the pin configuration, a non-zero value sets the pin in the 'active'
 * state (high active, low active) and a zero value sets the pin in the
 * inactive state.
 * Because we configured the pin direction mask earlier, nothing bad can happen
 * here.
 */
static int set_pin(PROGRAMMER * pgm, int pinfunc, int value)
{
	
	int pin, pin_mask;
	int inverted;
	
	avrftdi_t* pdata = to_pdata(pgm);

	pin = pgm->pinno[pinfunc] & PIN_MASK;
	inverted = pgm->pinno[pinfunc] & PIN_INVERSE;

	pin_mask = 1 << (pin - 1);

	/* make value 0 or 1 and invert, if necessary */
	value = (inverted) ? !value : !!value;

	if (!pin) {
		/* this error message is really annoying, maybe use a ratelimit? */
	/*
		avrftdi_print(2, "%s info: Pin is zero, value: %d!\n",
				progname, value);
	*/
		return 1;
	}

	if (value)
		value = pin_mask;

	avrftdi_print(1, "Setting pin %d (%s) as %s: %s (%s active)\n", pin,
			ftdi_pin_name(pdata, pin), avr_pin_name(pinfunc),
			(value) ? "high" : "low", (inverted) ? "low" : "high");

	/* set bits depending on value */
	//tval = (pdata->pin_value & (~pin_mask)) | pin_mask;
	pdata->pin_value ^= (-value ^ pdata->pin_value) & pin_mask;
	//fprintf(stderr, "%x %x\n", tval, pdata->pin_value);
	
	return write_flush(pdata);
}

/*
 * This function sets or clears a group of pins - VCC or BUFF.
 * the semantics are the same as for single pins, described above.
 */
static int set_pins(PROGRAMMER * pgm, int pinfunc, int value)
{
	int pin, pin_mask;
	int inverted;
	int pin_bit;
	
	avrftdi_t* pdata = to_pdata(pgm);

	pin = pgm->pinno[pinfunc] & PIN_MASK;
	inverted = pgm->pinno[pinfunc] & PIN_INVERSE;

	pin_mask = pin >> 1;
	
	value = (inverted) ? !value : !!value;

	if (!pin) {
		/* dito above */
		return 1;
	}

	if(value)
		value = pin_mask;

	/* conditional output */
	for(pin = 0; (1 << pin) & (PIN_MASK); pin++)
	{
		pin_bit = 1 << pin;

		/* skip if pin is not set */
		if(!(pin_bit & pin_mask))
			continue;

		/* remember, we count from 1, not 0 */
		avrftdi_print(0, "Setting pin %d (%s) as %s: %s (%s active)\n",
			pin+1, ftdi_pin_name(pdata, pin+1), avr_pin_name(pinfunc),
			(value) ? "high" : "low", (inverted) ? "low": "high");
	}

	/* set bits depending on value */
	/*pin_value ^= (-value ^ pin_value) & (1 << (pin - 1));  */
	pdata->pin_value ^= (-value ^ pdata->pin_value) & pin_mask;

	/*pdata->pin_value = (pdata->pin_value & (~pin_mask)) | value;*/
	
	return write_flush(pdata);
}

/* these functions are callbacks, which go into the
 * PROGRAMMER data structure ("optional functions")
 */
static int set_led_pgm(struct programmer_t * pgm, int value)
{
	return set_pin(pgm, PIN_LED_PGM, value);
}

static int set_led_rdy(struct programmer_t * pgm, int value)
{
	return set_pin(pgm, PIN_LED_RDY, value);
}

static int set_led_err(struct programmer_t * pgm, int value)
{
	return set_pin(pgm, PIN_LED_ERR, value);
}

static int set_led_vfy(struct programmer_t * pgm, int value)
{
	return set_pin(pgm, PIN_LED_VFY, value);
}

static int avrftdi_transmit(avrftdi_t* pdata, unsigned char mode, unsigned char *cmd,
			    unsigned char *data, int buf_size)
{
	int k = 0;
	int n;
	unsigned char buf[4 + buf_size];

	if (mode & MPSSE_DO_WRITE) {
		buf[0] = mode | MPSSE_WRITE_NEG;
		buf[1] = ((buf_size - 1) & 0xff);
		buf[2] = (((buf_size - 1) >> 8) & 0xff);

		memcpy(buf + 3, cmd, buf_size);
		buf[buf_size + 3] = 0x87;

#ifndef DRYRUN
		E(ftdi_write_data(pdata->ftdic, buf, buf_size + 4) != buf_size + 4, pdata->ftdic);
#endif
	}

	if (mode & MPSSE_DO_READ) {
		memset(buf, 0, sizeof(buf));
		do {
#ifndef DRYRUN
			n = ftdi_read_data(pdata->ftdic, buf + k, buf_size - k);
			E(n < 0, pdata->ftdic);
#else
			n = buf_size - k;
#endif
			k += n;
		} while (k < buf_size);

		memcpy(data, buf, buf_size);
	}

	return k;
}

static int write_flush(avrftdi_t* pdata)
{
	unsigned char buf[6];

	avrftdi_print(2, 
			"%s info: direction: 0x%04x, value: 0x%04x\n",
			progname, pdata->pin_direction, pdata->pin_value);

	buf[0] = SET_BITS_LOW;
	buf[1] = (pdata->pin_value) & 0xff;
	buf[2] = (pdata->pin_direction) & 0xff;
	buf[3] = SET_BITS_HIGH;
	buf[4] = ((pdata->pin_value) >> 8) & 0xff;
	buf[5] = ((pdata->pin_direction) >> 8) & 0xff;

#ifndef DRYRUN
	E(ftdi_write_data(pdata->ftdic, buf, 6) != 6, pdata->ftdic);

#endif

	avrftdi_print(3, "FTDI LOG: %02x %02x %02x %02x %02x %02x\n",
		       buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	/* we need to flush here, because set_pin is used as reset.
	 * if we want to sleep reset periods, we must be certain the
	 * avr has got the reset signal when we start sleeping.
	 * (it may be stuck in the USB stack or some USB hub)
	 *
	 * Add.: purge does NOT flush. It clears. Also, it is unkown, when the purge
	 * command actually arrives at the chip.
	 * Use read-pin-status command as sync.
	 */
#ifndef DRYRUN
	//E(ftdi_usb_purge_buffers(pdata->ftdic), pdata->ftdic);

	unsigned char cmd[] = { GET_BITS_LOW, SEND_IMMEDIATE };
	unsigned int n;
	int num = 0;
	E(ftdi_write_data(pdata->ftdic, cmd, sizeof(cmd)) != sizeof(cmd), pdata->ftdic);
	do
	{
		n = ftdi_read_data(pdata->ftdic, cmd, 1);
		E(n < 0, pdata->ftdic);
	} while(n < 1);
	
#endif

	return 0;
}


static int avrftdi_open(PROGRAMMER * pgm, char *port)
{
	int vid, pid, interface, snfound;
	char serial[255], *foundsn;
	struct ftdi_device_list* devlist;
	struct ftdi_device_list* devlist_ptr;
	struct usb_device *found_dev;
	
	avrftdi_t* pdata = to_pdata(pgm);

	/************************
	 * parameter validation *
	 ************************/

	/* use vid/pid in following priority: config,
	 * defaults. cmd-line is currently not supported */
	snfound = 0;
	foundsn = NULL;
	memset(serial, 0, sizeof(serial));

	if (pgm->usbvid)
		vid = pgm->usbvid;
	else
		vid = 0x0403;

	if (pgm->usbpid)
		pid = pgm->usbpid;
	else
		pid = 0x6010;

	if (0 == pgm->usbsn[0]) /* we don't care about SN. Use first avail. */
		snfound = 1;

	if (pgm->usbdev[0] == 'a' || pgm->usbdev[0] == 'A')
		interface = INTERFACE_A;
	else if (pgm->usbdev[0] == 'b' || pgm->usbdev[0] == 'B')
		interface = INTERFACE_B;
	else {
		fprintf(stderr,
			"%s: Invalid interface '%s'. Setting to Interface A\n",
			progname, pgm->usbdev);
		interface = INTERFACE_A;
	}

	/**************
	 * USB lookup *
	 **************/

#ifndef DRYRUN
	found_dev = NULL;
	if (ftdi_usb_find_all(pdata->ftdic, &devlist, vid, pid)) {
		devlist_ptr = devlist;
		do {
			ftdi_usb_get_strings(pdata->ftdic, devlist_ptr->dev,
					     NULL, 0, NULL, 0, serial, sizeof(serial));

			avrftdi_print(1, "%s: device: %s, serial number: %s type 0x%04x found\n",
					progname, devlist_ptr->dev->filename,	serial,
					devlist_ptr->dev->descriptor.bcdDevice);

			if (!snfound) {
				if (strcmp(pgm->usbsn, serial) == 0){
					foundsn = strdup(serial);
					snfound = 1;
					found_dev = devlist_ptr->dev;
				}
			}else {
				if (NULL == found_dev)
					found_dev = devlist_ptr->dev;
				if (NULL == foundsn)
					foundsn = strdup(serial);
			}
			memset(serial, 0, 255);
			devlist_ptr = devlist_ptr->next;
		} while (devlist_ptr);
		ftdi_list_free(&devlist);
	} else {
		fprintf(stderr,
			"%s: No devices with Vendor-ID:Product-ID %04x:%04x found.\n",
			progname, vid, pid);
		ftdi_list_free(&devlist);
		return -1;
	}
	if (!snfound) {
		fprintf(stderr,
			"%s: No devices with VID:PID %04x:%04x and SN '%s' found.\n",
			progname, vid, pid, pgm->usbsn);
		return -1;
	}

	avrftdi_print(1,
			"%s: Using device VID:PID %04x:%04x and SN '%s' on interface %c.\n",
			progname, vid, pid, foundsn, INTERFACE_A == interface? 'A': 'B');
	
	free(foundsn);
#endif

	/****************
	 * Device setup *
	 ****************/

	E(ftdi_set_interface(pdata->ftdic, interface) < 0, pdata->ftdic);
	E(ftdi_usb_open_dev(pdata->ftdic,found_dev) <0, pdata->ftdic);
	E(ftdi_usb_reset(pdata->ftdic) < 0, pdata->ftdic);
	ftdi_set_latency_timer(pdata->ftdic, 1);

#ifndef DRYRUN
	/* set SPI mode */
	E(ftdi_set_bitmode(pdata->ftdic, 0, BITMODE_RESET) < 0, pdata->ftdic);
	E(ftdi_set_bitmode(pdata->ftdic, pdata->pin_direction & 0xff, BITMODE_MPSSE) < 0, pdata->ftdic);
	E(ftdi_usb_purge_buffers(pdata->ftdic), pdata->ftdic);

#endif

	if (pgm->baudrate) {
		set_frequency(pdata, pgm->baudrate);
	} else if(pgm->bitclock) {
		set_frequency(pdata, (uint32_t)(1.0f/pgm->bitclock));
	} else {
		set_frequency(pdata, pgm->baudrate ? pgm->baudrate : 150000);
	}

	/*************
	 * pin setup *
	 *************/

	if ( FTDI_SCK != pgm->pinno[PIN_AVR_SCK]
		|| FTDI_MOSI != pgm->pinno[PIN_AVR_MOSI]
		|| FTDI_MISO != pgm->pinno[PIN_AVR_MISO])
	{
		fprintf(stderr, "%s failure: pinning for FTDI MPSSE must be:\n", progname);
		fprintf(stderr, "\t%s: 1, %s: 2, %s: 3(is: %d,%d,%d)\n",
			avr_pin_name(PIN_AVR_SCK), avr_pin_name(PIN_AVR_MOSI),
			avr_pin_name(PIN_AVR_MISO), pgm->pinno[PIN_AVR_SCK],
			pgm->pinno[PIN_AVR_MOSI],	pgm->pinno[PIN_AVR_MISO]);

		fprintf(stderr, "Setting pins accordingly ...\n");
			pgm->pinno[PIN_AVR_SCK] = FTDI_SCK;
			pgm->pinno[PIN_AVR_MOSI] = FTDI_MOSI;
			pgm->pinno[PIN_AVR_MISO] = FTDI_MISO;
	}
	
	avrftdi_print(1, "reset pin value: %x\n", pgm->pinno[PIN_AVR_RESET]-1);

	if ( pgm->pinno[PIN_AVR_RESET] < FTDI_RESET
		|| pgm->pinno[PIN_AVR_RESET] == 0)
	{
		fprintf(stderr,
			"%s failure: RESET pin clashes with data pin or is not set.\n",
			progname);
		fprintf(stderr, "Setting to default-value 4\n");
		pgm->pinno[PIN_AVR_RESET] = FTDI_RESET;
	}
	
	//pdata->pin_direction = (0x3 | (1 << (pgm->pinno[PIN_AVR_RESET] - 1)));

	/* set pin limit depending on chip type */
	switch(pdata->ftdic->type) {
#if 0
		//TODO: issue an error - no MPSSE. hint the user to syncbb?
		case TYPE_AM:
		case TYPE_BM:
		case TYPE_R:
#endif
		case TYPE_2232C:
			pdata->pin_limit = 11;
			break;
		case TYPE_2232H:
#ifdef HAVE_LIBFTDI_TYPE_232H
		case TYPE_232H:
#endif
			pdata->pin_limit = 15;
			break;
		case TYPE_4232H:
			pdata->pin_limit = 7;
			break;
		default:
		//TODO: error/unsupported device
			break;
	}
	
	/* add SCK, MOSI and RESET as output pins - MISO needs no configuration */
	if (add_pin(pgm, PIN_AVR_SCK)) return -1;
	if (add_pin(pgm, PIN_AVR_MOSI)) return -1;
	if (add_pin(pgm, PIN_AVR_RESET)) return -1;


	/* gather the rest of the pins */
	if (add_pins(pgm, PPI_AVR_VCC)) return -1;
	if (add_pins(pgm, PPI_AVR_BUFF)) return -1;
	if (add_pin(pgm, PIN_LED_ERR)) return -1;
	if (add_pin(pgm, PIN_LED_RDY)) return -1;
	if (add_pin(pgm, PIN_LED_PGM)) return -1;
	if (add_pin(pgm, PIN_LED_VFY)) return -1;

	avrftdi_print(1, "pin direction mask: %04x\n", pdata->pin_direction);
	avrftdi_print(1, "pin value mask: %04x\n", pdata->pin_value);

	/**********************************************
	 * set the ready LED and set our direction up *
	 **********************************************/

	set_led_rdy(pgm,0);
	set_led_pgm(pgm,1);

	return 0;
}

static void avrftdi_close(PROGRAMMER * pgm)
{
	avrftdi_t* pdata = to_pdata(pgm);

	if(pdata->ftdic->usb_dev) {
		set_pins(pgm, PPI_AVR_BUFF, ON);
		set_pin(pgm, PIN_AVR_RESET, ON);
		/**Stop driving the pins - except for the LEDs */
		
		avrftdi_print(1, "LED Mask=0x%04x value =0x%04x &=0x%04x\n",
				pdata->led_mask, pdata->pin_value, pdata->led_mask & pdata->pin_value);
		
		pdata->pin_direction = pdata->led_mask;
		pdata->pin_value &= pdata->led_mask;
		write_flush(pdata);
#ifndef DRYRUN
		/* reset state recommended by FTDI */
		ftdi_set_bitmode(pdata->ftdic, 0, BITMODE_RESET);
		E_VOID(ftdi_usb_close(pdata->ftdic), pdata->ftdic);
#endif
	}

	return;
}

static int avrftdi_initialize(PROGRAMMER * pgm, AVRPART * p)
{
	avrftdi_t* pdata = to_pdata(pgm);

	set_pin(pgm, PIN_AVR_RESET, OFF);
	set_pins(pgm, PPI_AVR_BUFF, OFF);
	set_pin(pgm, PIN_AVR_SCK, OFF);
	/*use speed optimization with CAUTION*/
	usleep(20 * 1000);

	/* giving rst-pulse of at least 2 avr-clock-cycles, for
	 * security (2us @ 1MHz) */
	set_pin(pgm, PIN_AVR_RESET, ON);
	usleep(20 * 1000);

	/*setting rst back to 0 */
	set_pin(pgm, PIN_AVR_RESET, OFF);
	/*wait at least 20ms bevor issuing spi commands to avr */
	usleep(20 * 1000);

	return pgm->program_enable(pgm, p);
}

static void avrftdi_disable(PROGRAMMER * pgm)
{
	return;
}

static void avrftdi_enable(PROGRAMMER * pgm)
{
	return;
}

static void avrftdi_display(PROGRAMMER * pgm, const char *p)
{
	return;
}


static int avrftdi_cmd(PROGRAMMER * pgm, unsigned char cmd[4], unsigned char res[4])
{
	/* Do not use 'sizeof(cmd)'. => message from cppcheck:
	   Using sizeof for array given as function argument returns the size of pointer. */
	return avrftdi_transmit(to_pdata(pgm), MPSSE_DO_READ | MPSSE_DO_WRITE, cmd, res, 4);
}


static int avrftdi_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
	int i;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));

	if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
		fprintf(stderr,
			"%s failure: Program Enable (PGM_ENABLE) command not defined for %s\n",
			progname, p->desc);
		return -1;
	}

	avr_set_bits(p->op[AVR_OP_PGM_ENABLE], buf);

	for(i = 0; i < 4; i++) {
		pgm->cmd(pgm, buf, buf);
		if (buf[p->pollindex-1] != p->pollvalue) {
			//try resetting
			set_pin(pgm, PIN_AVR_RESET, ON);
			usleep(20);
			set_pin(pgm, PIN_AVR_RESET, OFF);
			avr_set_bits(p->op[AVR_OP_PGM_ENABLE], buf);
		} else
			return 0;
	}
#ifndef DRYRUN
	return -1;
#else
	return 0;
#endif
}


static int avrftdi_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
	unsigned char cmd[4];
	unsigned char res[4];

	if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
		fprintf(stderr,
			"%s failure Chip Erase (CHIP_ERASE) command not defined for %s\n",
			progname, p->desc);
		return -1;
	}

	memset(cmd, 0, sizeof(cmd));

	avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
	pgm->cmd(pgm, cmd, res);
	usleep(p->chip_erase_delay);
	pgm->initialize(pgm, p);

	return 0;
}


/* Load extended address byte command */
static int
avrftdi_lext(avrftdi_t* pdata, AVRPART *p, AVRMEM *m, unsigned int address)
{
	unsigned char buf[] =
			{ MPSSE_DO_WRITE | MPSSE_WRITE_NEG, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };

	avr_set_bits(m->op[AVR_OP_LOAD_EXT_ADDR], &buf[3]);
	avr_set_addr(m->op[AVR_OP_LOAD_EXT_ADDR], &buf[3], address);

	if(verbose > 1)
		buf_dump(buf, sizeof(buf),
			 "load extended address command", 0, 16 * 3);

#ifndef DRYRUN
	E(ftdi_write_data(pdata->ftdic, buf, sizeof(buf)) != sizeof(buf),
			pdata->ftdic);
#endif
	return 0;
}

static int avrftdi_eeprom_write(PROGRAMMER *pgm, AVRPART *p, AVRMEM *m,
		unsigned int page_size, unsigned int addr, unsigned int len)
{
	unsigned char cmd[] =
			{ MPSSE_DO_WRITE | MPSSE_WRITE_NEG, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
	unsigned char *data = &m->buf[addr];
	unsigned int add;

	avr_set_bits(m->op[AVR_OP_WRITE], &cmd[3]);

	for (add = addr; add < addr + len; add++)
	{
		avr_set_addr(m->op[AVR_OP_WRITE], &cmd[3], add);
		avr_set_input(m->op[AVR_OP_WRITE], &cmd[3], *data++);

		E(ftdi_write_data(to_pdata(pgm)->ftdic, cmd, sizeof(cmd)) != sizeof(cmd),
				to_pdata(pgm)->ftdic);

		usleep((m->max_write_delay));
	}
	return len;
}

static int avrftdi_eeprom_read(PROGRAMMER *pgm, AVRPART *p, AVRMEM *m,
		unsigned int page_size, unsigned int addr, unsigned int len)
{
	unsigned char cmd[4];
	unsigned char buffer[len], *bufptr = buffer;
	unsigned int add;

	memset(buffer, 0, sizeof(buffer));
	for (add = addr; add < addr + len; add++)
	{
		memset(cmd, 0, sizeof(cmd));
		avr_set_bits(m->op[AVR_OP_READ], cmd);
		avr_set_addr(m->op[AVR_OP_READ], cmd, add);

		avrftdi_transmit(to_pdata(pgm), MPSSE_DO_READ | MPSSE_DO_WRITE, cmd, cmd, 4);

		avr_get_output(m->op[AVR_OP_READ], cmd, bufptr++);
	}

	memcpy(m->buf + addr, buffer, len);
	return len;
}

static int avrftdi_flash_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
		unsigned int page_size, unsigned int addr, unsigned int len)
{
	int use_lext_address = m->op[AVR_OP_LOAD_EXT_ADDR] != NULL;
	
	unsigned int word;
	unsigned int poll_index;
	unsigned int buf_size;

	unsigned char poll_byte;
	unsigned char *buffer = &m->buf[addr];
	unsigned char buf[4*len+4], *bufptr = buf;

	memset(buf, 0, sizeof(buf));

	/* pre-check opcodes */
	if (m->op[AVR_OP_LOADPAGE_LO] == NULL) {
		fprintf(stderr,
			"%s failure: %s command not defined for %s\n",
			progname, "AVR_OP_LOADPAGE_LO", p->desc);
		return -1;
	}
	if (m->op[AVR_OP_LOADPAGE_HI] == NULL) {
		fprintf(stderr,
			"%s failure: %s command not defined for %s\n",
			progname, "AVR_OP_LOADPAGE_HI", p->desc);
		return -1;
	}

	if(page_size != m->page_size) {
		fprintf(stderr,
			"%s: Something funny is going on. Parameter"
			"page_size is %d, buf m->page_size is %d. Using"
			"the latter.\n", progname, page_size, m->page_size);
	}

	page_size = m->page_size;

	/* if we do cross a 64k word boundary (or write the
	 * first page), we need to issue a 'load extended
	 * address byte' command, which is defined as 0x4d
	 * 0x00 <address byte> 0x00.  As far as i know, this
	 * is only available on 256k parts.  64k word is 128k
	 * bytes.
	 * write the command only once.
	 */
	if(use_lext_address && (((addr/2) & 0xffff0000))) {
		avrftdi_lext(to_pdata(pgm), p, m, addr/2);
	}
	
	/* prepare the command stream for the whole page */
	/* addr is in bytes, but we program in words. addr/2 should be something
	 * like addr >> WORD_SHIFT, though */
	for(word = addr/2; word < (len + addr)/2; word++)
	{
		avrftdi_print(2, "-< bytes = %d of %d\n", word * 2, len + addr);

		/*setting word*/
		avr_set_bits(m->op[AVR_OP_LOADPAGE_LO], bufptr);
		/* here is the second byte increment, just if you're wondering */
		avr_set_addr(m->op[AVR_OP_LOADPAGE_LO], bufptr, word);
		avr_set_input(m->op[AVR_OP_LOADPAGE_LO], bufptr, *buffer++);
		bufptr += 4;
		avr_set_bits(m->op[AVR_OP_LOADPAGE_HI], bufptr);
		avr_set_addr(m->op[AVR_OP_LOADPAGE_HI], bufptr, word);
		avr_set_input(m->op[AVR_OP_LOADPAGE_HI], bufptr, *buffer++);
		bufptr += 4;
	}

	/* issue write page command, if available */
	if (m->op[AVR_OP_WRITEPAGE] == NULL) {
		fprintf(stderr,
			"%s failure: Write Page (WRITEPAGE) command not defined for %s\n",
			progname, p->desc);
		//FIXME: maybe not exit but return error code
		exit(1);
	} else {
		avr_set_bits(m->op[AVR_OP_WRITEPAGE], bufptr);
		/* setting page address highbyte */
		avr_set_addr(m->op[AVR_OP_WRITEPAGE],
					 bufptr, addr/2);
		bufptr += 4;
	}

	buf_size = bufptr - buf;

	if(verbose > 3)
		buf_dump(buf, buf_size, "command buffer", 0, 16*2);

	avrftdi_print(2, "%s info: buffer size: %d\n", progname, buf_size);

	avrftdi_transmit(to_pdata(pgm), MPSSE_DO_WRITE, buf, buf, buf_size);

	bufptr = buf;
	/* find a poll byte. we cannot poll a value of 0xff, so look
	 * for a value != 0xff
	 */
	for(poll_index = addr+len-1; poll_index > addr-1; poll_index--)
		if(m->buf[poll_index] != 0xff)
			break;

	if((poll_index < addr + len) && m->buf[poll_index] != 0xff)
	{
		avrftdi_print(2, "%s: using m->buf[%d] = 0x%02x as polling value ",
				progname, poll_index, m->buf[poll_index]);
		/* poll page write ready */
		do {
			avrftdi_print(2, ".");

			pgm->read_byte(pgm, p, m, poll_index, &poll_byte);
		} while (m->buf[poll_index] != poll_byte);

		avrftdi_print(2, "\n");
	}
	else
	{
		fprintf(stderr,	"%s: no suitable byte (!=0xff) for polling found.\n", progname);
		fprintf(stderr, "%s: trying to sleep, but programming errors may occur.\n", progname);
		fprintf(stderr, "%s: be sure to verify programmed memory (no -V option)\n", progname);
		/* TODO sync write */
		/* sleep */
		usleep((m->max_write_delay));
	}
	
	return len;
}

/*
 *Reading from flash
 */
static int avrftdi_flash_read(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
		unsigned int page_size, unsigned int addr, unsigned int len)
{
	OPCODE * readop;
	int byte, word;
	int use_lext_address = m->op[AVR_OP_LOAD_EXT_ADDR] != NULL;
	unsigned int address = addr/2;

	unsigned char o_buf[4*len+4];
	unsigned char i_buf[4*len+4];
	unsigned int index;

	memset(o_buf, 0, sizeof(o_buf));
	memset(i_buf, 0, sizeof(i_buf));

	/* pre-check opcodes */
	if (m->op[AVR_OP_READ_LO] == NULL) {
		fprintf(stderr,
			"%s failure: %s command not defined for %s\n",
			progname, "AVR_OP_READ_LO", p->desc);
		return -1;
	}
	if (m->op[AVR_OP_READ_HI] == NULL) {
		fprintf(stderr,
			"%s failure: %s command not defined for %s\n",
			progname, "AVR_OP_READ_HI", p->desc);
		return -1;
	}
	
	if(use_lext_address && ((address & 0xffff0000))) {
		avrftdi_lext(to_pdata(pgm), p, m, address);
	}
	
	/* word addressing! */
	for(word = addr/2, index = 0; word < (addr + len)/2; word++)
	{
		/* one byte is transferred via a 4-byte opcode.
		 * TODO: reduce magic numbers
		 */
		avr_set_bits(m->op[AVR_OP_READ_LO], &o_buf[index*4]);
		avr_set_addr(m->op[AVR_OP_READ_LO], &o_buf[index*4], word);
		index++;
		avr_set_bits(m->op[AVR_OP_READ_HI], &o_buf[index*4]);
		avr_set_addr(m->op[AVR_OP_READ_HI], &o_buf[index*4], word);
		index++;
	}

	/* transmit,
	 * if there was an error, we did not see, memory validation will
	 * subsequently fail.
	 */
	if(verbose > 2) {
		buf_dump(o_buf, sizeof(o_buf), "o_buf", 0, 32);
	}

	avrftdi_transmit(to_pdata(pgm), MPSSE_DO_READ | MPSSE_DO_WRITE, o_buf, i_buf, len * 4);
				
	if(verbose > 2) {
		buf_dump(i_buf, sizeof(i_buf), "i_buf", 0, 32);
	}

	memset(&m->buf[addr], 0, page_size);

	/* every (read) op is 4 bytes in size and yields one byte of memory data */
	for(byte = 0; byte < page_size; byte++) {
		if(byte & 1)
			readop = m->op[AVR_OP_READ_HI];
		else
			readop = m->op[AVR_OP_READ_LO];
		
		/* take 4 bytes and put the memory byte in the buffer at
		 * offset addr + offset of the current byte
		 */
		avr_get_output(readop, &i_buf[byte*4], &m->buf[addr+byte]);
	}
	
	if(verbose > 2)
		buf_dump(&m->buf[addr], page_size, "page:", 0, 32);

	return len;
}

static int avrftdi_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
		unsigned int page_size, unsigned int addr, unsigned int n_bytes)
{
	if (strcmp(m->desc, "flash") == 0)
		return avrftdi_flash_write(pgm, p, m, page_size, addr, n_bytes);
	else if (strcmp(m->desc, "eeprom") == 0)
		return avrftdi_eeprom_write(pgm, p, m, page_size, addr, n_bytes);
	else
		return -2;
}

static int avrftdi_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
		unsigned int page_size, unsigned int addr, unsigned int n_bytes)
{
	if (strcmp(m->desc, "flash") == 0)
		return avrftdi_flash_read(pgm, p, m, page_size, addr, n_bytes);
	else if(strcmp(m->desc, "eeprom") == 0)
		return avrftdi_eeprom_read(pgm, p, m, page_size, addr, n_bytes);
	else
		return -2;
}

static void
avrftdi_setup(PROGRAMMER * pgm)
{
	avrftdi_t* pdata;

	pgm->cookie = malloc(sizeof(avrftdi_t));
	pdata = to_pdata(pgm);

	#ifndef DRYRUN
	pdata->ftdic = ftdi_new();
	if(!pdata->ftdic)
	{
		fprintf(stderr, "%s: Error allocating memory.\n", progname);
		exit(-ENOMEM);
	}
	E_VOID(ftdi_init(pdata->ftdic), pdata->ftdic);
	#endif

	pdata->pin_value = 0;
	pdata->pin_direction = 0;
	pdata->led_mask = 0;
}

static void
avrftdi_teardown(PROGRAMMER * pgm)
{
	avrftdi_t* pdata = to_pdata(pgm);

#ifndef DRYRUN
	ftdi_deinit(pdata->ftdic);
	ftdi_free(pdata->ftdic);
#endif

	free(pdata);
}

void avrftdi_initpgm(PROGRAMMER * pgm)
{

	strcpy(pgm->type, "avrftdi");

	/*
	 * mandatory functions
	 */

	pgm->initialize = avrftdi_initialize;
	pgm->display = avrftdi_display;
	pgm->enable = avrftdi_enable;
	pgm->disable = avrftdi_disable;
	pgm->program_enable = avrftdi_program_enable;
	pgm->chip_erase = avrftdi_chip_erase;
	pgm->cmd = avrftdi_cmd;
	pgm->open = avrftdi_open;
	pgm->close = avrftdi_close;
	pgm->read_byte = avr_read_byte_default;
	pgm->write_byte = avr_write_byte_default;

	/*
	 * optional functions
	 */

	pgm->paged_write = avrftdi_paged_write;
	pgm->paged_load = avrftdi_paged_load;

	pgm->setup = avrftdi_setup;
	pgm->teardown = avrftdi_teardown;

	pgm->rdy_led = set_led_rdy;
	pgm->err_led = set_led_err;
	pgm->pgm_led = set_led_pgm;
	pgm->vfy_led = set_led_vfy;
}

#else /*HAVE_LIBFTDI*/

static int avrftdi_noftdi_open (struct programmer_t *pgm, char * name)
{
	fprintf(stderr,
		"%s: error: no libftdi support. please compile again with libftdi installed.\n",
		progname);

	exit(1);
}

void avrftdi_initpgm(PROGRAMMER * pgm)
{
	strcpy(pgm->type, "avrftdi");
	pgm->open = avrftdi_noftdi_open;
}

#endif  /* HAVE_LIBFTDI */

#else /*HAVE_LIBUSB*/

static int avrftdi_nousb_open (struct programmer_t *pgm, char * name)
{
	fprintf(stderr,
		"%s: error: no usb support. please compile again with libusb installed.\n",
		progname);

	exit(1);
}

void avrftdi_initpgm(PROGRAMMER * pgm)
{
	strcpy(pgm->type, "avrftdi");
	pgm->open = avrftdi_nousb_open;
}

#endif /*HAVE_LIBUSB*/

const char avrftdi_desc[] = "Interface to the MPSSE Engine of FTDI Chips using libftdi.";

