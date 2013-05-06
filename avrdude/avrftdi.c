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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
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
#include "pindefs.h"
#include "avrftdi.h"
#include "avrpart.h"
#include "avrftdi_tpi.h"
#include "avrftdi_private.h"

#ifdef HAVE_LIBUSB_1_0
#ifdef HAVE_LIBFTDI1

#include <libusb-1.0/libusb.h>
#include <libftdi1/ftdi.h>

enum { FTDI_SCK = 0, FTDI_MOSI, FTDI_MISO, FTDI_RESET };

/* This is for running the code without having a FTDI-device.
 * The generated code is useless! For debugging purposes only.
 * This should never be defined, unless you know what you are
 * doing.
 * If you think you know what you are doing: YOU DONT!
 */
//#define DRYRUN

static int write_flush(avrftdi_t *);

/*
 * returns a human-readable name for a pin number. the name should match with
 * the pin names used in FTDI datasheets.
 */
static char*
ftdi_pin_name(avrftdi_t* pdata, struct pindef_t pin)
{
	static char str[128];

	char interface = '@';

	/* INTERFACE_ANY is zero, so @ is used
	 * INTERFACE_A is one, so '@' + 1 = 'A'
	 * and so forth ...
	 * be aware, there is an 'interface' member in ftdi_context,
	 * however, we really want the 'index' member here.
	 */
	interface += pdata->ftdic->index;

	int pinno;
	int n = 0;
	int mask = pin.mask[0];

	const char * fmt;

	str[0] = 0;

	for(pinno = 0; mask; mask >>= 1, pinno++) {
		if(!(mask & 1))
			continue;

		int chars = 0;

		char port;
		/* This is FTDI's naming scheme.
		 * probably 'D' is for data and 'C' for control
		 */
		if(pinno < 8)
			port = 'D';
		else
			port = 'C';

		if(str[0] == 0)
			fmt = "%c%cBUS%d%n";
		else
			fmt = ", %c%cBUS%d%n";

		snprintf(&str[n], sizeof(str) - n, fmt, interface, port, pinno, &chars);
		n += chars;
	}

	return str;
}

/*
 * output function, to save if(vebose>level)-constructs. also prefixes output
 * with "avrftdi function-name(line-number):" to identify were messages came
 * from.
 * This function is the backend of the log_*-macros, but it can be used
 * directly.
 */
void avrftdi_log(int level, const char * func, int line,
		const char * fmt, ...) {
	static int skip_prefix = 0;
	const char *p = fmt;
	va_list ap;

	if(verbose >= level)
	{
		if(!skip_prefix)
		{
			switch(level) {
				case ERR: fprintf(stderr, "E "); break;
				case WARN:  fprintf(stderr, "W "); break;
				case INFO:  fprintf(stderr, "I "); break;
				case DEBUG: fprintf(stderr, "D "); break;
				case TRACE: fprintf(stderr, "T "); break;
				default: fprintf(stderr, "  ");
			}
			fprintf(stderr, "%s(%d): ", func, line);
		}
		va_start(ap, fmt);
		vfprintf(stderr, fmt, ap);
		va_end(ap);
	}

	skip_prefix = 1;
	while(*p++)
		if(*p == '\n' && !(*(p+1)))
			skip_prefix = 0;
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
		log_warn("Frequency too high (%u > 6 MHz)\n", freq);
		log_warn("Resetting Frequency to 6MHz\n");
		divisor = 0;
	}

	if (divisor > 65535) {
		log_warn("Frequency too low (%u < 91.553 Hz)\n", freq);
		log_warn("Resetting Frequency to 91.553Hz\n");
		divisor = 65535;
	}

	log_info("Using frequency: %d\n", 6000000/(divisor+1));
	log_info("Clock divisor: 0x%04x\n", divisor);

	buf[0] = TCK_DIVISOR;
	buf[1] = (uint8_t)(divisor & 0xff);
	buf[2] = (uint8_t)((divisor >> 8) & 0xff);

#ifndef DRYRUN
	E(ftdi_write_data(ftdi->ftdic, buf, 3) < 0, ftdi->ftdic);
#endif

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
	avrftdi_t* pdata = to_pdata(pgm);
	struct pindef_t pin = pgm->pin[pinfunc];
	
	if (pins_check(pgm, pdata->pin_checklist, N_PINS - 1)) {
		/* this error message is really annoying, maybe use a ratelimit? */
	/*
		avrftdi_print(2, "%s info: Pin is zero, value: %d!\n",
				progname, value);
	*/
		return -1;
	
	}

	log_debug("Setting pin %s (%s) as %s: %s (%s active)\n",
	          pinmask_to_str(pin.mask), ftdi_pin_name(pdata, pin),
						avr_pin_name(pinfunc),
	          (value) ? "high" : "low", (pin.inverse[0]) ? "low" : "high");

	pdata->pin_value = SET_BITS_0(pdata->pin_value, pgm, pinfunc, value);

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

/* Send 'buf_size' bytes from 'cmd' to device and return data from device in
 * buffer 'data'.
 * Write is only performed when mode contains MPSSE_DO_WRITE.
 * Read is only performed when mode contains MPSSE_DO_WRITE and MPSSE_DO_READ.
 */
static int avrftdi_transmit(avrftdi_t* pdata, unsigned char mode, unsigned char *buf,
			    unsigned char *data, int buf_size)
{
	size_t blocksize;
	size_t remaining = buf_size;
	size_t written = 0;
	
	unsigned char cmd[3];
//	unsigned char si = SEND_IMMEDIATE;

	cmd[0] = mode | MPSSE_WRITE_NEG;
	cmd[1] = ((buf_size - 1) & 0xff);
	cmd[2] = (((buf_size - 1) >> 8) & 0xff);

	//if we are not reading back, we can just write the data out
	if(!(mode & MPSSE_DO_READ))
		blocksize = buf_size;
	else
		blocksize = pdata->rx_buffer_size;

#ifndef DRYRUN
	E(ftdi_write_data(pdata->ftdic, cmd, sizeof(cmd)) != sizeof(cmd), pdata->ftdic);
#endif

	while(remaining)
	{
		size_t transfer_size = (remaining > blocksize) ? blocksize : remaining;

#ifndef DRYRUN
		E(ftdi_write_data(pdata->ftdic, &buf[written], transfer_size) != transfer_size, pdata->ftdic);
#endif
#if 0
		if(remaining < blocksize)
			E(ftdi_write_data(pdata->ftdic, &si, sizeof(si)) != sizeof(si), pdata->ftdic);
#endif

		if (mode & MPSSE_DO_READ) {
			int n;
			int k = 0;
			do {
	#ifndef DRYRUN
				n = ftdi_read_data(pdata->ftdic, &data[written + k], transfer_size - k);
				E(n < 0, pdata->ftdic);
	#else
				n = transfer_size - k;
	#endif
				k += n;
			} while (k < transfer_size);

		}
		
		written += transfer_size;
		remaining -= transfer_size;
	}
	
	return written;
}

/* this function tries to sync up with the FTDI. See FTDI application note AN_129.
 * AN_135 uses 0xab as bad command and enables/disables loopback around synchronisation.
 * This may fail if data is left in the buffer (i.e. avrdude aborted with ctrl-c)
 * or the device is in an illegal state (i.e. a previous program).
 * If the FTDI is out of sync, the buffers are cleared ("purged") and the
 * sync is re-tried.
 * if it still fails, we return an error code. higher level code may than abort.
 * the device may be reset by unplugging the device and plugging it back in.
 * resetting the device did not always help for me.
 */
static int ftdi_sync(avrftdi_t* pdata)
{
	unsigned char illegal_cmd[] = {0xaa};
	unsigned char reply[2];
	unsigned int i, n;
	unsigned int retry = 0;
	unsigned char latency;

	ftdi_get_latency_timer(pdata->ftdic, &latency);
	fprintf(stderr, "Latency: %d\n", latency);

	do{
		n = ftdi_read_data(pdata->ftdic, reply, 1);
	} while(n > 0);
retry:
	/* send command "0xaa", which is an illegal command. */
	E(ftdi_write_data(pdata->ftdic, illegal_cmd, sizeof(illegal_cmd)) != sizeof(illegal_cmd), pdata->ftdic);
	
	i = 0;
	do {
#ifndef DRYRUN
		n = ftdi_read_data(pdata->ftdic, &reply[i], sizeof(reply) - i);
		E(n < 0, pdata->ftdic);
		//fprintf(stderr, "%s\n", ftdi_get_error_string(pdata->ftdic));
#else
		n = sizeof(reply) - i;
#endif
		i += n;
	} while (i < sizeof(reply));

	/* 0xfa is return code for illegal command - we expect that, since we issued an
	 * illegal command (0xaa)
	 * the next byte will be the illegal command, the FTDI is complaining about.
	 */
	if(reply[0] == 0xfa && reply[1] == illegal_cmd[0])
	{
		/* if the FTDI is complaining about the right thing, everything is fine */
		fprintf(stderr, "FTDI is in sync.\n");
		return 0;
	}
		else
	{
		fprintf(stderr, "FTDI out of sync. Received 0x%02x 0x%02x\n", reply[0], reply[1]);
		if(retry < 4)
		{
			fprintf(stderr, "Trying to re-sync by purging buffers. Attempt\n");
			E(ftdi_usb_purge_buffers(pdata->ftdic), pdata->ftdic);
			retry++;
			goto retry;
		} else
			fprintf(stderr, "Aborting. Try resetting or unplugging the device.\n");
	}
	return -1;
}

static int write_flush(avrftdi_t* pdata)
{
	unsigned char buf[6];

	log_debug("Setting pin direction (0x%04x) and value (0x%04x)\n",
	          pdata->pin_direction, pdata->pin_value);

	buf[0] = SET_BITS_LOW;
	buf[1] = (pdata->pin_value) & 0xff;
	buf[2] = (pdata->pin_direction) & 0xff;
	buf[3] = SET_BITS_HIGH;
	buf[4] = ((pdata->pin_value) >> 8) & 0xff;
	buf[5] = ((pdata->pin_direction) >> 8) & 0xff;

#ifndef DRYRUN
	E(ftdi_write_data(pdata->ftdic, buf, 6) != 6, pdata->ftdic);

#endif

	log_trace("Set pins command: %02x %02x %02x %02x %02x %02x\n",
	          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	/* we need to flush here, because set_pin is used as reset.
	 * if we want to sleep reset periods, we must be certain the
	 * avr has got the reset signal when we start sleeping.
	 * (it may be stuck in the USB stack or some USB hub)
	 *
	 * Add.: purge does NOT flush. It clears. Also, it is unkown, when the purge
	 * command actually arrives at the chip.
	 * Use read pin status command as sync.
	 */
#ifndef DRYRUN
	//E(ftdi_usb_purge_buffers(pdata->ftdic), pdata->ftdic);

	unsigned char cmd[] = { GET_BITS_LOW, SEND_IMMEDIATE };
	E(ftdi_write_data(pdata->ftdic, cmd, sizeof(cmd)) != sizeof(cmd), pdata->ftdic);
	
	int num = 0;
	do
	{
		int n = ftdi_read_data(pdata->ftdic, buf, sizeof(buf));
		if(n > 0)
			num += n;
		E(n < 0, pdata->ftdic);
	} while(num < 1);
	
	if(num > 1)
		log_warn("Read %d extra bytes\n", num-1);
#endif

	return 0;

}

static int avrftdi_pin_setup(PROGRAMMER * pgm)
{
	/*************
	 * pin setup *
	 *************/

	int pin;

	avrftdi_t* pdata = to_pdata(pgm);

	/* SCK/MOSI/MISO are fixed and not invertable?*/
	/* TODO: inverted SCK/MISO/MOSI */
	static const struct pindef_t valid_pins_SCK  = {{0x01},{0x00}} ;
	static const struct pindef_t valid_pins_MOSI = {{0x02},{0x00}} ;
	static const struct pindef_t valid_pins_MISO = {{0x04},{0x00}} ;

	/* value for 8/12/16 bit wide interface for other pins */
	int valid_mask = ((1 << pdata->pin_limit) - 1);
	/* mask out SCK/MISO/MOSI */
	valid_mask &= ~((1 << FTDI_SCK) | (1 << FTDI_MOSI) | (1 << FTDI_MISO));

	log_debug("Using valid mask: 0x%08x\n", valid_mask);
	static struct pindef_t valid_pins_others;
	valid_pins_others.mask[0] = valid_mask;
	valid_pins_others.inverse[0] = valid_mask ;

	/* build pin checklist */
	for(pin = PPI_AVR_VCC; pin < N_PINS; ++pin) {
		/* unfortunately, the pin name enum is one-based */
		pdata->pin_checklist[pin - 1].pinname = pin;
		pdata->pin_checklist[pin - 1].mandatory = 0;
		pdata->pin_checklist[pin - 1].valid_pins = &valid_pins_others;
	}
	pdata->pin_checklist[PIN_AVR_SCK - 1].mandatory = 1;
	pdata->pin_checklist[PIN_AVR_SCK - 1].valid_pins = &valid_pins_SCK;
	pdata->pin_checklist[PIN_AVR_MOSI - 1].mandatory = 1;
	pdata->pin_checklist[PIN_AVR_MOSI - 1].valid_pins = &valid_pins_MOSI;
	pdata->pin_checklist[PIN_AVR_MISO - 1].mandatory = 1;
	pdata->pin_checklist[PIN_AVR_MISO - 1].valid_pins = &valid_pins_MISO;


	/* everything is an output, except MISO */
	for(pin = PPI_AVR_VCC; pin < N_PINS; ++pin) {
		pdata->pin_direction |= pgm->pin[pin].mask[0];
		pdata->pin_value = SET_BITS_0(pdata->pin_value, pgm, pin, OFF);
	}
	pdata->pin_direction &= ~pgm->pin[PIN_AVR_MISO].mask[0];

	for(pin = PIN_LED_ERR; pin < N_PINS; ++pin) {
		pdata->led_mask |= pgm->pin[pin].mask[0];
	}

	/* assumes all checklists above have same number of entries */
	if (pins_check(pgm, pdata->pin_checklist,N_PINS - 1)) {
		log_err("Pin configuration for FTDI MPSSE must be:\n");
		log_err("%s: 0, %s: 1, %s: 2 (is: %s, %s, %s)\n", avr_pin_name(PIN_AVR_SCK),
		         avr_pin_name(PIN_AVR_MOSI), avr_pin_name(PIN_AVR_MISO),
						 pins_to_str(&pgm->pin[PIN_AVR_SCK]),
						 pins_to_str(&pgm->pin[PIN_AVR_MOSI]),
						 pins_to_str(&pgm->pin[PIN_AVR_MISO]));
		log_err("Please correct your cabling and/or configuration.\n");
		log_err("If your hardware is fixed, consider using a bitbang programmer.\n");

		return -1;
	}

	/*
	 * TODO: No need to fail for a wrongly configured led or something.
	 * Maybe we should only fail for SCK; MISO, MOSI, RST (and probably
	 * VCC and BUFF).
	 */

	log_info("Pin direction mask: %04x\n", pdata->pin_direction);
	log_info("Pin value mask: %04x\n", pdata->pin_value);

	return 0;
}

static int avrftdi_open(PROGRAMMER * pgm, char *port)
{
	int vid, pid, interface, index, err;
	char * serial, *desc;
	
	avrftdi_t* pdata = to_pdata(pgm);

	/************************
	 * parameter validation *
	 ************************/

	/* use vid/pid in following priority: config,
	 * defaults. cmd-line is currently not supported */
	
	if (pgm->usbvid)
		vid = pgm->usbvid;
	else
		vid = 0x0403;

	if (pgm->usbpid)
		pid = pgm->usbpid;
	else
		pid = 0x6010;

	if (0 == pgm->usbsn[0]) /* we don't care about SN. Use first avail. */
		serial = NULL;
	else
		serial = pgm->usbsn;

	/* not used yet, but i put them here, just in case someone does needs or
	 * wants to implement this.
	 */
	desc = NULL;
	index = 0;

	if (pgm->usbdev[0] == 'a' || pgm->usbdev[0] == 'A')
		interface = INTERFACE_A;
	else if (pgm->usbdev[0] == 'b' || pgm->usbdev[0] == 'B')
		interface = INTERFACE_B;
	else {
		log_warn("Invalid interface '%s'. Setting to Interface A\n", pgm->usbdev);
		interface = INTERFACE_A;
	}

	/****************
	 * Device setup *
	 ****************/

	E(ftdi_set_interface(pdata->ftdic, interface) < 0, pdata->ftdic);
	
	err = ftdi_usb_open_desc_index(pdata->ftdic, vid, pid, desc, serial, index);
	if(err) {
		log_err("Error %d occured: %s\n", err, ftdi_get_error_string(pdata->ftdic));
		//stupid hack, because avrdude calls pgm->close() even when pgm->open() fails
		//and usb_dev is intialized to the last usb device from probing
		pdata->ftdic->usb_dev = NULL;
		return err;
	} else {
		log_info("Using device VID:PID %04x:%04x and SN '%s' on interface %c.\n",
		         vid, pid, serial, INTERFACE_A == interface? 'A': 'B');
	}
	
	ftdi_set_latency_timer(pdata->ftdic, 1);
	//ftdi_write_data_set_chunksize(pdata->ftdic, 16);
	//ftdi_read_data_set_chunksize(pdata->ftdic, 16);

	/* set SPI mode */
	E(ftdi_set_bitmode(pdata->ftdic, 0, BITMODE_RESET) < 0, pdata->ftdic);
	E(ftdi_set_bitmode(pdata->ftdic, pdata->pin_direction & 0xff, BITMODE_MPSSE) < 0, pdata->ftdic);
	E(ftdi_usb_purge_buffers(pdata->ftdic), pdata->ftdic);

/*
	ret = ftdi_sync(pdata);
	if(ret < 0)
		return ret;
*/
	write_flush(pdata);

	if (pgm->baudrate) {
		set_frequency(pdata, pgm->baudrate);
	} else if(pgm->bitclock) {
		set_frequency(pdata, (uint32_t)(1.0f/pgm->bitclock));
	} else {
		set_frequency(pdata, pgm->baudrate ? pgm->baudrate : 150000);
	}

	/* set pin limit depending on chip type */
	switch(pdata->ftdic->type) {
		case TYPE_AM:
		case TYPE_BM:
		case TYPE_R:
			log_err("Found unsupported device type AM, BM or R. avrftdi ");
			log_err("cannot work with your chip. Try the 'synbb' programmer.\n");
			return -1;
		case TYPE_2232C:
			pdata->pin_limit = 12;
			pdata->rx_buffer_size = 384;
			break;
		case TYPE_2232H:
			pdata->pin_limit = 16;
			pdata->rx_buffer_size = 4096;
			break;
		case TYPE_232H:
			pdata->pin_limit = 16;
			pdata->rx_buffer_size = 1024;
			break;
		case TYPE_4232H:
			pdata->pin_limit = 8;
			pdata->rx_buffer_size = 2048;
			break;
		default:
			log_warn("Found unkown device %x. I will do my ", pdata->ftdic->type);
			log_warn("best to work with it, but no guarantees ...\n");
			pdata->pin_limit = 8;
			pdata->rx_buffer_size = pdata->ftdic->max_packet_size;
			break;
	}
	
	if(avrftdi_pin_setup(pgm))
		return -1;

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
		set_pin(pgm, PPI_AVR_BUFF, ON);
		set_pin(pgm, PIN_AVR_RESET, ON);

		/* Stop driving the pins - except for the LEDs */
		log_info("LED Mask=0x%04x value =0x%04x &=0x%04x\n",
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
	if(p->flags & AVRPART_HAS_TPI)
	{
		/* see avrftdi_tpi.c */
		avrftdi_tpi_initialize(pgm, p);
	}
	else
	{
		set_pin(pgm, PIN_AVR_RESET, OFF);
		set_pin(pgm, PPI_AVR_BUFF, OFF);
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
	}

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
		log_err("AVR_OP_PGM_ENABLE command not defined for %s\n", p->desc);
		return -1;
	}

	avr_set_bits(p->op[AVR_OP_PGM_ENABLE], buf);

	for(i = 0; i < 4; i++) {
		pgm->cmd(pgm, buf, buf);
		if (buf[p->pollindex-1] != p->pollvalue) {
			log_warn("Program enable command not successful. Retrying.\n");
			set_pin(pgm, PIN_AVR_RESET, ON);
			usleep(20);
			set_pin(pgm, PIN_AVR_RESET, OFF);
			avr_set_bits(p->op[AVR_OP_PGM_ENABLE], buf);
		} else
			return 0;
	}

	log_err("Device is not responding to program enable. Check connection.\n");
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
		log_err("AVR_OP_CHIP_ERASE command not defined for %s\n", p->desc);
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

	if(verbose > TRACE)
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

		//avrftdi_transmit(to_pdata(pgm), MPSSE_DO_WRITE, cmd, cmd, 4);
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
		log_err("AVR_OP_LOADPAGE_LO command not defined for %s\n", p->desc);
		return -1;
	}
	if (m->op[AVR_OP_LOADPAGE_HI] == NULL) {
		log_err("AVR_OP_LOADPAGE_HI command not defined for %s\n", p->desc);
		return -1;
	}

	if(page_size != m->page_size) {
		log_warn("Parameter page_size is %d, ", page_size);
		log_warn("but m->page_size is %d. Using the latter.\n", m->page_size);
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
		log_debug("-< bytes = %d of %d\n", word * 2, len + addr);

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
		log_err("AVR_OP_WRITEPAGE command not defined for %s\n", p->desc);
		return -1;
	} else {
		avr_set_bits(m->op[AVR_OP_WRITEPAGE], bufptr);
		/* setting page address highbyte */
		avr_set_addr(m->op[AVR_OP_WRITEPAGE],
					 bufptr, addr/2);
		bufptr += 4;
	}

	buf_size = bufptr - buf;

	if(verbose > TRACE)
		buf_dump(buf, buf_size, "command buffer", 0, 16*2);

	log_info("Transmitting buffer of size: %d\n", buf_size);
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
		log_info("Using m->buf[%d] = 0x%02x as polling value ", poll_index,
		         m->buf[poll_index]);
		/* poll page write ready */
		do {
			log_info(".");

			pgm->read_byte(pgm, p, m, poll_index, &poll_byte);
		} while (m->buf[poll_index] != poll_byte);

		log_info("\n");
	}
	else
	{
		log_warn("No suitable byte (!=0xff) for polling found.\n");
		log_warn("Trying to sleep instead, but programming errors may occur.\n");
		log_warn("Be sure to verify programmed memory (no -V option)\n");
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
		log_err("AVR_OP_READ_LO command not defined for %s\n", p->desc);
		return -1;
	}
	if (m->op[AVR_OP_READ_HI] == NULL) {
		log_err("AVR_OP_READ_HI command not defined for %s\n", p->desc);
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
	if(verbose > TRACE) {
		buf_dump(o_buf, sizeof(o_buf), "o_buf", 0, 32);
	}

	avrftdi_transmit(to_pdata(pgm), MPSSE_DO_READ | MPSSE_DO_WRITE, o_buf, i_buf, len * 4);
				
	if(verbose > TRACE) {
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
	
	if(verbose > TRACE)
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
		log_err("Error allocating memory.\n");
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

	if(pdata) {
#ifndef DRYRUN
		ftdi_deinit(pdata->ftdic);
		ftdi_free(pdata->ftdic);
#endif

		free(pdata);
	}
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

	pgm->setpin = set_pin;

	pgm->setup = avrftdi_setup;
	pgm->teardown = avrftdi_teardown;

	pgm->rdy_led = set_led_rdy;
	pgm->err_led = set_led_err;
	pgm->pgm_led = set_led_pgm;
	pgm->vfy_led = set_led_vfy;
}

#else /*HAVE_LIBFTDI1*/

static int avrftdi_noftdi_open (struct programmer_t *pgm, char * name)
{
	fprintf(stderr,
		"%s: Error: no libftdi1 support. Install libftdi1 and run configure/make again.\n",
		progname);

	exit(1);
}

void avrftdi_initpgm(PROGRAMMER * pgm)
{
	strcpy(pgm->type, "avrftdi");
	pgm->open = avrftdi_noftdi_open;
}

#endif  /* HAVE_LIBFTDI1 */

#else /*HAVE_LIBUSB_1_0*/

static int avrftdi_nousb_open (struct programmer_t *pgm, char * name)
{
	fprintf(stderr,
		"%s: Error: no USB support. Install libusb-1.0 and run configure/make again.\n",
		progname);

	exit(1);
}

void avrftdi_initpgm(PROGRAMMER * pgm)
{
	strcpy(pgm->type, "avrftdi");
	pgm->open = avrftdi_nousb_open;
}

#endif /*HAVE_LIBUSB_1_0*/

const char avrftdi_desc[] = "Interface to the MPSSE Engine of FTDI Chips using libftdi.";

