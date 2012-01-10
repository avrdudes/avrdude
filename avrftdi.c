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

#include "avrdude.h"
#include "avr.h"
#include "pgm.h"
#include "avrftdi.h"

#ifdef HAVE_LIBUSB
#ifdef HAVE_LIBFTDI

#include <ftdi.h>
#include <usb.h>

/* This is for running the code without having a FTDI-device.
 * The generated code is useless! For debugging purposes only.
 * This should never be defined, unless you know what you are
 * doing.
 * If you think you know what you are doing: YOU DONT!
 */
//#define DRYRUN

static struct ftdi_context ftdic;
static uint16_t pin_value, pin_direction, pin_inversion, led_mask;
static int type; /**type is  bcdDevice. C/D is 0x500 H is 0x700 4H is 0x800*/
static int ftype; /** is from FTDI. Use TYPE_2232C, TYPE_2232H, or TYPE_4232H*/

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

static int set_frequency(uint32_t freq)
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

	if(verbose)
		fprintf(stderr,
			"%s info: clock divisor: 0x%04x\n",
			progname, divisor);

	buf[0] = 0x86;
	buf[1] = (uint8_t)(divisor & 0xff);
	buf[2] = (uint8_t)((divisor >> 8) & 0xff);

#ifndef DRYRUN
	E(ftdi_write_data(&ftdic, buf, 3) < 0);
#endif

	return 0;
}

/*   Add a single pin (by pin number) to the pin masks (or to pins),
 *   update pinmask[pinfunc]  */
static int add_pin(PROGRAMMER *pgm, int pinfunc)
{
	int pin, inversion_mask, mlim;

	pin = pgm->pinno[pinfunc];
	if (verbose)
		fprintf(stderr,
			"add_pin: %d: bit 0x%04x inv=0x%04x\n",
			pinfunc, pin,
			(pin & PIN_INVERSE)? (1<< ((pin&PIN_MASK) - 1)): 0);

	/* non-existent definitions, go away */
	if (pin == 0)
		return 0;

	/* see if pin should be inverted */
	if(pin & PIN_INVERSE) {
		pin &= PIN_MASK;
		inversion_mask = 1 << (pin - 1);
	} else {
		inversion_mask = 0;
	}
	if(TYPE_4232H == ftype)
		mlim=7;
	else if(TYPE_2232C==ftype)
		mlim=11;
	else if(TYPE_2232H == ftype)
		mlim=15;
	else{
		fprintf(stderr, "Unknown type %d (0x%x)\n",
			ftype, ftype);
		mlim=15;
	}
	/* check that the pin number is in range */
	if (pin > mlim) {
		fprintf(stderr,
			"%s failure: invalid pin definition (pin no > %d) in config file\n",
			progname, mlim);
		fprintf(stderr,
			"pin function no %d, pin no: 0x%x\n",
			pinfunc, pin);
		return -1;
	}

	/* create the mask and check that the pin is available */
	if (pin_direction & (1 << (pin -1)) ) {
		fprintf(stderr,
			"%s failure: pin %d has two definitions in config file\n",
			progname, pin);
		return -1;
	} else {
		pin_direction |= (1 << (pin - 1));
		pin_inversion |= inversion_mask;
	}
	if(PIN_LED_ERR == pinfunc ||
	   PIN_LED_VFY == pinfunc ||
	   PIN_LED_RDY == pinfunc ||
	   PIN_LED_PGM == pinfunc) {
		led_mask|=(1 << (pin - 1));
	}

	return 0;
}

/*   Add pins by pin mask    */

static int add_pins(PROGRAMMER *pgm, int pinfunc)
{
	int i, pin, mlim;
	uint32_t mask, inversion_mask=0;

	pin = pgm->pinno[pinfunc];

	if(pin & PIN_INVERSE){
		pin &= PIN_MASK;
		inversion_mask = pin >>1;
	}
	pin >>= 1;
	if (verbose)
		fprintf(stderr,
			"add_pins: %d: 0x%04x, inv=0x%04x\n",
			pinfunc, pin, inversion_mask);
	mask = pin;
	if (TYPE_4232H == ftype)
		mlim = 8;
	else if (TYPE_2232C == ftype)
		mlim = 12;
	else if (TYPE_2232H == ftype)
		mlim = 16;
	else{
		fprintf(stderr, "Unknown type %d (0x%x)\n",
			ftype, ftype);
		mlim = 16;
	}
	if (mask >= 1 << mlim) {
		fprintf(stderr,
			"%s failure: pin list has pins out of range (%x>%x): ",
			progname, mask, 1 << mlim);
		mask &= ~(1 << mlim) - 1;
	}
	else if (mask & pin_direction) {
		fprintf(stderr,
			"%s failure: conflicting pins in pin list: ",
			progname);
		mask &= pin_direction;
	}
	else {
		pin_direction |= (uint16_t)mask;
		pin_inversion |= inversion_mask;
		return 0;
	}

	/* print the list of pins, if needed */
	i = 0;
	while (mask > 1) {
		if (mask & 1)
			fprintf(stderr, "%d, ", i);
		mask >>= 1;
		i++;
	}
	if (mask > 0)
		fprintf(stderr, "%d\n", i);
	return -1;
}


static int write_flush(void)
{
	unsigned char buf[6];
	if(verbose > 2)
		fprintf(stderr,
			"%s info: direction: 0x%04x, value: 0x%04x, inversion: 0x%04x\n",
			progname, pin_direction, pin_value, pin_inversion);

	buf[0] = 0x80;
	buf[1] = pin_value & 0xff;
	buf[2] = pin_direction & 0xff;
	buf[3] = 0x82;
	buf[4] = (pin_value >> 8) & 0xff;
	buf[5] = (pin_direction >> 8) & 0xff;

#ifndef DRYRUN
	E(ftdi_write_data(&ftdic, buf, 6) != 6);

#endif

	if (verbose > 3)
		fprintf(stderr, "FTDI LOG: %02x %02x %02x %02x %02x %02x\n",
		       buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	/* we need to flush here, because set_pin is used as reset.
	 * if we want to sleep reset periods, we must be certain the
	 * avr has got the reset signal when we start sleeping.
	 * (it may be stuck in the USB stack or some USB hub)
	 */
	E(ftdi_usb_purge_buffers(&ftdic));

	return 0;

}

/* this function sets or clears a GPIO pin */
static int set_pin(int pin, int value)
{
	int bit;
	uint16_t tval;
	if (0 == pin){
		if(verbose > 2)
			fprintf(stderr,
				"%s info: Pin is zero val %d!\n",
				progname, value);
		return 1;
	}
	--pin;
	bit= 1 << (pin);
	if (pin_inversion & bit) {
		value = !value;
	}
	if (value)
		value = bit;

	if (verbose > 1)
		fprintf(stderr,
			"%s info: pin %04x bit %04x value 0x%04x\n",
			progname, pin + 1, bit, value);
	/* set bits depending on value */
	/*pin_value ^= (-value ^ pin_value) & (1 << (pin - 1));  */
	tval = (pin_value & (~bit)) | value;
	if (tval != pin_value) {
		pin_value = tval;
		return write_flush();
	} else if (verbose > 1)
	    fprintf(stderr, "SameVal\n");
	return 0;

}

/* this function sets or clears one or more GPIO pin these are bit-mapped */
static int set_pins(int pin, int value)
{
	if (0 == pin) {
		if(verbose > 2)
			fprintf(stderr,"%s info: Pins is zero!\n",progname);
		return 1;
	}
	pin >>=1;
	if (pin_inversion & pin) {
		value = !value;
	}
	if (value)
		value = pin;

	if (verbose)
		fprintf(stderr,
			"%s info: pin %04x value %d\n",
			progname, pin, value);
	/* set bits depending on value */
	/*pin_value ^= (-value ^ pin_value) & (1 << (pin - 1));  */
	pin_value = (pin_value & (~pin)) | value;
	return write_flush();
}

/* these functions are callbacks, which go into the
 * PROGRAMMER data structure ("optional functions")
 */
static int set_led_pgm(struct programmer_t * pgm, int value)
{
	return set_pin(pgm->pinno[PIN_LED_PGM], value);
}

static int set_led_rdy(struct programmer_t * pgm, int value)
{
	return set_pin(pgm->pinno[PIN_LED_RDY], value);
}

static int set_led_err(struct programmer_t * pgm, int value)
{
	return set_pin(pgm->pinno[PIN_LED_ERR], value);
}

static int set_led_vfy(struct programmer_t * pgm, int value)
{
	return set_pin(pgm->pinno[PIN_LED_VFY], value);
}

static int avrftdi_transmit(unsigned char mode, unsigned char *cmd,
			    unsigned char *data, int buf_size)
{
	int k = 0;
	int n;
	unsigned char buf[4 + buf_size];

	if (mode & TX) {
		buf[0] = mode;
		buf[1] = ((buf_size - 1) & 0xff);
		buf[2] = (((buf_size - 1) >> 8) & 0xff);

		memcpy(buf + 3, cmd, buf_size);
		buf[buf_size + 3] = 0x87;

#ifndef DRYRUN
		E(ftdi_write_data(&ftdic, buf, buf_size + 4) != buf_size + 4);
#endif
	}

	if (mode & RX) {
		memset(buf, 0, sizeof(buf));
		do {
#ifndef DRYRUN
			n = ftdi_read_data(&ftdic, buf + k, buf_size - k);
			E(n < 0);
#else
			n = buf_size - k;
#endif
			k += n;
		} while (k < buf_size);

		memcpy(data, buf, buf_size);
	}

	return k;
}

static int avrftdi_open(PROGRAMMER * pgm, char *port)
{
	int vid, pid, interface, snfound;
	char serial[255], *foundsn;
	struct ftdi_device_list* devlist;
	struct ftdi_device_list* devlist_ptr;
	struct usb_device *found_dev;
	/* use vid/pid in following priority: config,
	 * defaults. cmd-line is currently not supported */
	type = 0;
	snfound = 0;
	foundsn = NULL;

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
#ifndef DRYRUN
	E(ftdi_init(&ftdic) < 0);
	found_dev = NULL;
	if (ftdi_usb_find_all(&ftdic, &devlist, vid, pid)) {
		devlist_ptr = devlist;
		do {
			ftdi_usb_get_strings(&ftdic, devlist_ptr->dev,
					     NULL, 0, NULL, 0, serial, 255);

			if (verbose)
				fprintf(stderr,
					"%s: device: %s, serial number: %s type 0x%04x found\n",
					progname, devlist_ptr->dev->filename,
					serial, devlist_ptr->dev->descriptor.bcdDevice);

			if (!snfound) {
				if (strcmp(pgm->usbsn, serial) == 0){
					foundsn = strdup(serial);
					snfound = 1;
					found_dev = devlist_ptr->dev;
					type = devlist_ptr->dev->descriptor.bcdDevice;
				}
			}else {
				if (0 == type)	/**we assume it will attach to first found.  */
					type = devlist_ptr->dev->descriptor.bcdDevice;
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
	if (verbose) {
		fprintf(stderr,
			"%s: Using device VID:PID %04x:%04x type 0x%04x(",
			progname, vid, pid, type);
		switch (type) {
			case TYPE_C_D:
			    fprintf(stderr,"C/D"); break;
			case TYPE_H:
			    fprintf(stderr,"H"); break;
			case TYPE_4H:
			    fprintf(stderr,"4H"); break;
			default:
			    fprintf(stderr,"unknown %04x",type); break;
		}

		fprintf(stderr,") and SN '%s'.\n", foundsn);
	}
	if (type == TYPE_C_D && INTERFACE_B == interface){
		fprintf(stderr,
			"%s: Type C/D found. Setting interface to A\n",
			progname);
		interface = INTERFACE_A;
	}
	/*must be A for mpsse if C/D, can be A/B for H */
	if (verbose)
		fprintf(stderr,
			"%s: Using USB Interface %c\n",
			progname, INTERFACE_A == interface? 'A': 'B');
	free(foundsn);
	E(ftdi_set_interface(&ftdic, interface) < 0);
	E(ftdi_usb_open_dev(&ftdic,found_dev) <0);
/*	E(ftdi_usb_open_desc(&ftdic, vid,pid,NULL,0==pgm->usbsn[0]?NULL:pgm->usbsn) < 0);  */
	ftype=ftdic.type;
#endif


	if (SCK != (1 << (pgm->pinno[PIN_AVR_SCK] - 1))
		|| SDO != (1 << (pgm->pinno[PIN_AVR_MOSI] - 1))
		|| SDI != (1 << (pgm->pinno[PIN_AVR_MISO] - 1))) {
		fprintf(stderr,
			"%s failure: pinning for FTDI MPSSE must be:\n"
			"\tSCK: 1, SDO: 2, SDI: 3(is: %d,%d,%d)\n",
			progname,
			pgm->pinno[PIN_AVR_SCK],
			pgm->pinno[PIN_AVR_MOSI],
			pgm->pinno[PIN_AVR_MISO]);
		fprintf(stderr, "Setting pins accordingly ...\n");
			pgm->pinno[PIN_AVR_SCK] = 1;
			pgm->pinno[PIN_AVR_MOSI] = 2;
			pgm->pinno[PIN_AVR_MISO] = 3;

	}
	if(verbose)
		fprintf(stderr,
			"%s info: reset pin value: %x\n",
			progname, pgm->pinno[PIN_AVR_RESET]-1);
	if (pgm->pinno[PIN_AVR_RESET] < 4 || pgm->pinno[PIN_AVR_RESET] == 0) {
		fprintf(stderr,
			"%s failure: RESET pin clashes with data pin or is not set.\n",
			progname);
		fprintf(stderr, "Setting to default-value 4\n");
		pgm->pinno[PIN_AVR_RESET] = 4;
	}
	/**sync our internal state with the chip  */
	pin_direction = 0;
	pin_value = 0;
	write_flush();
	pin_direction = (0x3 | (1 << (pgm->pinno[PIN_AVR_RESET] - 1)));

	/* gather the rest of the pins */
	if (add_pins(pgm, PPI_AVR_VCC)) return -1;
	if (add_pins(pgm, PPI_AVR_BUFF)) return -1;
	if (add_pin(pgm, PIN_LED_ERR)) return -1;
	if (add_pin(pgm, PIN_LED_RDY)) return -1;
	if (add_pin(pgm, PIN_LED_PGM)) return -1;
	if (add_pin(pgm, PIN_LED_VFY)) return -1;
#ifndef DRYRUN
	E(ftdi_set_bitmode(&ftdic, pin_direction & 0xff, BITMODE_MPSSE) < 0);	/*set SPI */
#endif
	if (verbose > 1) {
		fprintf(stderr, "pin direction mask: %04x\n", pin_direction);
		fprintf(stderr, "pin value mask: %04x\n", pin_value);
	}

	if (pgm->baudrate) {
		set_frequency(pgm->baudrate);
	} else if(pgm->bitclock) {
		set_frequency((uint32_t)(1.0f/pgm->bitclock));
	} else {
		set_frequency(pgm->baudrate ? pgm->baudrate : 150000);
	}
	/**set the ready LED, if we have one .. and set our direction up */
	set_led_rdy(pgm,0);
	set_led_rdy(pgm,1);
	return 0;
}

static void avrftdi_close(PROGRAMMER * pgm)
{
	if(ftdic.usb_dev) {
		set_pins(pgm->pinno[PPI_AVR_BUFF], ON);
		set_pin(pgm->pinno[PIN_AVR_RESET], ON);
		/**Stop driving the pins - except for the LEDs */
		if (verbose > 1)
			fprintf(stderr,
				"LED Mask=0x%04x value =0x%04x &=0x%04x\n",
				led_mask, pin_value, led_mask & pin_value);
		pin_direction = led_mask;
		pin_value &= led_mask;
		write_flush();
#ifndef DRYRUN
		E_VOID(ftdi_usb_close(&ftdic));
#endif
	}

#ifndef DRYRUN
	ftdi_deinit(&ftdic);
#endif

	return;
}


static int avrftdi_initialize(PROGRAMMER * pgm, AVRPART * p)
{
	set_pin(pgm->pinno[PIN_AVR_RESET], OFF);
	set_pins(pgm->pinno[PPI_AVR_BUFF], OFF);
	set_pin(pgm->pinno[PIN_AVR_SCK], OFF);
	/*use speed optimization with CAUTION*/
	usleep(20 * 1000);

	/* giving rst-pulse of at least 2 avr-clock-cycles, for
	 * security (2us @ 1MHz) */
	set_pin(pgm->pinno[PIN_AVR_RESET], ON);
	usleep(20 * 1000);

	/*setting rst back to 0 */
	set_pin(pgm->pinno[PIN_AVR_RESET], OFF);
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
	return avrftdi_transmit(TRX, cmd, res, 4);
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
			set_pin(pgm->pinno[PIN_AVR_RESET], ON);
			usleep(20);
			set_pin(pgm->pinno[PIN_AVR_RESET], OFF);
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
static int avrftdi_lext(PROGRAMMER *pgm, AVRPART *p, AVRMEM *m, unsigned int address)
{
	unsigned char buf[] = {0x11, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};

	avr_set_bits(m->op[AVR_OP_LOAD_EXT_ADDR], &buf[3]);
	avr_set_addr(m->op[AVR_OP_LOAD_EXT_ADDR], &buf[3], address);

	if(verbose > 1)
		buf_dump(buf, sizeof(buf),
			 "load extended address command", 0, 16 * 3);

#ifndef DRYRUN
	E(ftdi_write_data(&ftdic, buf, sizeof(buf)) != sizeof(buf));
#endif
	return 0;
}

static int avrftdi_eeprom_write(PROGRAMMER *pgm, AVRPART *p, AVRMEM *m,
		unsigned int page_size, unsigned int addr, unsigned int len)
{
	unsigned char cmd[4];
	unsigned char *data = m->buf;
	unsigned int add;

	avr_set_bits(m->op[AVR_OP_WRITE], cmd);

	for (add = addr; add < addr + len; add++)
	{
		avr_set_addr(m->op[AVR_OP_WRITE], cmd, add);
		avr_set_input(m->op[AVR_OP_WRITE], cmd, *data++);

		E(avrftdi_transmit(TX, cmd, cmd, 4) < 0);

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
		avr_set_bits(m->op[AVR_OP_READ], cmd);
		avr_set_addr(m->op[AVR_OP_READ], cmd, add);

		E(avrftdi_transmit(TRX, cmd, cmd, 4) < 0);

		avr_get_output(m->op[AVR_OP_READ], cmd, bufptr++);
	}

	memcpy(m->buf + addr, buffer, len);
	return len;
}

static int avrftdi_flash_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
		unsigned int page_size, unsigned int addr, unsigned int len)
{
	int i;
	unsigned int address = addr/2, buf_size;
	unsigned int address_prev_block = ~address; /* start address of previous block,
	                                            init to different than address */
	unsigned int bytes = len;
	unsigned int blocksize;
	int use_lext_address = m->op[AVR_OP_LOAD_EXT_ADDR] != NULL;
	unsigned char buf[4*len+4], *bufptr = buf;
	unsigned char *buffer = m->buf;
	unsigned char byte;

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

	//page_size = (page_size > m->page_size) ? m->page_size : page_size - 8;
	page_size = m->page_size;

	while (bytes) {
		if (bytes > page_size) {
			blocksize = (page_size)/2;
			bytes -= (page_size);
		} else {
			blocksize = bytes/2;
			bytes = 0;
		}

		if(verbose > 2)
			fprintf(stderr,
				"-< bytes = %d of %d, blocksize = %d of %d\n",
				len - bytes, len, blocksize, m->page_size / 2);

		/* if we do cross a 64k word boundary (or write the
		 * first page), we need to issue a 'load extended
		 * address byte' command, which is defined as 0x4d
		 * 0x00 <address byte> 0x00.  As far as i know, this
		 * is only available on 256k parts.  64k word is 128k
		 * bytes.
		 */
		if(use_lext_address && ((address & 0xffff0000) != (address_prev_block & 0xffff0000))) {
			avrftdi_lext(pgm, p, m, address);
		}
		address_prev_block = address;

		for (i = 0; i < blocksize; i++) {
			/*setting word*/
			avr_set_bits(m->op[AVR_OP_LOADPAGE_LO], bufptr);
			avr_set_addr(m->op[AVR_OP_LOADPAGE_LO], bufptr, address);
			avr_set_input(m->op[AVR_OP_LOADPAGE_LO], bufptr, *buffer++);
			bufptr += 4;
			avr_set_bits(m->op[AVR_OP_LOADPAGE_HI], bufptr);
			avr_set_addr(m->op[AVR_OP_LOADPAGE_HI], bufptr, address);
			avr_set_input(m->op[AVR_OP_LOADPAGE_HI], bufptr, *buffer++);
			bufptr += 4;
			address++;
		}

		if (verbose > 2)
			fprintf(stderr,
				"address = %d, page_size = %d\n",
				address, m->page_size);

		if (((address * 2) % m->page_size) == 0 || bytes == 0) {
			if (m->op[AVR_OP_WRITEPAGE] == NULL) {
				fprintf(stderr,
					"%s failure: Write Page (WRITEPAGE) command not defined for %s\n",
					progname, p->desc);
				exit(1);
			} else {
				avr_set_bits(m->op[AVR_OP_WRITEPAGE], bufptr);
			}
			/* setting page address highbyte */
			avr_set_addr(m->op[AVR_OP_WRITEPAGE],
				     bufptr, address - 1);
			bufptr += 4;
		}

		buf_size = bufptr - buf;

		if(verbose > 3)
			buf_dump(buf, buf_size, "command buffer", 0, 16*3);
		if(verbose > 2)
			fprintf(stderr,
				"%s info: buffer size: %d\n",
				progname, buf_size);

		E(avrftdi_transmit(TX, buf, buf, buf_size) < 0);

		bufptr = buf;
		if (((address * 2) % m->page_size) == 0 || bytes == 0) {
			do {
				pgm->read_byte(pgm, p, m,
					       (address * 2) - 1, &byte);
			} while (m->buf[(address*2) - 1] != byte);
		}

		if (verbose < 3)
			report_progress(2 * address - addr, len, NULL);
	}
	return len;
}

static int avrftdi_flash_read(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
		unsigned int page_size, unsigned int addr, unsigned int len)
{
	/*
	 *Reading from flash
	 */
	int use_lext_address = m->op[AVR_OP_LOAD_EXT_ADDR] != NULL;
	int i, buf_index, buf_size = 0, psize = m->page_size;
	unsigned char o_buf[4*len+4], *o_ptr = o_buf;
	unsigned char i_buf[4*len+4];
	unsigned int address = addr/2;
	unsigned int address_prev_block = ~address; /* start address of previous block,
	                                            init to different than address */
	unsigned int bytes = len;
	unsigned int blocksize;
	unsigned char buffer[m->size], *bufptr = buffer;

	memset(o_buf, 0, sizeof(o_buf));
	memset(i_buf, 0, sizeof(i_buf));
	memset(buffer, 0, sizeof(buffer));

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

	while (bytes) {
		if (bytes > psize) {
			blocksize = psize/2;
			bytes -= psize;
		} else {
			blocksize = bytes/2;
			bytes = 0;
		}

		if(use_lext_address && ((address & 0xffff0000) != (address_prev_block & 0xffff0000))) {
			avrftdi_lext(pgm, p, m, address);
		}
		address_prev_block = address;

		for(i = 0; i < blocksize; i++) {
			if(verbose > 3)
				fprintf(stderr,
					"bufsize: %d, i: %d, add: %d\n",
					buf_size, i, address);
			avr_set_bits(m->op[AVR_OP_READ_LO], o_ptr);
			avr_set_addr(m->op[AVR_OP_READ_LO], o_ptr, address);
			o_ptr += 4;
			avr_set_bits(m->op[AVR_OP_READ_HI], o_ptr);
			avr_set_addr(m->op[AVR_OP_READ_HI], o_ptr, address);
			o_ptr += 4;

			address++;

			//FIXME: why not program on per-page basis?
			//maybe this covered a timing error in an earlier version?
			buf_size = o_ptr - o_buf;

			if((buf_size >= (page_size - 8)) || ( i == blocksize-1)) {
				E(avrftdi_transmit(TRX, o_buf, i_buf, buf_size) < 0);

				for(buf_index = 0; buf_index < buf_size; buf_index+=8) {
					avr_get_output(m->op[AVR_OP_READ_LO], i_buf+buf_index, bufptr++);
					avr_get_output(m->op[AVR_OP_READ_HI], i_buf+buf_index+4, bufptr++);
				}

				if(verbose > 3) {
					buf_dump(i_buf, buf_size, "i_buf", 0, 16);
				}
				o_ptr = o_buf;
			}
		}
	}
	memcpy(m->buf + addr, buffer, len);

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

void avrftdi_initpgm(PROGRAMMER * pgm)
{
	strcpy(pgm->type, "avrftdi");

	pin_value=pin_direction=pin_inversion=led_mask=0;
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

