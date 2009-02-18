/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2007 Dick Streefland, adapted for 5.4 by Limor Fried
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

/*
 * Driver for "usbtiny"-type programmers
 * Please see http://www.xs4all.nl/~dicks/avr/usbtiny/
 *        and http://www.ladyada.net/make/usbtinyisp/
 * For example schematics and detailed documentation
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#include "avrdude.h"
#include "avr.h"
#include "pgm.h"
#include "usbtiny.h"

#if defined(HAVE_LIBUSB)      // we use LIBUSB to talk to the board
#include <usb.h>

typedef	unsigned int	uint_t;
typedef	unsigned long	ulong_t;

extern int avr_write_byte_default ( PROGRAMMER* pgm, AVRPART* p,
				    AVRMEM* mem, ulong_t addr,
				    unsigned char data );
/*
 * Private data for this programmer.
 */
struct pdata
{
  usb_dev_handle *usb_handle;
  int sck_period;
  int chunk_size;
  int retries;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

// ----------------------------------------------------------------------

static void usbtiny_setup(PROGRAMMER * pgm)
{
  if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0) {
    fprintf(stderr,
	    "%s: usbtiny_setup(): Out of memory allocating private data\n",
	    progname);
    exit(1);
  }
  memset(pgm->cookie, 0, sizeof(struct pdata));
}

static void usbtiny_teardown(PROGRAMMER * pgm)
{
  free(pgm->cookie);
}

// Wrapper for simple usb_control_msg messages
static int usb_control (PROGRAMMER * pgm,
			unsigned int requestid, unsigned int val, unsigned int index )
{
  int nbytes;
  nbytes = usb_control_msg( PDATA(pgm)->usb_handle,
			    USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			    requestid,
			    val, index,           // 2 bytes each of data
			    NULL, 0,              // no data buffer in control messge
			    USB_TIMEOUT );        // default timeout
  if(nbytes < 0){
    fprintf(stderr, "\n%s: error: usbtiny_transmit: %s\n", progname, usb_strerror());
    exit(1);
  }

  return nbytes;
}

// Wrapper for simple usb_control_msg messages to receive data from programmer
static int usb_in (PROGRAMMER * pgm,
		   unsigned int requestid, unsigned int val, unsigned int index,
		   unsigned char* buffer, int buflen, int bitclk )
{
  int nbytes;
  int timeout;
  int i;

  // calculate the amout of time we expect the process to take by
  // figuring the bit-clock time and buffer size and adding to the standard USB timeout.
  timeout = USB_TIMEOUT + (buflen * bitclk) / 1000;

  for (i = 0; i < 10; i++) {
    nbytes = usb_control_msg( PDATA(pgm)->usb_handle,
			      USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			      requestid,
			      val, index,
			      (char *)buffer, buflen,
			      timeout);
    if (nbytes == buflen) {
      return nbytes;
    }
    PDATA(pgm)->retries++;
  }
  fprintf(stderr, "\n%s: error: usbtiny_receive: %s (expected %d, got %d)\n",
          progname, usb_strerror(), buflen, nbytes);
  exit(1);
}

// Report the number of retries, and reset the counter.
static void check_retries (PROGRAMMER * pgm, const char* operation)
{
  if (PDATA(pgm)->retries > 0 && quell_progress < 2) {
    printf("%s: %d retries during %s\n", progname,
           PDATA(pgm)->retries, operation);
  }
  PDATA(pgm)->retries = 0;
}

// Wrapper for simple usb_control_msg messages to send data to programmer
static int usb_out (PROGRAMMER * pgm,
		    unsigned int requestid, unsigned int val, unsigned int index,
		    unsigned char* buffer, int buflen, int bitclk )
{
  int nbytes;
  int timeout;

  // calculate the amout of time we expect the process to take by
  // figuring the bit-clock time and buffer size and adding to the standard USB timeout.
  timeout = USB_TIMEOUT + (buflen * bitclk) / 1000;

  nbytes = usb_control_msg( PDATA(pgm)->usb_handle,
			    USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			    requestid,
			    val, index,
			    (char *)buffer, buflen,
			    timeout);
  if (nbytes != buflen) {
    fprintf(stderr, "\n%s: error: usbtiny_send: %s (expected %d, got %d)\n",
	    progname, usb_strerror(), buflen, nbytes);
    exit(1);
  }

  return nbytes;
}

// Sometimes we just need to know the SPI command for the part to perform
// a function. Here we wrap this request for an operation so that we
// can just specify the part and operation and it'll do the right stuff
// to get the information from AvrDude and send to the USBtiny
static int usbtiny_avr_op (PROGRAMMER * pgm, AVRPART * p,
			   int op,
			   unsigned char res[4])
{
  unsigned char	cmd[4];

  if (p->op[op] == NULL) {
    fprintf( stderr, "Operation %d not defined for this chip!\n", op );
    return -1;
  }
  memset(cmd, 0, sizeof(cmd));
  avr_set_bits(p->op[op], cmd);

  return pgm->cmd(pgm, cmd, res);
}

// ----------------------------------------------------------------------

/* Find a device with the correct VID/PID match for USBtiny */

static	int	usbtiny_open(PROGRAMMER* pgm, char* name)
{
  struct usb_bus      *bus;
  struct usb_device   *dev = 0;

  usb_init();                    // initialize the libusb system
  usb_find_busses();             // have libusb scan all the usb busses available
  usb_find_devices();            // have libusb scan all the usb devices available

  PDATA(pgm)->usb_handle = NULL;

  // now we iterate through all the busses and devices
  for ( bus = usb_busses; bus; bus = bus->next ) {
    for	( dev = bus->devices; dev; dev = dev->next ) {
      if (dev->descriptor.idVendor == USBTINY_VENDOR
	  && dev->descriptor.idProduct == USBTINY_PRODUCT ) {   // found match?

	PDATA(pgm)->usb_handle = usb_open(dev);           // attempt to connect to device

	// wrong permissions or something?
	if (!PDATA(pgm)->usb_handle) {
	  fprintf(stderr, "%s: Warning: cannot open USB device: %s\n",
		  progname, usb_strerror());
	  continue;
	}
      }
    }
  }

  if (!PDATA(pgm)->usb_handle) {
    fprintf( stderr, "%s: Error: Could not find USBtiny device (0x%x/0x%x)\n",
	     progname, USBTINY_VENDOR, USBTINY_PRODUCT );
    exit(1);
  }

  return 0;                  // If we got here, we must have found a good USB device
}

/* Clean up the handle for the usbtiny */
static	void usbtiny_close ( PROGRAMMER* pgm )
{
  if (! PDATA(pgm)->usb_handle) {
    return;                // not a valid handle, bail!
  }
  usb_close(PDATA(pgm)->usb_handle);   // ask libusb to clean up
  PDATA(pgm)->usb_handle = NULL;
}

/* A simple calculator function determines the maximum size of data we can
   shove through a USB connection without getting errors */
static void usbtiny_set_chunk_size (PROGRAMMER * pgm, int period)
{
  PDATA(pgm)->chunk_size = CHUNK_SIZE;       // start with the maximum (default)
  while	(PDATA(pgm)->chunk_size > 8 && period > 16) {
    // Reduce the chunk size for a slow SCK to reduce
    // the maximum time of a single USB transfer.
    PDATA(pgm)->chunk_size >>= 1;
    period >>= 1;
  }
}

/* Given a SCK bit-clock speed (in useconds) we verify its an OK speed and tell the
   USBtiny to update itself to the new frequency */
static int usbtiny_set_sck_period (PROGRAMMER *pgm, double v)
{
  PDATA(pgm)->sck_period = (int)(v * 1e6 + 0.5);   // convert from us to 'int', the 0.5 is for rounding up

  // Make sure its not 0, as that will confuse the usbtiny
  if (PDATA(pgm)->sck_period < SCK_MIN)
    PDATA(pgm)->sck_period = SCK_MIN;

  // We can't go slower, due to the byte-size of the clock variable
  if  (PDATA(pgm)->sck_period > SCK_MAX)
    PDATA(pgm)->sck_period = SCK_MAX;

  if (verbose) {
    printf( "%s: Setting SCK period to %d usec\n", progname,
	    PDATA(pgm)->sck_period );
  }

  // send the command to the usbtiny device.
  usb_control(pgm, USBTINY_POWERUP, PDATA(pgm)->sck_period, RESET_LOW);  // MEME: for at90's fix resetstate?

  // with the new speed, we'll have to update how much data we send per usb transfer
  usbtiny_set_chunk_size(pgm, PDATA(pgm)->sck_period);
  return 0;
}


static int usbtiny_initialize (PROGRAMMER *pgm, AVRPART *p )
{
  unsigned char res[4];        // store the response from usbtinyisp

  // Check for bit-clock and tell the usbtiny to adjust itself
  if (pgm->bitclock > 0.0) {
    // -B option specified: convert to valid range for sck_period
    usbtiny_set_sck_period(pgm, pgm->bitclock);
  } else {
    // -B option not specified: use default
    PDATA(pgm)->sck_period = SCK_DEFAULT;
    if	(verbose) {
      printf( "%s: Using SCK period of %d usec\n",
	      progname, PDATA(pgm)->sck_period );
    }
    usb_control(pgm,  USBTINY_POWERUP, PDATA(pgm)->sck_period, RESET_LOW );
    usbtiny_set_chunk_size(pgm, PDATA(pgm)->sck_period);
  }

  // Let the device wake up.
  usleep(50000);

  // Attempt to use the underlying avrdude methods to connect (MEME: is this kosher?)
  if (! usbtiny_avr_op(pgm, p, AVR_OP_PGM_ENABLE, res)) {
    // no response, RESET and try again
    usb_control(pgm, USBTINY_POWERUP, PDATA(pgm)->sck_period, RESET_HIGH);
    usb_control(pgm, USBTINY_POWERUP, PDATA(pgm)->sck_period, RESET_LOW);
    usleep(50000);
    if	( ! usbtiny_avr_op( pgm, p, AVR_OP_PGM_ENABLE, res)) {
      // give up
      return -1;
    }
  }
  return 0;
}

/* Tell the USBtiny to release the output pins, etc */
static void usbtiny_powerdown(PROGRAMMER * pgm)
{
  if (!PDATA(pgm)->usb_handle) {
    return;                 // wasn't connected in the first place
  }
  usb_control(pgm, USBTINY_POWERDOWN, 0, 0);      // Send USB control command to device
}

/* Send a 4-byte SPI command to the USBtinyISP for execution
   This procedure is used by higher-level Avrdude procedures */
static int usbtiny_cmd(PROGRAMMER * pgm, unsigned char cmd[4], unsigned char res[4])
{
  int nbytes;

  // Make sure its empty so we don't read previous calls if it fails
  memset(res, '\0', 4 );

  nbytes = usb_in( pgm, USBTINY_SPI,
		   (cmd[1] << 8) | cmd[0],  // convert to 16-bit words
		   (cmd[3] << 8) | cmd[2],  //  "
			res, 4, 8 * PDATA(pgm)->sck_period );
  check_retries(pgm, "SPI command");
  if (verbose > 1) {
    // print out the data we sent and received
    printf( "CMD: [%02x %02x %02x %02x] [%02x %02x %02x %02x]\n",
	    cmd[0], cmd[1], cmd[2], cmd[3],
	    res[0], res[1], res[2], res[3] );
  }
  return ((nbytes == 4) &&      // should have read 4 bytes
	  res[2] == cmd[1]);              // AVR's do a delayed-echo thing
}

/* Send the chip-erase command */
static int usbtiny_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char res[4];

  if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
    fprintf(stderr, "Chip erase instruction not defined for part \"%s\"\n",
            p->desc);
    return -1;
  }

  // get the command for erasing this chip and transmit to avrdude
  if (! usbtiny_avr_op( pgm, p, AVR_OP_CHIP_ERASE, res )) {
    return -1;
  }
  usleep( p->chip_erase_delay );

  // prepare for further instruction
  pgm->initialize(pgm, p);

  return 0;
}

// These are required functions but don't actually do anything
static	void	usbtiny_enable ( PROGRAMMER* pgm ) {}

static void usbtiny_disable ( PROGRAMMER* pgm ) {}


/* To speed up programming and reading, we do a 'chunked' read.
 *  We request just the data itself and the USBtiny uses the SPI function
 *  given to read in the data. Much faster than sending a 4-byte SPI request
 *  per byte
*/
static int usbtiny_paged_load (PROGRAMMER * pgm, AVRPART * p, AVRMEM* m,
				    int page_size, int n_bytes )
{
  int i;
  int chunk;
  int function;


  // First determine what we're doing
  if (strcmp( m->desc, "flash" ) == 0) {
    function = USBTINY_FLASH_READ;
  } else {
    function = USBTINY_EEPROM_READ;
  }

  for (i = 0; i < n_bytes; i += chunk) {
    chunk = PDATA(pgm)->chunk_size;         // start with the maximum chunk size possible

    // If we want to xmit less than a chunk, thats OK
    if	(chunk > n_bytes-i)
      chunk = n_bytes - i;

    // Send the chunk of data to the USBtiny with the function we want
    // to perform
    usb_in(pgm,
	   function,          // EEPROM or flash
	   0,                 // delay between SPI commands
	   i,                 // index
	   m->buf + i,        // pointer to where we store data
	   chunk,             // number of bytes
	   32 * PDATA(pgm)->sck_period);  // each byte gets turned into a 4-byte SPI cmd
                              // usb_in() multiplies this per byte.

    // Tell avrdude how we're doing to provide user feedback
    report_progress(i + chunk, n_bytes, NULL );
  }

  check_retries(pgm, "read");
  return n_bytes;
}

/* To speed up programming and reading, we do a 'chunked' write.
 *  We send just the data itself and the USBtiny uses the SPI function
 *  given to write the data. Much faster than sending a 4-byte SPI request
 *  per byte.
*/
static int usbtiny_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
			       int page_size, int n_bytes)
{
  int i;
  int chunk;        // Size of data to write at once
  int next;
  int function;     // which SPI command to use
  int delay;        // delay required between SPI commands

  // First determine what we're doing
  if (strcmp( m->desc, "flash" ) == 0) {
    function = USBTINY_FLASH_WRITE;
  } else {
    function = USBTINY_EEPROM_WRITE;
  }

  delay = 0;
  if (! m->paged) {
    // Does this chip not support paged writes?
    i = (m->readback[1] << 8) | m->readback[0];
    usb_control(pgm, USBTINY_POLL_BYTES, i, 0 );
    delay = m->max_write_delay;
  }

  for (i=0; i < n_bytes; i=next) {
    // start with the max chunk size
    chunk = PDATA(pgm)->chunk_size;

    // we can only write a page at a time anyways
    if (m->paged && chunk > page_size)
      chunk = page_size;

    // if there's less data remaining than one chunk
    if (chunk > n_bytes-i)
      chunk = n_bytes - i;

    usb_out(pgm,
	    function,       // Flash or EEPROM
	    delay,          // How much to wait between each byte
	    i,              // Index of data
	    m->buf + i,     // Pointer to data
	    chunk,          // Number of bytes to write
	    32 * PDATA(pgm)->sck_period + delay  // each byte gets turned into a
	                             // 4-byte SPI cmd  usb_out() multiplies
	                             // this per byte. Then add the cmd-delay
	    );

    next = i + chunk;       // Calculate what address we're at now
    if (m->paged
	&& ((next % page_size) == 0 || next == n_bytes) ) {
      // If we're at a page boundary, send the SPI command to flush it.
      avr_write_page(pgm, p, m, (unsigned long) i);
    }

    report_progress( next, n_bytes, NULL );
  }
  return n_bytes;
}

extern void usbtiny_initpgm ( PROGRAMMER* pgm )
{
  strcpy(pgm->type, "USBtiny");

  /* Mandatory Functions */
  pgm->initialize	= usbtiny_initialize;
  pgm->enable	        = usbtiny_enable;
  pgm->disable	        = usbtiny_disable;
  pgm->program_enable	= NULL;
  pgm->chip_erase	= usbtiny_chip_erase;
  pgm->cmd		= usbtiny_cmd;
  pgm->open		= usbtiny_open;
  pgm->close		= usbtiny_close;
  pgm->read_byte        = avr_read_byte_default;
  pgm->write_byte       = avr_write_byte_default;

  /* Optional Functions */
  pgm->powerup	        = NULL;
  pgm->powerdown	= usbtiny_powerdown;
  pgm->paged_load	= usbtiny_paged_load;
  pgm->paged_write	= usbtiny_paged_write;
  pgm->set_sck_period	= usbtiny_set_sck_period;
  pgm->setup            = usbtiny_setup;
  pgm->teardown         = usbtiny_teardown;
}

#else  /* !HAVE_LIBUSB */

// Give a proper error if we were not compiled with libusb

static int usbtiny_nousb_open(struct programmer_t *pgm, char * name)
{
  fprintf(stderr, "%s: error: no usb support. Please compile again with libusb installed.\n",
	  progname);

  exit(1);
}

void usbtiny_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "usbtiny");

  pgm->open = usbtiny_nousb_open;
}

#endif /* HAVE_LIBUSB */
