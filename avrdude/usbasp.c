/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2006  Thomas Fischl
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
 * Interface to the USBasp programmer.
 *
 * See http://www.fischl.de/usbasp/
 */
#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#include "avr.h"
#include "pgm.h"
#include "usbasp.h"

#ifdef HAVE_LIBUSB
#include <usb.h>

extern char * progname;
extern int do_cycles;

static usb_dev_handle *usbhandle;

/*
 * wrapper for usb_control_msg call
 */
static int usbasp_transmit(unsigned char receive, unsigned char functionid,
			   unsigned char send[4], unsigned char * buffer, int buffersize)
{
  int nbytes;
  nbytes = usb_control_msg(usbhandle,
			   USB_TYPE_VENDOR | USB_RECIP_DEVICE | (receive << 7),
			   functionid,
			   (send[1] << 8) | send[0],
			   (send[3] << 8) | send[2],
			   buffer, buffersize,
			   5000);
  if(nbytes < 0){
    fprintf(stderr, "%s: error: usbasp_transmit: %s\n", progname, usb_strerror());
    exit(1);
  }

  return nbytes;
}

static int usbasp_open(PROGRAMMER * pgm, char * port)
{
  struct usb_bus	*bus;
  struct usb_device *dev = 0;

  usb_init();
  usb_find_busses();
  usb_find_devices();
  for(bus=usb_busses; bus; bus=bus->next){
    for(dev=bus->devices; dev; dev=dev->next){
      if(dev->descriptor.idVendor == USBDEV_VENDOR && dev->descriptor.idProduct == USBDEV_PRODUCT)
	break;
    }
    if(dev)
      break;
  }
  if(!dev){
    fprintf(stderr, "%s: error: could not find USB device vendor=0x%x product=0x%x\n",
	    progname, USBDEV_VENDOR, USBDEV_PRODUCT);
    exit(1);
  }

  usbhandle = usb_open(dev);
  if(!usbhandle){
    fprintf(stderr, "%s: error: opening usb device: %s\n",
	    progname, usb_strerror());
    exit(1);
  }

  return 0;
}


static void usbasp_close(PROGRAMMER * pgm)
{
  unsigned char temp[4];
  memset(temp, 0, sizeof(temp));
  usbasp_transmit(1, USBASP_FUNC_DISCONNECT, temp, temp, sizeof(temp));

  usb_close(usbhandle);
}


static int usbasp_initialize(PROGRAMMER * pgm, AVRPART * p)
{

  unsigned char temp[4];
  memset(temp, 0, sizeof(temp));
  usbasp_transmit(1, USBASP_FUNC_CONNECT, temp, temp, sizeof(temp));

  usleep(100000);

  pgm->program_enable(pgm, p);
  return 0;
}

static void usbasp_disable(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}

static void usbasp_enable(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}

static void usbasp_display(PROGRAMMER * pgm, char * p)
{
  return;
}


static int usbasp_cmd(PROGRAMMER * pgm, unsigned char cmd[4],
                   unsigned char res[4])
{
  int nbytes =
    usbasp_transmit(1, USBASP_FUNC_TRANSMIT, cmd, res, sizeof(res));

  if(nbytes != 4){
    fprintf(stderr, "%s: error: wrong responds size\n",
	    progname);
    return -1;
  }

  return 0;
}


static int usbasp_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char res[4];
  unsigned char cmd[4];
  memset(cmd, 0, sizeof(cmd));
  memset(res, 0, sizeof(res));

  cmd[0] = 0;

  int nbytes =
    usbasp_transmit(1, USBASP_FUNC_ENABLEPROG, cmd, res, sizeof(res));

  if ((nbytes != 1) | (res[0] != 0)) {
    fprintf(stderr, "%s: error: programm enable: target doesn't answer. %x \n",
	    progname, res[0]);
    return -1;
  }

  return 0;
}


static int usbasp_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char cmd[4];
  unsigned char res[4];

  if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
    fprintf(stderr, "chip erase instruction not defined for part \"%s\"\n",
            p->desc);
    return -1;
  }

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
  pgm->cmd(pgm, cmd, res);
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);

  return 0;
}


static int usbasp_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                             int page_size, int n_bytes)
{
  int n;
  unsigned char cmd[4];
  int address = 0;
  int wbytes = n_bytes;
  int blocksize;
  unsigned char * buffer = m->buf;
  int function;

  if (strcmp(m->desc, "flash") == 0) {
    function = USBASP_FUNC_READFLASH;
  } else if (strcmp(m->desc, "eeprom") == 0) {
    function = USBASP_FUNC_READEEPROM;
  } else {
    return -2;
  }

  while (wbytes) {
    if (wbytes > USBASP_READBLOCKSIZE) {
      blocksize = USBASP_READBLOCKSIZE;
      wbytes -= USBASP_READBLOCKSIZE;
    } else {
      blocksize = wbytes;
      wbytes = 0;
    }

    cmd[0] = address & 0xFF;
    cmd[1] = address >> 8;

    n = usbasp_transmit(1, function, cmd, buffer, blocksize);

    if (n != blocksize) {
      fprintf(stderr, "%s: error: wrong reading bytes %x\n",
	      progname, n);
      exit(1);
    }

    buffer += blocksize;
    address += blocksize;

    report_progress (address, n_bytes, NULL);
  }

  return n_bytes;
}

static int usbasp_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                              int page_size, int n_bytes)
{
  int n;
  unsigned char cmd[4];
  int address = 0;
  int wbytes = n_bytes;
  int blocksize;
  unsigned char * buffer = m->buf;
  unsigned char blockflags = USBASP_BLOCKFLAG_FIRST;
  int function;

  if (strcmp(m->desc, "flash") == 0) {
    function = USBASP_FUNC_WRITEFLASH;
  } else if (strcmp(m->desc, "eeprom") == 0) {
    function = USBASP_FUNC_WRITEEEPROM;
  } else {
    return -2;
  }

  while (wbytes) {
    if (wbytes > USBASP_WRITEBLOCKSIZE) {
      blocksize = USBASP_WRITEBLOCKSIZE;
      wbytes -= USBASP_WRITEBLOCKSIZE;
    } else {
      blocksize = wbytes;
      wbytes = 0;
      blockflags |= USBASP_BLOCKFLAG_LAST;
    }

    cmd[0] = address & 0xFF;
    cmd[1] = address >> 8;
    cmd[2] = page_size & 0xFF;
    cmd[3] = (blockflags & 0x0F) + ((page_size & 0xF00) >> 4); //TP: Mega128 fix
    blockflags = 0;

    n = usbasp_transmit(0, function, cmd, buffer, blocksize);

    if (n != blocksize) {
      fprintf(stderr, "%s: error: wrong count at writing %x\n",
	      progname, n);
      exit(1);
    }


    buffer += blocksize;
    address += blocksize;

    report_progress (address, n_bytes, NULL);
  }

  return n_bytes;
}

void usbasp_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "usbasp");

  /*
   * mandatory functions
   */

  pgm->initialize     = usbasp_initialize;
  pgm->display        = usbasp_display;
  pgm->enable         = usbasp_enable;
  pgm->disable        = usbasp_disable;
  pgm->program_enable = usbasp_program_enable;
  pgm->chip_erase     = usbasp_chip_erase;
  pgm->cmd            = usbasp_cmd;
  pgm->open           = usbasp_open;
  pgm->close          = usbasp_close;

  /*
   * optional functions
   */

  pgm->paged_write = usbasp_paged_write;
  pgm->paged_load = usbasp_paged_load;

}


#else /* HAVE_LIBUSB */

extern char * progname;

static int usbasp_nousb_open (struct programmer_t *pgm, char * name)
{
  fprintf(stderr, "%s: error: no usb support. please compile again with libusb installed.\n",
	  progname);

  exit(1);
}

void usbasp_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "usbasp");

  pgm->open           = usbasp_nousb_open;
}

#endif  /* HAVE_LIBUSB */
