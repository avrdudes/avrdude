/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005 Joerg Wunsch
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
 * USB interface via libusb for avrdude.
 */

#include "ac_cfg.h"
#if defined(HAVE_LIBUSB)


#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>

#include <usb.h>

#include "serial.h"

extern char *progname;
extern int verbose;

#define USB_VENDOR_ATMEL 1003
#define USB_DEVICE_JTAGICEMKII 0x2103
/*
 * Should we query the endpoint number and max transfer size from USB?
 * After all, the JTAG ICE mkII docs document these values.
 */
#define JTAGICE_BULK_EP 2
#define JTAGICE_MAX_XFER 64

static char usbbuf[JTAGICE_MAX_XFER];
static int buflen = -1, bufptr;

static int usbdev_open(char * port, long baud)
{
  char string[256];
  struct usb_bus *bus;
  struct usb_device *dev;
  usb_dev_handle *udev;
  char *serno, *cp2;
  size_t x;

  /*
   * The syntax for usb devices is defined as:
   *
   * -P usb[:serialnumber]
   *
   * See if we've got a serial number passed here.  The serial number
   * might contain colons which we remove below, and we compare it
   * right-to-left, so only the least significant nibbles need to be
   * specified.
   */
  if ((serno = strchr(port, ':')) != NULL)
    {
      /* first, drop all colons there if any */
      cp2 = ++serno;

      while ((cp2 = strchr(cp2, ':')) != NULL)
	{
	  x = strlen(cp2) - 1;
	  memmove(cp2, cp2 + 1, x);
	  cp2[x] = '\0';
	}

      if (strlen(serno) > 12)
	{
	  fprintf(stderr,
		  "%s: usbdev_open(): invalid serial number \"%s\"\n",
		  progname, serno);
	  exit(1);
	}
    }

  usb_init();

  usb_find_busses();
  usb_find_devices();

  for (bus = usb_busses; bus; bus = bus->next)
    {
      for (dev = bus->devices; dev; dev = dev->next)
	{
	  udev = usb_open(dev);
	  if (udev)
	    {
	      if (dev->descriptor.idVendor == USB_VENDOR_ATMEL &&
		  dev->descriptor.idProduct == USB_DEVICE_JTAGICEMKII)
		{
		  /* yeah, we found something */
		  if (usb_get_string_simple(udev,
					    dev->descriptor.iSerialNumber,
					    string, sizeof(string)) < 0)
		    {
		      fprintf(stderr,
			      "%s: usb_open(): cannot read serial number \"%s\"\n",
			      progname, usb_strerror());
		      /*
		       * On some systems, libusb appears to have
		       * problems sending control messages.  Catch the
		       * benign case where the user did not request a
		       * particular serial number, so we could
		       * continue anyway.
		       */
		      if (serno != NULL)
			exit(1); /* no chance */
		      else
			strcpy(string, "[unknown]");
		    }

		  if (verbose)
		    fprintf(stderr,
			    "%s: usb_open(): Found JTAG ICE, serno: %s\n",
			    progname, string);
		  if (serno != NULL)
		    {
		      /*
		       * See if the serial number requested by the
		       * user matches what we found, matching
		       * right-to-left.
		       */
		      x = strlen(string) - strlen(serno);
		      if (strcasecmp(string + x, serno) != 0)
			{
			  if (verbose > 2)
			    fprintf(stderr,
				    "%s: usbdev_open(): serial number doesn't match\n",
				    progname);
			  usb_close(udev);
			      continue;
			}
		    }

		  return (int)udev;
		}
	      usb_close(udev);
	    }
	}
    }

  fprintf(stderr, "%s: usbdev_open(): did not find any%s USB device \"%s\"\n",
	  progname, serno? " (matching)": "", port);
  exit(1);
}

static int usbdev_setspeed(int fd, long baud)
{
  return 0;
}

static void usbdev_close(int fd)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd;

  usb_close(udev);
}


static int usbdev_send(int fd, char *bp, size_t mlen)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd;

  return usb_bulk_write(udev, JTAGICE_BULK_EP, (char *)bp, mlen, 5000) != mlen;
}

/*
 * As calls to usb_bulk_read() result in exactly one USB request, we
 * have to buffer the read results ourselves, so the single-char read
 * requests performed by the upper layers will be handled.  In order
 * to do this, we maintain a private buffer of what we've got so far,
 * and transparently issue another USB read request if the buffer is
 * empty and more data are requested.
 */
static int
usb_fill_buf(usb_dev_handle *udev)
{
  int rv;

  rv = usb_bulk_read(udev, JTAGICE_BULK_EP, usbbuf, JTAGICE_MAX_XFER, 5000);
  if (rv < 0)
    {
      if (verbose > 1)
	fprintf(stderr, "%s: usb_fill_buf(): usb_bulk_read() error %s\n",
		progname, usb_strerror());
      return -1;
    }

  buflen = rv;
  bufptr = 0;

  return 0;
}

static int usbdev_recv(int fd, char *buf, size_t nbytes)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd;
  int i, amnt;
  char * p = buf;

  for (i = 0; nbytes > 0;)
    {
      if (buflen <= bufptr)
	{
	  if (usb_fill_buf(udev) < 0)
	    return -1;
	}
      amnt = buflen - bufptr > nbytes? nbytes: buflen - bufptr;
      memcpy(buf + i, usbbuf + bufptr, amnt);
      bufptr += amnt;
      nbytes -= amnt;
      i += amnt;
    }

  if (verbose > 3)
  {
      fprintf(stderr, "%s: Recv: ", progname);

      while (i) {
        unsigned char c = *p;
        if (isprint(c)) {
          fprintf(stderr, "%c ", c);
        }
        else {
          fprintf(stderr, ". ");
        }
        fprintf(stderr, "[%02x] ", c);

        p++;
        i--;
      }
      fprintf(stderr, "\n");
  }

  return 0;
}


static int usbdev_drain(int fd, int display)
{
  return 0;
}

struct serial_device usb_serdev =
{
  .open = usbdev_open,
  .setspeed = usbdev_setspeed,
  .close = usbdev_close,
  .send = usbdev_send,
  .recv = usbdev_recv,
  .drain = usbdev_drain,
};

#endif  /* HAVE_LIBUSB */
