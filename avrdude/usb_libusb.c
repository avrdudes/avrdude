/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005,2006 Joerg Wunsch
 * Copyright (C) 2006 David Moore
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

#include "avrdude.h"
#include "serial.h"
#include "usbdevs.h"

#if defined(WIN32NATIVE)
/* someone has defined "interface" to "struct" in Cygwin */
#  undef interface
#endif

static char usbbuf[USBDEV_MAX_XFER];
static int buflen = -1, bufptr;

static int usb_interface;

/*
 * The "baud" parameter is meaningless for USB devices, so we reuse it
 * to pass the desired USB device ID.
 */
static void usbdev_open(char * port, long baud, union filedescriptor *fd)
{
  char string[256];
  char product[256];
  struct usb_bus *bus;
  struct usb_device *dev;
  usb_dev_handle *udev;
  char *serno, *cp2;
  int i;
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

  for (bus = usb_get_busses(); bus; bus = bus->next)
    {
      for (dev = bus->devices; dev; dev = dev->next)
	{
	  udev = usb_open(dev);
	  if (udev)
	    {
	      if (dev->descriptor.idVendor == USB_VENDOR_ATMEL &&
		  dev->descriptor.idProduct == (unsigned short)baud)
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

		  if (usb_get_string_simple(udev,
					    dev->descriptor.iProduct,
					    product, sizeof(product)) < 0)
		    {
		      fprintf(stderr,
			      "%s: usb_open(): cannot read product name \"%s\"\n",
			      progname, usb_strerror());
		      strcpy(product, "[unnamed product]");
		    }

		  if (verbose)
		    fprintf(stderr,
			    "%s: usbdev_open(): Found %s, serno: %s\n",
			    progname, product, string);
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

		  if (dev->config == NULL)
		    {
		      fprintf(stderr,
			      "%s: usbdev_open(): USB device has no configuration\n",
			      progname);
		      goto trynext;
		    }

		  if (usb_set_configuration(udev, dev->config[0].bConfigurationValue))
		    {
		      fprintf(stderr,
			      "%s: usbdev_open(): error setting configuration %d: %s\n",
			      progname, dev->config[0].bConfigurationValue,
			      usb_strerror());
		      goto trynext;
		    }

		  usb_interface = dev->config[0].interface[0].altsetting[0].bInterfaceNumber;
		  if (usb_claim_interface(udev, usb_interface))
		    {
		      fprintf(stderr,
			      "%s: usbdev_open(): error claiming interface %d: %s\n",
			      progname, usb_interface, usb_strerror());
		      goto trynext;
		    }

		  fd->usb.handle = udev;
		  fd->usb.ep = -1;
		  /* Try finding out what our read endpoint is. */
		  for (i = 0; i < dev->config[0].interface[0].altsetting[0].bNumEndpoints; i++)
		    {
		      int possible_ep = dev->config[0].interface[0].altsetting[0].
		      endpoint[i].bEndpointAddress;

		      if ((possible_ep & USB_ENDPOINT_DIR_MASK) != 0)
			{
			  if (verbose > 1)
			    {
			      fprintf(stderr,
				      "%s: usbdev_open(): using read endpoint 0x%02x\n",
				      progname, possible_ep);
			    }
			  fd->usb.ep = possible_ep;
			  break;
			}
		    }
		  if (fd->usb.ep == -1)
		    {
		      fprintf(stderr,
			      "%s: usbdev_open(): cannot find a read endpoint, using 0x%02x\n",
			      progname, USBDEV_BULK_EP_READ);
		      fd->usb.ep = USBDEV_BULK_EP_READ;
		    }
                  return;
		}
	      trynext:
	      usb_close(udev);
	    }
	}
    }

  fprintf(stderr, "%s: usbdev_open(): did not find any%s USB device \"%s\"\n",
	  progname, serno? " (matching)": "", port);
  exit(1);
}

static void usbdev_close(union filedescriptor *fd)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;

  (void)usb_release_interface(udev, usb_interface);

  /*
   * Without this reset, the AVRISP mkII seems to stall the second
   * time we try to connect to it.
   */
  usb_reset(udev);

  usb_close(udev);
}


static int usbdev_send(union filedescriptor *fd, unsigned char *bp, size_t mlen)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;
  int rv;
  int i = mlen;
  unsigned char * p = bp;
  int tx_size;

  /*
   * Split the frame into multiple packets.  It's important to make
   * sure we finish with a short packet, or else the device won't know
   * the frame is finished.  For example, if we need to send 64 bytes,
   * we must send a packet of length 64 followed by a packet of length
   * 0.
   */
  do {
    tx_size = (mlen < USBDEV_MAX_XFER)? mlen: USBDEV_MAX_XFER;
    rv = usb_bulk_write(udev, USBDEV_BULK_EP_WRITE, (char *)bp, tx_size, 5000);
    if (rv != tx_size)
    {
        fprintf(stderr, "%s: usbdev_send(): wrote %d out of %d bytes, err = %s\n",
                progname, rv, tx_size, usb_strerror());
        return -1;
    }
    bp += tx_size;
    mlen -= tx_size;
  } while (tx_size == USBDEV_MAX_XFER);

  if (verbose > 3)
  {
      fprintf(stderr, "%s: Sent: ", progname);

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

/*
 * As calls to usb_bulk_read() result in exactly one USB request, we
 * have to buffer the read results ourselves, so the single-char read
 * requests performed by the upper layers will be handled.  In order
 * to do this, we maintain a private buffer of what we've got so far,
 * and transparently issue another USB read request if the buffer is
 * empty and more data are requested.
 */
static int
usb_fill_buf(usb_dev_handle *udev, int ep)
{
  int rv;

  rv = usb_bulk_read(udev, ep, usbbuf, USBDEV_MAX_XFER, 5000);
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

static int usbdev_recv(union filedescriptor *fd, unsigned char *buf, size_t nbytes)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;
  int i, amnt;
  unsigned char * p = buf;

  for (i = 0; nbytes > 0;)
    {
      if (buflen <= bufptr)
	{
	  if (usb_fill_buf(udev, fd->usb.ep) < 0)
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

/*
 * This version of recv keeps reading packets until we receive a short
 * packet.  Then, the entire frame is assembled and returned to the
 * user.  The length will be unknown in advance, so we return the
 * length as the return value of this function, or -1 in case of an
 * error.
 *
 * This is used for the AVRISP mkII device.
 */
static int usbdev_recv_frame(union filedescriptor *fd, unsigned char *buf, size_t nbytes)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;
  int rv, n;
  int i;
  unsigned char * p = buf;

  n = 0;
  do
    {
      rv = usb_bulk_read(udev, fd->usb.ep, usbbuf,
			 USBDEV_MAX_XFER, 10000);
      if (rv < 0)
	{
	  if (verbose > 1)
	    fprintf(stderr, "%s: usbdev_recv_frame(): usb_bulk_read(): %s\n",
		    progname, usb_strerror());
	  return -1;
	}

      if (rv <= nbytes)
	{
	  memcpy (buf, usbbuf, rv);
	  buf += rv;
	}

      n += rv;
      nbytes -= rv;
    }
  while (rv == USBDEV_MAX_XFER);

  if (nbytes < 0)
    return -1;

  if (verbose > 3)
  {
      i = n;
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
  return n;
}

static int usbdev_drain(union filedescriptor *fd, int display)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;
  int rv;

  do {
    rv = usb_bulk_read(udev, fd->usb.ep, usbbuf, USBDEV_MAX_XFER, 100);
    if (rv > 0 && verbose >= 4)
      fprintf(stderr, "%s: usbdev_drain(): flushed %d characters\n",
	      progname, rv);
  } while (rv > 0);

  return 0;
}

/*
 * Device descriptor for the JTAG ICE mkII.
 */
struct serial_device usb_serdev =
{
  .open = usbdev_open,
  .close = usbdev_close,
  .send = usbdev_send,
  .recv = usbdev_recv,
  .drain = usbdev_drain,
  .flags = SERDEV_FL_NONE,
};

/*
 * Device descriptor for the AVRISP mkII.
 */
struct serial_device usb_serdev_frame =
{
  .open = usbdev_open,
  .close = usbdev_close,
  .send = usbdev_send,
  .recv = usbdev_recv_frame,
  .drain = usbdev_drain,
  .flags = SERDEV_FL_NONE,
};

#endif  /* HAVE_LIBUSB */
