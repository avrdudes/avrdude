/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005, 2006 Joerg Wunsch
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * USB interface via libusb for avrdude.
 */

#include <ac_cfg.h>
#if defined(HAVE_LIBUSB)


#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>

#if defined(HAVE_USB_H)
#  include <usb.h>
#elif defined(HAVE_LUSB0_USB_H)
#  include <lusb0_usb.h>
#else
#  error "libusb needs either <usb.h> or <lusb0_usb.h>"
#endif

#include "avrdude.h"
#include "libavrdude.h"

#include "usbdevs.h"

#if defined(WIN32)
/* someone has defined "interface" to "struct" in Cygwin */
#  undef interface
#endif

/*
 * The "baud" parameter is meaningless for USB devices, so we reuse it
 * to pass the desired USB device ID.
 */
static int usbdev_open(const char *port, union pinfo pinfo, union filedescriptor *fd) {
  char string[256];
  char product[256];
  struct usb_bus *bus;
  struct usb_device *dev;
  usb_dev_handle *udev;
  char *s, serno[64] = {0};
  const char *serp;
  int i, iface;

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
  if((serp = strchr(port, ':')) && *++serp) {
    // First, get a copy of the serial number w/out colons
    for(s = serno; *serp && s < serno + sizeof serno - 1; serp++)
      if(*serp != ':')
        *s++ = *serp;
    *s = 0;
  }

  if (fd->usb.max_xfer == 0)
    fd->usb.max_xfer = USBDEV_MAX_XFER_MKII;

  usb_init();

  usb_find_busses();
  usb_find_devices();

  for (bus = usb_get_busses(); bus; bus = bus->next)
    {
      for (dev = bus->devices; dev; dev = dev->next)
	{
	  if (dev->descriptor.idVendor == pinfo.usbinfo.vid &&
	      dev->descriptor.idProduct == pinfo.usbinfo.pid)
	    {
	      udev = usb_open(dev);
	      if (udev)
		{
		  /* yeah, we found something */
		  if (usb_get_string_simple(udev,
					    dev->descriptor.iSerialNumber,
					    string, sizeof(string)) < 0)
		    {
		      pmsg_warning("reading serial number, %s\n", usb_strerror());
		      /*
		       * On some systems, libusb appears to have
		       * problems sending control messages.  Catch the
		       * benign case where the user did not request a
		       * particular serial number, so we could
		       * continue anyway.
		       */
		      cx->usb_access_error = 1;
		      if(*serno)
			goto none_matching; // No chance of serno matches
		      strcpy(string, "[unknown]");
		    }
		  if(serdev)
		    serdev->usbsn = cache_string(string);
		  if (usb_get_string_simple(udev,
					    dev->descriptor.iProduct,
					    product, sizeof(product)) < 0)
		    {
		      pmsg_warning("reading product name, %s\n", usb_strerror());
		      strcpy(product, "[unnamed product]");
		    }

		  /* We need to write to endpoint 2 to switch the PICkit4 and SNAP
		   * from PIC to AVR mode
		   */
		  if(str_casestarts(product, "MPLAB") && (str_caseends(product, "Snap ICD")
		    || str_caseends(product, "PICkit 4")))
		  {
		    pinfo.usbinfo.flags = 0;
		    fd->usb.wep = 2;
		  }
		  /*
		   * The CMSIS-DAP specification mandates the string
		   * "CMSIS-DAP" must be present somewhere in the
		   * product name string for a device compliant to
		   * that protocol.  Use this for the decisision
		   * whether we have to search for a HID interface
		   * below.
		   */
		  if(str_contains(product, "CMSIS-DAP"))
		  {
		      pinfo.usbinfo.flags |= PINFO_FL_USEHID;
		      /* The JTAGICE3 running the CMSIS-DAP firmware doesn't
		       * use a separate endpoint for event reception. */
		      fd->usb.eep = 0;
		  }

		  if(str_contains(product, "mEDBG"))
		  {
		      /* The AVR Xplained Mini uses different endpoints. */
		      fd->usb.rep = 0x81;
		      fd->usb.wep = 0x02;
		  }

		  pmsg_notice2("%s(): found %s, serno: %s\n", __func__, product, string);
		  if (*serno)
		    {
		      /*
		       * See if the serial number requested by the
		       * user matches what we found, matching
		       * right-to-left.
		       */
		      int x = strlen(string) - strlen(serno);
		      if (x < 0 || !str_caseeq(string + x, serno))
			{
			  pmsg_debug("%s(): serial number does not match\n", __func__);
			  usb_close(udev);
			  continue;
			}
		    }

		  if (dev->config == NULL)
		    {
		      pmsg_warning("USB device has no configuration\n");
		      goto trynext;
		    }

		  if (usb_set_configuration(udev, dev->config[0].bConfigurationValue))
		    {
		      pmsg_warning("(config %d) %s\n",
			dev->config[0].bConfigurationValue, usb_strerror());
		      /* let's hope it has already been configured */
		      // goto trynext;
		    }

		  for (iface = 0; iface < dev->config[0].bNumInterfaces; iface++)
		    {
		      cx->usb_interface = dev->config[0].interface[iface].altsetting[0].bInterfaceNumber;
#ifdef LIBUSB_HAS_GET_DRIVER_NP
		      /*
		       * Many Linux systems attach the usbhid driver
		       * by default to any HID-class device.  On
		       * those, the driver needs to be detached before
		       * we can claim the interface.
		       */
		      (void) usb_detach_kernel_driver_np(udev, cx->usb_interface);
#endif
		      if (usb_claim_interface(udev, cx->usb_interface))
			{
			  pmsg_warning("(i/face %d) %s\n", cx->usb_interface, usb_strerror());
			  cx->usb_access_error = 1;
			}
		      else
			{
			  if (pinfo.usbinfo.flags & PINFO_FL_USEHID)
			    {
			      /* only consider an interface that is of class HID */
			      if (dev->config[0].interface[iface].altsetting[0].bInterfaceClass !=
				  USB_CLASS_HID)
				continue;
			      fd->usb.use_interrupt_xfer = 1;
			    }
			  break;
			}
		    }
		  if (iface == dev->config[0].bNumInterfaces)
		    {
		      pmsg_warning("no usable interface found\n");
		      goto trynext;
		    }

		  fd->usb.handle = udev;
		  if (fd->usb.rep == 0)
		    {
		      /* Try finding out what our read endpoint is. */
		      for (i = 0; i < dev->config[0].interface[iface].altsetting[0].bNumEndpoints; i++)
			{
			  int possible_ep = dev->config[0].interface[iface].altsetting[0].
			  endpoint[i].bEndpointAddress;

			  if ((possible_ep & USB_ENDPOINT_DIR_MASK) != 0)
			    {
                              pmsg_notice2("%s(): using read endpoint 0x%02x\n", __func__, possible_ep);
			      fd->usb.rep = possible_ep;
			      break;
			    }
			}
		      if (fd->usb.rep == 0)
			{
			  pmsg_warning("cannot find a read endpoint, using 0x%02x\n",
                            USBDEV_BULK_EP_READ_MKII);
			  fd->usb.rep = USBDEV_BULK_EP_READ_MKII;
			}
		    }
		  for (i = 0; i < dev->config[0].interface[iface].altsetting[0].bNumEndpoints; i++)
		    {
		      if ((dev->config[0].interface[iface].altsetting[0].endpoint[i].bEndpointAddress == fd->usb.rep ||
			   dev->config[0].interface[iface].altsetting[0].endpoint[i].bEndpointAddress == fd->usb.wep) &&
			  dev->config[0].interface[iface].altsetting[0].endpoint[i].wMaxPacketSize < fd->usb.max_xfer)
			{
			    pmsg_notice("max packet size expected %d, but found %d due to EP 0x%02x's wMaxPacketSize\n",
			      fd->usb.max_xfer,
			      dev->config[0].interface[iface].altsetting[0].endpoint[i].wMaxPacketSize,
			      dev->config[0].interface[iface].altsetting[0].endpoint[i].bEndpointAddress);
			    fd->usb.max_xfer = dev->config[0].interface[iface].altsetting[0].endpoint[i].wMaxPacketSize;
			}
		    }
		  if (pinfo.usbinfo.flags & PINFO_FL_USEHID)
		    {
		      if (usb_control_msg(udev, 0x21, 0x0a /* SET_IDLE */, 0, 0, NULL, 0, 100) < 0)
			pmsg_warning("SET_IDLE failed\n");
		    }
		  return 0;

		trynext:
		  usb_close(udev);
		}
	      else
		pmsg_warning("cannot open device: %s\n", usb_strerror());
	    }
	}
    }

none_matching:
  if ((pinfo.usbinfo.flags & PINFO_FL_SILENT) == 0)
    pmsg_error("%s%s USB device %s (%04x:%04x)\n",
      cx->usb_access_error? "found but could not access": "did not find any",
      *serno && !cx->usb_access_error? " (matching)": "",
      port, pinfo.usbinfo.vid, pinfo.usbinfo.pid);

  return -1;
}

static void usbdev_close(union filedescriptor *fd)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;

  if (udev == NULL)
    return;

  (void) usb_release_interface(udev, cx->usb_interface);

#if defined(__linux__)
  /*
   * Without this reset, the AVRISP mkII seems to stall the second
   * time we try to connect to it.  This is not necessary on
   * FreeBSD.
   */
  usb_reset(udev);
#endif

  usb_close(udev);
}


static int usbdev_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;
  int rv;
  int i = mlen;
  const unsigned char * p = bp;
  int tx_size;

  if (udev == NULL)
    return -1;

  /*
   * Split the frame into multiple packets.  It's important to make
   * sure we finish with a short packet, or else the device won't know
   * the frame is finished.  For example, if we need to send 64 bytes,
   * we must send a packet of length 64 followed by a packet of length
   * 0.
   */
  do {
    tx_size = ((int) mlen < fd->usb.max_xfer)? (int) mlen: fd->usb.max_xfer;
    if (fd->usb.use_interrupt_xfer)
      rv = usb_interrupt_write(udev, fd->usb.wep, (char *)bp, tx_size, 10000);
    else
      rv = usb_bulk_write(udev, fd->usb.wep, (char *)bp, tx_size, 10000);
    if (rv != tx_size)
    {
        pmsg_error("wrote %d out of %d bytes, err = %s\n", rv, tx_size, usb_strerror());
        return -1;
    }
    bp += tx_size;
    mlen -= tx_size;
  } while (mlen > 0);

  if(verbose >= MSG_TRACE)
    trace_buffer(__func__, p, i);
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
static int usb_fill_buf(usb_dev_handle *udev, int maxsize, int ep, int use_interrupt_xfer)
{
  int rv;

  if (use_interrupt_xfer)
    rv = usb_interrupt_read(udev, ep, cx->usb_buf, maxsize, 10000);
  else
    rv = usb_bulk_read(udev, ep, cx->usb_buf, maxsize, 10000);
  if (rv < 0)
    {
      pmsg_notice2("%s(): usb_%s_read() error: %s\n", __func__,
        use_interrupt_xfer? "interrupt": "bulk", usb_strerror());
      return -1;
    }

  cx->usb_buflen = rv;
  cx->usb_bufptr = 0;

  return 0;
}

static int usbdev_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;
  int i, amnt;
  unsigned char * p = buf;

  if (udev == NULL)
    return -1;

  for (i = 0; nbytes > 0;)
    {
      if (cx->usb_buflen <= cx->usb_bufptr)
	{
	  if (usb_fill_buf(udev, fd->usb.max_xfer, fd->usb.rep, fd->usb.use_interrupt_xfer) < 0)
	    return -1;
	}
      amnt = cx->usb_buflen - cx->usb_bufptr > (int) nbytes? (int) nbytes: cx->usb_buflen - cx->usb_bufptr;
      memcpy(buf + i, cx->usb_buf + cx->usb_bufptr, amnt);
      cx->usb_bufptr += amnt;
      nbytes -= amnt;
      i += amnt;
    }

  if(verbose >= MSG_TRACE2)
    trace_buffer(__func__, p, i);

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
static int usbdev_recv_frame(const union filedescriptor *fd, unsigned char *buf, size_t nbytes)
{
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;
  int rv, n;
  unsigned char *p = buf;

  if (udev == NULL)
    return -1;

  /* If there's an event EP, and it has data pending, return it first. */
  if (fd->usb.eep != 0)
  {
      rv = usb_bulk_read(udev, fd->usb.eep, cx->usb_buf,
                         fd->usb.max_xfer, 1);
      if (rv > 4)
      {
	  memcpy(buf, cx->usb_buf, rv);
	  n = rv;
	  n |= USB_RECV_FLAG_EVENT;
	  goto printout;
      }
      else if (rv > 0)
      {
	  pmsg_warning("short event len = %d, ignored\n", rv);
	  /* fallthrough */
      }
  }

  n = 0;
  do
    {
      if (fd->usb.use_interrupt_xfer)
	rv = usb_interrupt_read(udev, fd->usb.rep, cx->usb_buf,
				fd->usb.max_xfer, 10000);
      else
	rv = usb_bulk_read(udev, fd->usb.rep, cx->usb_buf,
			   fd->usb.max_xfer, 10000);
      if (rv < 0)
	{
          pmsg_notice2("%s(): usb_%s_read(): %s\n", __func__,
            fd->usb.use_interrupt_xfer? "interrupt": "bulk", usb_strerror());
	  return -1;
	}

      if (rv <= (int) nbytes)
	{
	  memcpy (buf, cx->usb_buf, rv);
	  buf += rv;
	}
      else
        {
            return -1; // buffer overflow
        }

      n += rv;
      nbytes -= rv;
    }
  while (nbytes > 0 && rv == fd->usb.max_xfer);

/*
 this ends when the buffer is completly filled (nbytes=0) or was too small (nbytes< 0)
 or a short packet is found.
 however we cannot say for nbytes=0 that there was really a packet completed, 
 we had to check the last rv value than for a short packet,
 but what happens if the packet does not end with a short packet?
 and what if the buffer is filled without the packet was completed?

 preconditions:
    expected packet is not a multiple of usb.max_xfer. (prevents further waiting)

    expected packet is shorter than the provided buffer (so it cannot filled completely)
    or buffer size is not a multiple of usb.max_xfer. (so it can clearly detected if the buffer was overflown.)
*/

  printout:
  if(verbose >= MSG_TRACE)
    trace_buffer(__func__, p, n & USB_RECV_LENGTH_MASK);

  return n;
}

static int usbdev_drain(const union filedescriptor *fd, int display)
{
  /*
   * There is not much point in trying to flush any data
   * on an USB endpoint, as the endpoint is supposed to
   * start afresh after being configured from the host.
   *
   * As trying to flush the data here caused strange effects
   * in some situations (see
   * https://savannah.nongnu.org/bugs/index.php?43268 )
   * better avoid it.
   */

  return 0;
}

/*
 * Device descriptor for the JTAG ICE mkII.
 */
struct serial_device usb_serdev =
{
  .open = usbdev_open,
  .close = usbdev_close,
  .rawclose = usbdev_close,
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
  .rawclose = usbdev_close,
  .send = usbdev_send,
  .recv = usbdev_recv_frame,
  .drain = usbdev_drain,
  .flags = SERDEV_FL_NONE,
};

#endif  /* HAVE_LIBUSB */
