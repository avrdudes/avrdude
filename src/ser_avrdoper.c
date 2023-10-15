/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003-2004  Theodore A. Roth  <troth@openavr.org>
 * Copyright (C) 2006 Joerg Wunsch <j@uriah.heep.sax.de>
 * Copyright (C) 2006 Christian Starkjohann
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
 * Serial Interface emulation for USB programmer "AVR-Doper" in HID mode.
 */

#include "ac_cfg.h"

#if defined(HAVE_LIBHIDAPI)

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <hidapi/hidapi.h>

#include "avrdude.h"
#include "libavrdude.h"

/* ------------------------------------------------------------------------ */

/* Numeric constants for 'reportType' parameters */
#define USB_HID_REPORT_TYPE_INPUT   1
#define USB_HID_REPORT_TYPE_OUTPUT  2
#define USB_HID_REPORT_TYPE_FEATURE 3

/* These are the error codes which can be returned by functions of this
 * module.
 */
#define USB_ERROR_NONE      0
#define USB_ERROR_ACCESS    1
#define USB_ERROR_NOTFOUND  2
#define USB_ERROR_BUSY      16
#define USB_ERROR_IO        5

#define USB_VENDOR_ID   0x16c0
#define USB_PRODUCT_ID  0x05df

static const int reportDataSizes[4] = {13, 29, 61, 125};

static unsigned char    avrdoperRxBuffer[280];  /* buffer for receive data */
static int              avrdoperRxLength = 0;   /* amount of valid bytes in rx buffer */
static int              avrdoperRxPosition = 0; /* amount of bytes already consumed in rx buffer */

/* ------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */

static int usbOpenDevice(union filedescriptor *fdp, int vendor, const char *vendorName,
			 int product, const char *productName, int doReportIDs)
{
    hid_device *dev;

    dev = hid_open(vendor, product, NULL);
    if (dev == NULL)
    {
      pmsg_ext_error("no device found\n");
      return USB_ERROR_NOTFOUND;
    }
    fdp->usb.handle = dev;
    return USB_ERROR_NONE;
}

/* ------------------------------------------------------------------------- */

static void usbCloseDevice(union filedescriptor *fdp)
{
  hid_device *udev = (hid_device *)fdp->usb.handle;
  fdp->usb.handle = NULL;

  if (udev == NULL)
    return;

  hid_close(udev);
}

/* ------------------------------------------------------------------------- */

static int usbSetReport(const union filedescriptor *fdp, int reportType, char *buffer, int len) {
  hid_device *udev = (hid_device *)fdp->usb.handle;
  int bytesSent = -1;

  switch(reportType){
  case USB_HID_REPORT_TYPE_INPUT:
      break;
  case USB_HID_REPORT_TYPE_OUTPUT:
      bytesSent = hid_write(udev, (unsigned char*) buffer, len);
      break;
  case USB_HID_REPORT_TYPE_FEATURE:
      bytesSent = hid_send_feature_report(udev, (unsigned char*) buffer, len);
      break;
  }

  if(bytesSent != len){
      if(bytesSent < 0)
          pmsg_error("unable to send message: %ls\n", hid_error(udev));
      return USB_ERROR_IO;
  }
  return USB_ERROR_NONE;
}

/* ------------------------------------------------------------------------- */

static int usbGetReport(const union filedescriptor *fdp, int reportType, int reportNumber,
			char *buffer, int *len)
{
  hid_device *udev = (hid_device *)fdp->usb.handle;
  int bytesReceived = -1;

  switch(reportType){
  case USB_HID_REPORT_TYPE_INPUT:
      bytesReceived = hid_read_timeout(udev, (unsigned char*) buffer, *len, 300);
      break;
  case USB_HID_REPORT_TYPE_OUTPUT:
      break;
  case USB_HID_REPORT_TYPE_FEATURE:
      bytesReceived = hid_get_feature_report(udev, (unsigned char*) buffer, *len);
      break;
  }
  if(bytesReceived < 0){
      pmsg_error("unable to send message: %ls\n", hid_error(udev));
      return USB_ERROR_IO;
  }
  *len = bytesReceived;
  return USB_ERROR_NONE;
}
/* ------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */


/* ------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */

/* ------------------------------------------------------------------------- */

static void dumpBlock(const char *prefix, const unsigned char *buf, int len)
{
    int i;

    if(len <= 8){   /* more compact format for short blocks */
        msg_info("%s: %d bytes: ", prefix, len);
        for(i = 0; i < len; i++){
            msg_info("%02x ", buf[i]);
        }
        msg_info(" \"");
        for(i = 0; i < len; i++)
            msg_info("%c", buf[i] >= 0x20 && buf[i] < 0x7f? buf[i]: '.');
        msg_info("\"\n");
    }else{
        msg_info("%s: %d bytes:\n", prefix, len);
        while(len > 0){
            for(i = 0; i < 16; i++){
                if(i < len){
                    msg_info("%02x ", buf[i]);
                }else{
                    msg_info("   ");
                }
                if(i == 7)
                    msg_info(" ");
            }
            msg_info("  \"");
            for(i = 0; i < 16 && i < len; i++)
                msg_info("%c", buf[i] >= 0x20 && buf[i] < 0x7f? buf[i]: '.');
            msg_info("\"\n");
            buf += 16;
            len -= 16;
        }
    }
}

static char *usbErrorText(int usbErrno)
{
    static char buffer[32];

    switch(usbErrno){
        case USB_ERROR_NONE:    return "Success";
        case USB_ERROR_ACCESS:  return "Access denied";
        case USB_ERROR_NOTFOUND:return "Device not found";
        case USB_ERROR_BUSY:    return "Device is busy";
        case USB_ERROR_IO:      return "I/O Error";
        default:
            sprintf(buffer, "Unknown error %d", usbErrno);
            return buffer;
    }
}

/* ------------------------------------------------------------------------- */

static int avrdoper_open(const char *port, union pinfo pinfo, union filedescriptor *fdp)
{
    int rval;
    char *vname = "obdev.at";
    char *devname = "AVR-Doper";

    rval = usbOpenDevice(fdp, USB_VENDOR_ID, vname, USB_PRODUCT_ID, devname, 1);
    if(rval != 0){
        pmsg_ext_error("%s\n", usbErrorText(rval));
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */

static void avrdoper_close(union filedescriptor *fdp)
{
    usbCloseDevice(fdp);
}

/* ------------------------------------------------------------------------- */

static int  chooseDataSize(int len)
{
    size_t i;

    for(i = 0; i < sizeof(reportDataSizes)/sizeof(reportDataSizes[0]); i++){
        if(reportDataSizes[i] >= len)
            return i;
    }
    return i - 1;
}

static int avrdoper_send(const union filedescriptor *fdp, const unsigned char *buf, size_t buflen)
{
    if(buflen > INT_MAX) {
        pmsg_error("%s() called with too large buflen = %lu\n", __func__, (unsigned long) buflen);
        return -1;
    }
    if(verbose > 3)
        dumpBlock("Send", buf, buflen);
    while(buflen > 0){
        unsigned char buffer[256];
        int rval, lenIndex = chooseDataSize(buflen);
        int thisLen = (int) buflen > reportDataSizes[lenIndex]?
            reportDataSizes[lenIndex]: (int) buflen;
        buffer[0] = lenIndex + 1;   /* report ID */
        buffer[1] = thisLen;
        memcpy(buffer + 2, buf, thisLen);
        msg_trace("Sending %d bytes data chunk\n", thisLen);
        rval = usbSetReport(fdp, USB_HID_REPORT_TYPE_FEATURE, (char *)buffer,
			    reportDataSizes[lenIndex] + 2);
        if(rval != 0){
            pmsg_error("%s\n", usbErrorText(rval));
            return -1;
        }
        buflen -= thisLen;
        buf += thisLen;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */

static int avrdoperFillBuffer(const union filedescriptor *fdp) {
    int bytesPending = reportDataSizes[1];  /* guess how much data is buffered in device */

    avrdoperRxPosition = avrdoperRxLength = 0;
    while(bytesPending > 0){
        int len, usbErr, lenIndex = chooseDataSize(bytesPending);
        unsigned char buffer[128];
        len = sizeof(avrdoperRxBuffer) - avrdoperRxLength;  /* bytes remaining */
        if(reportDataSizes[lenIndex] + 2 > len) /* requested data would not fit into buffer */
            break;
        len = reportDataSizes[lenIndex] + 2;
        usbErr = usbGetReport(fdp, USB_HID_REPORT_TYPE_FEATURE, lenIndex + 1,
			      (char *)buffer, &len);
        if(usbErr != 0){
            pmsg_error("%s\n", usbErrorText(usbErr));
            return -1;
        }
        msg_trace("Received %d bytes data chunk of total %d\n", len - 2, buffer[1]);
        len -= 2;   /* compensate for report ID and length byte */
        bytesPending = buffer[1] - len; /* amount still buffered */
        if(len > buffer[1])             /* cut away padding */
            len = buffer[1];
        if(avrdoperRxLength + len > (int) sizeof(avrdoperRxBuffer)){
            pmsg_error("buffer overflow\n");
            return -1;
        }
        memcpy(avrdoperRxBuffer + avrdoperRxLength, buffer + 2, len);
        avrdoperRxLength += len;
    }
    return 0;
}

static int avrdoper_recv(const union filedescriptor *fdp, unsigned char *buf, size_t buflen)
{
    unsigned char   *p = buf;
    int             remaining = buflen;

    while(remaining > 0){
        int len, available = avrdoperRxLength - avrdoperRxPosition;
        if(available <= 0){ /* buffer is empty */
            if (avrdoperFillBuffer(fdp) < 0)
                return -1;
            continue;
        }
        len = remaining < available ? remaining : available;
        memcpy(p, avrdoperRxBuffer + avrdoperRxPosition, len);
        p += len;
        remaining -= len;
        avrdoperRxPosition += len;
    }
    if(verbose > 3)
        dumpBlock("Receive", buf, buflen);
    return 0;
}

/* ------------------------------------------------------------------------- */

static int avrdoper_drain(const union filedescriptor *fdp, int display)
{
    do{
        if (avrdoperFillBuffer(fdp) < 0)
            return -1;
    }while(avrdoperRxLength > 0);
    return 0;
}

/* ------------------------------------------------------------------------- */

static int avrdoper_set_dtr_rts(const union filedescriptor *fdp, int is_on)
{
    pmsg_error("AVR-Doper does not support DTR/RTS setting\n");
    return -1;
}

/* ------------------------------------------------------------------------- */

struct serial_device avrdoper_serdev =
{
  .open = avrdoper_open,
  .close = avrdoper_close,
  .rawclose = avrdoper_close,
  .send = avrdoper_send,
  .recv = avrdoper_recv,
  .drain = avrdoper_drain,
  .set_dtr_rts = avrdoper_set_dtr_rts,
  .flags = SERDEV_FL_NONE,
};

#endif /* defined(HAVE_LIBHIDAPI) */
