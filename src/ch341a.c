/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 *
 * avrdude support for CH341A/B
 * Copyright (C) 2016  Alexey Sadkov, paged access by smr 2023
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
 * Interface to the CH341A programmer
 *
 */
#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "ch341a.h"
#include "usbdevs.h"

#if defined(HAVE_LIBUSB_1_0)

#define USE_LIBUSB_1_0

#if defined(HAVE_LIBUSB_1_0_LIBUSB_H)
#include <libusb-1.0/libusb.h>
#else
#include <libusb.h>
#endif

#include <sys/time.h>

#ifdef USE_LIBUSB_1_0
static libusb_context *ctx = NULL;

static int libusb_to_errno(int result) {
  switch (result) {
  case LIBUSB_SUCCESS:
    return 0;
  case LIBUSB_ERROR_IO:
    return EIO;
  case LIBUSB_ERROR_INVALID_PARAM:
    return EINVAL;
  case LIBUSB_ERROR_ACCESS:
    return EACCES;
  case LIBUSB_ERROR_NO_DEVICE:
    return ENXIO;
  case LIBUSB_ERROR_NOT_FOUND:
    return ENOENT;
  case LIBUSB_ERROR_BUSY:
    return EBUSY;

#ifdef ETIMEDOUT
  case LIBUSB_ERROR_TIMEOUT:
    return ETIMEDOUT;
#endif

#ifdef EOVERFLOW
  case LIBUSB_ERROR_OVERFLOW:
    return EOVERFLOW;
#endif

  case LIBUSB_ERROR_PIPE:
    return EPIPE;
  case LIBUSB_ERROR_INTERRUPTED:
    return EINTR;
  case LIBUSB_ERROR_NO_MEM:
    return ENOMEM;
  case LIBUSB_ERROR_NOT_SUPPORTED:
    return ENOSYS;
  default:
    return ERANGE;
  }
}
#endif

// Private data for this programmer
struct pdata {
  libusb_device_handle *usbhandle;
  int sckfreq_hz;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

// ----------------------------------------------------------------------

static void ch341a_setup(PROGRAMMER *pgm);
static void ch341a_teardown(PROGRAMMER *pgm);
static int ch341a_open(PROGRAMMER *pgm, const char *port);
static void ch341a_close(PROGRAMMER *pgm);
static int ch341a_initialize(const PROGRAMMER *pgm, const AVRPART *p);
static int ch341a_spi_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res);
static int ch341a_spi(const PROGRAMMER *pgm, const unsigned char *in, unsigned char *out, int size);
static int ch341a_spi_program_enable(const PROGRAMMER *pgm, const AVRPART *p);
static int ch341a_spi_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);

static void ch341a_disable(const PROGRAMMER *pgm);
static void ch341a_enable(PROGRAMMER *pgm, const AVRPART *p);
static void ch341a_display(const PROGRAMMER *pgm, const char *p);


// ch341 requires LSB first: invert the bit order before sending and after receiving
static unsigned char swap_byte(unsigned char byte) {
  byte = ((byte >> 1) & 0x55) | ((byte << 1) & 0xaa);
  byte = ((byte >> 2) & 0x33) | ((byte << 2) & 0xcc);
  byte = ((byte >> 4) & 0x0f) | ((byte << 4) & 0xf0);

  return byte;
}

static int CH341USBTransferPart(const PROGRAMMER *pgm, enum libusb_endpoint_direction dir,
  unsigned char *buff, unsigned int size) {

  int ret, bytestransferred;

  if(!PDATA(pgm)->usbhandle)
    return 0;

  if((ret = libusb_bulk_transfer(PDATA(pgm)->usbhandle, CH341A_USB_BULK_ENDPOINT | dir,
    buff, size, &bytestransferred, CH341A_USB_TIMEOUT))) {

    pmsg_error("libusb_bulk_transfer for IN_EP failed, return value %d (%s)\n", ret, libusb_error_name(ret));
    return -1;
  }

  return bytestransferred;
}

static bool CH341USBTransfer(const PROGRAMMER *pgm, enum libusb_endpoint_direction dir,
  unsigned char *buff, unsigned int size) {

  int pos = 0, bytestransferred;

  while(size) {
    bytestransferred = CH341USBTransferPart(pgm, dir, buff + pos, size);
    if(bytestransferred <= 0)
      return false;
    pos += bytestransferred;
    size -= bytestransferred;
  }

  return true;
}

/*
 * Below the assumed map between UIO command bits, pins on CH341A chip and
 * pins on SPI chip. The UIO stream commands only have 6 bits of output,
 * D6/D7 are SPI inputs.
 *
 * UIO  CH341A pin/name  AVR target
 * -------------------------------------------
 *  D0           15/CS0  RESET
 *  D1           16/CS1  (unused)
 *  D2           17/CS2  (unused)
 *  D3           18/DCK  SCK
 *  D4         19/DOUT2  (unused)
 *  D5          20/DOUT  SDI
 *  D6          21/DIN2  (unused)
 *  D7           22/DIN  SDO
 */

bool CH341ChipSelect(const PROGRAMMER *pgm, unsigned int cs, bool enable) {
  unsigned char cmd[4], res[4];

  memset(cmd, 0, sizeof(cmd));
  memset(res, 0, sizeof(res));
  pmsg_trace("ch341a_ChipSelect()\n");
  if(cs > 2) {
    pmsg_error("invalid CS pin %d, 0~2 are available\n", cs);
    return false;
  }
  cmd[0] = CH341A_CMD_UIO_STREAM;
  if(enable)
    cmd[1] = CH341A_CMD_UIO_STM_OUT | (0x37 & ~(1 << cs));
  else
    cmd[1] = CH341A_CMD_UIO_STM_OUT | 0x37;
  cmd[2] = CH341A_CMD_UIO_STM_DIR | 0x3F;
  cmd[3] = CH341A_CMD_UIO_STM_END;

  return CH341USBTransferPart(pgm, LIBUSB_ENDPOINT_OUT, cmd, 4) > 0;
}


static int ch341a_open(PROGRAMMER *pgm, const char *port) {
  LNODEID usbpid = lfirst(pgm->usbpid);
  int pid, vid, j, r;
  int errorCode = USB_ERROR_NOTFOUND;
  libusb_device_handle *handle = NULL;
  static int didUsbInit = 0;

  pmsg_trace("ch341a_open(\"%s\")\n", port);

  if(!didUsbInit) {
    didUsbInit = 1;
    libusb_init(&ctx);
  }

  if(usbpid) {
    pid = *(int *) (ldata(usbpid));
    if(lnext(usbpid))
      pmsg_warning("using PID 0x%04x, ignoring remaining PIDs in list\n", pid);
  } else {
    pid = CH341A_PID;
  }
  vid = pgm->usbvid? pgm->usbvid: CH341A_VID;

  libusb_device **dev_list;
  int dev_list_len = libusb_get_device_list(ctx, &dev_list);

  for(j = 0; j < dev_list_len; ++j) {
    libusb_device *dev = dev_list[j];
    struct libusb_device_descriptor descriptor;

    libusb_get_device_descriptor(dev, &descriptor);
    if(descriptor.idVendor == vid && descriptor.idProduct == pid) {
      r = libusb_open(dev, &handle);
      if(!handle) {
        errorCode = USB_ERROR_ACCESS;
        pmsg_warning("cannot open USB device: %s\n", strerror(libusb_to_errno(r)));
        continue;
      }
    }
  }
  libusb_free_device_list(dev_list, 1);
  if(handle != NULL) {
    errorCode = 0;
    PDATA(pgm)->usbhandle = handle;
  }

  if(errorCode != 0) {
    pmsg_error("could not find USB device with vid=0x%x pid=0x%x\n", vid, pid);
    return -1;
  }
  if((r = libusb_claim_interface(PDATA(pgm)->usbhandle, 0))) {
    pmsg_error("libusb_claim_interface failed, return value %d (%s)\n", r, libusb_error_name(r));
    libusb_close(PDATA(pgm)->usbhandle);
    libusb_exit(ctx);
    return -1;
  }
  return 0;
}

static void ch341a_close(PROGRAMMER *pgm) {
  pmsg_trace("ch341a_close()\n");
  CH341ChipSelect(pgm, 0, false);

  if(PDATA(pgm)->usbhandle != NULL) {
    libusb_release_interface(PDATA(pgm)->usbhandle, 0);
    libusb_close(PDATA(pgm)->usbhandle);
  }
  libusb_exit(ctx);
}


static int ch341a_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_trace("ch341a_initialize()\n");
  if(!CH341ChipSelect(pgm, 0, false)) {
    pmsg_error("CH341ChipSelect(..., false) failed\n");
    return -1;
  }
  usleep(20 * 1000);
  if(!CH341ChipSelect(pgm, 0, true)) {
    pmsg_error("CH341ChipSelect(..., true) failed\n");
    return -1;
  }

  return pgm->program_enable(pgm, p);
}


static int ch341a_spi(const PROGRAMMER *pgm, const unsigned char *in, unsigned char *out, int size) {
  unsigned char pkt[CH341A_PACKET_LENGTH];

  if(!size)
    return 0;

  if(size > CH341A_PACKET_LENGTH - 1)
    size = CH341A_PACKET_LENGTH - 1;

  pkt[0] = CH341A_CMD_SPI_STREAM;

  for(int i = 0; i < size; i++)
    pkt[i+1] = swap_byte(in[i]);

  if(!CH341USBTransfer(pgm, LIBUSB_ENDPOINT_OUT, pkt, size + 1)) {
    pmsg_error("failed to transfer data to CH341\n");
    return -1;
  }

  if(!CH341USBTransfer(pgm, LIBUSB_ENDPOINT_IN, pkt, size)) {
    pmsg_error("failed to transfer data from CH341\n");
    return -1;
  }

  for(int i = 0; i < size; i++)
    out[i] = swap_byte(pkt[i]);

  return size;
}

static int ch341a_spi_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  return pgm->spi(pgm, cmd, res, 4);
}


static int ch341a_spi_chip_erase(const struct programmer_t *pgm, const AVRPART *p) {
  unsigned char cmd[4];
  unsigned char res[4];

  if(p->op[AVR_OP_CHIP_ERASE] == NULL) {
    pmsg_error("chip erase instruction not defined for part %s\n", p->desc);
    return -1;
  }
  memset(cmd, 0, sizeof(cmd));
  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
  pgm->cmd(pgm, cmd, res);
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);
  return 0;
}


// Fall back on bytewise write (followed by write page if flash)
static int ch341a_spi_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  int isflash = avr_mem_is_flash_type(m);

  if(n_bytes) {
    if(!isflash && !avr_mem_is_eeprom_type(m))
      return -2;

    // Always called with addr at page boundary and n_bytes == m->page_size
    for(unsigned int end = addr + n_bytes; addr < end; addr++)
      if(pgm->write_byte(pgm, p, m, addr, m->buf[addr]) < 0)
        return -1;
  }

  if(isflash && avr_write_page(pgm, p, m, addr - n_bytes) < 0)
    return -1;

  return n_bytes;
}

// Fall back on bytewise read
static int ch341a_spi_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  int isflash = avr_mem_is_flash_type(m);

  if(n_bytes) {
    if(!isflash && !avr_mem_is_eeprom_type(m))
      return -2;

    // Always called with addr at page boundary and n_bytes == m->page_size
    if(isflash && m->op[AVR_OP_LOAD_EXT_ADDR]) {
      unsigned char cmd[4], res[4];

      memset(cmd, 0, sizeof cmd);
      avr_set_bits(m->op[AVR_OP_LOAD_EXT_ADDR], cmd);
      avr_set_addr(m->op[AVR_OP_LOAD_EXT_ADDR], cmd, addr / 2);
      if(pgm->cmd(pgm, cmd, res) < 0)
        return -1;
    }

    for(unsigned int end = addr + n_bytes; addr < end; addr++)
      if(pgm->read_byte(pgm, p, m, addr, m->buf + addr) < 0)
        return -1;
  }

  return n_bytes;
}


static int ch341a_spi_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char res[4];
  unsigned char cmd[4];

  memset(cmd, 0, sizeof(cmd));
  memset(res, 0, sizeof(res));

  cmd[0] = 0;
  pmsg_trace("ch341a_program_enable() %p\n", p->op[AVR_OP_PGM_ENABLE]);

  if(p->op[AVR_OP_PGM_ENABLE] == NULL) {
    pmsg_error("program enable instruction not defined for part %s\n", p->desc);
    return -1;
  }
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
  pgm->cmd(pgm, cmd, res);

  pmsg_debug("%s resp = %02x %02x %02x %02x\n", __func__, res[0], res[1], res[2], res[3]);
  // Check for sync character
  if(res[2] != cmd[1])
    return -2;
  return 0;
}


// Interface management
static void ch341a_setup(PROGRAMMER *pgm) {
  pgm->cookie = cfg_malloc(__func__, sizeof(struct pdata));
}

static void ch341a_teardown(PROGRAMMER *pgm) {
  free(pgm->cookie);
}

// Dummy functions
static void ch341a_disable(const PROGRAMMER *pgm) {
  return;
}

static void ch341a_enable(PROGRAMMER *pgm, const AVRPART *p) {
  return;
}

static void ch341a_display(const PROGRAMMER *pgm, const char *p) {
  return;
}

void ch341a_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "ch341a");

  // Mandatory functions
  pgm->initialize = ch341a_initialize;
  pgm->display = ch341a_display;
  pgm->enable = ch341a_enable;
  pgm->disable = ch341a_disable;
  pgm->program_enable = ch341a_spi_program_enable;
  pgm->chip_erase = ch341a_spi_chip_erase;
  pgm->cmd = ch341a_spi_cmd;
  pgm->spi = ch341a_spi;
  pgm->open = ch341a_open;
  pgm->close = ch341a_close;
  pgm->read_byte = avr_read_byte_default;
  pgm->write_byte = avr_write_byte_default;

  // Optional functions
  pgm->paged_write = ch341a_spi_paged_write;
  pgm->paged_load = ch341a_spi_paged_load;
  pgm->setup = ch341a_setup;
  pgm->teardown = ch341a_teardown;
}

// ----------------------------------------------------------------------
#else // !defined(HAVE_LIBUSB_1_0)

static int ch341a_nousb_open(struct programmer_t *pgm, const char *name) {
  pmsg_error("no usb support, please compile again with libusb installed\n");
  return -1;
}

void ch341a_initpgm(PROGRAMMER * pgm) {
  strcpy(pgm->type, "ch341a");
  pgm->open = ch341a_nousb_open;
}
#endif // !defined(HAVE_LIBUSB_1_0)

const char ch341a_desc[] = "Driver for \"ch341a\"-type programmers";
