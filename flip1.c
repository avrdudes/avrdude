/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2014 Joerg Wunsch
 *
 * This implementation has been cloned from FLIPv2 implementation
 * written by Kirill Levchenko.
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

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>

#if HAVE_STDINT_H
#include <stdint.h>
#elif HAVE_INTTYPES_H
#include <inttypes.h>
#endif

#include "flip1.h"
#include "dfu.h"

#include "avrdude.h"
#include "avr.h"
#include "pgm.h"

#include "usbdevs.h" /* for USB_VENDOR_ATMEL */

/* There are three versions of the FLIP protocol:
 *
 * Version 0: C51 parts
 * Version 1: megaAVR parts ("USB DFU Bootloader Datasheet" [doc7618])
 * Version 2: XMEGA parts (AVR4023 [doc8457])
 *
 * This implementation handles protocol version 1.
 */

/* EXPORTED CONSTANT STRINGS */

const char flip1_desc[] = "FLIP USB DFU protocol version 1 (doc7618)";

/* PRIVATE DATA STRUCTURES */

struct flip1
{
  struct dfu_dev *dfu;
  unsigned char part_sig[3];
  unsigned char part_rev;
  unsigned char boot_ver;
};

#define FLIP1(pgm) ((struct flip1 *)(pgm->cookie))

/* FLIP1 data structures and constants. */

#define FLIP1_CMD_PROG_START 0x01
#define FLIP1_CMD_DISPLAY_DATA 0x03
#define FLIP1_CMD_WRITE_COMMAND 0x04
#define FLIP1_CMD_READ_COMMAND 0x05
#define FLIP1_CMD_CHANGE_BASE_ADDRESS 0x06

enum flip1_mem_unit {
  FLIP1_MEM_UNIT_FLASH = 0x00,
  FLIP1_MEM_UNIT_EEPROM = 0x01
};

/* EXPORTED PROGRAMMER FUNCTION PROTOTYPES */

static int flip1_open(PROGRAMMER *pgm, char *port_spec);
static int flip1_initialize(PROGRAMMER* pgm, AVRPART *part);
static void flip1_close(PROGRAMMER* pgm);
static void flip1_enable(PROGRAMMER* pgm);
static void flip1_disable(PROGRAMMER* pgm);
static void flip1_display(PROGRAMMER* pgm, const char *prefix);
static int flip1_program_enable(PROGRAMMER* pgm, AVRPART *part);
static int flip1_chip_erase(PROGRAMMER* pgm, AVRPART *part);
static int flip1_read_byte(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned long addr, unsigned char *value);
static int flip1_write_byte(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned long addr, unsigned char value);
static int flip1_page_erase(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned int base_addr);
static int flip1_paged_load(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes);
static int flip1_paged_write(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes);
static int flip1_read_sig_bytes(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem);
static void flip1_setup(PROGRAMMER * pgm);
static void flip1_teardown(PROGRAMMER * pgm);

/* INTERNAL PROGRAMMER FUNCTION PROTOTYPES */

static void flip1_show_info(struct flip1 *flip1);

static int flip1_read_memory(struct dfu_dev *dfu,
  enum flip1_mem_unit mem_unit, uint32_t addr, void *ptr, int size);
static int flip1_write_memory(struct dfu_dev *dfu,
  enum flip1_mem_unit mem_unit, uint32_t addr, const void *ptr, int size);

static int flip1_read_max1k(struct dfu_dev *dfu,
  unsigned short offset, void *ptr, unsigned short size);
static int flip1_write_max1k(struct dfu_dev *dfu,
  unsigned short offset, const void *ptr, unsigned short size);

static const char * flip1_status_str(const struct dfu_status *status);
static const char * flip1_mem_unit_str(enum flip1_mem_unit mem_unit);
static enum flip1_mem_unit flip1_mem_unit(const char *name);

/* THE INITPGM FUNCTION DEFINITIONS */

void flip1_initpgm(PROGRAMMER *pgm)
{
  strcpy(pgm->type, "flip1");

  /* Mandatory Functions */
  pgm->initialize       = flip1_initialize;
  pgm->enable           = flip1_enable;
  pgm->disable          = flip1_disable;
  pgm->display          = flip1_display;
  pgm->program_enable   = flip1_program_enable;
  pgm->chip_erase       = flip1_chip_erase;
  pgm->open             = flip1_open;
  pgm->close            = flip1_close;
  pgm->page_erase       = flip1_page_erase;
  pgm->paged_load       = flip1_paged_load;
  pgm->paged_write      = flip1_paged_write;
  pgm->read_byte        = flip1_read_byte;
  pgm->write_byte       = flip1_write_byte;
  pgm->read_sig_bytes   = flip1_read_sig_bytes;
  pgm->setup            = flip1_setup;
  pgm->teardown         = flip1_teardown;
}

/* EXPORTED PROGRAMMER FUNCTION DEFINITIONS */

int flip1_open(PROGRAMMER *pgm, char *port_spec)
{
  FLIP1(pgm)->dfu = dfu_open(port_spec);
  return (FLIP1(pgm)->dfu != NULL) ? 0 : -1;
}

int flip1_initialize(PROGRAMMER* pgm, AVRPART *part)
{
  unsigned short vid, pid;
  int result;

  /* A note about return values. Negative return values from this function are
   * interpreted as failure by main(), from where this function is called.
   * However such failures are interpreted as a device signature check failure
   * and the user is adviced to use the -F option to override this check. In
   * our case, this is misleading, so we defer reporting an error until another
   * function is called. Thus, we always return 0 (success) from initialize().
   * I don't like this, but I don't want to mess with main().
   */

  /* The dfu_init() function will try to find the target part either based on
   * a USB address provided by the user with the -P option or by matching the
   * VID and PID of the device. The VID may be specified in the programmer
   * definition; if not specified, it defaults to USB_VENDOR_ATMEL (defined
   * in usbdevs.h). The PID may be specified either in the programmer
   * definition or the part definition; the programmer definition takes
   * priority. The default PID value is 0, which causes dfu_init() to ignore
   * the PID when matching a target device.
   */

  vid = (pgm->usbvid != 0) ? pgm->usbvid : USB_VENDOR_ATMEL;
  pid = (pgm->usbpid != 0) ? pgm->usbpid : part->usbpid;

  if (!ovsigck && (part->flags & AVRPART_HAS_PDI)) {
    fprintf(stderr,
            "%s: \"flip1\" (FLIP protocol version 1) is for AT90USB* and ATmega*U* devices.\n"
            "%s For Xmega devices, use \"flip2\".\n"
            "%s (Use -F to bypass this check.)\n",
            progname, progbuf, progbuf);
    return -1;
  }

  result = dfu_init(FLIP1(pgm)->dfu, vid, pid);

  if (result != 0)
    goto flip1_initialize_fail;

#if 0
  result = flip1_read_memory(FLIP1(pgm)->dfu,
    FLIP1_MEM_UNIT_SIGNATURE, 0, FLIP1(pgm)->part_sig, 4);

  if (result != 0)
    goto flip1_initialize_fail;

  result = flip1_read_memory(FLIP1(pgm)->dfu,
    FLIP1_MEM_UNIT_BOOTLOADER, 0, &FLIP1(pgm)->boot_ver, 1);

  if (result != 0)
    goto flip1_initialize_fail;
#endif

  if (verbose)
    flip1_show_info(FLIP1(pgm));

  return 0;

flip1_initialize_fail:
  dfu_close(FLIP1(pgm)->dfu);
  FLIP1(pgm)->dfu = NULL;
  return 0;
}

void flip1_close(PROGRAMMER* pgm)
{
  if (FLIP1(pgm)->dfu != NULL) {
    dfu_close(FLIP1(pgm)->dfu);
    FLIP1(pgm)->dfu = NULL;
  }
}

void flip1_enable(PROGRAMMER* pgm)
{
  /* Nothing to do. */
}

void flip1_disable(PROGRAMMER* pgm)
{
  /* Nothing to do. */
}

void flip1_display(PROGRAMMER* pgm, const char *prefix)
{
  /* Nothing to do. */
}

int flip1_program_enable(PROGRAMMER* pgm, AVRPART *part)
{
  /* I couldn't find anything that uses this function, although it is marked
   * as "mandatory" in pgm.c. In case anyone does use it, we'll report an
   * error if we failed to initialize.
   */

  return (FLIP1(pgm)->dfu != NULL) ? 0 : -1;
}

int flip1_chip_erase(PROGRAMMER* pgm, AVRPART *part)
{
#if 0
  struct dfu_status status;
  int cmd_result = 0;
  int aux_result;

  if (verbose > 1)
    fprintf(stderr, "%s: flip_chip_erase()\n", progname);

  struct flip1_cmd cmd = {
    FLIP1_CMD_GROUP_EXEC, FLIP1_CMD_CHIP_ERASE, { 0xFF, 0, 0, 0 }
  };

  for (;;) {
    cmd_result = dfu_dnload(FLIP1(pgm)->dfu, &cmd, sizeof(cmd));
    aux_result = dfu_getstatus(FLIP1(pgm)->dfu, &status);

    if (aux_result != 0)
      return aux_result;

    if (status.bStatus != DFU_STATUS_OK) {
      if (status.bStatus == ((FLIP1_STATUS_ERASE_ONGOING >> 8) & 0xFF) &&
          status.bState == ((FLIP1_STATUS_ERASE_ONGOING >> 0) & 0xFF))
      {
        continue;
      } else
        fprintf(stderr, "%s: Error: DFU status %s\n", progname,
          flip1_status_str(&status));
      dfu_clrstatus(FLIP1(pgm)->dfu);
    } else
      break;
  }

  return cmd_result;
#endif
}

int flip1_read_byte(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned long addr, unsigned char *value)
{
#if 0
  enum flip1_mem_unit mem_unit;

  if (FLIP1(pgm)->dfu == NULL)
    return -1;

  mem_unit = flip1_mem_unit(mem->desc);

  if (mem_unit == FLIP1_MEM_UNIT_UNKNOWN) {
    fprintf(stderr, "%s: Error: "
      "\"%s\" memory not accessible using FLIP",
      progname, mem->desc);
    fprintf(stderr, "\n");
    return -1;
  }

  return flip1_read_memory(FLIP1(pgm)->dfu, mem_unit, addr, value, 1);
#endif
}

int flip1_write_byte(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned long addr, unsigned char value)
{
#if 0
  enum flip1_mem_unit mem_unit;

  if (FLIP1(pgm)->dfu == NULL)
    return -1;

  mem_unit = flip1_mem_unit(mem->desc);

  if (mem_unit == FLIP1_MEM_UNIT_UNKNOWN) {
    fprintf(stderr, "%s: Error: "
      "\"%s\" memory not accessible using FLIP",
      progname, mem->desc);
    fprintf(stderr, "\n");
    return -1;
  }

  return flip1_write_memory(FLIP1(pgm)->dfu, mem_unit, addr, &value, 1);
#endif
}

int flip1_page_erase(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned int base_addr)
{
  return 0;
}

int flip1_paged_load(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes)
{
#if 0
  enum flip1_mem_unit mem_unit;
  int result;

  if (FLIP1(pgm)->dfu == NULL)
    return -1;

  mem_unit = flip1_mem_unit(mem->desc);

  if (mem_unit == FLIP1_MEM_UNIT_UNKNOWN) {
    fprintf(stderr, "%s: Error: "
      "\"%s\" memory not accessible using FLIP",
      progname, mem->desc);
    fprintf(stderr, "\n");
    return -1;
  }

  if (n_bytes > INT_MAX) {
    /* This should never happen, unless the int type is only 16 bits. */
    fprintf(stderr, "%s: Error: Attempting to read more than %d bytes\n",
      progname, INT_MAX);
    exit(1);
  }

  result = flip1_read_memory(FLIP1(pgm)->dfu, mem_unit, addr,
    mem->buf + addr, n_bytes);

  return (result == 0) ? n_bytes : -1;
#endif
}

int flip1_paged_write(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes)
{
#if 0
  enum flip1_mem_unit mem_unit;
  int result;

  if (FLIP1(pgm)->dfu == NULL)
    return -1;

  mem_unit = flip1_mem_unit(mem->desc);

  if (mem_unit == FLIP1_MEM_UNIT_UNKNOWN) {
    fprintf(stderr, "%s: Error: "
      "\"%s\" memory not accessible using FLIP",
      progname, mem->desc);
    fprintf(stderr, "\n");
    return -1;
  }

  if (n_bytes > INT_MAX) {
    /* This should never happen, unless the int type is only 16 bits. */
    fprintf(stderr, "%s: Error: Attempting to read more than %d bytes\n",
      progname, INT_MAX);
    exit(1);
  }

  result = flip1_write_memory(FLIP1(pgm)->dfu, mem_unit, addr,
    mem->buf + addr, n_bytes);

  return (result == 0) ? n_bytes : -1;
#endif
}

int flip1_read_sig_bytes(PROGRAMMER* pgm, AVRPART *part, AVRMEM *mem)
{
#if 0
  if (FLIP1(pgm)->dfu == NULL)
    return -1;

  if (mem->size < sizeof(FLIP1(pgm)->part_sig)) {
    fprintf(stderr, "%s: Error: Signature read must be at least %u bytes\n",
      progname, (unsigned int) sizeof(FLIP1(pgm)->part_sig));
    return -1;
  }

  memcpy(mem->buf, FLIP1(pgm)->part_sig, sizeof(FLIP1(pgm)->part_sig));
  return 0;
#endif
}

void flip1_setup(PROGRAMMER * pgm)
{
  pgm->cookie = calloc(1, sizeof(struct flip1));

  if (pgm->cookie == NULL) {
    perror(progname);
    exit(1);
  }
}

void flip1_teardown(PROGRAMMER * pgm)
{
  free(pgm->cookie);
  pgm->cookie = NULL;
}

/* INTERNAL FUNCTION DEFINITIONS
 */

void flip1_show_info(struct flip1 *flip1)
{
  dfu_show_info(flip1->dfu);

  fprintf(stderr, "    Part signature      : 0x%02X%02X%02X\n",
    (int) flip1->part_sig[0],
    (int) flip1->part_sig[1],
    (int) flip1->part_sig[2]);

  if (flip1->part_rev < 26)
    fprintf(stderr, "    Part revision       : %c\n",
      (char) (flip1->part_rev + 'A'));
  else
    fprintf(stderr, "    Part revision       : %c%c\n",
      (char) (flip1->part_rev / 26 - 1 + 'A'),
      (char) (flip1->part_rev % 26 + 'A'));

  fprintf(stderr, "    Bootloader version  : 2.%hu.%hu\n",
    ((unsigned short) flip1->boot_ver >> 4) & 0xF,
    ((unsigned short) flip1->boot_ver >> 0) & 0xF);

  fprintf(stderr, "    USB max packet size : %hu\n",
    (unsigned short) flip1->dfu->dev_desc.bMaxPacketSize0);
}

int flip1_read_memory(struct dfu_dev *dfu,
  enum flip1_mem_unit mem_unit, uint32_t addr, void *ptr, int size)
{
#if 0
  unsigned short prev_page_addr;
  unsigned short page_addr;
  const char * mem_name;
  int read_size;
  int result;

  if (verbose > 1)
    fprintf(stderr,
            "%s: flip_read_memory(%s, 0x%04x, %d)\n",
            progname, flip1_mem_unit_str(mem_unit), addr, size);

  result = flip1_set_mem_unit(dfu, mem_unit);

  if (result != 0) {
    if ((mem_name = flip1_mem_unit_str(mem_unit)) != NULL)
      fprintf(stderr, "%s: Error: Failed to set memory unit 0x%02X (%s)\n",
        progname, (int) mem_unit, mem_name);
    else
      fprintf(stderr, "%s: Error: Failed to set memory unit 0x%02X\n",
        progname, (int) mem_unit);
    return -1;
  }

  page_addr = addr >> 16;
  result = flip1_set_mem_page(dfu, page_addr);

  if (result != 0) {
    fprintf(stderr, "%s: Error: Failed to set memory page 0x%04hX\n",
        progname, page_addr);
    return -1;
  }

  while (size > 0) {
    prev_page_addr = page_addr;
    page_addr = addr >> 16;

    if (page_addr != prev_page_addr) {
      result = flip1_set_mem_page(dfu, page_addr);
      if (result != 0) {
        fprintf(stderr, "%s: Error: Failed to set memory page 0x%04hX\n",
            progname, page_addr);
        return -1;
      }
    }

    read_size = (size > 0x400) ? 0x400 : size;
    result = flip1_read_max1k(dfu, addr & 0xFFFF, ptr, read_size);

    if (result != 0) {
      fprintf(stderr, "%s: Error: Failed to read 0x%04X bytes at 0x%04lX\n",
          progname, read_size, (unsigned long) addr);
      return -1;
    }

    ptr += read_size;
    addr += read_size;
    size -= read_size;
  }

  return 0;
#endif
}

int flip1_write_memory(struct dfu_dev *dfu,
  enum flip1_mem_unit mem_unit, uint32_t addr, const void *ptr, int size)
{
#if 0
  unsigned short prev_page_addr;
  unsigned short page_addr;
  const char * mem_name;
  int write_size;
  int result;

  if (verbose > 1)
    fprintf(stderr,
            "%s: flip_write_memory(%s, 0x%04x, %d)\n",
            progname, flip1_mem_unit_str(mem_unit), addr, size);

  result = flip1_set_mem_unit(dfu, mem_unit);

  if (result != 0) {
    if ((mem_name = flip1_mem_unit_str(mem_unit)) != NULL)
      fprintf(stderr, "%s: Error: Failed to set memory unit 0x%02X (%s)\n",
        progname, (int) mem_unit, mem_name);
    else
      fprintf(stderr, "%s: Error: Failed to set memory unit 0x%02X\n",
        progname, (int) mem_unit);
    return -1;
  }

  page_addr = addr >> 16;
  result = flip1_set_mem_page(dfu, page_addr);

  if (result != 0) {
    fprintf(stderr, "%s: Error: Failed to set memory page 0x%04hX\n",
        progname, page_addr);
    return -1;
  }

  while (size > 0) {
    prev_page_addr = page_addr;
    page_addr = addr >> 16;

    if (page_addr != prev_page_addr) {
      result = flip1_set_mem_page(dfu, page_addr);
      if (result != 0) {
        fprintf(stderr, "%s: Error: Failed to set memory page 0x%04hX\n",
            progname, page_addr);
        return -1;
      }
    }

    write_size = (size > 0x800) ? 0x800 : size;
    result = flip1_write_max1k(dfu, addr & 0xFFFF, ptr, write_size);

    if (result != 0) {
      fprintf(stderr, "%s: Error: Failed to write 0x%04X bytes at 0x%04lX\n",
          progname, write_size, (unsigned long) addr);
      return -1;
    }

    ptr += write_size;
    addr += write_size;
    size -= write_size;
  }

  return 0;
#endif
}

int flip1_read_max1k(struct dfu_dev *dfu,
  unsigned short offset, void *ptr, unsigned short size)
{
#if 0
  struct dfu_status status;
  int cmd_result = 0;
  int aux_result;

  struct flip1_cmd cmd = {
    FLIP1_CMD_GROUP_UPLOAD, FLIP1_CMD_READ_MEMORY, { 0, 0, 0, 0 }
  };

  cmd.args[0] = (offset >> 8) & 0xFF;
  cmd.args[1] = (offset >> 0) & 0xFF;
  cmd.args[2] = ((offset+size-1) >> 8) & 0xFF;
  cmd.args[3] = ((offset+size-1) >> 0) & 0xFF;

  cmd_result = dfu_dnload(dfu, &cmd, sizeof(cmd));

  if (cmd_result != 0)
    goto flip1_read_max1k_status;

  cmd_result = dfu_upload(dfu, (char*) ptr, size);

flip1_read_max1k_status:

  aux_result = dfu_getstatus(dfu, &status);

  if (aux_result != 0)
    return aux_result;

  if (status.bStatus != DFU_STATUS_OK) {
    if (status.bStatus == ((FLIP1_STATUS_OUTOFRANGE >> 8) & 0xFF) &&
        status.bState == ((FLIP1_STATUS_OUTOFRANGE >> 0) & 0xFF))
    {
      fprintf(stderr, "%s: Error: Address out of range [0x%04hX,0x%04hX]\n",
        progname, offset, offset+size-1);
    } else
      fprintf(stderr, "%s: Error: DFU status %s\n", progname,
        flip1_status_str(&status));
    dfu_clrstatus(dfu);
  }

  return cmd_result;
#endif
}

int flip1_write_max1k(struct dfu_dev *dfu,
  unsigned short offset, const void *ptr, unsigned short size)
{
#if 0
  char buffer[64+64+0x400];
  unsigned short data_offset;
  struct dfu_status status;
  int cmd_result = 0;
  int aux_result;

  struct flip1_cmd cmd = {
    FLIP1_CMD_GROUP_DOWNLOAD, FLIP1_CMD_PROG_START, { 0, 0, 0, 0 }
  };

  cmd.args[0] = (offset >> 8) & 0xFF;
  cmd.args[1] = (offset >> 0) & 0xFF;
  cmd.args[2] = ((offset+size-1) >> 8) & 0xFF;
  cmd.args[3] = ((offset+size-1) >> 0) & 0xFF;

  if (size > 0x400) {
    fprintf(stderr, "%s: Error: Write block too large (%hu > 1024)\n",
      progname, size);
    exit(1);
  }

  /* There are some special padding requirements for writes. The first packet
   * must consist only of the FLIP1 command data, which must be padded to
   * fill out the USB packet (the packet size is given by bMaxPacketSize0 in
   * the device descriptor). In addition, the data must be padded so that the
   * first byte of data to be written is at located at position (offset mod
   * bMaxPacketSize0) within the packet.
   */

  data_offset = dfu->dev_desc.bMaxPacketSize0;
  data_offset += offset % dfu->dev_desc.bMaxPacketSize0;

  memcpy(buffer, &cmd, sizeof(cmd));
  memset(buffer + sizeof(cmd), 0, data_offset - sizeof(cmd));
  memcpy(buffer + data_offset, ptr, size);

  cmd_result = dfu_dnload(dfu, buffer, data_offset + size);

  aux_result = dfu_getstatus(dfu, &status);

  if (aux_result != 0)
    return aux_result;

  if (status.bStatus != DFU_STATUS_OK) {
    if (status.bStatus == ((FLIP1_STATUS_OUTOFRANGE >> 8) & 0xFF) &&
        status.bState == ((FLIP1_STATUS_OUTOFRANGE >> 0) & 0xFF))
    {
      fprintf(stderr, "%s: Error: Address out of range [0x%04hX,0x%04hX]\n",
        progname, offset, offset+size-1);
    } else
      fprintf(stderr, "%s: Error: DFU status %s\n", progname,
        flip1_status_str(&status));
    dfu_clrstatus(dfu);
  }

  return cmd_result;
#endif
}

const char * flip1_status_str(const struct dfu_status *status)
{
  // XXX
}

const char * flip1_mem_unit_str(enum flip1_mem_unit mem_unit)
{
  switch (mem_unit) {
  case FLIP1_MEM_UNIT_FLASH: return "Flash";
  case FLIP1_MEM_UNIT_EEPROM: return "EEPROM";
  default: return "unknown";
  }
}

enum flip1_mem_unit flip1_mem_unit(const char *name) {
  if (strcasecmp(name, "flash") == 0)
    return FLIP1_MEM_UNIT_FLASH;
  if (strcasecmp(name, "eeprom") == 0)
    return FLIP1_MEM_UNIT_EEPROM;
}
