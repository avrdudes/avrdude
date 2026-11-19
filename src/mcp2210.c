/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2026 David Hubner
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
 *
 * This code is not affiliated in any way with Microchip®
 */

#include <ac_cfg.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#if defined(HAVE_LIBHIDAPI)

#include <hidapi/hidapi.h>

#define MCP2210_CHIP_STATUS 0x10
#define MCP2210_SPI_CANCEL_TRANSFER 0x11
#define MCP2210_SET_SETTINGS 0x40
#define MCP2210_GET_SETTINGS 0x41
#define MCP2210_SPI_XFER_COMMAND 0x42

#define MCP2210_SUCCESS 0x00
#define MCP2210_TRANSFER_IN_PROGRESS 0xF8
#define MCP2210_SPI_BUS_UNAVALIABLE 0xF7
#define MCP2210_XFER_DONE 0x10
#define MCP2210_XFER_STARTED 0x20
#define MCP2210_XFER_NOT_FINISHED 0x30

#define MCP2210_DEFAULT_VID 0x04d8
#define MCP2210_DEFAULT_PID 0x00DE
#define HID_REPORT_LENGTH 65
#define TRANSFER_RETRIES 30
#define SPI_MAX_CHUNK 60
#define MAXIMUM_TRANSFER 3000000

static int mcp2210_run_query(const PROGRAMMER *pgm, const unsigned char *request, unsigned char *response, const char *caller);
static int mcp2210_cancel_transfer(const PROGRAMMER *pgm);

struct mcp {
  hid_device *handle;
  uint8_t bitclock[4];
};

#define mine ((struct mcp *)(pgm->cookie))

static void mcp2210_setup(PROGRAMMER *pgm) {
  pgm->cookie = mmt_malloc(sizeof *mine);
}

static int mcp2210_set_settings(const PROGRAMMER *pgm, unsigned short len) {
  unsigned char settings_request[HID_REPORT_LENGTH] = {0x00, MCP2210_SET_SETTINGS}, settings_response[HID_REPORT_LENGTH] = {0};

  memcpy(settings_request + 5, mine->bitclock, 4);
  settings_request[19] = len;
  settings_request[20] = len >> 8;

  if(mcp2210_run_query(pgm, settings_request, settings_response, "set settings") == -1)
    return -1;

  if(settings_response[2] != MCP2210_SUCCESS) {
    if(settings_response[2] == MCP2210_TRANSFER_IN_PROGRESS)
      pmsg_error("USB transfer in progress; settings not written\n");
    else
      pmsg_error("unknown error code 0x%02x\n", settings_response[2]);

    return -1;
  }

  return 0;
}

static void mcp2210_teardown(PROGRAMMER *pgm) {
  free(pgm->cookie);
  pgm->cookie = NULL;
}

static int mcp2210_open(PROGRAMMER *pgm, const char *port) {
  int numids;
  char serno[64] = {0};
  unsigned short param_vid = 0, param_pid = 0;
  struct hid_device_info *list;

  if((numids = str_set_vid_pid_serno(port, &param_vid, &param_pid, serno, sizeof serno)) < 0) {
    pmsg_error("invalid -P %s; drop -P option or use -P usb[:<vid>:<pid>][:<serno>]\n", port);
    return -1;
  }

  LNODEID usbpid = lfirst(pgm->usbpid);
  int pid, vid = numids >= 2? param_vid: pgm->usbvid? pgm->usbvid: MCP2210_DEFAULT_VID;

  if(numids >= 2) {
    pid = param_pid;
  } else if(usbpid) {
    pid = *(int *) ldata(usbpid);

    if(lnext(usbpid))
      pmsg_warning("using PID 0x%04x, ignoring remaining PIDs in list\n", pid);
  } else
    pid = MCP2210_DEFAULT_PID;

  if(!*serno)
    mine->handle = hid_open(vid, pid, NULL);
  else {
    wchar_t wserno[sizeof serno] = {0};
    mbstowcs(wserno, serno, sizeof serno);
    size_t serlen = strlen(serno);

    list = hid_enumerate(vid, pid);
    while(list) {
      if(list->serial_number) {
        size_t slen = wcslen(list->serial_number);

        pmsg_notice("found device on %04x:%04x with serial number %ls\n", vid, pid, list->serial_number);
        if(slen >= serlen && wcscmp(list->serial_number + slen - serlen, wserno) == 0)
          break;
      }
      list = list->next;
    }

    if(list == NULL) {
      pmsg_error("no matching device found\n");
      hid_free_enumeration(list);
      return -1;
    }

    mine->handle = hid_open_path(list->path);
    hid_free_enumeration(list);
  }

  if(mine->handle == NULL) {
    pmsg_error("could not find MCP2210 device with vid=0x%04x pid=0x%04x%s\n",
      vid, pid, *serno? str_ccprintf(" serno=%s", serno): "");
    return -1;
  }

  return 0;
}

static void mcp2210_close(PROGRAMMER *pgm) {
  mcp2210_cancel_transfer(pgm);
  hid_close(mine->handle);
}

static int mcp2210_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  if(pgm->bitclock > 0 && !(pgm->extra_features & HAS_BITCLOCK_ADJ))
    pmsg_warning("setting bitclock despite HAS_BITCLOCK_ADJ missing in pgm->extra_features\n");

  uint32_t frequency = pgm->bitclock > 0? 1/pgm->bitclock: p->factory_fcpu > 0? p->factory_fcpu/4: 250000;

  // Set a limit of 3 Mbps as that is the maximium transfer rate of the MCP2210
  if(frequency > MAXIMUM_TRANSFER) {
    pmsg_error("MCP2210 maximum transfer limit is 3 Mbps; use at most -B 3MHz");
    return -1;
  }

  uint32tobuf(mine->bitclock, frequency);

  pgm->program_enable(pgm, p);
  return 0;
}

static int mcp2210_cancel_transfer(const PROGRAMMER *pgm) {
  unsigned char cancel_response[HID_REPORT_LENGTH] = {0};
  unsigned char cancel_request[HID_REPORT_LENGTH] = {0x00, MCP2210_SPI_CANCEL_TRANSFER};

  if(mcp2210_run_query(pgm, cancel_request, cancel_response, "cancel transfer") == -1)
    return -1;

  if(cancel_response[2] != MCP2210_SUCCESS) {
    pmsg_error("unable to cancel transfer; error code 0x%02x", cancel_response[3]);
    return -1;
  }

  return 0;
}

static void mcp2210_disable(const PROGRAMMER *pgm) {
  return;
}

static void mcp2210_enable(PROGRAMMER *pgm, const AVRPART *p) {
  return;
}

static void mcp2210_display(const PROGRAMMER *pgm, const char *s) {
  wchar_t serno[64] = {0};

  if(hid_get_serial_number_string(mine->handle, serno, sizeof serno/sizeof(wchar_t)) == -1) {
    pmsg_error("unable to fetch serial number");
    return;
  }

  msg_info("Serial number         : %ls\n", serno);
  return;
}

static void mcp2210_powerup(const PROGRAMMER *pgm) {
  return;
}

static void mcp2210_powerdown(const PROGRAMMER *pgm) {
  pgm->disable(pgm);
}

static int mcp2210_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4] = {0}, res[4] = {0};

  if(p->op[AVR_OP_PGM_ENABLE] == NULL) {
    pmsg_error("program enable instruction not defined for part %s\n", pgm->desc);
    return -1;
  }

  memset(cmd, 0, sizeof cmd);
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
  pgm->cmd(pgm, cmd, res);

  if(res[2] != cmd[1])
    return -2;

  return 0;
}

static int mcp2210_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4] = {0}, res[4] = {0};

  if(p->op[AVR_OP_CHIP_ERASE] == NULL) {
    pmsg_error("chip erase instruction not defined for part %s\n", pgm->desc);
    return -1;
  }

  memset(cmd, 0, sizeof cmd);

  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
  pgm->cmd(pgm, cmd, res);
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);

  return 0;
}

static int mcp2210_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {
  // Only supporting flash & eeprom page reads
  if(!mem->paged || page_size <= 1 || (!mem_is_flash(mem) && !mem_is_eeprom(mem)))
    return -1;

  OPCODE *lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
  uint8_t cmd[SPI_MAX_CHUNK] = {0}, res[SPI_MAX_CHUNK] = {0};
  unsigned int past_end = addr + n_bytes;
  int hilo = mem->op[AVR_OP_READ_HI] && mem->op[AVR_OP_READ_LO];

  if(!hilo && !mem->op[AVR_OP_READ]) { // Neither HI/LO read commands nor single read command
    pmsg_error("no read command specified for %s\n", mem->desc);
    return -1;
  }

  if(lext) {
    memset(cmd, 0, sizeof cmd);
    avr_set_bits(lext, cmd);
    avr_set_addr(lext, cmd, addr / 2);
    pgm->cmd(pgm, cmd, res);
  }

  while(addr < past_end) {      // Process stride bytes in one go
    int stride = minm(65536 - (addr % 65536), minm(past_end - addr, SPI_MAX_CHUNK / 4));

    memset(cmd, 0, sizeof cmd);
    memset(res, 0, sizeof res);

    for(int caddr = addr, k = 0; k < stride; k++, caddr++) {
      OPCODE *readop = hilo? mem->op[caddr & 1? AVR_OP_READ_HI: AVR_OP_READ_LO]: mem->op[AVR_OP_READ];
      avr_set_bits(readop, cmd + 4*k);
      avr_set_addr(readop, cmd + 4*k, hilo? caddr/2: caddr);
    }

    if(pgm->spi(pgm, cmd, res, 4*stride) != 4*stride)
      return -1;

    memset(mem->buf + addr, 0, stride);

    for(int caddr = addr, k = 0; k < stride; k++, caddr++) {
      OPCODE *readop = hilo? mem->op[caddr & 1? AVR_OP_READ_HI: AVR_OP_READ_LO]: mem->op[AVR_OP_READ];
      avr_get_output(readop, res + 4*k, mem->buf + addr + k);
    }

    addr += stride;
  }

  return n_bytes;
}

static int mcp2210_commit_page(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned long addr) {
  OPCODE *wp = mem->op[AVR_OP_WRITEPAGE];
  if(wp == NULL) {
    pmsg_error("memory %s not configured for page writes\n", mem->desc);
    return -1;
  }

  // Adjust the address if this memory is word-addressable
  if(mem->op[AVR_OP_LOADPAGE_LO] || mem->op[AVR_OP_READ_LO])
    addr /= 2;

  unsigned char cmd[8], *start = cmd + 4, *pastend = cmd + 8;
  memset(cmd, 0, sizeof cmd);

  // Make up the write page command in the 2nd half of cmd
  avr_set_bits(wp, cmd + 4);
  avr_set_addr(wp, cmd + 4, addr);

  // Prepend the Load extended address command if available
  OPCODE *lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
  if(lext) {
    avr_set_bits(lext, cmd);
    avr_set_addr(lext, cmd, addr);
    start = cmd;
  }

  if(pgm->spi(pgm, start, NULL, pastend - start) != pastend - start)
    return -1;
  usleep(mem->max_write_delay);

  return 0;
}

static int mcp2210_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned int page_size, unsigned int addr, unsigned int n_bytes) {
  if(!mem_is_flash(mem) && !mem_is_eeprom(mem)) {
    pmsg_info("part does not support %d-byte paged write of %s\n", page_size, mem->desc);
    return -1;
  }

  uint8_t cmd[SPI_MAX_CHUNK] = {0}, res[SPI_MAX_CHUNK] = {0};
  unsigned int past_end = addr + n_bytes;
  int oldparts = !!mem->op[AVR_OP_WRITE_LO];
  int hilo = oldparts || (mem->paged && mem->op[AVR_OP_LOADPAGE_LO]); // Word-addressable

  while(addr < past_end) {
    int stride = mem->paged? minm(page_size - addr%page_size, minm(past_end - addr, SPI_MAX_CHUNK/4)): 1;

    memset(cmd, 0, sizeof cmd);
    memset(res, 0, sizeof res);

    for(int caddr = addr, k = 0; k < stride; k++, caddr++) {
      OPCODE *writeop = mem->op[
        oldparts? (caddr & 1? AVR_OP_WRITE_HI: AVR_OP_WRITE_LO): // Flash of ATtiny12/15/22 and AT90S...
        hilo? (caddr & 1? AVR_OP_LOADPAGE_HI: AVR_OP_LOADPAGE_LO): // Flash of *other* ISP parts
        AVR_OP_WRITE            // EEPROM
      ];

      if(writeop == NULL) {
        pmsg_error("missing appropriate ISP write definition for %s\n", mem->desc);
        return -1;
      }

      avr_set_bits(writeop, cmd + 4*k);
      avr_set_addr(writeop, cmd + 4*k, hilo? caddr/2: caddr);
      avr_set_input(writeop, cmd + 4*k, mem->buf[caddr]);
    }

    if(pgm->spi(pgm, cmd, res, 4*stride) != 4*stride)
      return -1;

    addr += stride;

    if(mem->paged && (addr % page_size == 0 || addr == past_end)) {
      if(mcp2210_commit_page(pgm, p, mem, addr - stride) < 0)
        return -1;
    } else if(!mem->paged)
      usleep(mem->max_write_delay);
  }

  return n_bytes;
}

static int mcp2210_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  return pgm->spi(pgm, cmd, res, 4);
}

// SPI communication; returns -1 on error (and will have printed error message) or n_bytes on success
static int mcp2210_spi(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res, int n_bytes) {
  if(n_bytes > SPI_MAX_CHUNK) {
    pmsg_error("SPI chunk of %d bytes is too long (max %d)", n_bytes, SPI_MAX_CHUNK);
    return -1;
  }

  if(mcp2210_set_settings(pgm, n_bytes) == -1)
    return -1;

  uint8_t response[HID_REPORT_LENGTH] = {0};
  uint8_t transfer_request[HID_REPORT_LENGTH] = {0, MCP2210_SPI_XFER_COMMAND, n_bytes};
  uint8_t check_request[HID_REPORT_LENGTH] = {0, MCP2210_SPI_XFER_COMMAND};
  memcpy(transfer_request + 5, cmd, n_bytes);  // Payload

  if(mcp2210_run_query(pgm, transfer_request, response, "SPI transfer") == -1)
    return -1;

  int retries, frequency = buf2uint32(mine->bitclock);
  for(retries = 0; retries < TRANSFER_RETRIES; retries++) {
    usleep(frequency > 0? 1 + 8*n_bytes*1000000/frequency + 10: 1000); // Estimated SPI transfer time plus 10 us

    memset(response, 0, HID_REPORT_LENGTH);
    if(mcp2210_run_query(pgm, check_request, response, "SPI check") == -1)
      return -1;

    if(response[2] == MCP2210_SUCCESS && response[4] == MCP2210_XFER_DONE) {
      pmsg_debug("SPI transfer finished; no more data to send\n");
      break;
    }
    if(response[2] == MCP2210_SPI_BUS_UNAVALIABLE) {
      pmsg_error("SPI bus not available\n");
      return -1;
    }
    pmsg_debug("SPI transfer %s, %s ...\n",
      response[2] == MCP2210_TRANSFER_IN_PROGRESS? "in progress; cannot currently accept more data":
      response[2] != MCP2210_SUCCESS? str_ccprintf("unknown response[2] 0x%02x", response[2]):
      response[4] == MCP2210_XFER_NOT_FINISHED? "not finished; received data available":
      response[4] == MCP2210_XFER_STARTED? "started; no data to receive":
      str_ccprintf("unknown response[4] 0x%02x", response[4]),
      retries < TRANSFER_RETRIES-1? "retrying": "aborting"
    );
  }

  if(retries >= TRANSFER_RETRIES) {
    pmsg_error("transfer failed\n");
    return -1;
  }

  if(res)
    memcpy(res, response + 5, minm(response[3], SPI_MAX_CHUNK));

  return n_bytes;
}

static int mcp2210_run_query(const PROGRAMMER *pgm, const unsigned char *request_report, unsigned char *response_report, const char *caller) {
  if(hid_write(mine->handle, request_report, HID_REPORT_LENGTH) == -1) {
    pmsg_error("HID write for %s has failed with error %ls", caller, hid_error(mine->handle));
    return -1;
  }

  if(hid_read(mine->handle, (response_report + 1), (HID_REPORT_LENGTH - 1)) == -1) {
    pmsg_error("HID read for %s has failed with error %ls", caller, hid_error(mine->handle));
    return -1;
  }

  return 0;
}

void mcp2210_initpgm(PROGRAMMER *pgm) {
  pgm->ptyp = "MCP2210";

  /*
   * Mandatory functions - these are called without checking to see
   * whether they are assigned or not
   */

  pgm->initialize = mcp2210_initialize;
  pgm->display = mcp2210_display;
  pgm->enable = mcp2210_enable;
  pgm->disable = mcp2210_disable;
  pgm->powerup = mcp2210_powerup;
  pgm->powerdown = mcp2210_powerdown;
  pgm->program_enable = mcp2210_program_enable;
  pgm->chip_erase = mcp2210_chip_erase;
  pgm->open = mcp2210_open;
  pgm->close = mcp2210_close;

  pgm->read_byte = avr_read_byte_default;
  pgm->write_byte = avr_write_byte_default;

  /*
   * Optional functions - these are checked to make sure they are
   * assigned before they are called
   */

  pgm->cmd = mcp2210_cmd;
  pgm->spi = mcp2210_spi;
  pgm->paged_write = mcp2210_paged_write;
  pgm->paged_load = mcp2210_paged_load;
  pgm->setup = mcp2210_setup;
  pgm->teardown = mcp2210_teardown;
}

#else

static int mcp2210_nohidapi_open(const PROGRAMMER *pgm, char *name) {
  pmsg_info("no hidapi support; please compile again with hidapi installed\n");
  return -1;
}

void mcp2210_initpgm(const PROGRAMMER *pgm) {
  pgm->open = mcp2210_nohidapi_open;
}

#endif

const char mcp2210_desc[] = "MCP2210 IC using SPI for programming via usb_hidapi";
