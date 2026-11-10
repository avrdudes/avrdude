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

static int mcp2210_run_query(const PROGRAMMER *pgm, const unsigned char request_report[65], unsigned char *response_report);
static int mcp2210_cancel_transfer(const PROGRAMMER *pgm);

struct mcp {
  hid_device *handle;
  union {
    u_int32_t bitrate_long;
    u_int8_t bitrate_byte[4];
  } bitrate;
};

#define mine ((struct mcp *)(pgm->cookie))

static void mcp2210_setup(PROGRAMMER *pgm) {
  pgm->cookie = mmt_malloc(sizeof *mine);
}

static int mcp2210_set_settings(const PROGRAMMER *pgm, unsigned short len) {
  unsigned char settings_request[HID_REPORT_LENGTH] = {0x00, MCP2210_GET_SETTINGS}, settings_response[HID_REPORT_LENGTH];

  if(!mine->bitrate.bitrate_long) {
    unsigned char current_settings[HID_REPORT_LENGTH];

    if(mcp2210_run_query(pgm, settings_request, current_settings) == -1) {
      msg_error("unable to run query to fetch current settings\n");
      return -1;
    }

    mine->bitrate.bitrate_long = le32toh(current_settings[5]);
  }

  memset(settings_request, 0, HID_REPORT_LENGTH);

  settings_request[1] = MCP2210_SET_SETTINGS;
  for(int i=5, j=0; j < 4; j++, i++)
      settings_request[i] =  mine->bitrate.bitrate_byte[j];
  settings_request[19] = len;
  settings_request[20] = len >> 8;

  if(mcp2210_run_query(pgm, settings_request, settings_response) == -1) {
    msg_error("unable to run query to set settings\n");
    return -1;
  }

  if(settings_response[2] != MCP2210_SUCCESS) {
    if(settings_response[2] == MCP2210_TRANSFER_IN_PROGRESS)
      msg_error("USB transfer in progress - settings not written\n");
    else
      msg_error("unknown error, return code: %x\n", settings_response[2]);

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

  if((numids = str_set_vid_pid_serno(port, &param_vid, &param_pid, serno, sizeof serno)) < 0) {
    msg_error("usb[:<vid>:<pid>][serno]\n");
    return -1;
  }

  LNODEID usbpid = lfirst(pgm->usbpid);
  int pid, vid = numids >= 2 ? param_vid : pgm->usbvid ? pgm->usbvid : MCP2210_DEFAULT_VID;

  if(numids >= 2) {
    pid = param_pid;
  } else if(pgm->usbvid) {
    pid = *(int *)ldata(usbpid);

    if (lnext(usbpid))
      msg_warning("using PID 0x%04x, ignoring remaining PIDs in list\n", pid);
  } else
    pid = MCP2210_DEFAULT_PID;

  mine->handle = hid_open(vid, pid, numids >= 2 ? (wchar_t *)serno : NULL);
  if(mine->handle == NULL) {
    msg_error("could not find mcp2210 device with vid=0x%x pid=0x%x serno=%s\n", vid, pid, serno);
    return -1;
  }

  return 0;
}

static void mcp2210_close(PROGRAMMER *pgm) {
  mcp2210_cancel_transfer(pgm);
  hid_close(mine->handle);
}

static int mcp2210_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  pgm->program_enable(pgm, p);

  if(pgm->baudrate)
    mine->bitrate.bitrate_long = htole32(pgm->baudrate);

  return 0;
}

static int mcp2210_cancel_transfer(const PROGRAMMER *pgm) {
  unsigned char cancel_response[HID_REPORT_LENGTH];
  unsigned char cancel_request[HID_REPORT_LENGTH] = {0x00, MCP2210_SPI_CANCEL_TRANSFER};

  if(mcp2210_run_query(pgm, cancel_request, cancel_response) == -1) {
    msg_error("unable to run query to cancel response\n");
    return -1;
  }

  if(cancel_response[2] != MCP2210_SUCCESS) {
    msg_error("unable to cancel transfer - error code: %i", cancel_response[3]);
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
  uint8_t status_request[HID_REPORT_LENGTH] = {0, 0x10}, status_response[HID_REPORT_LENGTH];

  if(mcp2210_run_query(pgm, status_request, status_response) == -1) {
    msg_error("unable to run query to get status\n");
    return;
  }

  if(status_response[2] != MCP2210_SUCCESS) {
    msg_error("unable to get chip status\n");
    return;
  }

  msg_info("SPI Bus current owner: ");
  switch(status_response[4]) {
    case 0x00:
      msg_info("no owner\n");
      break;
    case 0x01:
      msg_info("spi bridge\n");
      break;
    case 0x02:
      msg_info("external host\n");
      break;
    default:
      msg_info("unknown response\n");
  }

  msg_info("password attempts: %i\n", status_response[5]);
  return;
}

static void mcp2210_powerup(const PROGRAMMER *pgm) {
  return;
}

static void mcp2210_powerdown(const PROGRAMMER *pgm) {
  pgm->disable(pgm);
}

static int mcp2210_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4], res[4];

  if(p->op[AVR_OP_PGM_ENABLE] == NULL) {
    msg_error("program enable instruction not defined for part \"%s\"\n", pgm->desc);
    return -1;
  }

  memset(cmd, 0, sizeof(cmd));
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
  pgm->cmd(pgm, cmd, res);

  if(res[2] != cmd[1])
    return -2;

  return 0;
}

static int mcp2210_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4], res[4];

  if(p->op[AVR_OP_CHIP_ERASE] == NULL) {
    msg_error("chip erase instruction not defined for part \"%s\"\n", pgm->desc);
    return -1;
  }

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
  pgm->cmd(pgm, cmd, res);
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);

  return 0;
}

static int mcp2210_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {
  // Only supporting flash & eeprom page reads
  if((!mem->paged || page_size <= 1) || (!mem_is_flash(mem) && !mem_is_eeprom(mem)))
    return -1;

  OPCODE *readop = 0, *lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
  uint8_t data = 0, cmd[SPI_MAX_CHUNK], res[SPI_MAX_CHUNK];
  unsigned int addr_base;
  unsigned int max_addr = addr + n_bytes;

  if(lext) {
    memset(cmd, 0, sizeof(cmd));
    avr_set_bits(lext, cmd);
    avr_set_addr(lext, cmd, addr / 2);
    pgm->cmd(pgm, cmd, res);
  }

  for(addr_base = addr; addr_base < max_addr;) {
    uint32_t blockSize = minm(65536 - (addr_base % 65536), minm(max_addr - addr_base, SPI_MAX_CHUNK / 4));

    memset(cmd, 0, sizeof(cmd));
    memset(res, 0, sizeof(res));

    uint8_t addr_off;

    for(addr_off = 0; addr_off < blockSize; addr_off++) {
      int addr = addr_base + addr_off, caddr = addr;

      if(mem->op[AVR_OP_READ_LO] != NULL && mem->op[AVR_OP_READ_HI] != NULL) {
        if (addr & 0x00000001)
          readop = mem->op[AVR_OP_READ_HI];
        else
          readop = mem->op[AVR_OP_READ_LO];

        caddr /= 2;
      } else if(mem->op[AVR_OP_READ] != NULL) {
        readop = mem->op[AVR_OP_READ];
      } else {
        msg_error("no read command specified\n");
        return -1;
      }

      avr_set_bits(readop, &cmd[addr_off * 4]);
      avr_set_addr(readop, &cmd[addr_off * 4], caddr);
    }

    int bytes_read = pgm->spi(pgm, cmd, res, blockSize * 4);

    if(bytes_read < 0) {
      msg_error("failed @ p->spi()\n");
      return -1;
    }

    for(addr_off = 0; addr_off < bytes_read / 4; addr_off++) {
      data = 0;
      avr_get_output(readop, &res[addr_off * 4], &data);
      mem->buf[addr_base + addr_off] = data;
    }

    addr_base += blockSize;
  }

  return n_bytes;
}

static int mcp2210_commit_page(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned long addr) {
  OPCODE *wp, *lext;

  wp = mem->op[AVR_OP_WRITEPAGE];
  if(wp == NULL) {
    msg_error("memory \"%s\" not configured for page writes\n", mem->desc);
    return -1;
  }

  // Adjust the address if this memory is word-addressable
  if(mem->op[AVR_OP_LOADPAGE_LO] || mem->op[AVR_OP_READ_LO])
    addr /= 2;

  unsigned char cmd[8];
  memset(cmd, 0, sizeof(cmd));

  // Use the "load extended address" command, if available
  lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
  if(lext != NULL) {
    avr_set_bits(lext, cmd);
    avr_set_addr(lext, cmd, addr);
  }

  // Make up the write page command in the 2nd cmd position
  avr_set_bits(wp, cmd + 4);
  avr_set_addr(wp, cmd + 4, addr);

  if(lext != NULL)
    // Write the load extended address cmd && the write_page cmd
    pgm->spi(pgm, cmd, NULL, 8);
  else
    // Write just the write_page cmd
    pgm->spi(pgm, cmd + 4, NULL, 4);

  // Just delay the max
  usleep(mem->max_write_delay);

  return 0;
}

static int mcp2210_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned int page_size, unsigned int addr, unsigned int n_bytes) {
  if (!mem_is_flash(mem) && !mem_is_eeprom(mem)) {
    msg_info("part does not support %d paged write of %s\n", page_size, mem->desc);
    return -1;
  }

  OPCODE *writeop;
  uint8_t cmd[SPI_MAX_CHUNK], res[SPI_MAX_CHUNK];
  unsigned int addr_base;
  unsigned int max_addr = addr + n_bytes;

  for(addr_base = addr; addr_base < max_addr;) {
    uint32_t blockSize;

    if(mem->paged) {
      blockSize = minm(page_size - (addr_base % page_size), minm(max_addr - addr_base, SPI_MAX_CHUNK / 4)); // Bytes remaining in page
    } else
      blockSize = 1;

    memset(cmd, 0, sizeof(cmd));
    memset(res, 0, sizeof(res));

    uint8_t addr_off;
    for(addr_off = 0; addr_off < blockSize; addr_off++) {
      int addr = addr_base + addr_off;
      int caddr = 0;
      /*
       * Determine which memory opcode to use
       */
      if(mem->paged && mem->op[AVR_OP_LOADPAGE_HI] && mem->op[AVR_OP_LOADPAGE_LO]) {
        if(addr & 0x01)
          writeop = mem->op[AVR_OP_LOADPAGE_HI];
        else
          writeop = mem->op[AVR_OP_LOADPAGE_LO];
        caddr = addr / 2;
      } else if(mem->paged && mem->op[AVR_OP_LOADPAGE_LO]) {
        writeop = mem->op[AVR_OP_LOADPAGE_LO];
        caddr = addr;
      } else if(mem->op[AVR_OP_WRITE_LO]) {
        writeop = mem->op[AVR_OP_WRITE_LO];
        caddr = addr;

        msg_error("error AVR_OP_WRITE_LO defined only (where's the HIGH command?)\n");
        return -1;
      } else {
        writeop = mem->op[AVR_OP_WRITE];
        caddr = addr;
      }

      if(writeop == NULL)
        return -1;

      avr_set_bits(writeop, &cmd[addr_off * 4]);
      avr_set_addr(writeop, &cmd[addr_off * 4], caddr);
      avr_set_input(writeop, &cmd[addr_off * 4], mem->buf[addr]);
    }

    int bytes_read = pgm->spi(pgm, cmd, res, blockSize * 4);

    if(bytes_read < 0) {
      msg_error("failed @ p->spi()\n");
      return -1;
    }

    addr_base += blockSize;

    if(mem->paged && (((addr_base % page_size) == 0) || (addr_base == max_addr))) {
      mcp2210_commit_page(pgm, p, mem, addr_base - blockSize);
    }
    else if(!mem->paged) {
      usleep(mem->max_write_delay);
    }
  }

  return n_bytes;
}

static int mcp2210_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  return pgm->spi(pgm, cmd, res, 4);
}

static int mcp2210_spi(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res, int n_bytes) {
  if(n_bytes > SPI_MAX_CHUNK) {
    msg_error("spi chunk longer than %i", n_bytes);
    return -1;
  }

  if(mcp2210_set_settings(pgm, n_bytes) == -1)
    return 1;

  uint8_t transfer_request[HID_REPORT_LENGTH] = {0, MCP2210_SPI_XFER_COMMAND, n_bytes}, transfer_response[HID_REPORT_LENGTH];
  int retries = 0, timeout = (100000 / mine->bitrate.bitrate_long) + 1;
  bool success = false;

  for(int i = 5; i < n_bytes + 5; i++)
    transfer_request[i] = *cmd++;

  if(mcp2210_run_query(pgm, transfer_request, transfer_response) == -1) {
    msg_error("unable to run query to send spi command\n");
    return -1;
  }

  while(retries < TRANSFER_RETRIES) {
    usleep(timeout * 1000);

    if(mcp2210_run_query(pgm, (const unsigned char[65]){0x00, MCP2210_SPI_XFER_COMMAND, 0}, transfer_response) == -1) {
      msg_error("unable to run query to send spi command\n");
      return -1;
    }

    if(transfer_response[2] != MCP2210_SUCCESS) {
      if(transfer_response[2] == MCP2210_TRANSFER_IN_PROGRESS)
        msg_debug("SPI transfer in progress - cannot accept any data for the moment\n");

      if(transfer_response[2] == MCP2210_SPI_BUS_UNAVALIABLE) {
        msg_error("SPI bus not available\n");
        return -1;
      }
    } else {
      if(transfer_response[4] == MCP2210_XFER_DONE) {
        msg_debug("SPI transfer finished - no more data to send\n");
        success = true;
        break;
      }

      if(transfer_response[4] == MCP2210_XFER_NOT_FINISHED)
        msg_debug("SPI transfer not finished; received data available\n");

      if(transfer_response[4] == MCP2210_XFER_STARTED)
        msg_debug("SPI transfer started - no data to receive\n");
    }

    retries++;
  }

  if(!success) {
    msg_error("transfer failed\n");
    return -1;
  }

  if(res)
    memcpy(res, &transfer_response[5], transfer_response[3]);

  return n_bytes;
}

static int mcp2210_run_query(const PROGRAMMER *pgm, const unsigned char request_report[HID_REPORT_LENGTH], unsigned char *response_report) {
  if(hid_write(mine->handle, request_report, HID_REPORT_LENGTH) == -1) {
    msg_error("HID write has failed with error: %ls", hid_error(mine->handle));
    return -1;
  }

  memset(response_report, 0, sizeof(HID_REPORT_LENGTH));

  if(hid_read(mine->handle, (response_report + 1), (HID_REPORT_LENGTH - 1)) == -1) {
    msg_error("HID read has failed with error: %ls", hid_error(mine->handle));
    return -1;
  }

  return 0;
}

void mcp2210_initpgm(PROGRAMMER *pgm) {
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
  msg_info("no hidapi support. Please compile again with hidapi installed.\n");
  return -1;
}

void mcp2210_initpgm(const PROGRAMMER *pgm) {
  /*
   * Mandatory functions - these are called without checking to see
   * whether they are assigned or not
   */

  pgm->open = mcp2210_nohidapi_open;
}

#endif

const char mcp2210_desc[] = "MCP2210 IC";
