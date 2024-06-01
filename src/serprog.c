/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Support for using serprog programmers to program over ISP.
 * For information on serprog see:
 * https://flashrom.org/supported_hw/supported_prog/serprog/index.html
 *
 * Copyright (C) 2024 Sydney Louisa Wilke <git@funkeleinhorn.com>
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
 *
 * Known limitations:
 *  - performance is suboptimal
 *  - connecting over TCP/IP to programmers is not implemented yet
 */

#include "ac_cfg.h"

#include "avrdude.h"
#include "libavrdude.h"

#include "serprog.h"

#include <fcntl.h>
#include <unistd.h>

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char serprog_desc[] = "Program via the Serprog protocol from Flashrom";

// Private data for this programmer
struct pdata {
  unsigned char cmd_bitmap[32];
  unsigned int cs;
  uint32_t actual_frequency;
};

#define my (*(struct pdata *)(pgm->cookie))

// Serprog protocol specification

// According to Serial Flasher Protocol Specification - version 1
#define S_ACK 0x06
#define S_NAK 0x15
#define S_CMD_NOP           0x00 // No operation
#define S_CMD_Q_IFACE       0x01 // Query interface version
#define S_CMD_Q_CMDMAP      0x02 // Query supported commands bitmap
#define S_CMD_Q_PGMNAME     0x03 // Query programmer name
#define S_CMD_Q_SERBUF      0x04 // Query Serial Buffer Size
#define S_CMD_Q_BUSTYPE     0x05 // Query supported bustypes
#define S_CMD_Q_CHIPSIZE    0x06 // Query supported chipsize (2^n format)
#define S_CMD_Q_OPBUF       0x07 // Query operation buffer size
#define S_CMD_Q_WRNMAXLEN   0x08 // Query Write to opbuf: Write-N maximum length
#define S_CMD_R_BYTE        0x09 // Read a single byte
#define S_CMD_R_NBYTES      0x0A // Read n bytes
#define S_CMD_O_INIT        0x0B // Initialize operation buffer
#define S_CMD_O_WRITEB      0x0C // Write opbuf: Write byte with address
#define S_CMD_O_WRITEN      0x0D // Write to opbuf: Write-N
#define S_CMD_O_DELAY       0x0E // Write opbuf: udelay
#define S_CMD_O_EXEC        0x0F // Execute operation buffer
#define S_CMD_SYNCNOP       0x10 // Special no-operation that returns NAK+ACK
#define S_CMD_Q_RDNMAXLEN   0x11 // Query read-n maximum length
#define S_CMD_S_BUSTYPE     0x12 // Set used bustype(s).
#define S_CMD_O_SPIOP       0x13 // Perform SPI operation.
#define S_CMD_S_SPI_FREQ    0x14 // Set SPI clock frequency
#define S_CMD_S_PIN_STATE   0x15 // Enable/disable output drivers
#define S_CMD_S_SPI_CS      0x16 // Set SPI chip select to use
#define S_CMD_S_SPI_MODE    0x17 // Sets the SPI mode used by S_CMD_O_SPIOP
#define S_CMD_S_CS_MODE     0x18 // Sets the way the CS is controlled

enum spi_mode {
  SPI_MODE_HALF_DUPLEX = 0,
  SPI_MODE_FULL_DUPLEX = 1,
  SPI_MODE_MAX = SPI_MODE_FULL_DUPLEX,
};

enum cs_mode {
  CS_MODE_AUTO = 0,
  CS_MODE_SELECTED = 1,
  CS_MODE_DESELECTED = 2,
  CS_MODE_MAX = CS_MODE_DESELECTED,
};

// Little endian helper functions

static uint16_t read_le16(const unsigned char *buf) {
  return buf[0] | (buf[1] << 8);
}

static uint32_t read_le32(const unsigned char *buf) {
  return buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
}

static void write_le24(unsigned char *buf, uint32_t val) {
  buf[0] = val;
  buf[1] = val >> 8;
  buf[2] = val >> 16;
}

static void write_le32(unsigned char *buf, uint32_t val) {
  buf[0] = val;
  buf[1] = val >> 8;
  buf[2] = val >> 16;
  buf[3] = val >> 24;
}

// Serprog communication functions

static int perform_serprog_cmd_full(const PROGRAMMER *pgm, uint8_t cmd,
  const unsigned char *params, int params_len,
  const unsigned char *send_buf, int send_len, unsigned char *recv_buf, int recv_len) {

  unsigned char resp_status_code = 0;

  if(serial_send(&pgm->fd, &cmd, 1) < 0)
    return -1;
  if(params_len > 0)
    if(serial_send(&pgm->fd, params, params_len) < 0)
      return -1;
  if(send_len > 0)
    if(serial_send(&pgm->fd, send_buf, send_len) < 0)
      return -1;

  if(serial_recv(&pgm->fd, &resp_status_code, 1) < 0 || serial_recv(&pgm->fd, recv_buf, recv_len) < 0)
    return -1;

  return resp_status_code == S_ACK? 0: resp_status_code == S_NAK? 1: -1;
}

static int perform_serprog_cmd(const PROGRAMMER *pgm, uint8_t cmd,
  const unsigned char *params, int params_len, unsigned char *recv_buf, int recv_len) {
  return perform_serprog_cmd_full(pgm, cmd, params, params_len, NULL, 0, recv_buf, recv_len);
}

/**
 * @brief Sends/receives a message to the AVR in full duplex mode
 * @return -1 on failure, otherwise number of bytes sent/received
 */
static int serprog_spi_duplex(const PROGRAMMER *pgm, const unsigned char *tx, unsigned char *rx, int len) {
  unsigned char params[6];

  write_le24(params, len);
  write_le24(params + 3, len);
  if(perform_serprog_cmd_full(pgm, S_CMD_O_SPIOP, params, sizeof params, tx, len, rx, len) != 0)
    return -1;

  return len;
}

static bool is_serprog_cmd_supported(const unsigned char *cmd_bitmap, unsigned char cmd) {
  return (cmd_bitmap[cmd / 8] >> (cmd % 8)) & 1;
}

// Programmer lifecycle handlers

static int serprog_open(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;

  pgm->port = port;
  pinfo.serialinfo.baud = pgm->baudrate? pgm->baudrate: 115200;
  pinfo.serialinfo.cflags = SERIAL_8N1;
  if(serial_open(port, pinfo, &pgm->fd) == -1)
    return -1;

  unsigned char buf[32];

  // Sync
  memset(buf, 0, sizeof buf);
  if(perform_serprog_cmd(pgm, S_CMD_SYNCNOP, NULL, 0, buf, 1) != 1 || buf[0] != S_ACK) {
    pmsg_error("cannot sync; is this a serprog programmer?\n");
    return -1;
  }

  // Get command bitmap
  memset(my.cmd_bitmap, 0, sizeof my.cmd_bitmap);
  if(perform_serprog_cmd(pgm, S_CMD_Q_CMDMAP, NULL, 0, my.cmd_bitmap, 32) != 0) {
    pmsg_error("cannot get list of supported serprog commands\n");
    return -1;
  }

  // Get protocol version
  memset(buf, 0, sizeof buf);
  if(!is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_Q_IFACE)
    || perform_serprog_cmd(pgm, S_CMD_Q_IFACE, NULL, 0, buf, 2) != 0) {
    pmsg_error("cannot get serprog protocol version\n");
    return -1;
  }
  if(read_le16(buf) != 0x01) {
    pmsg_error("unsupported serprog protocol version: %d\n", read_le16(buf));
    return -1;
  }

  pmsg_info("serprog protocol version: %d\n", read_le16(buf));

  // Get programmer name
  if(is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_Q_PGMNAME)) {
    memset(buf, 0, sizeof buf);
    if(perform_serprog_cmd(pgm, S_CMD_Q_PGMNAME, NULL, 0, buf, 16) != 0) {
      pmsg_error("cannot get programmer name\n");
      return -1;
    }
    pmsg_info("programmer name: %s\n", buf);
  }

  // Check if required commands are supported
  if(!is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_O_SPIOP)) {
    pmsg_error("the %s programmer does not support SPI operations\n", pgmid);
    return -1;
  }

  if(!is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_S_CS_MODE)) {
    pmsg_error("the %s programmer does not support setting the CS mode\n", pgmid);
    return -1;
  }

  if(!is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_S_SPI_MODE)) {
    pmsg_error("the %s programmer does not support setting the SPI mode\n", pgmid);
    return -1;
  }

  if(my.cs > 0 && !is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_S_SPI_CS)) {
    pmsg_error("the %s programmer does not support changing the CS\n", pgmid);
    return -1;
  }

  return 0;
}

static void serprog_disable(const PROGRAMMER *pgm) {
  unsigned char buf[32];

  // Switch CS to auto
  const unsigned char cs_mode = CS_MODE_AUTO;

  if(perform_serprog_cmd(pgm, S_CMD_S_CS_MODE, &cs_mode, 1, NULL, 0) != 0) {
    pmsg_error("cannot reset the CS mode to auto\n");
  }
  // Disable output
  if(is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_S_PIN_STATE)) {
    memset(buf, 0, sizeof buf);
    buf[0] = 0;                 // Pin state disable
    if(perform_serprog_cmd(pgm, S_CMD_S_PIN_STATE, buf, 1, NULL, 0) != 0) {
      pmsg_error("cannot disable pin state\n");
    }
  }
  // Restore half duplex
  memset(buf, 0, sizeof buf);
  buf[0] = SPI_MODE_HALF_DUPLEX;
  if(perform_serprog_cmd(pgm, S_CMD_S_SPI_MODE, buf, 1, NULL, 0) != 0)
    pmsg_error("cannot reset SPI half duplex mode\n");

  // Reset CS to CS_0
  if(is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_S_SPI_CS)) {
    memset(buf, 0, sizeof buf);
    buf[0] = 0;
    if(perform_serprog_cmd(pgm, S_CMD_S_SPI_CS, buf, 1, NULL, 0) != 0)
      pmsg_error("cannot reset CS to CS_0\n");
  }
}

static void serprog_close(PROGRAMMER *pgm) {
  serial_close(&pgm->fd);
}

static int serprog_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  return serprog_spi_duplex(pgm, cmd, res, 4);
}

static int serprog_initialize(const PROGRAMMER *pgm, const AVRPART *part) {
  if(part->prog_modes & PM_TPI) {
    // We do not support TPI; this is a dedicated SPI thing
    pmsg_error("the %s programmer does not support TPI\n", pgmid);
    return -1;
  }

  unsigned char buf[32];

  // Set SPI clock frequency
  if(is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_S_SPI_FREQ)) {
    memset(buf, 0, sizeof buf);
    uint32_t frequency =
      pgm->bitclock > 0? 1/pgm->bitclock:
      part->factory_fcpu > 0? part->factory_fcpu/4:
      250000;
    write_le32(buf, frequency);
    if(perform_serprog_cmd(pgm, S_CMD_S_SPI_FREQ, buf, 4, buf, 4) != 0) {
      pmsg_error("cannot set SPI frequency %u Hz\n", (unsigned) frequency);
      return -1;
    }
    my.actual_frequency = read_le32(buf);
  }

  // Set active chip select
  if(is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_S_SPI_CS)) {
    memset(buf, 0, sizeof buf);
    buf[0] = my.cs;
    if(perform_serprog_cmd(pgm, S_CMD_S_SPI_CS, buf, 1, NULL, 0) != 0) {
      pmsg_error("cannot change CS\n");
      return -1;
    }
  }

  // Set full duplex
  memset(buf, 0, sizeof buf);
  buf[0] = SPI_MODE_FULL_DUPLEX;
  if(perform_serprog_cmd(pgm, S_CMD_S_SPI_MODE, buf, 1, NULL, 0) != 0) {
    pmsg_error("cannot set SPI full duplex mode\n");
    return -1;
  }

  // Set output
  if(is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_S_PIN_STATE)) {
    memset(buf, 0, sizeof buf);
    buf[0] = 1;                 // Pin state enable
    if(perform_serprog_cmd(pgm, S_CMD_S_PIN_STATE, buf, 1, NULL, 0) != 0) {
      pmsg_error("cannot enable pin state\n");
      return -1;
    }
  }

  // Enable the CS/reset pin
  const unsigned char cs_mode = CS_MODE_SELECTED;

  if(perform_serprog_cmd(pgm, S_CMD_S_CS_MODE, &cs_mode, 1, NULL, 0) != 0) {
    pmsg_error("cannot enable the reset pin\n");
    return -1;
  }

  int tries, ret;

  // Enable programming on the part
  tries = 0;
  do {
    ret = pgm->program_enable(pgm, part);
    if(ret == 0 || ret == -1)
      break;
  } while(tries++ < 65);

  if(ret)
    pmsg_error("AVR device not responding\n");

  return ret;
}

/* used linuxspi.c as a template:
 *   Copyright (C) 2013 Kevin Cuzner <kevin@kevincuzner.com>
 *   Copyright (C) 2018 Ralf Ramsauer <ralf@vmexit.de>
 */
static int serprog_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4], res[4];

  if(!p->op[AVR_OP_PGM_ENABLE]) {
    pmsg_error("program enable instruction not defined for part %s\n", p->desc);
    return -1;
  }

  memset(cmd, 0, sizeof cmd);
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);  // Set the cmd
  pgm->cmd(pgm, cmd, res);

  if(res[2] != cmd[1]) {

    /** From ATtiny441 datasheet:
     *
     * In some systems, the programmer cannot guarantee that SCK is held low
     * during power-up. In this case, RESET must be given a positive pulse after
     * SCK has been set to '0'. The duration of the pulse must be at least t RST
     * plus two CPU clock cycles. See Table 25-5 on page 240 for definition of
     * minimum pulse width on RESET pin, t RST
     * 2. Wait for at least 20 ms and then enable serial programming by sending
     * the Programming Enable serial instruction to the SDO pin
     * 3. The serial programming instructions will not work if the communication
     * is out of synchronization. When in sync, the second byte (0x53) will echo
     * back when issuing the third byte of the Programming Enable instruction
     * ...
     * If the 0x53 did not echo back, give RESET a positive pulse and issue a
     * new Programming Enable command
     */

    unsigned char cs_mode = CS_MODE_DESELECTED;
    if(perform_serprog_cmd(pgm, S_CMD_S_CS_MODE, &cs_mode, 1, NULL, 0) != 0)
      return -1;

    usleep(5);
    cs_mode = CS_MODE_SELECTED;
    if(perform_serprog_cmd(pgm, S_CMD_S_CS_MODE, &cs_mode, 1, NULL, 0) != 0)
      return -1;

    usleep(20000);

    return -2;
  }

  return 0;
}

static int serprog_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4], res[4];

  if(!p->op[AVR_OP_CHIP_ERASE]) {
    pmsg_error("chip erase instruction not defined for part %s\n", p->desc);
    return -1;
  }

  memset(cmd, 0, sizeof cmd);
  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
  pgm->cmd(pgm, cmd, res);
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);

  return 0;
}

static void serprog_display(const PROGRAMMER *pgm, const char *p) {
}

static void serprog_enable(PROGRAMMER *pgm, const AVRPART *p) {
}

static void serprog_setup(PROGRAMMER *pgm) {
  pgm->cookie = mmt_malloc(sizeof(struct pdata));
}

static void serprog_teardown(PROGRAMMER *pgm) {
  mmt_free(pgm->cookie);
  pgm->cookie = NULL;
}

static int serprog_set_sck_period(const PROGRAMMER *pgm, double v) {
  if(!is_serprog_cmd_supported(my.cmd_bitmap, S_CMD_S_SPI_FREQ))
    return -1;

  unsigned char buf[8];

  memset(buf, 0, sizeof buf);
  write_le32(buf, v);
  if(perform_serprog_cmd(pgm, S_CMD_S_SPI_FREQ, buf, 4, buf, 4) != 0) {
    pmsg_error("cannot set SPI frequency\n");
    return -1;
  }
  my.actual_frequency = read_le32(buf);
  return 0;
}

static int serprog_get_sck_period(const PROGRAMMER *pgm, double *v) {
  *v = my.actual_frequency;
  return 0;
}

static int serprog_parseextparams(const PROGRAMMER *pgm, const LISTID extparms) {
  LNODEID ln;
  const char *extended_param;
  int rv = 0;

  for(ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if(str_starts(extended_param, "cs=")) {
      unsigned int cs;

      if(sscanf(extended_param, "cs=%u", &cs) != 1) {
        pmsg_error("invalid chip select '%s'\n", extended_param);
        rv = -1;
      }
      my.cs = cs;
      continue;
    }

    if(str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      msg_error("  -xcs=cs_num    Sets the chip select (CS) to use on supported programmers\n");
      msg_error("  -xhelp         Show this help menu and exit\n");
      return LIBAVRDUDE_EXIT;
    }

    pmsg_error("invalid extended parameter '%s'\n", extended_param);
    rv = -1;
  }

  return rv;
}

void serprog_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "serprog");

  // Required fields
  pgm->initialize = serprog_initialize;
  pgm->display = serprog_display;
  pgm->enable = serprog_enable;
  pgm->disable = serprog_disable;
  pgm->program_enable = serprog_program_enable;
  pgm->chip_erase = serprog_chip_erase;
  pgm->cmd = serprog_cmd;
  pgm->open = serprog_open;
  pgm->close = serprog_close;
  pgm->read_byte = avr_read_byte_default;
  pgm->write_byte = avr_write_byte_default;

  // Optional fields
  pgm->setup = serprog_setup;
  pgm->teardown = serprog_teardown;
  pgm->parseextparams = serprog_parseextparams;
  pgm->get_sck_period = serprog_get_sck_period;
  pgm->set_sck_period = serprog_set_sck_period;
}
