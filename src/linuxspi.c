/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Support for using spidev userspace drivers to communicate directly over SPI
 *
 * Copyright (C) 2013 Kevin Cuzner <kevin@kevincuzner.com>
 * Copyright (C) 2018 Ralf Ramsauer <ralf@vmexit.de>
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
 * Support for inversion of reset pin, Tim Chilton 02/05/2014
 * Review code, rebase to latest trunk, add linux/gpio.h support, Ralf Ramsauer 2018-09-07
 */

#include <ac_cfg.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "linuxspi.h"

#if HAVE_LINUXSPI

/*
 * Linux Kernel SPI Drivers
 *
 * Copyright (C) 2006 SWAPP
 *      Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define LINUXSPI "linuxspi"

// Private data for this programmer
struct pdata {
  int disable_no_cs;
  int fd_spidev, fd_gpiochip, fd_linehandle;
};

// Use private programmer data as if they were a global structure my
#define my (*(struct pdata *)(pgm->cookie))

/*
 * @brief Sends/receives a message in full duplex mode
 * @return -1 on failure, otherwise number of bytes sent/received
 */
static int linuxspi_spi_duplex(const PROGRAMMER *pgm, const unsigned char *tx, unsigned char *rx, int len) {
  struct spi_ioc_transfer tr;
  int ret;

  tr = (struct spi_ioc_transfer) {
    .tx_buf = (unsigned long) tx,
    .rx_buf = (unsigned long) rx,
    .len = len,
    .delay_usecs = 1,
    .speed_hz = 1.0/pgm->bitclock,
    .bits_per_word = 8,
  };

  errno = 0;

  ret = ioctl(my.fd_spidev, SPI_IOC_MESSAGE(1), &tr);
  if(ret != len) {
    int ioctl_errno = errno;

    msg_error("\n");
    pmsg_error("unable to send SPI message");
    if(ioctl_errno)
      msg_error("%s", strerror(ioctl_errno));
    msg_error("\n");
  }

  return ret == -1? -1: 0;
}

static void linuxspi_setup(PROGRAMMER *pgm) {
  pgm->cookie = mmt_malloc(sizeof(struct pdata));
}

static void linuxspi_teardown(PROGRAMMER *pgm) {
  mmt_free(pgm->cookie);
  pgm->cookie = NULL;
}

static int linuxspi_reset_mcu(const PROGRAMMER *pgm, bool active) {
  struct gpiohandle_data data;
  int ret;

  /*
   * Set the reset state and keep it. The pin will be released and set back to
   * its initial value, once the my.fd_gpiochip is closed.
   */
  data.values[0] = active ^ !(pgm->pinno[PIN_AVR_RESET] & PIN_INVERSE);
  ret = ioctl(my.fd_linehandle, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);

#ifdef GPIO_V2_LINE_SET_VALUES_IOCTL
  if(ret == -1) {
    struct gpio_v2_line_values val;

    val.mask = 1;
    val.bits = active ^ !(pgm->pinno[PIN_AVR_RESET] & PIN_INVERSE);

    ret = ioctl(my.fd_linehandle, GPIO_V2_LINE_SET_VALUES_IOCTL, &val);
  }
#endif

  if(ret == -1) {
    ret = -errno;
    pmsg_ext_error("unable to set GPIO line %d value: %s\n", pgm->pinno[PIN_AVR_RESET] & PIN_MASK, strerror(errno));
    return ret;
  }

  return 0;
}

static int linuxspi_open(PROGRAMMER *pgm, const char *pt) {
  const char *port_error = "unknown port specification, "
    "please use the format /dev/spidev:/dev/gpiochip[:resetno]\n";
  char port_default[] = "/dev/spidev0.0:/dev/gpiochip0";
  char *spidev, *gpiochip, *reset_pin;
  char *port = mmt_strdup(pt);
  struct gpiohandle_request req;
  int ret;

  if(str_eq(port, "unknown")) {
    port = port_default;
  }

  spidev = strtok(port, ":");
  if(!spidev) {
    pmsg_error("%s", port_error);
    return -1;
  }

  gpiochip = strtok(NULL, ":");
  if(!gpiochip) {
    pmsg_error("%s", port_error);
    return -1;
  }

  // Optional: override reset pin in configuration
  reset_pin = strtok(NULL, ":");
  if(reset_pin) {
    const char *errstr;

    pgm->pinno[PIN_AVR_RESET] = str_int(reset_pin, STR_UINT32, &errstr);
    if(errstr) {
      pmsg_error("pin number %s: %s", reset_pin, errstr);
      return -1;
    }
  }

  pgm->port = port;
  my.fd_spidev = open(pgm->port, O_RDWR);
  if(my.fd_spidev < 0) {
    pmsg_ext_error("unable to open the spidev device %s: %s\n", pgm->port, strerror(errno));
    return -1;
  }

  uint32_t mode = SPI_MODE_0;

  if(!my.disable_no_cs)
    mode |= SPI_NO_CS;

  ret = ioctl(my.fd_spidev, SPI_IOC_WR_MODE32, &mode);
  if(ret == -1) {
    int ioctl_errno = errno;

    pmsg_ext_error("unable to set SPI mode %02X on %s: %s\n", mode, spidev, strerror(errno));
    if(ioctl_errno == EINVAL && !my.disable_no_cs)
      pmsg_error("try -x disable_no_cs\n");
    goto close_spidev;
  }
  my.fd_gpiochip = open(gpiochip, 0);
  if(my.fd_gpiochip < 0) {
    pmsg_ext_error("unable to open the gpiochip %s: %s\n", gpiochip, strerror(errno));
    ret = -1;
    goto close_spidev;
  }

  strcpy(req.consumer_label, progname);
  req.lines = 1;
  req.lineoffsets[0] = pgm->pinno[PIN_AVR_RESET] & PIN_MASK;
  req.default_values[0] = !!(pgm->pinno[PIN_AVR_RESET] & PIN_INVERSE);
  req.flags = GPIOHANDLE_REQUEST_OUTPUT;

  ret = ioctl(my.fd_gpiochip, GPIO_GET_LINEHANDLE_IOCTL, &req);
  if(ret != -1)
    my.fd_linehandle = req.fd;

#ifdef GPIO_V2_GET_LINE_IOCTL
  if(ret == -1) {
    struct gpio_v2_line_request reqv2;

    memset(&reqv2, 0, sizeof(reqv2));
    reqv2.offsets[0] = pgm->pinno[PIN_AVR_RESET] & PIN_MASK;
    strncpy(reqv2.consumer, progname, sizeof(reqv2.consumer) - 1);
    reqv2.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    reqv2.config.num_attrs = 1;
    reqv2.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    reqv2.config.attrs[0].attr.values = !!(pgm->pinno[PIN_AVR_RESET] & PIN_INVERSE);
    reqv2.config.attrs[0].mask = 1;
    reqv2.num_lines = 1;

    ret = ioctl(my.fd_gpiochip, GPIO_V2_GET_LINE_IOCTL, &reqv2);
    if(ret != -1)
      my.fd_linehandle = reqv2.fd;
  }
#endif

  if(ret == -1) {
    ret = -errno;
    pmsg_ext_error("unable to get GPIO line %d. %s\n", pgm->pinno[PIN_AVR_RESET] & PIN_MASK, strerror(errno));
    goto close_gpiochip;
  }

  // Ensure part is up for some 100 ms before resetting it
  if((ret = linuxspi_reset_mcu(pgm, false)))
    goto close_out;
  usleep(100*1000);

  if((ret = linuxspi_reset_mcu(pgm, true)))
    goto close_out;
  usleep(20000);

  if(pgm->baudrate != 0) {
    pmsg_warning("obsolete use of -b <clock> option for bit clock; use -B <clock>\n");
    pgm->bitclock = 1.0/pgm->baudrate;
  }
  if(pgm->bitclock == 0) {
    pmsg_notice("defaulting bit clock to 200 kHz\n");
    pgm->bitclock = 5E-6;       // 200 kHz - 5 µs
  }

  return 0;

close_out:
  close(my.fd_linehandle);
close_gpiochip:
  close(my.fd_gpiochip);
close_spidev:
  close(my.fd_spidev);
  return ret;
}

static void linuxspi_close(PROGRAMMER *pgm) {
  switch(pgm->exit_reset) {
  case EXIT_RESET_ENABLED:
    linuxspi_reset_mcu(pgm, true);
    break;

  case EXIT_RESET_DISABLED:
    linuxspi_reset_mcu(pgm, false);
    break;

  default:
    break;
  }

  close(my.fd_linehandle);
  close(my.fd_spidev);
  close(my.fd_gpiochip);
}

static void linuxspi_disable(const PROGRAMMER *pgm) {
}

static void linuxspi_enable(PROGRAMMER *pgm, const AVRPART *p) {
}

static void linuxspi_display(const PROGRAMMER *pgm, const char *p) {
}

static int linuxspi_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  int tries, ret;

  if(is_tpi(p)) {
    // We do not support TPI; this is a dedicated SPI thing
    pmsg_error("programmer " LINUXSPI " does not support TPI\n");
    return -1;
  }
  // Enable programming on the part
  tries = 0;
  do {
    ret = pgm->program_enable(pgm, p);
    if(ret == 0 || ret == -1)
      break;
  } while(tries++ < 65);

  if(ret)
    pmsg_error("AVR device not responding\n");

  return ret;
}

static int linuxspi_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  return linuxspi_spi_duplex(pgm, cmd, res, 4);
}

static int linuxspi_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4], res[4];

  if(!p->op[AVR_OP_PGM_ENABLE]) {
    pmsg_error("program enable instruction not defined for part %s\n", p->desc);
    return -1;
  }

  memset(cmd, 0, sizeof(cmd));
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);  // Set the cmd
  pgm->cmd(pgm, cmd, res);

  if(res[2] != cmd[1]) {
    /*
     * From ATtiny441 datasheet:
     *
     * In some systems, the programmer can not guarantee that SCK is held low
     * during power-up. In this case, RESET must be given a positive pulse
     * after SCK has been set to '0'. The duration of the pulse must be at
     * least t RST plus two CPU clock cycles. See Table 25-5 on page 240 for
     * definition of minimum pulse width on RESET pin, t RST 2. Wait for at
     * least 20 ms and then enable serial programming by sending the
     * Programming Enable serial instruction to the SDO pin 3. The serial
     * programming instructions will not work if the communication is out of
     * synchronization. When in sync, the second byte (0x53) will echo back
     * when issuing the third byte of the Programming Enable instruction ... If
     * the 0x53 did not echo back, give RESET a positive pulse and issue a new
     * Programming Enable command
     */
    if(linuxspi_reset_mcu(pgm, false))
      return -1;
    usleep(5);
    if(linuxspi_reset_mcu(pgm, true))
      return -1;
    usleep(20000);

    return -2;
  }

  return 0;
}

static int linuxspi_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4], res[4];

  if(!p->op[AVR_OP_CHIP_ERASE]) {
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

static int linuxspi_parseexitspecs(PROGRAMMER *pgm, const char *sp) {
  char *cp, *s, *str = mmt_strdup(sp);
  int rv = 0;
  bool help = false;

  s = str;
  while((cp = strtok(s, ","))) {
    s = NULL;
    if(str_eq(cp, "reset")) {
      pgm->exit_reset = EXIT_RESET_ENABLED;
      continue;
    }
    if(str_eq(cp, "noreset")) {
      pgm->exit_reset = EXIT_RESET_DISABLED;
      continue;
    }
    if(str_eq(cp, "help")) {
      help = true;
      rv = LIBAVRDUDE_EXIT;
    }

    if(!help) {
      pmsg_error("invalid exitspec parameter -E %s\n", cp);
      rv = -1;
    }
    msg_error("%s -c %s exitspec parameter options:\n", progname, pgmid);
    msg_error("  -E reset   Programmer will keep the reset line low after programming session\n");
    msg_error("  -E noreset Programmer will not keep the reset line low after programming session\n");
    msg_error("  -E help    Show this help menu and exit\n");
    mmt_free(str);
    return rv;
  }

  mmt_free(str);
  return rv;
}

static int linuxspi_parseextparams(const PROGRAMMER *pgm, const LISTID extparms) {
  int rc = 0;
  bool help = false;

  for(LNODEID ln = lfirst(extparms); ln; ln = lnext(ln)) {
    const char *extended_param = ldata(ln);

    if(str_eq(extended_param, "disable_no_cs")) {
      my.disable_no_cs = 1;
      continue;
    }

    if(str_eq(extended_param, "help")) {
      help = true;
      rc = LIBAVRDUDE_EXIT;
    }

    if(!help) {
      pmsg_error("invalid extended parameter -x %s\n", extended_param);
      rc = -1;
    }
    msg_error("%s -c %s extended options:\n", progname, pgmid);
    msg_error("  -x disable_no_cs Do not use the SPI_NO_CS bit for the SPI driver\n");
    msg_error("  -x help          Show this help menu and exit\n");
    return rc;
  }

  return rc;
}

void linuxspi_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, LINUXSPI);

  pgm_fill_old_pins(pgm);       // TODO to be removed if old pin data no longer needed

  // Mandatory functions
  pgm->initialize = linuxspi_initialize;
  pgm->display = linuxspi_display;
  pgm->enable = linuxspi_enable;
  pgm->disable = linuxspi_disable;
  pgm->program_enable = linuxspi_program_enable;
  pgm->chip_erase = linuxspi_chip_erase;
  pgm->cmd = linuxspi_cmd;
  pgm->open = linuxspi_open;
  pgm->close = linuxspi_close;
  pgm->read_byte = avr_read_byte_default;
  pgm->write_byte = avr_write_byte_default;

  // Optional functions
  pgm->setup = linuxspi_setup;
  pgm->teardown = linuxspi_teardown;
  pgm->parseexitspecs = linuxspi_parseexitspecs;
  pgm->parseextparams = linuxspi_parseextparams;
}

const char linuxspi_desc[] = "SPI using Linux spidev driver";

#else                           // ! HAVE_LINUXSPI

void linuxspi_initpgm(PROGRAMMER *pgm) {
  pmsg_error("Linux SPI driver not available in this configuration\n");
}

const char linuxspi_desc[] = "SPI using Linux spidev driver (not available)";
#endif                          // HAVE_LINUXSPI
