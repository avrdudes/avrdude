/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Support for bitbanging GPIO pins using libgpiod or the /sys/class/gpio interface
 *
 * Copyright (C) 2013 Radoslav Kolev <radoslav@kolev.info>
 * Copyright (C) 2023 Sebastian Kuzminsky <seb@highlab.com>
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

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>

#ifdef HAVE_LIBGPIOD
#include <gpiod.h>
#endif

#include "avrdude.h"
#include "libavrdude.h"

#include "bitbang.h"

#if HAVE_LINUXGPIO

/*
 * Sysfs GPIO user space helpers
 *
 * Copyright 2009 Analog Devices Inc.
 * Michael Hennerich (hennerich@blackfin.uclinux.org)
 *
 * Licensed under the GPL-2 or later
 */

/*
 * Sysfs GPIO user space helpers
 * The following functions are acting on an "unsigned gpio" argument, which corresponds to the
 * gpio numbering scheme in the kernel (starting from 0).
 */

#define GPIO_DIR_IN	0
#define GPIO_DIR_OUT	1

static int linuxgpio_sysfs_export(unsigned int gpio)
{
  int fd, len, r;
  char buf[11];

  fd = open("/sys/class/gpio/export", O_WRONLY);
  if (fd < 0) {
    pmsg_ext_error("cannot open /sys/class/gpio/export: %s\n", strerror(errno));
    return fd;
  }

  len = snprintf(buf, sizeof(buf), "%u", gpio);
  r = write(fd, buf, len);
  close(fd);

  return r;
}

static int linuxgpio_sysfs_unexport(unsigned int gpio)
{
  int fd, len, r;
  char buf[11];

  fd = open("/sys/class/gpio/unexport", O_WRONLY);
  if (fd < 0) {
    pmsg_ext_error("cannot open /sys/class/gpio/unexport: %s\n", strerror(errno));
    return fd;
  }

  len = snprintf(buf, sizeof(buf), "%u", gpio);
  r = write(fd, buf, len);
  close(fd);

  return r;
}

static int linuxgpio_sysfs_openfd(unsigned int gpio)
{
  char filepath[60];

  snprintf(filepath, sizeof(filepath), "/sys/class/gpio/gpio%u/value", gpio);
  return (open(filepath, O_RDWR));
}

static int linuxgpio_sysfs_dir(unsigned int gpio, unsigned int dir)
{
  int fd, r;
  char buf[60];

  snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%u/direction", gpio);

  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    pmsg_ext_error("cannot open %s: %s\n", buf, strerror(errno));
    return fd;
  }

  if (dir == GPIO_DIR_OUT)
    r = write(fd, "out", 4);
  else
    r = write(fd, "in", 3);

  close(fd);

  return r;
}

static int linuxgpio_sysfs_dir_out(unsigned int gpio)
{
  return linuxgpio_sysfs_dir(gpio, GPIO_DIR_OUT);
}

static int linuxgpio_sysfs_dir_in(unsigned int gpio)
{
  return linuxgpio_sysfs_dir(gpio, GPIO_DIR_IN);
}

/*
 * End of Sysfs GPIO user space helpers
 */

#define N_GPIO (PIN_MAX + 1)

/* Delay between checks for successful GPIO export (100ms) */
#define GPIO_SYSFS_OPEN_DELAY      100000
/* Number of retries to check for successful GPIO exports */
#define GPIO_SYSFS_OPEN_RETRIES    10

/*
* an array which holds open FDs to /sys/class/gpio/gpioXX/value for all needed pins
*/
static int linuxgpio_sysfs_fds[N_GPIO] ;


static int linuxgpio_sysfs_setpin(const PROGRAMMER *pgm, int pinfunc, int value) {
  if(pinfunc < 0 || pinfunc >= N_PINS)
    return -1;

  unsigned pin = pgm->pinno[pinfunc];
  if (pin & PIN_INVERSE)
    value = !value;
  pin &= PIN_MASK;

  if (pin > PIN_MAX || linuxgpio_sysfs_fds[pin] < 0)
    return -1;

  if (write(linuxgpio_sysfs_fds[pin], value? "1": "0", 1) != 1)
    return -1;

  if (pgm->ispdelay > 1)
    bitbang_delay(pgm->ispdelay);

  return 0;
}

static int linuxgpio_sysfs_getpin(const PROGRAMMER *pgm, int pinfunc) {
  if(pinfunc < 0 || pinfunc >= N_PINS)
    return -1;

  unsigned int pin = pgm->pinno[pinfunc];
  int invert = !!(pin & PIN_INVERSE);
  pin &= PIN_MASK;

  if(pin > PIN_MAX || linuxgpio_sysfs_fds[pin] < 0)
    return -1;

  if(lseek(linuxgpio_sysfs_fds[pin], 0, SEEK_SET) < 0)
    return -1;

  char c;
  if(read(linuxgpio_sysfs_fds[pin], &c, 1) != 1)
    return -1;

  return c=='0'? 0+invert: c=='1'? 1-invert: -1;
}

static int linuxgpio_sysfs_highpulsepin(const PROGRAMMER *pgm, int pinfunc) {
  if(pinfunc < 0 || pinfunc >= N_PINS)
    return -1;

  unsigned int pin = pgm->pinno[pinfunc] & PIN_MASK;

  if (pin > PIN_MAX || linuxgpio_sysfs_fds[pin] < 0 )
    return -1;

  linuxgpio_sysfs_setpin(pgm, pinfunc, 1);
  linuxgpio_sysfs_setpin(pgm, pinfunc, 0);

  return 0;
}



static void linuxgpio_sysfs_display(const PROGRAMMER *pgm, const char *p) {
    msg_info("%sPin assignment        : /sys/class/gpio/gpio{n}\n",p);
    pgm_display_generic_mask(pgm, p, SHOW_AVR_PINS);
}

static void linuxgpio_enable(PROGRAMMER *pgm, const AVRPART *p) {
  /* nothing */
}

static void linuxgpio_disable(const PROGRAMMER *pgm) {
  /* nothing */
}

static void linuxgpio_powerup(const PROGRAMMER *pgm) {
  /* nothing */
}

static void linuxgpio_powerdown(const PROGRAMMER *pgm) {
  /* nothing */
}

static int linuxgpio_sysfs_open(PROGRAMMER *pgm, const char *port) {
  int r, i, pin;
  char gpio_path[60];
  struct stat stat_buf;

  if (bitbang_check_prerequisites(pgm) < 0)
    return -1;


  for (i=0; i<N_GPIO; i++)
    linuxgpio_sysfs_fds[i] = -1;
  // Avrdude assumes that if a pin number is invalid it means not used/available
  for (i = 1; i < N_PINS; i++) { // The pin enumeration in libavrdude.h starts with PPI_AVR_VCC = 1
    if ((pgm->pinno[i] & PIN_MASK) <= PIN_MAX) {
        pin = pgm->pinno[i] & PIN_MASK;
        if ((r=linuxgpio_sysfs_export(pin)) < 0) {
            pmsg_ext_error("cannot export GPIO %d, already exported/busy?: %s",
                    pin, strerror(errno));
            return r;
        }

        /* Wait until GPIO directory appears */
        snprintf(gpio_path, sizeof(gpio_path), "/sys/class/gpio/gpio%u", pin);
        unsigned int retry_count;
        for (retry_count = 0; retry_count < GPIO_SYSFS_OPEN_RETRIES; retry_count++) {
            int ret = stat(gpio_path, &stat_buf);
            if (ret == 0) {
                break;
            } else if (ret < 0 && errno != ENOENT) {
                linuxgpio_sysfs_unexport(pin);
                return ret;
            }

            usleep(GPIO_SYSFS_OPEN_DELAY);
        }

        /* Write direction, looping in case of EACCES errors due to delayed
         * udev permission rule application after export */
        for (retry_count = 0; retry_count < GPIO_SYSFS_OPEN_RETRIES; retry_count++) {
            usleep(GPIO_SYSFS_OPEN_DELAY);
            if (i == PIN_AVR_SDI)
                r=linuxgpio_sysfs_dir_in(pin);
            else
                r=linuxgpio_sysfs_dir_out(pin);

            if (r >= 0)
                break;

            if (errno != EACCES) {
                linuxgpio_sysfs_unexport(pin);
                return r;
            }
        }

        if (retry_count)
            pmsg_notice2("needed %d retr%s for linuxgpio_sysfs_dir_%s(%s)\n",
              retry_count, retry_count > 1? "ies": "y",
              i == PIN_AVR_SDI? "in": "out", avr_pin_name(pin));

        if (r < 0) {
            linuxgpio_sysfs_unexport(pin);
            return r;
        }

        if ((linuxgpio_sysfs_fds[pin]=linuxgpio_sysfs_openfd(pin)) < 0)
            return linuxgpio_sysfs_fds[pin];
    }
  }

 return(0);
}

static void linuxgpio_sysfs_close(PROGRAMMER *pgm)
{
  int i, reset_pin;

  reset_pin = pgm->pinno[PIN_AVR_RESET] & PIN_MASK;

  //first configure all pins as input, except RESET
  //this should avoid possible conflicts when AVR firmware starts
  for (i=0; i<N_GPIO; i++) {
    if (linuxgpio_sysfs_fds[i] >= 0 && i != reset_pin) {
       close(linuxgpio_sysfs_fds[i]);
       linuxgpio_sysfs_fds[i] = -1;
       linuxgpio_sysfs_dir_in(i);
       linuxgpio_sysfs_unexport(i);
    }
  }
  //configure RESET as input, if there's external pull up it will go high
  if(reset_pin <= PIN_MAX && linuxgpio_sysfs_fds[reset_pin] >= 0) {
    close(linuxgpio_sysfs_fds[reset_pin]);
    linuxgpio_sysfs_fds[reset_pin] = -1;
    linuxgpio_sysfs_dir_in(reset_pin);
    linuxgpio_sysfs_unexport(reset_pin);
  }
}


//
// libgpiod backend for the linuxgpio programmer.
//

#ifdef HAVE_LIBGPIOD

struct gpiod_line * linuxgpio_libgpiod_lines[N_PINS];

// Try to tell if libgpiod is going to work.
// Returns True (non-zero) if it looks like libgpiod will work, False
// (zero) if libgpiod will not work.
static int libgpiod_is_working(void) {
  char const * filename = "/dev/gpiochip0";
  struct gpiod_chip * gpiod_chip_ptr;
  gpiod_chip_ptr = gpiod_chip_open(filename);
  if (gpiod_chip_ptr == NULL) {
    msg_info("failed to open gpiod chip %s: %s\n", filename, strerror(errno));
    return 0;
  }
  gpiod_chip_close(gpiod_chip_ptr);
  return 1;
}


static void linuxgpio_libgpiod_display(const PROGRAMMER *pgm, const char *p) {
  msg_info("%sPin assignment        : libgpiod\n", p);
  pgm_display_generic_mask(pgm, p, SHOW_AVR_PINS);
}


static int linuxgpio_libgpiod_open(PROGRAMMER *pgm, const char *port) {
  int i;

  if (bitbang_check_prerequisites(pgm) < 0) {
    return -1;
  }

  for (i = 0; i < N_PINS; ++i) {
    linuxgpio_libgpiod_lines[i] = NULL;
  }

  // Avrdude assumes that if a pin number is invalid it means not used/available
  for (i = 1; i < N_PINS; i++) { // The pin enumeration in libavrdude.h starts with PPI_AVR_VCC = 1
    int r;
    int gpio_num;

    gpio_num = pgm->pinno[i] & PIN_MASK;
    if (gpio_num > PIN_MAX) {
      continue;
    }

    linuxgpio_libgpiod_lines[i] = gpiod_line_get(port, gpio_num);
    if (linuxgpio_libgpiod_lines[i] == NULL) {
      msg_error("failed to open %s line %d: %s\n", port, gpio_num, strerror(errno));
      return -1;
    }

    // Request the pin, select direction.
    if (i == PIN_AVR_SDI) {
        r = gpiod_line_request_input(linuxgpio_libgpiod_lines[i], "avrdude");
    } else {
        r = gpiod_line_request_output(linuxgpio_libgpiod_lines[i], "avrdude", 0);
    }
    if (r != 0) {
      msg_error("failed to request %s line %d: %s\n", port, gpio_num, strerror(errno));
      return -1;
    }

  }

  return(0);
}


static void linuxgpio_libgpiod_close(PROGRAMMER *pgm) {
  int i;

  // First configure all pins as input, except RESET.
  // This should avoid possible conflicts when AVR firmware starts.
  for (i = 0; i < N_PINS; ++i) {
    if (linuxgpio_libgpiod_lines[i] != NULL && i != PIN_AVR_RESET) {
      int r = gpiod_line_set_direction_input(linuxgpio_libgpiod_lines[i]);
      if (r != 0) {
        msg_error("failed to set pin %s to input: %s\n", gpiod_line_name(linuxgpio_libgpiod_lines[i]), strerror(errno));
      }
      gpiod_line_release(linuxgpio_libgpiod_lines[i]);
      linuxgpio_libgpiod_lines[i] = NULL;
    }
  }

  // Configure RESET as input.
  if (linuxgpio_libgpiod_lines[PIN_AVR_RESET] != NULL) {
    int r = gpiod_line_set_direction_input(linuxgpio_libgpiod_lines[PIN_AVR_RESET]);
    if (r != 0) {
      msg_error("failed to set pin %s to input: %s\n", gpiod_line_name(linuxgpio_libgpiod_lines[PIN_AVR_RESET]), strerror(errno));
    }
    gpiod_line_release(linuxgpio_libgpiod_lines[PIN_AVR_RESET]);
    linuxgpio_libgpiod_lines[PIN_AVR_RESET] = NULL;
  }
}


static int linuxgpio_libgpiod_setpin(const PROGRAMMER *pgm, int pinfunc, int value) {
  if (pinfunc < 0 || pinfunc >= N_PINS) {
    return -1;
  }

  unsigned pin = pgm->pinno[pinfunc];
  if (pin & PIN_INVERSE) {
    value = !value;
  }
  pin &= PIN_MASK;

  if (pin > PIN_MAX || linuxgpio_libgpiod_lines[pinfunc] == NULL) {
    return -1;
  }

  int r = gpiod_line_set_value(linuxgpio_libgpiod_lines[pinfunc], value);
  if (r != 0) {
    msg_error("failed to set value of %s (%s) to %d: %s\n", avr_pin_name(pinfunc), gpiod_line_name(linuxgpio_libgpiod_lines[pinfunc]), value, strerror(errno));
    return -1;
  }

  if (pgm->ispdelay > 1) {
    bitbang_delay(pgm->ispdelay);
  }

  return 0;
}

static int linuxgpio_libgpiod_getpin(const PROGRAMMER *pgm, int pinfunc) {
  if (pinfunc < 0 || pinfunc >= N_PINS) {
    return -1;
  }

  unsigned int pin = pgm->pinno[pinfunc];
  int invert = !!(pin & PIN_INVERSE);
  pin &= PIN_MASK;

  if (pin > PIN_MAX || linuxgpio_libgpiod_lines[pinfunc] == NULL) {
    return -1;
  }

  int r = gpiod_line_get_value(linuxgpio_libgpiod_lines[pinfunc]);
  if (r == -1) {
    msg_error("failed to read %s: %s\n", gpiod_line_name(linuxgpio_libgpiod_lines[pinfunc]), strerror(errno));
    return -1;
  }

  return r ^ invert;
}


static int linuxgpio_libgpiod_highpulsepin(const PROGRAMMER *pgm, int pinfunc) {
  if(pinfunc < 0 || pinfunc >= N_PINS) {
    return -1;
  }

  unsigned int pin = pgm->pinno[pinfunc] & PIN_MASK;

  if (pin > PIN_MAX || linuxgpio_libgpiod_lines[pinfunc] == NULL ) {
    return -1;
  }

  int r = gpiod_line_set_value(linuxgpio_libgpiod_lines[pinfunc], 1);
  if (r != 0) {
    msg_error("failed to set value\n");
    return -1;
  }

  r = gpiod_line_set_value(linuxgpio_libgpiod_lines[pinfunc], 0);
  if (r != 0) {
    msg_error("failed to set value\n");
    return -1;
  }

  return 0;
}


#endif // HAVE_LIBGPIOD


void linuxgpio_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "linuxgpio");

  pgm_fill_old_pins(pgm); // TODO to be removed if old pin data no longer needed

  pgm->rdy_led        = bitbang_rdy_led;
  pgm->err_led        = bitbang_err_led;
  pgm->pgm_led        = bitbang_pgm_led;
  pgm->vfy_led        = bitbang_vfy_led;
  pgm->initialize     = bitbang_initialize;
  pgm->display        = linuxgpio_sysfs_display;
  pgm->enable         = linuxgpio_enable;
  pgm->disable        = linuxgpio_disable;
  pgm->powerup        = linuxgpio_powerup;
  pgm->powerdown      = linuxgpio_powerdown;
  pgm->program_enable = bitbang_program_enable;
  pgm->chip_erase     = bitbang_chip_erase;
  pgm->cmd            = bitbang_cmd;
  pgm->cmd_tpi        = bitbang_cmd_tpi;
  pgm->open           = linuxgpio_sysfs_open;
  pgm->close          = linuxgpio_sysfs_close;
  pgm->setpin         = linuxgpio_sysfs_setpin;
  pgm->getpin         = linuxgpio_sysfs_getpin;
  pgm->highpulsepin   = linuxgpio_sysfs_highpulsepin;
  pgm->read_byte      = avr_read_byte_default;
  pgm->write_byte     = avr_write_byte_default;

#ifdef HAVE_LIBGPIOD
  if (libgpiod_is_working()) {
    msg_info("using libgpiod for linuxgpio\n");
    pgm->display        = linuxgpio_libgpiod_display;
    pgm->open           = linuxgpio_libgpiod_open;
    pgm->close          = linuxgpio_libgpiod_close;
    pgm->setpin         = linuxgpio_libgpiod_setpin;
    pgm->getpin         = linuxgpio_libgpiod_getpin;
    pgm->highpulsepin   = linuxgpio_libgpiod_highpulsepin;
  } else {
    msg_info("falling back to sysfs for linuxgpio\n");
  }
#endif
}

const char linuxgpio_desc[] = "GPIO bitbanging using the Linux libgpiod or sysfs interface";

#else  /* !HAVE_LINUXGPIO */

void linuxgpio_initpgm(PROGRAMMER *pgm) {
  pmsg_error("Linux libgpiod/sysfs GPIO support not available in this configuration\n");
}

const char linuxgpio_desc[] = "GPIO bitbanging using the Linux libgpiod or sysfs interface (not available)";

#endif /* HAVE_LINUXGPIO */
