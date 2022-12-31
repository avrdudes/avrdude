/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Support for bitbanging GPIO pins using the /sys/class/gpio interface
 * 
 * Copyright (C) 2013 Radoslav Kolev <radoslav@kolev.info>
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

#include "avrdude.h"
#include "libavrdude.h"

#include "bitbang.h"

#if HAVE_LINUXGPIO

/*
 * GPIO user space helpers
 *
 * Copyright 2009 Analog Devices Inc.
 * Michael Hennerich (hennerich@blackfin.uclinux.org)
 *
 * Licensed under the GPL-2 or later
 */

/*
 * GPIO user space helpers
 * The following functions are acting on an "unsigned gpio" argument, which corresponds to the
 * gpio numbering scheme in the kernel (starting from 0).
 */

#define GPIO_DIR_IN	0
#define GPIO_DIR_OUT	1

static int linuxgpio_export(unsigned int gpio)
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

static int linuxgpio_unexport(unsigned int gpio)
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

static int linuxgpio_openfd(unsigned int gpio)
{
  char filepath[60];

  snprintf(filepath, sizeof(filepath), "/sys/class/gpio/gpio%u/value", gpio);
  return (open(filepath, O_RDWR));
}

static int linuxgpio_dir(unsigned int gpio, unsigned int dir)
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

static int linuxgpio_dir_out(unsigned int gpio)
{
  return linuxgpio_dir(gpio, GPIO_DIR_OUT);
}

static int linuxgpio_dir_in(unsigned int gpio)
{
  return linuxgpio_dir(gpio, GPIO_DIR_IN);
}

/*
 * End of GPIO user space helpers
 */

#define N_GPIO (PIN_MAX + 1)

/* Delay between checks for successful GPIO export (100ms) */
#define GPIO_SYSFS_OPEN_DELAY      100000
/* Number of retries to check for successful GPIO exports */
#define GPIO_SYSFS_OPEN_RETRIES    10

/*
* an array which holds open FDs to /sys/class/gpio/gpioXX/value for all needed pins
*/
static int linuxgpio_fds[N_GPIO] ;


static int linuxgpio_setpin(const PROGRAMMER *pgm, int pinfunc, int value) {
  if(pinfunc < 0 || pinfunc >= N_PINS)
    return -1;

  unsigned pin = pgm->pinno[pinfunc];
  if (pin & PIN_INVERSE)
    value = !value;
  pin &= PIN_MASK;

  if (pin > PIN_MAX || linuxgpio_fds[pin] < 0)
    return -1;

  if (write(linuxgpio_fds[pin], value? "1": "0", 1) != 1)
    return -1;

  if (pgm->ispdelay > 1)
    bitbang_delay(pgm->ispdelay);

  return 0;
}

static int linuxgpio_getpin(const PROGRAMMER *pgm, int pinfunc) {
  if(pinfunc < 0 || pinfunc >= N_PINS)
    return -1;

  unsigned int pin = pgm->pinno[pinfunc];
  int invert = !!(pin & PIN_INVERSE);
  pin &= PIN_MASK;

  if(pin > PIN_MAX || linuxgpio_fds[pin] < 0)
    return -1;

  if(lseek(linuxgpio_fds[pin], 0, SEEK_SET) < 0)
    return -1;

  char c;
  if(read(linuxgpio_fds[pin], &c, 1) != 1)
    return -1;

  return c=='0'? 0+invert: c=='1'? 1-invert: -1;
}

static int linuxgpio_highpulsepin(const PROGRAMMER *pgm, int pinfunc) {
  if(pinfunc < 0 || pinfunc >= N_PINS)
    return -1;

  unsigned int pin = pgm->pinno[pinfunc] & PIN_MASK;

  if (pin > PIN_MAX || linuxgpio_fds[pin] < 0 )
    return -1;

  linuxgpio_setpin(pgm, pinfunc, 1);
  linuxgpio_setpin(pgm, pinfunc, 0);

  return 0;
}



static void linuxgpio_display(const PROGRAMMER *pgm, const char *p) {
    msg_info("%sPin assignment  : /sys/class/gpio/gpio{n}\n",p);
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

static int linuxgpio_open(PROGRAMMER *pgm, const char *port) {
  int r, i, pin;
  char gpio_path[60];
  struct stat stat_buf;

  if (bitbang_check_prerequisites(pgm) < 0)
    return -1;


  for (i=0; i<N_GPIO; i++)
    linuxgpio_fds[i] = -1;
  // Avrdude assumes that if a pin number is invalid it means not used/available
  for (i=0; i<N_PINS; i++) {
    if ((pgm->pinno[i] & PIN_MASK) <= PIN_MAX) {
        pin = pgm->pinno[i] & PIN_MASK;
        if ((r=linuxgpio_export(pin)) < 0) {
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
                linuxgpio_unexport(pin);
                return ret;
            }

            usleep(GPIO_SYSFS_OPEN_DELAY);
        }

        /* Write direction, looping in case of EACCES errors due to delayed
         * udev permission rule application after export */
        for (retry_count = 0; retry_count < GPIO_SYSFS_OPEN_RETRIES; retry_count++) {
            usleep(GPIO_SYSFS_OPEN_DELAY);
            if (i == PIN_AVR_SDI)
                r=linuxgpio_dir_in(pin);
            else
                r=linuxgpio_dir_out(pin);

            if (r >= 0)
                break;

            if (errno != EACCES) {
                linuxgpio_unexport(pin);
                return r;
            }
        }

        if (retry_count)
            pmsg_notice2("needed %d retr%s for linuxgpio_dir_%s(%s)\n",
              retry_count, retry_count > 1? "ies": "y",
              i == PIN_AVR_SDI? "in": "out", avr_pin_name(pin));

        if (r < 0) {
            linuxgpio_unexport(pin);
            return r;
        }

        if ((linuxgpio_fds[pin]=linuxgpio_openfd(pin)) < 0)
            return linuxgpio_fds[pin];
    }
  }

 return(0);
}

static void linuxgpio_close(PROGRAMMER *pgm)
{
  int i, reset_pin;

  reset_pin = pgm->pinno[PIN_AVR_RESET] & PIN_MASK;

  //first configure all pins as input, except RESET
  //this should avoid possible conflicts when AVR firmware starts
  for (i=0; i<N_GPIO; i++) {
    if (linuxgpio_fds[i] >= 0 && i != reset_pin) {
       close(linuxgpio_fds[i]);
       linuxgpio_fds[i] = -1;
       linuxgpio_dir_in(i);
       linuxgpio_unexport(i);
    }
  }
  //configure RESET as input, if there's external pull up it will go high
  if(reset_pin <= PIN_MAX && linuxgpio_fds[reset_pin] >= 0) {
    close(linuxgpio_fds[reset_pin]);
    linuxgpio_fds[reset_pin] = -1;
    linuxgpio_dir_in(reset_pin);
    linuxgpio_unexport(reset_pin);
  }
}

void linuxgpio_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "linuxgpio");

  pgm_fill_old_pins(pgm); // TODO to be removed if old pin data no longer needed

  pgm->rdy_led        = bitbang_rdy_led;
  pgm->err_led        = bitbang_err_led;
  pgm->pgm_led        = bitbang_pgm_led;
  pgm->vfy_led        = bitbang_vfy_led;
  pgm->initialize     = bitbang_initialize;
  pgm->display        = linuxgpio_display;
  pgm->enable         = linuxgpio_enable;
  pgm->disable        = linuxgpio_disable;
  pgm->powerup        = linuxgpio_powerup;
  pgm->powerdown      = linuxgpio_powerdown;
  pgm->program_enable = bitbang_program_enable;
  pgm->chip_erase     = bitbang_chip_erase;
  pgm->cmd            = bitbang_cmd;
  pgm->cmd_tpi        = bitbang_cmd_tpi;
  pgm->open           = linuxgpio_open;
  pgm->close          = linuxgpio_close;
  pgm->setpin         = linuxgpio_setpin;
  pgm->getpin         = linuxgpio_getpin;
  pgm->highpulsepin   = linuxgpio_highpulsepin;
  pgm->read_byte      = avr_read_byte_default;
  pgm->write_byte     = avr_write_byte_default;
}

const char linuxgpio_desc[] = "GPIO bitbanging using the Linux sysfs interface";

#else  /* !HAVE_LINUXGPIO */

void linuxgpio_initpgm(PROGRAMMER *pgm) {
  pmsg_error("Linux sysfs GPIO support not available in this configuration\n");
}

const char linuxgpio_desc[] = "GPIO bitbanging using the Linux sysfs interface (not available)";

#endif /* HAVE_LINUXGPIO */
