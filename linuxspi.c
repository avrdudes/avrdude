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


#include "ac_cfg.h"

#include "avrdude.h"
#include "libavrdude.h"

#include "linuxspi.h"

#if HAVE_LINUXSPI

/**
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

static int fd_spidev, fd_gpiochip, fd_linehandle;

/**
 * @brief Sends/receives a message in full duplex mode
 * @return -1 on failure, otherwise number of bytes sent/recieved
 */
static int linuxspi_spi_duplex(PROGRAMMER *pgm, const unsigned char *tx, unsigned char *rx, int len)
{
    struct spi_ioc_transfer tr;
    int ret;

    tr = (struct spi_ioc_transfer) {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .delay_usecs = 1,
        //should settle around 400Khz, a standard SPI speed. Adjust using baud parameter (-b)
	.speed_hz = pgm->baudrate == 0 ? 400000 : pgm->baudrate,
        .bits_per_word = 8,
    };

    ret = ioctl(fd_spidev, SPI_IOC_MESSAGE(1), &tr);
    if (ret != len)
        avrdude_message(MSG_INFO, "\n%s: error: Unable to send SPI message\n", progname);

    return (ret == -1) ? -1 : 0;
}

static void linuxspi_setup(PROGRAMMER *pgm)
{
}

static void linuxspi_teardown(PROGRAMMER* pgm)
{
}

static int linuxspi_open(PROGRAMMER *pgm, char *port)
{
    const char *port_error = "%s: error: Unknown port specification. Please use the format /dev/spidev:/dev/gpiochip[:resetno]\n";
    char *spidev, *gpiochip, *reset_pin;
    struct gpiohandle_request req;
    struct gpiohandle_data data;
    int ret;

    if (!port || !strcmp(port, "unknown")) {
        avrdude_message(MSG_INFO, "%s: error: No port specified. Port should point to an spidev device.\n", progname);
        return -1;
    }

    spidev = strtok(port, ":");
    if (!spidev) {
        avrdude_message(MSG_INFO, port_error, progname);
        return -1;
    }

    gpiochip = strtok(NULL, ":");
    if (!gpiochip) {
        avrdude_message(MSG_INFO, port_error, progname);
        return -1;
    }

    /* optional: override reset pin in configuration */
    reset_pin = strtok(NULL, ":");
    if (reset_pin)
        pgm->pinno[PIN_AVR_RESET] = strtoul(reset_pin, NULL, 0);

    strcpy(pgm->port, port);
    fd_spidev = open(pgm->port, O_RDWR);
    if (fd_spidev < 0) {
        avrdude_message(MSG_INFO, "\n%s: error: Unable to open the spidev device %s", progname, pgm->port);
        return -1;
    }

    fd_gpiochip = open(gpiochip, 0);
    if (fd_gpiochip < 0) {
        close(fd_spidev);
        avrdude_message(MSG_INFO, "\n%s error: Unable to open the gpiochip %s", progname, gpiochip);
        ret = -1;
        goto close_spidev;
    }

    strcpy(req.consumer_label, progname);
    req.lines = 1;
    req.lineoffsets[0] = pgm->pinno[PIN_AVR_RESET];
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;

    ret = ioctl(fd_gpiochip, GPIO_GET_LINEHANDLE_IOCTL, &req);
    if (ret == -1) {
        ret = -errno;
        goto close_gpiochip;
    }

    fd_linehandle = req.fd;

    /*
     * Set the reset state and keep it. The pin will be released and set back to
     * its initial value, once the fd_gpiochip is closed.
     */
    data.values[0] = !!(pgm->pinno[PIN_AVR_RESET] & PIN_INVERSE);
    ret = ioctl(fd_linehandle, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
    if (ret == -1) {
        ret = -errno;
        goto close_out;
    }

    return 0;

close_out:
    close(fd_linehandle);
close_gpiochip:
    close(fd_gpiochip);
close_spidev:
    close(fd_spidev);
    return ret;
}

static void linuxspi_close(PROGRAMMER *pgm)
{
    close(fd_spidev);
    close(fd_gpiochip);
}

static void linuxspi_disable(PROGRAMMER* pgm)
{
}

static void linuxspi_enable(PROGRAMMER* pgm)
{
}

static void linuxspi_display(PROGRAMMER* pgm, const char* p)
{
}

static int linuxspi_initialize(PROGRAMMER *pgm, AVRPART *p)
{
    int tries, ret;

    if (p->flags & AVRPART_HAS_TPI) {
        /* We do not support tpi. This is a dedicated SPI thing */
        avrdude_message(MSG_INFO, "%s: error: Programmer " LINUXSPI " does not support TPI\n", progname);
        return -1;
    }

    //enable programming on the part
    tries = 0;
    do
    {
        ret = pgm->program_enable(pgm, p);
        if (ret == 0 || ret == -1)
            break;
    } while(tries++ < 65);

    if (ret)
        avrdude_message(MSG_INFO, "%s: error: AVR device not responding\n", progname);

    return ret;
}

static int linuxspi_cmd(PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res)
{
    return linuxspi_spi_duplex(pgm, cmd, res, 4);
}

static int linuxspi_program_enable(PROGRAMMER *pgm, AVRPART *p)
{
    unsigned char cmd[4], res[4];

    if (!p->op[AVR_OP_PGM_ENABLE]) {
        avrdude_message(MSG_INFO, "%s: error: program enable instruction not defined for part \"%s\"\n", progname, p->desc);
        return -1;
    }

    memset(cmd, 0, sizeof(cmd));
    avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd); //set the cmd
    pgm->cmd(pgm, cmd, res);

    if (res[2] != cmd[1])
        return -2;

    return 0;
}

static int linuxspi_chip_erase(PROGRAMMER *pgm, AVRPART *p)
{
    unsigned char cmd[4], res[4];

    if (!p->op[AVR_OP_CHIP_ERASE]) {
        avrdude_message(MSG_INFO, "%s: error: chip erase instruction not defined for part \"%s\"\n", progname, p->desc);
        return -1;
    }

    memset(cmd, 0, sizeof(cmd));
    avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
    pgm->cmd(pgm, cmd, res);
    usleep(p->chip_erase_delay);
    pgm->initialize(pgm, p);

    return 0;
}

void linuxspi_initpgm(PROGRAMMER *pgm)
{
    strcpy(pgm->type, LINUXSPI);

    pgm_fill_old_pins(pgm); // TODO to be removed if old pin data no longer needed

    /* mandatory functions */
    pgm->initialize     = linuxspi_initialize;
    pgm->display        = linuxspi_display;
    pgm->enable         = linuxspi_enable;
    pgm->disable        = linuxspi_disable;
    pgm->program_enable = linuxspi_program_enable;
    pgm->chip_erase     = linuxspi_chip_erase;
    pgm->cmd            = linuxspi_cmd;
    pgm->open           = linuxspi_open;
    pgm->close          = linuxspi_close;
    pgm->read_byte      = avr_read_byte_default;
    pgm->write_byte     = avr_write_byte_default;

    /* optional functions */
    pgm->setup          = linuxspi_setup;
    pgm->teardown       = linuxspi_teardown;
}

const char linuxspi_desc[] = "SPI using Linux spidev driver";

#else /* !HAVE_LINUXSPI */

void linuxspi_initpgm(PROGRAMMER * pgm)
{
    avrdude_message(MSG_INFO, "%s: Linux SPI driver not available in this configuration\n",
                    progname);
}

const char linuxspi_desc[] = "SPI using Linux spidev driver (not available)";

#endif /* HAVE_LINUXSPI */
