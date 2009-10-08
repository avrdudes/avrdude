/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 *
 * avrdude support for The Bus Pirate - universal serial interface
 *
 * Copyright (C) 2009 Michal Ludvig <mludvig@logix.net.nz>
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

/*
 * BusPirate       AVR Chip
 * ---------       --------
 *       GND  <->  GND
 *       +5V  <->  Vcc
 *        CS  <->  RESET
 *      MOSI  <->  MOSI
 *      MISO  <->  MISO
 *   SCL/CLK  <->  SCK
 *
 * Tested with BusPirate PTH, firmware version 2.1 programming ATmega328P
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "avrdude.h"
#include "avr.h"
#include "pgm.h"
#include "serial.h"

/* ====== Serial talker functions ====== */

static int buspirate_getc(struct programmer_t *pgm)
{
	int rc;
	unsigned char ch = 0;

	rc = serial_recv(&pgm->fd, &ch, 1);
	if (rc < 0)
		return EOF;
	return ch;
}

static char *buspirate_readline(struct programmer_t *pgm, char *buf, size_t len)
{
	char *buf_p;
	long orig_serial_recv_timeout = serial_recv_timeout;

	/* Static local buffer - this may come handy at times */
	static char buf_local[100];

	if (buf == NULL) {
		buf = buf_local;
		len = sizeof(buf_local);
	}
	buf_p = buf;
	memset(buf, 0, len);
	while (buf_p < (buf + len - 1)) { /* keep the very last byte == 0 */
		*buf_p = buspirate_getc(pgm);
		if (*buf_p == '\r')
			continue;
		if (*buf_p == '\n')
			break;
		if (*buf_p == EOF) {
			*buf_p = '\0';
			break;
		}
		buf_p++;
		serial_recv_timeout = 100;
	}
	serial_recv_timeout = orig_serial_recv_timeout;
	if (verbose)
		fprintf(stderr, "%s: buspirate_readline(): %s%s",
				progname, buf,
				buf[strlen(buf) - 1] == '\n' ? "" : "\n");
	if (! buf[0]) {
		fprintf(stderr,
				"%s: buspirate_readline(): programmer is not responding\n",
				progname);
		exit(1);
	}
	return buf;
}

static int buspirate_send(struct programmer_t *pgm, char *str)
{
	int rc;

	if (verbose)
		fprintf(stderr, "%s: buspirate_send(): %s", progname, str);

	rc = serial_send(&pgm->fd, (unsigned char *)str, strlen(str));
	if (rc)
		return rc;
	while (strcmp(buspirate_readline(pgm, NULL, 0), str) != 0)
		/* keep reading until we get what we sent there */
		;
	/* by now we should be in sync */
	return 0;
}

static int buspirate_is_prompt(char *str)
{
	/* Prompt ends with '>' all other input probably ends with '\n' */
	return (str[strlen(str) - 1] == '>');
}

static int buspirate_expect(struct programmer_t *pgm, char *send,
							char *expect, int wait_for_prompt)
{
	int got_it = 0;
	size_t expect_len = strlen(expect);
	char *rcvd;

	buspirate_send(pgm, send);
	while (1) {
		rcvd = buspirate_readline(pgm, NULL, 0);

		if (strncmp(rcvd, expect, expect_len) == 0)
			got_it = 1;

		if (buspirate_is_prompt(rcvd))
			break;
	}
	return got_it;
}

/* ====== Do-nothing functions ====== */
static void buspirate_dummy_6(struct programmer_t *pgm,
				const char *p)
{
}

/* ====== Programmer methods ======= */
static int buspirate_open(struct programmer_t *pgm, char * port)
{
	/* BusPirate runs at 115200 by default */
	if(pgm->baudrate == 0)
		pgm->baudrate = 115200;

	strcpy(pgm->port, port);
	serial_open(port, pgm->baudrate, &pgm->fd);

	/* drain any extraneous input */
	serial_drain(&pgm->fd, 0);

	return 0;
}

static void buspirate_close(struct programmer_t *pgm)
{
	serial_close(&pgm->fd);
	pgm->fd.ifd = -1;
}

static int buspirate_start_spi_mode(struct programmer_t *pgm)
{
	int spi_cmd = -1;
	int cmd;
	char *rcvd, *mode, buf[5];

	buspirate_send(pgm, "M\n");
	while(1) {
		rcvd = buspirate_readline(pgm, NULL, 0);
		if (spi_cmd == -1 && sscanf(rcvd, "%d. %as", &cmd, &mode)) {
			if (strcmp(mode, "SPI") == 0)
				spi_cmd = cmd;
		}
		if (buspirate_is_prompt(rcvd))
			break;
	}
	if (spi_cmd == -1) {
		fprintf(stderr,
				"%s: SPI mode number not found. Does your BusPirate support SPI?\n",
				progname);
		fprintf(stderr, "%s: Try powercycling your BusPirate and try again.\n",
				progname);
		return -1;
	}
	snprintf(buf, sizeof(buf), "%d\n", spi_cmd);
	buspirate_send(pgm, buf);
	buf[0] = '\0';
	while (1) {
		rcvd = buspirate_readline(pgm, NULL, 0);
		if (strstr(rcvd, "Normal (H=3.3V, L=GND)")) {
			/* BP firmware 2.1 defaults to Open-drain output.
			 * That doesn't work on my board, even with pull-up
			 * resistors. Select 3.3V output mode instead. */
			sscanf(rcvd, " %d.", &cmd);
			snprintf(buf, sizeof(buf), "%d\n", cmd);
		}
		if (buspirate_is_prompt(rcvd)) {
			if (strncmp(rcvd, "SPI>", 4) == 0) {
				printf("BusPirate is now configured for SPI\n");
				break;
			}
			/* Not yet 'SPI>' prompt */
			if (buf[0]) {
				buspirate_send(pgm, buf);
				buf[0] = '\0';
			} else
				buspirate_send(pgm, "\n");
		}
	}
	return 0;
}

static void buspirate_enable(struct programmer_t *pgm)
{
	char *rcvd;

	printf("Detecting BusPirate...\n");
	buspirate_send(pgm, "#\n");
	while(1) {
		rcvd = buspirate_readline(pgm, NULL, 0);
		if (strncmp(rcvd, "RESET", 5) == 0)
			continue;
		if (buspirate_is_prompt(rcvd)) {
			puts("**");
			break;
		}
		printf("**  %s", rcvd);
	}

	if (buspirate_start_spi_mode(pgm) < 0)
		fprintf(stderr, "%s: Failed to start SPI mode\n", progname);
}

static void buspirate_disable(struct programmer_t *pgm)
{
	buspirate_expect(pgm, "#\n", "RESET", 1);
}

static int buspirate_initialize(struct programmer_t *pgm, AVRPART * p)
{
	pgm->powerup(pgm);

	return pgm->program_enable(pgm, p);
}

static void buspirate_powerup(struct programmer_t *pgm)
{
	if (!buspirate_expect(pgm, "W\n", "POWER SUPPLIES ON", 1)) {
		fprintf(stderr, "%s: warning: did not get a response to PowerUp command.\n", progname);
		fprintf(stderr, "%s: warning: Trying to continue anyway...\n", progname);
	}
}

static void buspirate_powerdown(struct programmer_t *pgm)
{
	if (!buspirate_expect(pgm, "w\n", "POWER SUPPLIES OFF", 1))
		fprintf(stderr, "%s: warning: did not get a response to PowerDown command.\n", progname);
}

static int buspirate_cmd(struct programmer_t *pgm,
						 unsigned char cmd[4],
						 unsigned char res[4])
{
	char buf[25];
	char *rcvd;
	int spi_write, spi_read, i = 0;

	snprintf(buf, sizeof(buf), "0x%02x 0x%02x 0x%02x 0x%02x\n",
			 cmd[0], cmd[1], cmd[2], cmd[3]);
	buspirate_send(pgm, buf);
	while (1) {
		rcvd = buspirate_readline(pgm, NULL, 0);
		/* WRITE: 0xAC READ: 0x04 */
		if (sscanf(rcvd, "WRITE: 0x%x READ: 0x%x", &spi_write, &spi_read) == 2) {
			res[i++] = spi_read;
		}
		if (buspirate_is_prompt(rcvd))
			break;
	}
	if (i != 4) {
		fprintf(stderr, "%s: error: SPI has not read 4 bytes back\n", progname);
		return -1;
	}
	return 0;
}

static int buspirate_program_enable(struct programmer_t *pgm, AVRPART * p)
{
	unsigned char cmd[4];
	unsigned char res[4];

	buspirate_expect(pgm, "{\n", "CS ENABLED", 1);

	if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
		fprintf(stderr,
				"program enable instruction not defined for part \"%s\"\n",
				p->desc);
		return -1;
	}

	memset(cmd, 0, sizeof(cmd));
	avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
	pgm->cmd(pgm, cmd, res);

	if (res[2] != cmd[1])
		return -2;

	return 0;
}

static int buspirate_chip_erase(struct programmer_t *pgm, AVRPART * p)
{
	unsigned char cmd[4];
	unsigned char res[4];

	if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
		fprintf(stderr,
				"chip erase instruction not defined for part \"%s\"\n",
				p->desc);
		return -1;
	}

	pgm->pgm_led(pgm, ON);

	memset(cmd, 0, sizeof(cmd));

	avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
	pgm->cmd(pgm, cmd, res);
	usleep(p->chip_erase_delay);
	pgm->initialize(pgm, p);

	pgm->pgm_led(pgm, OFF);

	return 0;
}

void buspirate_initpgm(struct programmer_t *pgm)
{
	strcpy(pgm->type, "BusPirate");

	pgm->display        = buspirate_dummy_6;

	/* BusPirate itself related methods */
	pgm->open           = buspirate_open;
	pgm->close          = buspirate_close;
	pgm->enable         = buspirate_enable;
	pgm->disable        = buspirate_disable;
	pgm->initialize     = buspirate_initialize;

	/* Chip related methods */
	pgm->powerup        = buspirate_powerup;
	pgm->powerdown      = buspirate_powerdown;
	pgm->program_enable = buspirate_program_enable;
	pgm->chip_erase     = buspirate_chip_erase;
	pgm->cmd            = buspirate_cmd;
	pgm->read_byte      = avr_read_byte_default;
	pgm->write_byte     = avr_write_byte_default;
}

