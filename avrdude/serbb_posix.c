/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
 * Copyright (C) 2005 Michael Holzt <kju-avr@fqdn.org>
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
/* $Id$ */

/*
 * Posix serial bitbanging interface for avrdude.
 */

#if !defined(WIN32NATIVE)

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <termios.h>

#include "avr.h"
#include "pindefs.h"
#include "pgm.h"
#include "bitbang.h"

#undef DEBUG

extern char *progname;
struct termios oldmode;

/*
  serial port/pin mapping

  1	cd	<-
  2	rxd	<-
  3	txd	->
  4	dtr	->
  5	dsr	<-
  6	rts	->
  7	cts	<-
*/

static int serregbits[] =
{ TIOCM_CD, 0, 0, TIOCM_DTR, TIOCM_DSR, TIOCM_RTS, TIOCM_CTS };

#ifdef DEBUG
static char *serpins[7] =
  { "CD", "RXD", "TXD ~RESET", "DTR MOSI", "DSR", "RTS SCK", "CTS MISO" };
#endif

static int serbb_setpin(PROGRAMMER * pgm, int pin, int value)
{
  unsigned int	ctl;

  if (pin & PIN_INVERSE)
  {
    value  = !value;
    pin   &= PIN_MASK;
  }

  if ( pin < 1 || pin > 7 )
    return -1;

  pin--;

#ifdef DEBUG
  printf("%s to %d\n",serpins[pin],value);
#endif

  switch ( pin )
  {
    case 2:  /* txd */
             ioctl(pgm->fd, value ? TIOCSBRK : TIOCCBRK, 0);
             return 0;

    case 3:  /* dtr, rts */
    case 5:  ioctl(pgm->fd, TIOCMGET, &ctl);
             if ( value )
               ctl |= serregbits[pin];
             else
               ctl &= ~(serregbits[pin]);
             ioctl(pgm->fd, TIOCMSET, &ctl);
             return 0;

    default: /* impossible */
             return -1;
  }
}

static int serbb_getpin(PROGRAMMER * pgm, int pin)
{
  unsigned int	ctl;
  unsigned char invert;

  if (pin & PIN_INVERSE)
  {
    invert = 1;
    pin   &= PIN_MASK;
  } else
    invert = 0;

  if ( pin < 1 || pin > 7 )
    return(-1);

  pin --;

  switch ( pin )
  {
    case 1:  /* rxd, currently not implemented, FIXME */
             return(-1);

    case 0:  /* cd, dsr, dtr, rts, cts */
    case 3:
    case 4:
    case 5:
    case 6:  ioctl(pgm->fd, TIOCMGET, &ctl);
             if ( !invert )
             {
#ifdef DEBUG
               printf("%s is %d\n",serpins[pin],(ctl & serregbits[pin]) ? 1 : 0 );
#endif
               return ( (ctl & serregbits[pin]) ? 1 : 0 );
             }
             else
             {
#ifdef DEBUG
               printf("%s is %d (~)\n",serpins[pin],(ctl & serregbits[pin]) ? 0 : 1 );
#endif
               return (( ctl & serregbits[pin]) ? 0 : 1 );
             }

    default: /* impossible */
             return(-1);
  }
}

static int serbb_highpulsepin(PROGRAMMER * pgm, int pin)
{
  if (pin < 1 || pin > 7)
    return -1;

  serbb_setpin(pgm, pin, 1);
  if (pgm->ispdelay > 1)
    bitbang_delay(pgm->ispdelay);

  serbb_setpin(pgm, pin, 0);
  if (pgm->ispdelay > 1)
    bitbang_delay(pgm->ispdelay);

  return 0;
}



static void serbb_display(PROGRAMMER *pgm, char *p)
{
  /* MAYBE */
}

static void serbb_enable(PROGRAMMER *pgm)
{
  /* nothing */
}

static void serbb_disable(PROGRAMMER *pgm)
{
  /* nothing */
}

static void serbb_powerup(PROGRAMMER *pgm)
{
  /* nothing */
}

static void serbb_powerdown(PROGRAMMER *pgm)
{
  /* nothing */
}

static int serbb_open(PROGRAMMER *pgm, char *port)
{
  struct termios mode;
  int flags;

  bitbang_check_prerequisites(pgm);

  /* adapted from uisp code */

  pgm->fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (pgm->fd < 0)
    return(-1);

  tcgetattr(pgm->fd, &mode);
  oldmode = mode;

  mode.c_iflag = IGNBRK | IGNPAR;
  mode.c_oflag = 0;
  mode.c_cflag = CLOCAL | CREAD | CS8 | B9600;
  mode.c_cc [VMIN] = 1;
  mode.c_cc [VTIME] = 0;

  tcsetattr(pgm->fd, TCSANOW, &mode);

  /* Clear O_NONBLOCK flag.  */
  flags = fcntl(pgm->fd, F_GETFL, 0);
  if (flags == -1)
    {
      fprintf(stderr, "%s: Can not get flags\n", progname);
      return(-1);
    }
  flags &= ~O_NONBLOCK;
  if (fcntl(pgm->fd, F_SETFL, flags) == -1)
    {
      fprintf(stderr, "%s: Can not clear nonblock flag\n", progname);
      return(-1);
    }

  return(0);
}

static void serbb_close(PROGRAMMER *pgm)
{
  tcsetattr(pgm->fd, TCSANOW, &oldmode);
  return;
}

void serbb_initpgm(PROGRAMMER *pgm)
{
  strcpy(pgm->type, "SERBB");

  pgm->rdy_led        = bitbang_rdy_led;
  pgm->err_led        = bitbang_err_led;
  pgm->pgm_led        = bitbang_pgm_led;
  pgm->vfy_led        = bitbang_vfy_led;
  pgm->initialize     = bitbang_initialize;
  pgm->display        = serbb_display;
  pgm->enable         = serbb_enable;
  pgm->disable        = serbb_disable;
  pgm->powerup        = serbb_powerup;
  pgm->powerdown      = serbb_powerdown;
  pgm->program_enable = bitbang_program_enable;
  pgm->chip_erase     = bitbang_chip_erase;
  pgm->cmd            = bitbang_cmd;
  pgm->open           = serbb_open;
  pgm->close          = serbb_close;
  pgm->setpin         = serbb_setpin;
  pgm->getpin         = serbb_getpin;
  pgm->highpulsepin   = serbb_highpulsepin;
}

#endif  /* WIN32NATIVE */
