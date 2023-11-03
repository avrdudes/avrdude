/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) 2005 Juliane Holzt <avrdude@juliane.holzt.de>
 * Copyright (C) 2011 Darell Tan <darell.tan@gmail.com>
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
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#if defined(WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <signal.h>
#include <sys/time.h>
#endif

#include "avrdude.h"
#include "libavrdude.h"

#include "par.h"
#include "serbb.h"
#include "tpi.h"
#include "bitbang.h"

static int delay_decrement;

#if defined(WIN32)
static int has_perfcount;
static LARGE_INTEGER freq;
#else
static volatile int done;

typedef void (*mysighandler_t)(int);
static mysighandler_t saved_alarmhandler;

static void alarmhandler(int signo)
{
  done = 1;
  signal(SIGALRM, saved_alarmhandler);
}
#endif /* WIN32 */

/*
 * Calibrate the microsecond delay loop below.
 */
static void bitbang_calibrate_delay(void)
{
#if defined(WIN32)
  /*
   * If the hardware supports a high-resolution performance counter,
   * we ultimately prefer that one, as it gives quite accurate delays
   * on modern high-speed CPUs.
   */
  if (QueryPerformanceFrequency(&freq))
  {
    has_perfcount = 1;
    pmsg_notice2("using performance counter for bitbang delays\n");
  }
  else
  {
    /*
     * If a high-resolution performance counter is not available, we
     * don't have any Win32 implementation for setting up the
     * per-microsecond delay count, so we can only run on a
     * preconfigured delay stepping there.  The figure below should at
     * least be correct within an order of magnitude, judging from the
     * auto-calibration figures seen on various Unix systems on
     * comparable hardware.
     */
    pmsg_notice2("using guessed per-microsecond delay count for bitbang delays\n");
    delay_decrement = 100;
  }
#else  /* !WIN32 */
  struct itimerval itv;
  volatile int i;

  pmsg_notice2("calibrating delay loop ...");
  i = 0;
  done = 0;
  saved_alarmhandler = signal(SIGALRM, alarmhandler);
  /*
   * Set ITIMER_REAL to 100 ms.  All known systems have a timer
   * granularity of 10 ms or better, so counting the delay cycles
   * accumulating over 100 ms should give us a rather realistic
   * picture, without annoying the user by a lengthy startup time (as
   * an alarm(1) would do).  Of course, if heavy system activity
   * happens just during calibration but stops before the remaining
   * part of AVRDUDE runs, this will yield wrong values.  There's not
   * much we can do about this.
   */
  itv.it_value.tv_sec = 0;
  itv.it_value.tv_usec = 100000;
  itv.it_interval.tv_sec = itv.it_interval.tv_usec = 0;
  setitimer(ITIMER_REAL, &itv, 0);
  while (!done)
    i--;
  itv.it_value.tv_sec = itv.it_value.tv_usec = 0;
  setitimer(ITIMER_REAL, &itv, 0);
  /*
   * Calculate back from 100 ms to 1 us.
   */
  delay_decrement = -i / 100000;
  msg_notice2(" calibrated to %d cycles per us\n",
                  delay_decrement);
#endif /* WIN32 */
}

/*
 * Delay for approximately the number of microseconds specified.
 * usleep()'s granularity is usually like 1 ms or 10 ms, so it's not
 * really suitable for short delays in bit-bang algorithms.
 */
void bitbang_delay(unsigned int us)
{
#if defined(WIN32)
  LARGE_INTEGER countNow, countEnd;

  if (has_perfcount)
  {
    QueryPerformanceCounter(&countNow);
    countEnd.QuadPart = countNow.QuadPart + freq.QuadPart * us / 1000000ll;

    while (countNow.QuadPart < countEnd.QuadPart)
      QueryPerformanceCounter(&countNow);
  }
  else /* no performance counters -- run normal uncalibrated delay */
  {
#endif  /* WIN32 */
  volatile unsigned int del = us * delay_decrement;

  while (del > 0)
    del--;
#if defined(WIN32)
  }
#endif /* WIN32 */
}

/*
 * transmit and receive a byte of data to/from the AVR device
 */
static unsigned char bitbang_txrx(const PROGRAMMER *pgm, unsigned char byte) {
  int i;
  unsigned char r, b, rbyte;

  rbyte = 0;
  for (i=7; i>=0; i--) {
    /*
     * Write and read one bit on SPI.
     * Some notes on timing: Let T be the time it takes to do
     * one pgm->setpin()-call resp. par clrpin()-call, then
     * - SCK is high for 2T
     * - SCK is low for 2T
     * - SDO setuptime is 1T
     * - SDO holdtime is 3T
     * - SCK low to SDI read is 2T to 3T
     * So we are within programming specs (expect for AT90S1200),
     * if and only if T>t_CLCL (t_CLCL=clock period of target system).
     *
     * Due to the delay introduced by "IN" and "OUT"-commands,
     * T is greater than 1us (more like 2us) on x86-architectures.
     * So programming works safely down to 1MHz target clock.
    */

    b = (byte >> i) & 0x01;

    /* set the data input line as desired */
    pgm->setpin(pgm, PIN_AVR_SDO, b);

    pgm->setpin(pgm, PIN_AVR_SCK, 1);

    /*
     * read the result bit (it is either valid from a previous falling
     * edge or it is ignored in the current context)
     */
    r = pgm->getpin(pgm, PIN_AVR_SDI);

    pgm->setpin(pgm, PIN_AVR_SCK, 0);

    rbyte |= r << i;
  }

  return rbyte;
}

static int bitbang_tpi_clk(const PROGRAMMER *pgm)  {
  unsigned char r = 0;
  pgm->setpin(pgm, PIN_AVR_SCK, 1);

  r = pgm->getpin(pgm, PIN_AVR_SDI);

  pgm->setpin(pgm, PIN_AVR_SCK, 0);

  return r;
}

void bitbang_tpi_tx(const PROGRAMMER *pgm, unsigned char byte)  {
  int i;
  unsigned char b, parity;

  /* start bit */
  pgm->setpin(pgm, PIN_AVR_SDO, 0);
  bitbang_tpi_clk(pgm);

  parity = 0;
  for (i = 0; i <= 7; i++) {
    b = (byte >> i) & 0x01;
    parity ^= b;

    /* set the data input line as desired */
    pgm->setpin(pgm, PIN_AVR_SDO, b);
    bitbang_tpi_clk(pgm);
  }
  
  /* parity bit */
  pgm->setpin(pgm, PIN_AVR_SDO, parity);
  bitbang_tpi_clk(pgm);

  /* 2 stop bits */
  pgm->setpin(pgm, PIN_AVR_SDO, 1);
  bitbang_tpi_clk(pgm);
  bitbang_tpi_clk(pgm);
}

int bitbang_tpi_rx(const PROGRAMMER *pgm)  {
  int i;
  unsigned char b, rbyte, parity;

  /* make sure pin is on for "pullup" */
  pgm->setpin(pgm, PIN_AVR_SDO, 1);

  /* wait for start bit (up to 10 bits) */
  b = 1;
  for (i = 0; i < 10; i++) {
    b = bitbang_tpi_clk(pgm);
    if (b == 0)
      break;
  }
  if (b != 0) {
    pmsg_error("start bit not received correctly\n");
    return -1;
  }

  rbyte = 0;
  parity = 0;
  for (i=0; i<=7; i++) {
    b = bitbang_tpi_clk(pgm);
    parity ^= b;

    rbyte |= b << i;
  }

  /* parity bit */
  if (bitbang_tpi_clk(pgm) != parity) {
    pmsg_error("parity bit is wrong\n");
    return -1;
  }

  /* 2 stop bits */
  b = 1;
  b &= bitbang_tpi_clk(pgm);
  b &= bitbang_tpi_clk(pgm);
  if (b != 1) {
    pmsg_error("stop bits not received correctly\n");
    return -1;
  }
  
  return rbyte;
}

int bitbang_rdy_led(const PROGRAMMER *pgm, int value) {
  pgm->setpin(pgm, PIN_LED_RDY, !value);
  return 0;
}

int bitbang_err_led(const PROGRAMMER *pgm, int value) {
  pgm->setpin(pgm, PIN_LED_ERR, !value);
  return 0;
}

int bitbang_pgm_led(const PROGRAMMER *pgm, int value) {
  pgm->setpin(pgm, PIN_LED_PGM, !value);
  return 0;
}

int bitbang_vfy_led(const PROGRAMMER *pgm, int value) {
  pgm->setpin(pgm, PIN_LED_VFY, !value);
  return 0;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
int bitbang_cmd(const PROGRAMMER *pgm, const unsigned char *cmd,
                   unsigned char *res)
{
  int i;

  for (i=0; i<4; i++) {
    res[i] = bitbang_txrx(pgm, cmd[i]);
  }

    if(verbose >= 2)
	{
        msg_notice2("bitbang_cmd(): [ ");
        for(i = 0; i < 4; i++)
            msg_notice2("%02X ", cmd[i]);
        msg_notice2("] [ ");
        for(i = 0; i < 4; i++)
		{
            msg_notice2("%02X ", res[i]);
		}
        msg_notice2("]\n");
	}

  return 0;
}

int bitbang_cmd_tpi(const PROGRAMMER *pgm, const unsigned char *cmd,
                       int cmd_len, unsigned char *res, int res_len)
{
  int i, r;

  for (i=0; i<cmd_len; i++) {
    bitbang_tpi_tx(pgm, cmd[i]);
  }

  r = 0;
  for (i=0; i<res_len; i++) {
    r = bitbang_tpi_rx(pgm);
    if (r == -1)
      break;
    res[i] = r;
  }

  if(verbose >= 2)
  {
    msg_notice2("bitbang_cmd_tpi(): [ ");
    for(i = 0; i < cmd_len; i++)
      msg_notice2("%02X ", cmd[i]);
    msg_notice2("] [ ");
    for(i = 0; i < res_len; i++)
    {
      msg_notice2("%02X ", res[i]);
    }
    msg_notice2("]\n");
  }

  if (r == -1)
    return -1;
  return 0;
}

/*
 * transmit bytes via SPI and return the results; 'cmd' and
 * 'res' must point to data buffers
 */
int bitbang_spi(const PROGRAMMER *pgm, const unsigned char *cmd,
                   unsigned char *res, int count)
{
  int i;

  pgm->setpin(pgm, PIN_LED_PGM, 0);

  for (i=0; i<count; i++) {
    res[i] = bitbang_txrx(pgm, cmd[i]);
  }

  pgm->setpin(pgm, PIN_LED_PGM, 1);

  if(verbose >= 2)
	{
        msg_notice2("bitbang_cmd(): [ ");
        for(i = 0; i < count; i++)
            msg_notice2("%02X ", cmd[i]);
        msg_notice2("] [ ");
        for(i = 0; i < count; i++)
		{
            msg_notice2("%02X ", res[i]);
		}
        msg_notice2("]\n");
	}

  return 0;
}


/*
 * issue the 'chip erase' command to the AVR device
 */
int bitbang_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4];
  unsigned char res[4];
  AVRMEM *mem;

  if (p->prog_modes & PM_TPI) {

    while (avr_tpi_poll_nvmbsy(pgm));

    /* NVMCMD <- CHIP_ERASE */
    bitbang_tpi_tx(pgm, TPI_CMD_SOUT | TPI_SIO_ADDR(TPI_IOREG_NVMCMD));
    bitbang_tpi_tx(pgm, TPI_NVMCMD_CHIP_ERASE); /* CHIP_ERASE */

    /* Set Pointer Register */
    mem = avr_locate_flash(p);
    if (mem == NULL) {
      pmsg_error("no flash memory to erase for part %s\n", p->desc);
      return -1;
    }
    bitbang_tpi_tx(pgm, TPI_CMD_SSTPR | 0);
    bitbang_tpi_tx(pgm, (mem->offset & 0xFF) | 1);  /* high byte */
    bitbang_tpi_tx(pgm, TPI_CMD_SSTPR | 1);
    bitbang_tpi_tx(pgm, (mem->offset >> 8) & 0xFF);

    /* write dummy value to start erase */
    bitbang_tpi_tx(pgm, TPI_CMD_SST);
    bitbang_tpi_tx(pgm, 0xFF);

    while (avr_tpi_poll_nvmbsy(pgm));

    return 0;
  }

  if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
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

/*
 * issue the 'program enable' command to the AVR device
 */
int bitbang_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4];
  unsigned char res[4];
  int i;

  if (p->prog_modes & PM_TPI) {
    /* enable NVM programming */
    bitbang_tpi_tx(pgm, TPI_CMD_SKEY);
    for (i = sizeof(tpi_skey) - 1; i >= 0; i--)
      bitbang_tpi_tx(pgm, tpi_skey[i]);

    /* check NVMEN bit */
    bitbang_tpi_tx(pgm, TPI_CMD_SLDCS | TPI_REG_TPISR);
    i = bitbang_tpi_rx(pgm);
    return (i != -1 && (i & TPI_REG_TPISR_NVMEN)) ? 0 : -2;
  }

  if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
    pmsg_error("program enable instruction not defined for part %s\n", p->desc);
    return -1;
  }

  memset(cmd, 0, sizeof(cmd));
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
  pgm->cmd(pgm, cmd, res);

  if (res[2] != cmd[1])
    return -2;

  return 0;
}

/*
 * initialize the AVR device and prepare it to accept commands
 */
int bitbang_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  int rc;
  int tries;
  int i;

  bitbang_calibrate_delay();

  pgm->powerup(pgm);
  usleep(20000);

  /* TPIDATA is a single line, so SDI & SDO should be connected */
  if (p->prog_modes & PM_TPI) {
    /* make sure cmd_tpi() is defined */
    if (pgm->cmd_tpi == NULL) {
      pmsg_error("%s programmer does not support TPI\n", pgm->type);
      return -1;
    }

	/* bring RESET high first */
    pgm->setpin(pgm, PIN_AVR_RESET, 1);
    usleep(128000);	/* wait t_TOUT (32-128ms) */

    /* RESET must be LOW in case the existing code is driving the TPI pins: */
    pgm->setpin(pgm, PIN_AVR_RESET, 0);

    msg_notice2("doing SDO-SDI link check\n");

    pgm->setpin(pgm, PIN_AVR_SDO, 0);
    if (pgm->getpin(pgm, PIN_AVR_SDI) != 0) {
      pmsg_error("SDO->SDI 0 failed\n");
      return -1;
    }
    pgm->setpin(pgm, PIN_AVR_SDO, 1);
    if (pgm->getpin(pgm, PIN_AVR_SDI) != 1) {
      pmsg_error("SDO->SDI 1 failed\n");
      return -1;
    }

    msg_notice2("SDO-SDI link present\n");
  }

  pgm->setpin(pgm, PIN_AVR_SCK, 0);
  pgm->setpin(pgm, PIN_AVR_RESET, 0);
  usleep(20000);

  if (p->prog_modes & PM_TPI) {
    /* keep TPIDATA high for 16 clock cycles */
    pgm->setpin(pgm, PIN_AVR_SDO, 1);
    for (i = 0; i < 16; i++)
      pgm->highpulsepin(pgm, PIN_AVR_SCK);

    /* remove extra guard timing bits */
    bitbang_tpi_tx(pgm, TPI_CMD_SSTCS | TPI_REG_TPIPCR);
    bitbang_tpi_tx(pgm, 0x7);

    /* read TPI ident reg */
    bitbang_tpi_tx(pgm, TPI_CMD_SLDCS | TPI_REG_TPIIR);
    rc = bitbang_tpi_rx(pgm);
    if (rc != 0x80) {
      pmsg_error("TPIIR not correct\n");
      return -1;
    }
  } else {
    pgm->highpulsepin(pgm, PIN_AVR_RESET);
  }

  usleep(20000); /* 20 ms XXX should be a per-chip parameter */

  /*
   * Enable programming mode.  If we are programming an AT90S1200, we
   * can only issue the command and hope it worked.  If we are using
   * one of the other chips, the chip will echo 0x53 when issuing the
   * third byte of the command.  In this case, try up to 32 times in
   * order to possibly get back into sync with the chip if we are out
   * of sync.
   */
  if (p->flags & AVRPART_IS_AT90S1200) {
    pgm->program_enable(pgm, p);
  }
  else {
    tries = 0;
    do {
      rc = pgm->program_enable(pgm, p);
      if ((rc == 0)||(rc == -1))
        break;
      pgm->highpulsepin(pgm, p->retry_pulse/*PIN_AVR_SCK*/);
      tries++;
    } while (tries < 65);

    /*
     * can't sync with the device, maybe it's not attached?
     */
    if (rc) {
      pmsg_error("AVR device not responding\n");
      return -1;
    }
  }

  return 0;
}

static int verify_pin_assigned(const PROGRAMMER *pgm, int pinfunc, char *desc) {
  if(pinfunc < 0 || pinfunc >= N_PINS) {
    pmsg_error("invalid pin function number %d\n", pinfunc);
    return -1;
  }

  if ((pgm->pinno[pinfunc] & PIN_MASK) > PIN_MAX) {
    pmsg_error("no pin has been assigned for %s\n", desc);
    return -1;
  }
  return 0;
}


/*
 * Verify all prerequisites for a bit-bang programmer are present.
 */
int bitbang_check_prerequisites(const PROGRAMMER *pgm) {

  if (verify_pin_assigned(pgm, PIN_AVR_RESET, "AVR RESET") < 0)
    return -1;
  if (verify_pin_assigned(pgm, PIN_AVR_SCK,   "AVR SCK") < 0)
    return -1;
  if (verify_pin_assigned(pgm, PIN_AVR_SDI,   "AVR SDI") < 0)
    return -1;
  if (verify_pin_assigned(pgm, PIN_AVR_SDO,   "AVR SDO") < 0)
    return -1;

  if (pgm->cmd == NULL) {
    pmsg_error("no cmd() method defined for bitbang programmer\n");
    return -1;
  }
  return 0;
}
