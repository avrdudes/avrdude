/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2002-2004 Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) 2008,2014 Joerg Wunsch
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

/*
 * avrdude interface for Atmel STK500 programmer
 *
 * Note: most commands use the "universal command" feature of the
 * programmer in a "pass through" mode, exceptions are "program
 * enable", "paged read", and "paged write".
 *
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "stk500.h"
#include "stk500_private.h"

#define STK500_XTAL 7372800U
#define MAX_SYNC_ATTEMPTS 10

static double f_to_kHz_MHz(double f, const char **unit) {
  if (f >= 1e6) {
    f /= 1e6;
    *unit = "MHz";
  } else if (f >= 1e3) {
    f /= 1000;
    *unit = "kHz";
  } else
    *unit = "Hz";
  return f;
}

static int get_decimals(double f) {
  if (f >= 1e6)
    return 6;
  if (f >= 1e3)
    return 3;
  return 0;
}

static int stk500_getparm(const PROGRAMMER *pgm, unsigned parm, unsigned *value);
static int stk500_setparm(const PROGRAMMER *pgm, unsigned parm, unsigned value);
static void stk500_print_parms1(const PROGRAMMER *pgm, const char *p, FILE *fp);


static int stk500_send(const PROGRAMMER *pgm, unsigned char *buf, size_t len) {
  return serial_send(&pgm->fd, buf, len);
}


static int stk500_recv(const PROGRAMMER *pgm, unsigned char *buf, size_t len) {
  int rv;

  rv = serial_recv(&pgm->fd, buf, len);
  if (rv < 0) {
    pmsg_error("programmer is not responding\n");
    return -1;
  }
  return 0;
}


int stk500_drain(const PROGRAMMER *pgm, int display) {
  return serial_drain(&pgm->fd, display);
}


int stk500_getsync(const PROGRAMMER *pgm) {
  unsigned char buf[32], resp[32];
  int attempt;
  int max_sync_attempts;

  buf[0] = Cmnd_STK_GET_SYNC;
  buf[1] = Sync_CRC_EOP;
  
  /*
   * First send and drain a few times to get rid of line noise 
   */
   
  stk500_send(pgm, buf, 2);
  stk500_drain(pgm, 0);
  stk500_send(pgm, buf, 2);
  stk500_drain(pgm, 0);

  if(PDATA(pgm)->retry_attempts)
    max_sync_attempts = PDATA(pgm)->retry_attempts;
  else
    max_sync_attempts = MAX_SYNC_ATTEMPTS;

  for (attempt = 0; attempt < max_sync_attempts; attempt++) {
    // Restart Arduino bootloader for every sync attempt
    if (str_eq(pgm->type, "Arduino") && attempt > 0) {
      // This code assumes a negative-logic USB to TTL serial adapter
      // Pull the RTS/DTR line low to reset AVR: it is still high from open()/last attempt
      serial_set_dtr_rts(&pgm->fd, 1);
      // Max 100 us: charging a cap longer creates a high reset spike above Vcc
      usleep(100);
      // Set the RTS/DTR line back to high, so direct connection to reset works
      serial_set_dtr_rts(&pgm->fd, 0);
      usleep(20*1000);
      stk500_drain(pgm, 0);
    }

    stk500_send(pgm, buf, 2);
    resp[0] = 0;
    if(stk500_recv(pgm, resp, 1) >= 0 && resp[0] == Resp_STK_INSYNC)
      break;

    pmsg_warning("attempt %d of %d: not in sync: resp=0x%02x\n", attempt + 1, max_sync_attempts, resp[0]);
  }
  if (attempt == max_sync_attempts) {
    stk500_drain(pgm, 0);
    return -1;
  }

  if (stk500_recv(pgm, resp, 1) < 0)
    return -1;
  if (resp[0] != Resp_STK_OK) {
    pmsg_error("cannot communicate with device: resp=0x%02x\n", resp[0]);
    return -1;
  }

  return 0;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
static int stk500_cmd(const PROGRAMMER *pgm, const unsigned char *cmd,
                      unsigned char *res)
{
  unsigned char buf[32];

  buf[0] = Cmnd_STK_UNIVERSAL;
  buf[1] = cmd[0];
  buf[2] = cmd[1];
  buf[3] = cmd[2];
  buf[4] = cmd[3];
  buf[5] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 6);

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] != Resp_STK_INSYNC) {
    pmsg_error("programmer is out of sync\n");
    return -1;
  }

  res[0] = cmd[1];
  res[1] = cmd[2];
  res[2] = cmd[3];
  if (stk500_recv(pgm, &res[3], 1) < 0)
    return -1;

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] != Resp_STK_OK) {
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[0]);
    return -1;
  }

  return 0;
}



/*
 * issue the 'chip erase' command to the AVR device
 */
static int stk500_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4];
  unsigned char res[4];

  if (pgm->cmd == NULL) {
    pmsg_error("%s programmer uses stk500_chip_erase() but does not\n", pgm->type);
    imsg_error("provide a cmd() method\n");
    return -1;
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
static int stk500_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char buf[16];
  int tries=0;

 retry:
  
  tries++;

  buf[0] = Cmnd_STK_ENTER_PROGMODE;
  buf[1] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 2);
  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      pmsg_error("cannot get into sync\n");
      return -1;
    }
    if (stk500_getsync(pgm) < 0)
      return -1;
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_OK) {
    return 0;
  }
  else if (buf[0] == Resp_STK_NODEVICE) {
    pmsg_error("no device\n");
    return -1;
  }

  if(buf[0] == Resp_STK_FAILED)
  {
      pmsg_error("unable to enter programming mode\n");
      return -1;
  }


  pmsg_error("unknown response=0x%02x\n", buf[0]);

  return -1;
}



static int stk500_set_extended_parms(const PROGRAMMER *pgm, int n,
                                     unsigned char * cmd)
{
  unsigned char buf[16];
  int tries=0;
  int i;

 retry:
  
  tries++;

  buf[0] = Cmnd_STK_SET_DEVICE_EXT;
  for (i=0; i<n; i++) {
    buf[1+i] = cmd[i];
  }
  i++;
  buf[i] = Sync_CRC_EOP;

  stk500_send(pgm, buf, i+1);
  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      pmsg_error("cannot get into sync\n");
      return -1;
    }
    if (stk500_getsync(pgm) < 0)
      return -1;
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_OK) {
    return 0;
  }
  else if (buf[0] == Resp_STK_NODEVICE) {
    pmsg_error("no device\n");
    return -1;
  }

  if(buf[0] == Resp_STK_FAILED) {
    pmsg_error("unable to set extended device programming parameters\n");
    return -1;
  }

  pmsg_error("unknown response=0x%02x\n", buf[0]);

  return -1;
}

/*
 * Crossbow MIB510 initialization and shutdown.  Use cmd = 1 to
 * initialize, cmd = 0 to close.
 */
static int mib510_isp(const PROGRAMMER *pgm, unsigned char cmd) {
  unsigned char buf[9];
  int tries = 0;

  buf[0] = 0xaa;
  buf[1] = 0x55;
  buf[2] = 0x55;
  buf[3] = 0xaa;
  buf[4] = 0x17;
  buf[5] = 0x51;
  buf[6] = 0x31;
  buf[7] = 0x13;
  buf[8] = cmd;


 retry:

  tries++;

  stk500_send(pgm, buf, 9);
  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      pmsg_error("cannot get into sync\n");
      return -1;
    }
    if (stk500_getsync(pgm) < 0)
      return -1;
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_OK) {
    return 0;
  }
  else if (buf[0] == Resp_STK_NODEVICE) {
    pmsg_error("no device\n");
    return -1;
  }

  if (buf[0] == Resp_STK_FAILED)
  {
      pmsg_error("command %d failed\n", cmd);
      return -1;
  }


  pmsg_error("unknown response=0x%02x\n", buf[0]);

  return -1;
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
static int stk500_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char buf[32];
  AVRMEM * m;
  int tries;
  unsigned maj = 0, min = 0;
  int rc;
  int n_extparms;

  if ((rc = stk500_getparm(pgm, Parm_STK_SW_MAJOR, &maj)) < 0
     || (rc = stk500_getparm(pgm, Parm_STK_SW_MINOR, &min)) < 0 ) {
    pmsg_error("cannot obtain SW version\n");
    return rc;
  }

  // MIB510 does not need extparams
  if (str_eq(pgmid, "mib510"))
    n_extparms = 0;
  else if ((maj > 1) || ((maj == 1) && (min > 10)))
    n_extparms = 4;
  else
    n_extparms = 3;

  tries = 0;

 retry:
  tries++;

  memset(buf, 0, sizeof(buf));

  /*
   * set device programming parameters
   */
  buf[0] = Cmnd_STK_SET_DEVICE;

  buf[1] = p->stk500_devcode;
  buf[2] = 0; /* device revision */

  if ((p->flags & AVRPART_SERIALOK) && (p->flags & AVRPART_PARALLELOK))
    buf[3] = 0; /* device supports parallel and serial programming */
  else
    buf[3] = 1; /* device supports parallel only */

  if (p->flags & AVRPART_PARALLELOK) {
    if (p->flags & AVRPART_PSEUDOPARALLEL) {
      buf[4] = 0; /* pseudo parallel interface */
      n_extparms = 0;
    }
    else {
      buf[4] = 1; /* full parallel interface */
    }
  }

#if 0
  pmsg_info("stk500_initialize(): n_extparms = %d\n", n_extparms);
#endif
    
  buf[5] = 1; /* polling supported - XXX need this in config file */
  buf[6] = 1; /* programming is self-timed - XXX need in config file */

  buf[7] = (m = avr_locate_lock(p))? m->size: 0;

  // Number of fuse bytes (for classic parts)
  buf[8] = 0;
  for(int fu = 0; fu < 3; fu++)
    if((m = avr_locate_fuse_by_offset(p, fu)))
      buf[8] += m->size;

  if ((m = avr_locate_flash(p))) {
    buf[9] = m->readback[0];
    buf[10] = m->readback[1];
    if (m->paged) {
      buf[13] = (m->page_size >> 8) & 0x00ff;
      buf[14] = m->page_size & 0x00ff;
    }
    buf[17] = (m->size >> 24) & 0xff;
    buf[18] = (m->size >> 16) & 0xff;
    buf[19] = (m->size >> 8) & 0xff;
    buf[20] = m->size & 0xff;
  } else {
    buf[9]  = 0xff;
    buf[10] = 0xff;
    buf[13] = 0;
    buf[14] = 0;
    buf[17] = 0;
    buf[18] = 0;
    buf[19] = 0;
    buf[20] = 0;
  }

  if ((m = avr_locate_eeprom(p))) {
    buf[11] = m->readback[0];
    buf[12] = m->readback[1];
    buf[15] = (m->size >> 8) & 0x00ff;
    buf[16] = m->size & 0x00ff;
  } else {
    buf[11] = 0xff;
    buf[12] = 0xff;
    buf[15] = 0;
    buf[16] = 0;
  }

  buf[21] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 22);
  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_NOSYNC) {
    pmsg_warning("programmer not in sync, resp=0x%02x\n", buf[0]);
    if (tries > 33)
      return -1;
    if (stk500_getsync(pgm) < 0)
      return -1;
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] != Resp_STK_OK) {
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[0]);
    return -1;
  }

  if (n_extparms) {
    if ((p->pagel == 0) || (p->bs2 == 0)) {
      pmsg_notice2("PAGEL and BS2 signals not defined in the configuration "
        "file for part %s, using dummy values\n", p->desc);
      buf[2] = 0xD7;            /* they look somehow possible, */
      buf[3] = 0xA0;            /* don't they? ;) */
    }
    else {
      buf[2] = p->pagel;
      buf[3] = p->bs2;
    }
    buf[0] = n_extparms+1;

    /*
     * m is currently pointing to eeprom memory if the part has it
     */
    if (m)
      buf[1] = m->page_size;
    else
      buf[1] = 0;


    if (n_extparms == 4) {
      if (p->reset_disposition == RESET_DEDICATED)
        buf[4] = 0;
      else
        buf[4] = 1;
    }

    rc = stk500_set_extended_parms(pgm, n_extparms+1, buf);
    if (rc) {
      pmsg_error("failed to initialise programmer\n");
      return -1;
    }
  }

  // Read or write target voltage
  if (PDATA(pgm)->vtarg_get || PDATA(pgm)->vtarg_set) {
    // Read current target voltage set value
    unsigned int vtarg_read = 0;
    if ((rc = stk500_getparm(pgm, Parm_STK_VTARGET, &vtarg_read)) < 0) {
      pmsg_error("cannot obtain V[target]\n");
      return rc;
    }
    if (PDATA(pgm)->vtarg_get)
      msg_info("Target voltage value read as %.2f V\n", (vtarg_read / 10.0));
    // Write target voltage value
    else {
      msg_info("Changing target voltage from %.2f V to %.2f V\n", (vtarg_read / 10.0), PDATA(pgm)->vtarg_data);
      if(pgm->set_vtarget(pgm, PDATA(pgm)->vtarg_data) < 0)
        return -1;
    }
  }

  // Read or write analog reference voltage
  if (PDATA(pgm)->varef_get || PDATA(pgm)->varef_set) {
    // Read current analog reference voltage
    unsigned int varef_read = 0;
    if ((rc = stk500_getparm(pgm, Parm_STK_VADJUST, &varef_read)) < 0) {
      pmsg_error("cannot obtain V[aref]\n");
      return rc;
    }
    if (PDATA(pgm)->varef_get)
      msg_info("Analog reference voltage value read as %.2f V\n", (varef_read / 10.0));
    // Write analog reference voltage
    else {
      msg_info("Changing analog reference voltage from %.2f V to %.2f V\n",
        (varef_read / 10.0), PDATA(pgm)->varef_data);
      if(pgm->set_varef(pgm, 0, PDATA(pgm)->varef_data) < 0)
        return -1;
    }
  }

  // Read or write clock generator frequency
  if (PDATA(pgm)->fosc_get || PDATA(pgm)->fosc_set) {
    // Read current target voltage set value
    unsigned int osc_pscale = 0;
    unsigned int osc_cmatch = 0;
    const char *unit_get = {"Hz"};
    double f_get = 0.0;
    if ((rc = stk500_getparm(pgm, Parm_STK_OSC_PSCALE, &osc_pscale)) < 0
      || (rc = stk500_getparm(pgm, Parm_STK_OSC_CMATCH, &osc_cmatch) < 0)) {
      pmsg_error("cannot obtain fosc values\n");
      return rc;
    }
    if(osc_pscale) {
      int prescale = 1;
      f_get = PDATA(pgm)->xtal / 2;
      switch (osc_pscale) {
        case 2: prescale = 8; break;
        case 3: prescale = 32; break;
        case 4: prescale = 64; break;
        case 5: prescale = 128; break;
        case 6: prescale = 256; break;
        case 7: prescale = 1024; break;
      }
      f_get /= prescale;
      f_get /= (osc_cmatch + 1);
      f_get = f_to_kHz_MHz(f_get, &unit_get);
    }
    if (PDATA(pgm)->fosc_get)
        msg_info("Oscillator currently set to %.3f %s\n", f_get, unit_get);
    // Write target voltage value
    else {
      const char *unit_set;
      double f_set = f_to_kHz_MHz(PDATA(pgm)->fosc_data, &unit_set);
      msg_info("Changing oscillator frequency from %.3f %s to %.3f %s\n", f_get, unit_get, f_set, unit_set);
      if(pgm->set_fosc(pgm, PDATA(pgm)->fosc_data) < 0)
        return -1;
    }
  }

  return pgm->program_enable(pgm, p);
}

static int stk500_parseextparms(const PROGRAMMER *pgm, const LISTID extparms)
 {
   LNODEID ln;
   const char *extended_param;
   int attempts;
   int rv = 0;

   for (ln = lfirst(extparms); ln; ln = lnext(ln)) {
     extended_param = ldata(ln);

    if (sscanf(extended_param, "attempts=%2d", &attempts) == 1) {
      PDATA(pgm)->retry_attempts = attempts;
      pmsg_info("setting number of retry attempts to %d\n", attempts);
      continue;
    }

    else if (str_starts(extended_param, "vtarg")) {
      if ((pgm->extra_features & HAS_VTARG_ADJ) && (str_starts(extended_param, "vtarg=")))  {
        // Set target voltage
        double vtarg_set_val = -1; // default = invlid value
        int sscanf_success = sscanf(extended_param, "vtarg=%lf", &vtarg_set_val);
        PDATA(pgm)->vtarg_data = (double)((int)(vtarg_set_val * 100 + .5)) / 100;
        if (sscanf_success < 1 || vtarg_set_val < 0) {
          pmsg_error("invalid vtarg value %s\n", extended_param);
          rv = -1;
          break;
        }
        PDATA(pgm)->vtarg_set = true;
        continue;
      } else if ((pgm->extra_features & HAS_VTARG_READ) && str_eq(extended_param, "vtarg")) {
        // Get target voltage
        PDATA(pgm)->vtarg_get = true;
        continue;
      }
    }

    else if (str_starts(extended_param, "varef")) {
      if (pgm->extra_features & HAS_VAREF_ADJ) {
        int sscanf_success = 0;
        double varef_set_val = -1;
        // Get new analog reference voltage for channel 0
        if (str_starts(extended_param, "varef=")) {
          sscanf_success = sscanf(extended_param, "varef=%lf", &varef_set_val);
          PDATA(pgm)->varef_set = true;
        }
        // Get new analog reference voltage for channel 0
        else if(str_starts(extended_param, "varef0=")) {
          sscanf_success = sscanf(extended_param, "varef0=%lf", &varef_set_val);
          PDATA(pgm)->varef_set = true;
        }
        // Get current analog reference voltage for channel 0
        else if(str_eq(extended_param, "varef") || str_eq(extended_param, "varef0")) {
          PDATA(pgm)->varef_get = true;
          continue;
        }
        // Set analog reference voltage
        if (PDATA(pgm)->varef_set) {
          PDATA(pgm)->varef_data = (double)((int)(varef_set_val * 100 + .5)) / 100;
          if (sscanf_success < 1 || varef_set_val < 0) {
            pmsg_error("invalid varef value %s\n", extended_param);
            PDATA(pgm)->varef_set = false;
            rv = -1;
            break;
          }
          continue;
        }
      }
    }

    else if (str_starts(extended_param, "fosc")) {
      if (pgm->extra_features & HAS_FOSC_ADJ) {
        // Set clock generator frequency
        if (str_starts(extended_param, "fosc=")) {
          char fosc_str[16] = {0};
          // allow spaces in fosc_str
          int sscanf_success = sscanf(extended_param, "fosc=%15[0-9.eE MmKkHhZzof]", fosc_str);
          if (sscanf_success < 1) {
            pmsg_error("invalid fosc value %s\n", extended_param);
            rv = -1;
            break;
          }
          char *endp;
          double v = strtod(fosc_str, &endp);
          if (endp == fosc_str){ // no number
            while ( *endp == ' ' ) // remove leading spaces
              ++endp;
            if (str_starts(endp, "off"))
              PDATA(pgm)->fosc_data = 0.0;
            else {
              pmsg_error("invalid fosc value %s\n", fosc_str);
              rv = -1;
              break;
            }
          }
          while ( *endp == ' ' ) // remove leading spaces before unit
            ++endp;
          if (*endp == 'm' || *endp == 'M')
            PDATA(pgm)->fosc_data =  v * 1e6;
          else if (*endp == 'k' || *endp == 'K')
            PDATA(pgm)->fosc_data =  v * 1e3;
          else if (*endp == 0 || *endp == 'h' || *endp == 'H')
            PDATA(pgm)->fosc_data =  v;
          PDATA(pgm)->fosc_set = true;
          continue;
        }
        // Get clock generator frequency
        else if(str_eq(extended_param, "fosc")) {
          PDATA(pgm)->fosc_get = true;
         continue;
        }
      }
    }

    else if (str_starts(extended_param, "xtal")) {
      // Set clock generator frequency
      if (str_starts(extended_param, "xtal=")) {
        char xtal_str[16] = {0};
        int sscanf_success = sscanf(extended_param, "xtal=%15[0-9.eE MmKkHhZz]", xtal_str);
        if (sscanf_success < 1) {
          pmsg_error("invalid xtal value %s\n", extended_param);
          rv = -1;
          break;
        }
        char *endp;
        double v = strtod(xtal_str, &endp);
        if (endp == xtal_str){
          pmsg_error("invalid xtal value %s\n", xtal_str);
          rv = -1;
          break;
        }
        while ( *endp == ' ' ) // remove leading spaces before unit
          ++endp;
        if (*endp == 'm' || *endp == 'M') // fits also e.g. "nnnnMHz"
          PDATA(pgm)->xtal = v * 1e6;
        else if (*endp == 'k' || *endp == 'K')
          PDATA(pgm)->xtal = v * 1e3;
        else if (*endp == 0 || *endp == 'h' || *endp == 'H') // "nnnn" or "nnnnHz"
          PDATA(pgm)->xtal = v;
        continue;
      }
    }

    else if (str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      msg_error("  -xattempts=<arg>      Specify no. connection retry attempts\n");
      if (pgm->extra_features & HAS_VTARG_READ) {
        msg_error("  -xvtarg               Read target supply voltage\n");
      }
      if (pgm->extra_features & HAS_VTARG_ADJ) {
        msg_error("  -xvtarg=<arg>         Set target supply voltage\n");
      }
      if (pgm->extra_features & HAS_VAREF_ADJ) {
        msg_error("  -xvaref               Read analog reference voltage\n");
        msg_error("  -xvaref=<arg>         Set analog reference voltage\n");
      }
      if (pgm->extra_features & HAS_FOSC_ADJ) {
        msg_error("  -xfosc                Read oscillator clock frequency\n");
        msg_error("  -xfosc=<arg>[M|k]|off Set oscillator clock frequency\n");
      }
      msg_error("  -xxtal=<arg>[M|k]     Set programmer xtal frequency\n");
      msg_error("  -xhelp                Show this help menu and exit\n");
      exit(0);
    }

     pmsg_error("invalid extended parameter %s\n", extended_param);
     rv = -1;
   }

   return rv;
 }

static void stk500_disable(const PROGRAMMER *pgm) {
  unsigned char buf[16];
  int tries=0;

 retry:
  
  tries++;

  buf[0] = Cmnd_STK_LEAVE_PROGMODE;
  buf[1] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 2);
  if (stk500_recv(pgm, buf, 1) < 0)
    return;
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      pmsg_error("cannot get into sync\n");
      return;
    }
    if (stk500_getsync(pgm) < 0)
      return;
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return;
  }

  if (stk500_recv(pgm, buf, 1) < 0)
    return;
  if (buf[0] == Resp_STK_OK) {
    return;
  }
  else if (buf[0] == Resp_STK_NODEVICE) {
    pmsg_error("no device\n");
    return;
  }

  pmsg_error("unknown response=0x%02x\n", buf[0]);

  return;
}

static void stk500_enable(PROGRAMMER *pgm, const AVRPART *p) {
  AVRMEM *mem;
  if(pgm->prog_modes & PM_SPM)  // For bootloaders (eg, arduino)
    if((mem = avr_locate_eeprom(p)))
      if(mem->page_size == 1)   // Change EEPROM page size from 1 to 16 to force paged r/w
        mem->page_size = 16;
  return;
}


static int stk500_open(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;
  strcpy(pgm->port, port);
  pinfo.serialinfo.baud = pgm->baudrate? pgm->baudrate: 115200;
  pinfo.serialinfo.cflags = SERIAL_8N1;
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  stk500_drain(pgm, 0);

  // MIB510 init
  if (str_eq(pgmid, "mib510") && mib510_isp(pgm, 1) != 0)
    return -1;

  if (stk500_getsync(pgm) < 0)
    return -1;

  if (pgm->bitclock != 0.0) {
    if (pgm->set_sck_period(pgm, pgm->bitclock) != 0)
      return -1;
  }

  return 0;
}


static void stk500_close(PROGRAMMER * pgm)
{
  // MIB510 close
  if (str_eq(pgmid, "mib510"))
    (void)mib510_isp(pgm, 0);

  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}


// Address is byte address; a_div == 2: send word address; a_div == 1: send byte address
static int stk500_loadaddr(const PROGRAMMER *pgm, const AVRMEM *mem, unsigned int addr, int a_div) {
  unsigned char buf[16];
  int tries;
  unsigned char ext_byte;

  addr /= a_div;

  tries = 0;
 retry:
  tries++;

  // Support large flash by sending the correct extended address byte when needed

  if(pgm->prog_modes & PM_SPM) { // Bootloaders, eg, optiboot, optiboot_dx, optiboot_x
    if(mem->size/a_div >  64*1024) { // Extended addressing needed
      ext_byte = (addr >> 16) & 0xff;
      if(ext_byte != PDATA(pgm)->ext_addr_byte) { // First addr load or a different 64k section
        buf[0] = 0x4d;          // Protocol bytes that bootloaders expect
        buf[1] = 0x00;
        buf[2] = ext_byte;
        buf[3] = 0x00;
        if(stk500_cmd(pgm, buf, buf) == 0)
          PDATA(pgm)->ext_addr_byte = ext_byte;
      }
      /*
       * Ensure next paged r/w will load ext addr again if page sits just below a 64k boundary
       *
       * Some bootloaders increment their copy of ext_addr_byte in that situation, eg, when they
       * use elpm rx,Z+ to read a byte from flash or spm Z+ to write to flash whilst they keep
       * ext_addr_byte in RAMPZ, which in turn gets incremented by Z+ at 64k page boundaries. So,
       * if an upload with automated verify finishes just below 64k, AVRDUDE still holds
       * ext_addr_byte at the current 64k segment whilst its copy in the bootloader has been
       * auto-incremented. Verifying the code from start exposes the discrepancy.
       */
      if((addr & 0xffff0000) != ((addr+mem->page_size/a_div) & 0xffff0000))
        PDATA(pgm)->ext_addr_byte = 0xff;
    }
  } else {                      // Programmer *not* for bootloaders? Original stk500v1 protocol!
    OPCODE *lext = mem->op[AVR_OP_LOAD_EXT_ADDR];

    if(lext) {
      ext_byte = (addr >> 16) & 0xff;
      if(ext_byte != PDATA(pgm)->ext_addr_byte) { // First addr load or a different 64k section
        memset(buf, 0, 4);      // Part's load_ext_addr command is typically 4d 00 ext_addr 00
        avr_set_bits(lext, buf);
        avr_set_addr(lext, buf, addr);
        if(stk500_cmd(pgm, buf, buf) == 0)
          PDATA(pgm)->ext_addr_byte = ext_byte;
      }
    }
  }

  buf[0] = Cmnd_STK_LOAD_ADDRESS;
  buf[1] = addr & 0xff;
  buf[2] = (addr >> 8) & 0xff;
  buf[3] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 4);

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      pmsg_error("cannot get into sync\n");
      return -1;
    }
    if (stk500_getsync(pgm) < 0)
      return -1;
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_OK)
    return 0;

  pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[0]);

  return -1;
}


static int set_memchr_a_div(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, int *memchrp, int *a_divp) {
  if(mem_is_in_flash(m)) {
    *memchrp = 'F';
    if(!(pgm->prog_modes & PM_SPM)) // Programmer *not* for bootloaders: original stk500v1 protocol
      *a_divp = m->op[AVR_OP_LOADPAGE_LO] || m->op[AVR_OP_READ_LO]? 2: 1;
    else if(!(p->prog_modes & (PM_UPDI | PM_PDI | PM_aWire)))
      *a_divp = 2;              // Bootloader where part is a "classic" part (eg, optiboot)
    else
      *a_divp = 1;              // Bootloader where part is Xmega or "new" families (optiboot_x, optiboot_dx)
    return 0;
  }

  if(mem_is_eeprom(m)) {
    *memchrp = 'E';
    // Word addr for bootloaders or Arduino as ISP if part is a "classic" part, byte addr otherwise
    *a_divp = ((pgm->prog_modes & PM_SPM) || str_caseeq(pgmid, "arduino_as_isp")) \
       && !(p->prog_modes & (PM_UPDI | PM_PDI))? 2: 1;
    return 0;
  }

  return -1;
}


static int stk500_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                              unsigned int page_size,
                              unsigned int addr, unsigned int n_bytes)
{
  unsigned char* buf = alloca(page_size + 16);
  int memchr;
  int a_div;
  int block_size;
  int tries;
  unsigned int n;
  unsigned int i;

  if(set_memchr_a_div(pgm, p, m, &memchr, &a_div) < 0)
    return -2;

  n = addr + n_bytes;
#if 0
  msg_debug(
    "n_bytes   = %d\n"
    "n         = %u\n"
    "a_div     = %d\n"
    "page_size = %d\n",
    n_bytes, n, a_div, page_size);
#endif

  for (; addr < n; addr += block_size) {
    // MIB510 uses fixed blocks size of 256 bytes
    if (str_eq(pgmid, "mib510")) {
      block_size = 256;
    } else {
      if (n - addr < page_size)
        block_size = n - addr;
      else
        block_size = page_size;
    }
    tries = 0;
  retry:
    tries++;
    stk500_loadaddr(pgm, m, addr, a_div);

    /* build command block and avoid multiple send commands as it leads to a crash
        of the silabs usb serial driver on mac os x */
    i = 0;
    buf[i++] = Cmnd_STK_PROG_PAGE;
    buf[i++] = (block_size >> 8) & 0xff;
    buf[i++] = block_size & 0xff;
    buf[i++] = memchr;
    memcpy(&buf[i], &m->buf[addr], block_size);
    i += block_size;
    buf[i++] = Sync_CRC_EOP;
    stk500_send( pgm, buf, i);

    if (stk500_recv(pgm, buf, 1) < 0)
      return -1;
    if (buf[0] == Resp_STK_NOSYNC) {
      if (tries > 33) {
        msg_error("\n");
        pmsg_error("cannot get into sync\n");
        return -3;
      }
      if (stk500_getsync(pgm) < 0)
        return -1;
      goto retry;
    }
    else if (buf[0] != Resp_STK_INSYNC) {
      msg_error("\n");
      pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
      return -4;
    }

    if (stk500_recv(pgm, buf, 1) < 0)
      return -1;
    if (buf[0] != Resp_STK_OK) {
      msg_error("\n");
      pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[0]);
      return -5;
    }
  }

  return n_bytes;
}

static int stk500_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                             unsigned int page_size,
                             unsigned int addr, unsigned int n_bytes)
{
  unsigned char buf[16];
  int memchr;
  int a_div;
  int tries;
  unsigned int n;
  int block_size;

  if(set_memchr_a_div(pgm, p, m, &memchr, &a_div) < 0)
    return -2;

  n = addr + n_bytes;
  for (; addr < n; addr += block_size) {
    // MIB510 uses fixed blocks size of 256 bytes
    if(str_eq(pgmid, "mib510")) {
      block_size = 256;
    } else {
      if (n - addr < page_size)
        block_size = n - addr;
      else
        block_size = page_size;
    }

    tries = 0;
  retry:
    tries++;
    stk500_loadaddr(pgm, m, addr, a_div);
    buf[0] = Cmnd_STK_READ_PAGE;
    buf[1] = (block_size >> 8) & 0xff;
    buf[2] = block_size & 0xff;
    buf[3] = memchr;
    buf[4] = Sync_CRC_EOP;
    stk500_send(pgm, buf, 5);

    if (stk500_recv(pgm, buf, 1) < 0)
      return -1;
    if (buf[0] == Resp_STK_NOSYNC) {
      if (tries > 33) {
        msg_error("\n");
        pmsg_error("cannot get into sync\n");
        return -3;
      }
      if (stk500_getsync(pgm) < 0)
        return -1;
      goto retry;
    }
    else if (buf[0] != Resp_STK_INSYNC) {
      msg_error("\n");
      pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
      return -4;
    }

    if (stk500_recv(pgm, &m->buf[addr], block_size) < 0)
      return -1;

    if (stk500_recv(pgm, buf, 1) < 0)
      return -1;

    if(str_eq(pgmid, "mib510")) {
      if (buf[0] != Resp_STK_INSYNC) {
        msg_error("\n");
        pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
        return -5;
      }
    }
    else {
      if (buf[0] != Resp_STK_OK) {
        msg_error("\n");
        pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[0]);
        return -5;
      }
    }
  }

  return n_bytes;
}


static int stk500_set_vtarget(const PROGRAMMER *pgm, double v) {
  unsigned uaref = 0;
  unsigned utarg = (unsigned)((v + 0.049) * 10);
  int rc = 0;

  if ((rc = stk500_getparm(pgm, Parm_STK_VADJUST, &uaref)) != 0) {
    pmsg_error("cannot obtain V[aref]\n");
    return rc;
  }

  if (uaref > utarg) {
    pmsg_warning("reducing V[aref] from %.1f to %.1f\n", uaref / 10.0, v);
    if ((rc = stk500_setparm(pgm, Parm_STK_VADJUST, utarg)) != 0) {
      pmsg_error("cannot set V[aref]\n");
      return rc;
    }
  }
  return stk500_setparm(pgm, Parm_STK_VTARGET, utarg);
}


static int stk500_get_vtarget(const PROGRAMMER *pgm, double *v) {
  unsigned utarg = 0;
  int rv;

  if ((rv = stk500_getparm(pgm, Parm_STK_VTARGET, &utarg)) != 0) {
    pmsg_error("cannot obtain V[target]\n");
    return rv;
  }

  *v = utarg / 10.0;
  return 0;
}


static int stk500_set_varef(const PROGRAMMER *pgm, unsigned int chan /* unused */,
                            double v)
{
  unsigned utarg = 0;
  unsigned uaref = (unsigned)((v + 0.049) * 10);
  int rc = 0;

  if ((rc = stk500_getparm(pgm, Parm_STK_VTARGET, &utarg)) != 0) {
    pmsg_error("cannot obtain V[target]\n");
    return rc;
  }

  if (uaref > utarg) {
    pmsg_error("V[aref] must not be greater than "
      "V[target] = %.1f\n", utarg/10.0);
    return -1;
  }

  if ((rc = stk500_setparm(pgm, Parm_STK_VADJUST, uaref)) < 0)
    pmsg_error("cannot set V[aref]\n");
  return rc;
}


static int stk500_get_varef(const PROGRAMMER *pgm, unsigned int chan /* unused */,
                            double *v) {
  unsigned uaref = 0;
  int rv;

  if ((rv = stk500_getparm(pgm, Parm_STK_VADJUST, &uaref)) != 0) {
    pmsg_error("cannot obtain V[aref]\n");
    return rv;
  }

  *v = uaref / 10.0;
  return 0;
}


static int stk500_set_fosc(const PROGRAMMER *pgm, double v) {
  unsigned prescale, cmatch, fosc;
  static unsigned ps[] = {
    1, 8, 32, 64, 128, 256, 1024
  };
  size_t idx;
  int rc = 0;

  prescale = cmatch = 0;
  if (v > 0.0) {
    if (v > PDATA(pgm)->xtal / 2.0) {
      const char *unit;
      if (v >= 1e6) {
        v /= 1e6;
        unit = "MHz";
      } else if (v >= 1e3) {
        v /= 1e3;
        unit = "kHz";
      } else
        unit = "Hz";
      pmsg_warning("f = %.3f %s too high, using %.3f MHz\n", v, unit, PDATA(pgm)->xtal/2e6);
      fosc = PDATA(pgm)->xtal / 2.0;
    } else
      fosc = (unsigned) v;
    
    for (idx = 0; idx < sizeof(ps) / sizeof(ps[0]); idx++) {
      if (fosc >= PDATA(pgm)->xtal / (256 * ps[idx] * 2)) {
        /* this prescaler value can handle our frequency */
        prescale = idx + 1;
        cmatch = (unsigned)(PDATA(pgm)->xtal / (2 * fosc * ps[idx])) - 1;
        break;
      }
    }
    if (idx == sizeof(ps) / sizeof(ps[0])) {
      pmsg_warning("f = %u Hz too low, using %u Hz\n", fosc, PDATA(pgm)->xtal / (256 * 1024 * 2));
      prescale = idx;
      cmatch = 255;
    }
  }
  
  if ((rc = stk500_setparm(pgm, Parm_STK_OSC_PSCALE, prescale)) != 0 ) {
    pmsg_error("cannot set Parm_STK_OSC_PSCALE\n");
    return rc;
  }

  if ((rc = stk500_setparm(pgm, Parm_STK_OSC_CMATCH, cmatch)) != 0) {
    pmsg_error("cannot set Parm_STK_OSC_CMATCH\n");
    return rc;
  }

  return 0;
}


static int stk500_get_fosc(const PROGRAMMER *pgm, double *v) {
  unsigned prescale=0, cmatch=0;
  static unsigned ps[] = {
    1, 8, 32, 64, 128, 256, 1024
  };
  int rc;

  if ((rc = stk500_getparm(pgm, Parm_STK_OSC_PSCALE, &prescale)) != 0) {
    pmsg_error("cannot get Parm_STK_OSC_PSCALE\n");
    return rc;
}

  if ((rc = stk500_getparm(pgm, Parm_STK_OSC_CMATCH, &cmatch)) != 0) {
    pmsg_error("cannot get Parm_STK_OSC_CMATCH\n");
    return rc;
  }

  *v = !prescale ? 0 : PDATA(pgm)->xtal / ((cmatch + 1) * 2 * ps[prescale - 1]);

  return 0;
}


/* This code assumes that each count of the SCK duration parameter
   represents 8/f, where f is the clock frequency of the STK500 controller
   processors (not the target).  This number comes from Atmel
   application note AVR061.  It appears that the STK500 bit bangs SCK.
   For small duration values, the actual SCK width is larger than
   expected.  As the duration value increases, the SCK width error
   diminishes. */
static int stk500_set_sck_period(const PROGRAMMER *pgm, double v) {
  int dur;
  double min, max;
  int rv = 0;

  min = 8.0 / PDATA(pgm)->xtal;
  max = 255 * min;
  dur = v / min + 0.5;

  if (v < min) {
      dur = 1;
      pmsg_warning("p = %.1f us too small, using %.1f us\n",
        v/1e-6, dur*min/1e-6);
  } else if (v > max) {
      dur = 255;
      pmsg_warning("p = %.1f us too large, using %.1f us\n",
        v/1e-6, dur*min/1e-6);
  }

  if ((rv = stk500_setparm(pgm, Parm_STK_SCK_DURATION, dur)) < 0) {
    pmsg_error("cannot set Parm_STK_SCK_DURATION\n");
    return rv;
  }
  return 0;
}


static int stk500_get_sck_period(const PROGRAMMER *pgm, double *v) {
  unsigned dur;
  int rv = 0;

  if ((rv = stk500_getparm(pgm, Parm_STK_SCK_DURATION, &dur)) < 0) {
    pmsg_error("cannot obtain Parm_STK_SCK_DURATION\n");
    return rv;
  }
  *v = dur * 8.0 / PDATA(pgm)->xtal;
  return 0;
}


static int stk500_getparm(const PROGRAMMER *pgm, unsigned parm, unsigned *value) {
  unsigned char buf[16];
  unsigned v;
  int tries = 0;

 retry:
  tries++;
  buf[0] = Cmnd_STK_GET_PARAMETER;
  buf[1] = parm;
  buf[2] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 3);

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      msg_error("\n");
      pmsg_error("cannot get into sync\n");
      return -1;
    }
    if (stk500_getsync(pgm) < 0)
      return -1;
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    msg_error("\n");
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -2;
  }

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  v = buf[0];

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_FAILED) {
    msg_error("\n");
    pmsg_error("parameter 0x%02x failed\n", v);
    return -3;
  }
  else if (buf[0] != Resp_STK_OK) {
    msg_error("\n");
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[0]);
    return -3;
  }

  *value = v;

  return 0;
}

  
static int stk500_setparm(const PROGRAMMER *pgm, unsigned parm, unsigned value) {
  unsigned char buf[16];
  int tries = 0;

 retry:
  tries++;
  buf[0] = Cmnd_STK_SET_PARAMETER;
  buf[1] = parm;
  buf[2] = value;
  buf[3] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 4);

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      msg_error("\n");
      pmsg_error("cannot get into sync\n");
      return -1;
    }
    if (stk500_getsync(pgm) < 0)
      return -1;
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    msg_error("\n");
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -2;
  }

  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_OK)
    return 0;

  parm = buf[0];                /* if not STK_OK, we've been echoed parm here */
  if (stk500_recv(pgm, buf, 1) < 0)
    return -1;
  if (buf[0] == Resp_STK_FAILED) {
    msg_error("\n");
    pmsg_error("parameter 0x%02x failed\n", parm);
    return -3;
  }
  else {
    msg_error("\n");
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[0]);
    return -3;
  }
}

  
static void stk500_display(const PROGRAMMER *pgm, const char *p) {
  unsigned maj = 0, min = 0, hdw = 0, topcard = 0;

  stk500_getparm(pgm, Parm_STK_HW_VER, &hdw);
  stk500_getparm(pgm, Parm_STK_SW_MAJOR, &maj);
  stk500_getparm(pgm, Parm_STK_SW_MINOR, &min);
  stk500_getparm(pgm, Param_STK500_TOPCARD_DETECT, &topcard);
  msg_info("%sHW Version            : %d\n", p, hdw);
  msg_info("%sFW Version            : %d.%d\n", p, maj, min);
  if (topcard < 3) {
    const char *n = "Unknown";

    switch (topcard) {
      case 1:
        n = "STK502";
        break;

      case 2:
        n = "STK501";
        break;
    }
    msg_info("%sTopcard               : %s\n", p, n);
  }
  if(!str_eq(pgm->type, "Arduino"))
    stk500_print_parms1(pgm, p, stderr);

  return;
}


static void stk500_print_parms1(const PROGRAMMER *pgm, const char *p, FILE *fp) {
  unsigned vtarget = 0, vadjust = 0;
  unsigned osc_pscale = 0, osc_cmatch = 0, sck_duration = 0;
  const char *unit;
  int decimals;

  if (pgm->extra_features & HAS_VTARG_READ) {
    stk500_getparm(pgm, Parm_STK_VTARGET, &vtarget);
    fmsg_out(fp, "%sVtarget               : %.1f V\n", p, vtarget / 10.0);
  }
  if (pgm->extra_features & HAS_VAREF_ADJ) {
    stk500_getparm(pgm, Parm_STK_VADJUST, &vadjust);
    fmsg_out(fp, "%sVaref                 : %.1f V\n", p, vadjust / 10.0);
  }
  if (pgm->extra_features & HAS_FOSC_ADJ) {
    stk500_getparm(pgm, Parm_STK_OSC_PSCALE, &osc_pscale);
    stk500_getparm(pgm, Parm_STK_OSC_CMATCH, &osc_cmatch);
    fmsg_out(fp, "%sOscillator            : ", p);
    if (osc_pscale == 0)
      fmsg_out(fp, "Off\n");
    else {
      int prescale = 1;
      double f = PDATA(pgm)->xtal / 2.0;

      switch (osc_pscale) {
        case 2: prescale = 8; break;
        case 3: prescale = 32; break;
        case 4: prescale = 64; break;
        case 5: prescale = 128; break;
        case 6: prescale = 256; break;
        case 7: prescale = 1024; break;
      }
      f /= prescale;
      f /= (osc_cmatch + 1);
      decimals = get_decimals(f);
      f = f_to_kHz_MHz(f, &unit);
      fmsg_out(fp, "%.*f %s\n", decimals, f, unit);
    }
  }

  stk500_getparm(pgm, Parm_STK_SCK_DURATION, &sck_duration);
  fmsg_out(fp, "%sSCK period            : %.1f us\n", p, sck_duration * 8.0e6 / PDATA(pgm)->xtal + 0.0499);

  double f = PDATA(pgm)->xtal;
  decimals = get_decimals(f);
  f = f_to_kHz_MHz(f, &unit);
  fmsg_out(fp, "%sXTAL frequency        : %.*f %s\n", p, decimals, f, unit);

  return;
}


static void stk500_print_parms(const PROGRAMMER *pgm, FILE *fp) {
  stk500_print_parms1(pgm, "", fp);
}

static void stk500_setup(PROGRAMMER * pgm)
{
  if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0) {
    pmsg_error("out of memory allocating private data\n");
    return;
  }
  memset(pgm->cookie, 0, sizeof(struct pdata));
  PDATA(pgm)->ext_addr_byte = 0xff;
  PDATA(pgm)->xbeeResetPin = XBEE_DEFAULT_RESET_PIN;
  // nanoSTK (Arduino Nano HW) uses 16 MHz
  if (str_starts(pgmid, "nanoSTK"))
    PDATA(pgm)->xtal = 16000000U;
  else
    PDATA(pgm)->xtal = STK500_XTAL;
}

static void stk500_teardown(PROGRAMMER * pgm)
{
  free(pgm->cookie);
  pgm->cookie = NULL;
}

const char stk500_desc[] = "Atmel STK500 Version 1.x firmware";

void stk500_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "STK500");

  /*
   * mandatory functions
   */
  pgm->initialize     = stk500_initialize;
  pgm->display        = stk500_display;
  pgm->enable         = stk500_enable;
  pgm->disable        = stk500_disable;
  pgm->program_enable = stk500_program_enable;
  pgm->chip_erase     = stk500_chip_erase;
  pgm->cmd            = stk500_cmd;
  pgm->open           = stk500_open;
  pgm->close          = stk500_close;
  pgm->read_byte      = avr_read_byte_default;
  pgm->write_byte     = avr_write_byte_default;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500_paged_write;
  pgm->paged_load     = stk500_paged_load;
  pgm->print_parms    = stk500_print_parms;
  pgm->set_sck_period = stk500_set_sck_period;
  pgm->get_sck_period = stk500_get_sck_period;
  pgm->parseextparams = stk500_parseextparms;
  pgm->setup          = stk500_setup;
  pgm->teardown       = stk500_teardown;
  pgm->page_size      = 256;

  /*
   * hardware dependent functions
   */
  if (pgm->extra_features & HAS_VTARG_ADJ)
    pgm->set_vtarget    = stk500_set_vtarget;
  if (pgm->extra_features & HAS_VTARG_READ)
    pgm->get_vtarget    = stk500_get_vtarget;
  if (pgm->extra_features & HAS_VAREF_ADJ) {
    pgm->set_varef      = stk500_set_varef;
    pgm->get_varef      = stk500_get_varef;
  }
  if (pgm->extra_features & HAS_FOSC_ADJ) {
    pgm->set_fosc       = stk500_set_fosc;
    pgm->get_fosc       = stk500_get_fosc;
  }
}
