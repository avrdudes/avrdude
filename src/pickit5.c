/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2024 MX682X
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
#include <stdint.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "pickit5.h"
#include "pickit5_lut.h"
#include "updi_constants.h"
#include "usbdevs.h"

#if defined(HAVE_USB_H)
#include <usb.h>                // Linux/Mac
#elif defined(HAVE_LUSB0_USB_H)
#include <lusb0_usb.h>          // Windows
#endif

// A USB driver is needed to talk with MacOS USB; it's unclear where to find it
// so remove the support under MacOS for the time being

#if defined(HAVE_USB_H) || defined(HAVE_LUSB0_USB_H)
#define USB_PK5_CMD_READ_EP   0x81
#define USB_PK5_CMD_WRITE_EP  0x02
#define USB_PK5_DATA_READ_EP  0x83
#define USB_PK5_DATA_WRITE_EP 0x04

#define USB_PK5_MAX_XFER    512

#define CHECK_ERROR        0x01
#define BIST_TEST          0x02
#define BIST_RESULT        0x03

#define PGM_TYPE_PK5       0x00 // Default
#define PGM_TYPE_PK4       0x01 // PICkit4
#define PGM_TYPE_SNAP      0x02 // SNAP

#define PK_OP_NONE         0x00 // Init
#define PK_OP_FOUND        0x01 // PK is connected to USB
#define PK_OP_RESPONDS     0x02 // Responds to get_fw() requests
#define PK_OP_READY        0x03 // Voltage Set, Clock Set

#define POWER_SOURCE_EXT   0x00
#define POWER_SOURCE_INT   0x01
#define POWER_SOURCE_NONE  0x02

// Private data for this programmer
struct pdata {
  unsigned char pgm_type;       // Used to skip unsupported functions
  unsigned char pk_op_mode;     // See PK_OP_ defines
  unsigned char power_source;   // 0: external / 1: from PICkit / 2: ignore check
  unsigned char hvupdi_enabled; // 0: no HV / 1: HV generation enabled
  double target_voltage;        // Voltage to supply to target

  double measured_vcc;          // This and below for print_params()
  unsigned int measured_current;
  unsigned int actual_updi_clk;

  unsigned char nvm_version;    // Used to determine the offset for SIGROW/DevID

  unsigned char devID[4];       // Last byte has the Chip Revision of the target
  unsigned char app_version[3]; // Buffer for display() sent by get_fw()
  unsigned char fw_info[16];    // Buffer for display() sent by get_fw()
  unsigned char sernum_string[20]; // Buffer for display() sent by get_fw()
  char sib_string[32];
  unsigned char txBuf[512];
  unsigned char rxBuf[512];
  SCRIPT scripts;
};

#define my (*(struct pdata *)(pgm->cookie))


static void pickit5_setup(PROGRAMMER *pgm);
static void pickit5_teardown(PROGRAMMER *pgm);
static int pickit5_parseextparms(const PROGRAMMER *pgm, const LISTID extparms);

static int pickit5_open(PROGRAMMER *pgm, const char *port);
static void pickit5_close(PROGRAMMER *pgm);

static void pickit5_disable(const PROGRAMMER *pgm);
static void pickit5_enable(PROGRAMMER *pgm, const AVRPART *p);
static void pickit5_display(const PROGRAMMER *pgm, const char *p);
static int pickit5_initialize(const PROGRAMMER *pgm, const AVRPART *p);

static int pickit5_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res);
static int pickit5_program_enable(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_program_disable(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes);
static int pickit5_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes);
static int pickit5_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char value);
static int pickit5_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char *value);
static int pickit5_read_dev_id(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_read_sib(const PROGRAMMER *pgm, const AVRPART *p, char *sib);
static int pickit5_read_chip_rev(const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev);
static int pickit5_read_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);
static int pickit5_write_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);

// UPDI-specific functions
static int pickit5_updi_write_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char value);
static int pickit5_updi_read_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char *value);

// Extra functions
static int pickit5_get_fw_info(const PROGRAMMER *pgm);
static int pickit5_set_vtarget(const PROGRAMMER *pgm, double v);
static int pickit5_get_vtarget(const PROGRAMMER *pgm, double *v);
static int pickit5_set_ptg_mode(const PROGRAMMER *pgm);
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod);
static int pickit5_write_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char value);
static int pickit5_read_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char *value);

// Internal functions
inline static void pickit5_uint32_to_array(unsigned char *buf, uint32_t num);
inline static unsigned int pickit5_array_to_uint32(unsigned char *buf);
inline static void pickit5_create_payload_header(unsigned char *buf, unsigned int type,
  unsigned int msg_len, unsigned int trans_len);
inline static void pickit5_create_script_header(unsigned char *buf, unsigned int arg_len, unsigned int script_len);
inline static int pickit5_check_ret_status(const PROGRAMMER *pgm);

static int pickit5_get_status(const PROGRAMMER *pgm, unsigned char status);
static int pickit5_send_script(const PROGRAMMER *pgm, unsigned int script_type,
  const unsigned char *script, unsigned int script_len,
  const unsigned char *param, unsigned int param_len, unsigned int payload_len);
static int pickit5_send_script_done(const PROGRAMMER *pgm, char *func);
static int pickit5_read_response(const PROGRAMMER *pgm, char *fn_name);

// Extra-USB related functions, because we need more then 2 endpoints
static int usbdev_data_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes);
static int usbdev_data_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen);

inline static void pickit5_uint32_to_array(unsigned char *buf, uint32_t num) {
  *(buf++) = (uint8_t) num;
  *(buf++) = (uint8_t) (num >> 8);
  *(buf++) = (uint8_t) (num >> 16);
  *(buf) = (uint8_t) (num >> 24);
}

inline static unsigned int pickit5_array_to_uint32(unsigned char *buf) {
  unsigned int retval = 0;

  retval |= *(buf++);
  retval |= *(buf++) << 8;
  retval |= *(buf++) << 16;
  retval |= *(buf++) << 24;
  return retval;
}

inline static void pickit5_create_payload_header(unsigned char *buf, unsigned int type,
  unsigned int msg_len, unsigned int trans_len) {
  pickit5_uint32_to_array(&buf[0], type);
  pickit5_uint32_to_array(&buf[4], 0);
  pickit5_uint32_to_array(&buf[8], msg_len);
  pickit5_uint32_to_array(&buf[12], trans_len);
}

inline static void pickit5_create_script_header(unsigned char *buf, unsigned int arg_len, unsigned int script_len) {
  pickit5_uint32_to_array(&buf[0], arg_len);
  pickit5_uint32_to_array(&buf[4], script_len);
}

static void pickit5_setup(PROGRAMMER *pgm) {
  pgm->cookie = mmt_malloc(sizeof(struct pdata));
}

static void pickit5_teardown(PROGRAMMER *pgm) {
  mmt_free(pgm->cookie);
  pgm->cookie = NULL;
}

static int pickit5_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  LNODEID ln;
  const char *extended_param;
  int rv = 0;

  for(ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if(str_starts(extended_param, "vtarg=")) {
      double voltage = -1.0;

      if(sscanf(extended_param, "vtarg=%lf", &voltage) != 1) {

        pmsg_error("invalid voltage parameter %s\n", extended_param);
        rv = -1;
        continue;
      }
      if(voltage < 0.1 && voltage > -1.0) {
        my.power_source = POWER_SOURCE_NONE;   // Voltage check disabled
        continue;
      } else if(voltage < 1.8 || voltage > 5.5) {
        pmsg_error("voltage %1.1lf V outside valid range [1.8 V, 5.5 V]\n", voltage);
        rv = -1;
        continue;
      } else {
        my.power_source = POWER_SOURCE_INT;    // PK supplies power
        my.target_voltage = voltage;
        continue;
      }
    }
    if(str_starts(extended_param, "hvupdi")) {
      my.hvupdi_enabled = 1;
      continue;
    }

    if(str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      msg_error("  -x vtarg=<dbl>  Enable power output; <dbl> must be in [1.8, 5.5] V\n");
      msg_error("  -x hvupdi       Enable high-voltage UPDI initialization\n");
      msg_error("  -x help         Show this help menu and exit\n");
      return LIBAVRDUDE_EXIT;
    }

    pmsg_error("invalid extended parameter %s\n", extended_param);
    rv = -1;
  }
  return rv;
}

// Internal functions

// Type can be CMD, UPLOAD or DOWNLOAD
#define SCR_CMD           0x0100
#define SCR_UPLOAD    0x80000102
#define SCR_DOWNLOAD 0x0C0000101

static int pickit5_send_script(const PROGRAMMER *pgm, unsigned int script_type,
  const unsigned char *script, unsigned int script_len,
  const unsigned char *param, unsigned int param_len, unsigned int payload_len) {

  if(script == NULL)
    return -1;

  unsigned int header_len = 16 + 8;     // Header info + script header
  unsigned int preamble_len = header_len + param_len;
  unsigned int message_len = preamble_len + script_len;

  if(message_len > 1023)        // Required memory will exceed buffer size, abort
    return -1;                  // 1 kB should be enough for everything

  unsigned char *buf = my.txBuf;

  pickit5_create_payload_header(&buf[0], script_type, message_len, payload_len);
  pickit5_create_script_header(&buf[16], param_len, script_len);

  if(param != NULL)
    memcpy(&buf[24], param, param_len);

  memcpy(&buf[preamble_len], script, script_len);

  return serial_send(&pgm->fd, buf, message_len);
}

static int pickit5_read_response(const PROGRAMMER *pgm, char *fn_name) {
  unsigned char *buf = my.rxBuf;

  if(serial_recv(&pgm->fd, buf, 512) < 0) {
    pmsg_error("reading from PICkit failed");
    return -1;
  }
  unsigned int status = pickit5_array_to_uint32(&buf[0]);

  if(status != 0x0D) {
    pmsg_error("unexpected response in function %s", fn_name);
    return -1;
  }

  return 0;
}

static int pickit5_send_script_done(const PROGRAMMER *pgm, char *func) {
  unsigned char script_done[16];
  unsigned int script_done_type = 0x0103;

  pickit5_create_payload_header(script_done, script_done_type, 16, 0);
  if(serial_send(&pgm->fd, script_done, 16) >= 0)
    return pickit5_read_response(pgm, func);
  pmsg_error("failed sending script done message");
  return -1;
}

static int pickit5_open(PROGRAMMER *pgm, const char *port) {
  if(!pgm->cookie)              // Sanity
    return -1;
  pmsg_debug("%s(\"%s\")\n", __func__, port);
  union pinfo pinfo;
  LNODEID usbpid;
  int rv = -1;

#if !defined(HAVE_LIBUSB)
  pmsg_error("need to be compiled with USB or HIDAPI support\n");
  return -1;
#endif

  if(!str_starts(port, "usb")) {
    pmsg_error("port names must start with usb\n");
    return -1;
  }
  unsigned int new_vid = 0, new_pid = 0;
  char *vidp, *pidp;

  /*
   * The syntax for usb devices is defined as:
   *
   * -P usb:vid:pid
   * -P usb::pid
   * -P usb:serialnumber
   * -P usb
   *
   * First we check if we have two colons.
   * Then check the filed between the two colons is empty
   * Parse VID as hex number
   * If it is empty, assume Microchip VID
   * The PID is handled similary but can not be empty
   *
   * If there are fewer than two colons nothing is changed
   */

  vidp = strchr(port, ':');
  if(vidp != NULL) {
    vidp += 1;
    pidp = strchr(vidp, ':');
    if(pidp != NULL) {
      if(vidp != pidp) {        // User specified an VID
        // First: Handle VID input
        if(sscanf(vidp, "%x", &new_vid) != 1) {
          pmsg_error("failed to parse -P VID input %s: unexpected format", vidp);
          return -1;
        }
      } else {                  // VID space empty: default to Microchip
        new_vid = USB_VENDOR_MICROCHIP;
      }

      // Now handle PID input
      if(sscanf(pidp + 1, "%x", &new_pid) != 1) {
        pmsg_error("failed to parse -P PID input %s: unexpected format", pidp+1);
        return -1;
      }

      if((new_vid != 0) && (new_pid != 0)) {
        pmsg_notice("overwriting VID:PID to %04x:%04x\n", new_vid, new_pid);
        port = "usb";           // Overwrite the string to avoid confusing the libusb
      }
    }                           // pidp == NULL means vidp could point to serial number
  }                             // vidp == NULL means just 'usb'

  // If the config entry did not specify a USB PID, insert the default one
  if(lfirst(pgm->usbpid) == NULL)
    ladd(pgm->usbpid, (void *) USB_DEVICE_PICKIT5);

  pinfo.usbinfo.vid = pgm->usbvid? pgm->usbvid: USB_VENDOR_MICROCHIP;

  // PICkit 5 does not have support for HID, so no need to support it
  serdev = &usb_serdev;
  if(new_pid != 0 && new_vid != 0) {    // In case a specific VID/PID was specified
    pinfo.usbinfo.vid = new_vid;
    pinfo.usbinfo.pid = new_pid;
    pinfo.usbinfo.flags = PINFO_FL_SILENT;
    pgm->fd.usb.max_xfer = USB_PK5_MAX_XFER;
    pgm->fd.usb.rep = USB_PK5_CMD_READ_EP;  // Command read
    pgm->fd.usb.wep = USB_PK5_CMD_WRITE_EP; // Command write
    pgm->fd.usb.eep = 0x00;
    pgm->port = port;
    rv = serial_open(port, pinfo, &pgm->fd);
  } else {                      // Otherwise walk the list of config file PIDs
    for(usbpid = lfirst(pgm->usbpid); rv < 0 && usbpid != NULL; usbpid = lnext(usbpid)) {
      pinfo.usbinfo.flags = PINFO_FL_SILENT;
      pinfo.usbinfo.pid = *(int *) (ldata(usbpid));
      pgm->fd.usb.max_xfer = USB_PK5_MAX_XFER;
      pgm->fd.usb.rep = USB_PK5_CMD_READ_EP;  // Command read
      pgm->fd.usb.wep = USB_PK5_CMD_WRITE_EP; // Command write
      pgm->fd.usb.eep = 0x00;

      pgm->port = port;
      rv = serial_open(port, pinfo, &pgm->fd);
    }
  }

  // Make USB serial number available to programmer
  if(serdev && serdev->usbsn) {
    pgm->usbsn = serdev->usbsn;
    my.pk_op_mode = PK_OP_FOUND;

    if(pinfo.usbinfo.pid == USB_DEVICE_PICKIT5)
      my.pgm_type = PGM_TYPE_PK5;
    else if(pinfo.usbinfo.pid == USB_DEVICE_PICKIT4_PIC_MODE)
      my.pgm_type = PGM_TYPE_PK4;
    else if(pinfo.usbinfo.pid == USB_DEVICE_SNAP_PIC_MODE)
      my.pgm_type = PGM_TYPE_SNAP;
  }

  return rv;
}

static void pickit5_close(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  pickit5_set_vtarget(pgm, 0.0); // Switches off PICkit voltage regulator if enabled

  serial_close(&pgm->fd);
}

static void pickit5_disable(const PROGRAMMER *pgm) {
  return;
}

static void pickit5_enable(PROGRAMMER *pgm, const AVRPART *p) {
  // Overwrite page sizes so that avrdude uses pages read/writes
  // This will reduce overhead and increase speed
  AVRMEM *mem;

  if((mem = avr_locate_sram(p)))
    mem->page_size = mem->size < 256? mem->size: 256;
  if((mem = avr_locate_eeprom(p)))
    mem->page_size = mem->size < 32? mem->size: 32;
  if((mem = avr_locate_sib(p))) {       // This is mandatory as PICkit is reading all 32 bytes at once
    mem->page_size = 32;
    mem->readsize = 32;
  }
}

static void pickit5_display(const PROGRAMMER *pgm, const char *p) {
  unsigned char *app = my.app_version;
  unsigned char *sn = my.sernum_string;

  if(pickit5_get_fw_info(pgm) < 0) {
    msg_error("failed to get firmware info\n");
    return;
  }

  msg_info("Application version   : %02x.%02x.%02x\n", app[0], app[1], app[2]);
  msg_info("Serial number         : %s\n", sn);
  my.pk_op_mode = PK_OP_RESPONDS;
}

static void pickit5_print_parms(const PROGRAMMER *pgm, FILE *fp) {
  pickit5_get_vtarget(pgm, NULL);
  fmsg_out(fp, "UPDI clock            : %u kHz\n", my.actual_updi_clk / 1000);
  fmsg_out(fp, "Target Vcc            : %1.2f V\n", my.measured_vcc);
  fmsg_out(fp, "Target current        : %3u mA\n", my.measured_current);
}

static int pickit5_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  if(!pgm->cookie)
    return -1;

  if(my.pk_op_mode < PK_OP_FOUND) {
    pmsg_error("failed to find a connected PICkit\n");
    return -1;
  }

  int rc = -1;

  if(pgm->prog_modes == PM_UPDI)
    rc = get_pickit_updi_script(&(my.scripts), p->desc);

  if(rc == -1)
    return -1;
  if(rc == -2) {
    pmsg_error("failed to match scripts to %s, aborting\n", p->desc);
    return -1;
  }

  if(my.hvupdi_enabled > 0) {
    if(p->hvupdi_variant == 0)
      pmsg_notice("high-voltage SYSCFG0 override on UPDI Pin enabled\n");
    if(p->hvupdi_variant == 2)
      pmsg_notice("high-voltage SYSCFG0 override on RST Pin enabled\n");
  }

  if(my.pk_op_mode < PK_OP_RESPONDS) {
    if(pickit5_get_fw_info(pgm) < 0)    // PK responds: we can try to enable voltage
      return -1;
    my.pk_op_mode = PK_OP_RESPONDS;
  }

  pickit5_set_ptg_mode(pgm);
  pickit5_set_vtarget(pgm, 0.0);        // Avoid the edge case when avrdude was CTRL+C'd but still provides power

  // Now we try to figure out if we have to supply power from PICkit
  double v_target;

  pickit5_get_vtarget(pgm, &v_target);
  if(v_target < 1.8) {
    if(my.power_source == POWER_SOURCE_NONE) {
      pmsg_warning("no external voltage detected but continuing anyway\n");
    } else if(my.power_source == POWER_SOURCE_INT) {
      pmsg_notice("no extenal Voltage detected; trying to supply from PICkit\n");

      if(pickit5_set_vtarget(pgm, my.target_voltage) < 0)
        return -1;              // Set requested voltage

      if(pickit5_get_vtarget(pgm, &v_target) < 0)
        return -1;              // Verify voltage

      if(v_target < my.target_voltage - 0.25   // Voltage supply is not accurate: allow some room
        || v_target > my.target_voltage + 0.15) {
        pmsg_error("target voltage out of range, aborting\n");
        return -1;
      }
    } else {
      pmsg_error("no external voltage detected, aborting; overwrite this check with -x vtarg=0\n");
      return -1;
    }
  } else {
    my.power_source = POWER_SOURCE_EXT;        // Overwrite user input
    pmsg_notice("external Voltage detected: will not supply power\n");
  }

  my.pk_op_mode = PK_OP_READY;

  // Target is powered, set the UPDI baudrate; adjust UPDICLKSEL if possible and neccessary
  if(pickit5_program_enable(pgm, p) < 0)
    return -1;

  // Get SIB so we can get the NVM Version
  if(pickit5_read_sib(pgm, p, my.sib_string) < 0) {
    pmsg_error("failed to obtain System Info Block\n");
    return -1;
  }

  if(pickit5_read_dev_id(pgm, p) < 0) {
    pmsg_error("failed to obtain device ID\n");
    return -1;
  }

  double bitclock = pgm->bitclock;
  unsigned int baud = pgm->baudrate;

  if(baud == 200000) {          // If baud unchanged
    if(bitclock > 0.0) {
      baud = (unsigned int) (1.0 / pgm->bitclock); // Bitclock in us
    }
  } else {
    if(bitclock > 0.0) {
      pmsg_error("both -b baudrate and -B bitclock given; please use only one, aborting\n");
      return -1;
    }
  }

  if(baud < 300) {              // Better be safe than sorry
    pmsg_warning("UPDI needs a higher clock for operation, increasing UPDI to 300 Hz\n");
    baud = 300;
  }
  if(baud > 225000) {
    if(v_target < 2.9) {
      pmsg_warning("UPDI needs a voltage of more than 2.9 V for a faster baudrate, limiting UPDI to 225 kHz\n");
      baud = 225000;
    } else {
      if(baud > 900000) {
        pmsg_warning("requested clock %u Hz too high, limiting UPDI to 900 kHz\n", baud);
        baud = 900000;
      }
      pickit5_set_sck_period(pgm, 1.0 / 100000);       // Start with 200 kHz
      pickit5_write_cs_reg(pgm, UPDI_ASI_CTRLA, 0x01); // Change UPDI clock to 16 MHz
      unsigned char ret_val = 0;

      pickit5_read_cs_reg(pgm, UPDI_ASI_CTRLA, &ret_val);
      if(ret_val != 0x01) {
        pmsg_warning("failed to change UPDI clock, falling back to 225 kHz\n");
        baud = 225000;
      }
    }
  }
  if(pickit5_set_sck_period(pgm, 1.0 / baud) >= 0) {
    pmsg_notice("UPDI speed set to %i kHz\n", baud / 1000);
    my.actual_updi_clk = baud;
  } else {
    pmsg_warning("failed to set UPDI speed, continuing\n");
    my.actual_updi_clk = 100000;       // Default clock?
  }

  pickit5_program_enable(pgm, p);
  return 0;
}

static int pickit5_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  return -2;
}

static int pickit5_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *enter_prog = my.scripts.EnterProgMode;
  unsigned int enter_prog_len = my.scripts.EnterProgMode_len;

  if(my.hvupdi_enabled && (my.pgm_type != PGM_TYPE_SNAP)) {   // SNAP has no HV generation
    if(p->hvupdi_variant == HV_UPDI_VARIANT_0) {        // High voltage generation on UPDI line
      enter_prog = my.scripts.EnterProgModeHvSp;
      enter_prog_len = my.scripts.EnterProgModeHvSp_len;
    } else if(p->hvupdi_variant == HV_UPDI_VARIANT_2) { // High voltage generation on RST line
      enter_prog = my.scripts.EnterProgModeHvSpRst;
      enter_prog_len = my.scripts.EnterProgModeHvSpRst_len;
    }
  }
  if(my.pk_op_mode == PK_OP_READY) {
    if(pickit5_send_script(pgm, SCR_CMD, enter_prog, enter_prog_len, NULL, 0, 0) < 0)
      return -1;

    if(pickit5_read_response(pgm, "Enter Programming Mode") < 0)
      return -1;
  }
  return 0;
}

static int pickit5_program_disable(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *enter_prog = my.scripts.ExitProgMode;
  unsigned int enter_prog_len = my.scripts.ExitProgMode_len;

  if(my.pk_op_mode == PK_OP_READY) {
    if(pickit5_send_script(pgm, SCR_CMD, enter_prog, enter_prog_len, NULL, 0, 0) < 0)
      return -1;

    if(pickit5_read_response(pgm, "Exit Programming Mode") < 0)
      return -1;
  }
  return 0;
}

static int pickit5_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);

  pickit5_program_enable(pgm, p);
  const unsigned char *chip_erase = my.scripts.EraseChip;
  unsigned int chip_erase_len = my.scripts.EraseChip_len;

  if(pickit5_send_script(pgm, SCR_CMD, chip_erase, chip_erase_len, NULL, 0, 0) >= 0) {
    if(pickit5_read_response(pgm, "Erase Chip") >= 0) {
      if(pickit5_array_to_uint32(&(my.rxBuf[16])) == 0x00) {
        pmsg_info("target successfully erased\n");
        my.pk_op_mode = PK_OP_READY;
        pickit5_program_enable(pgm, p);
        return 0;
      }
    }
  }

  pmsg_error("chip erase failed\n");
  return -1;
}

static int pickit5_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {

  return pickit5_read_array(pgm, p, mem, address, n_bytes, &mem->buf[address]);
}

static int pickit5_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {

  return pickit5_write_array(pgm, p, mem, address, n_bytes, &mem->buf[address]);
}

// Sets UPDI Frequency in kHz
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod) {
  pmsg_debug("%s()\n", __func__);
  double frq = (0.001 / sckperiod) + 0.5;       // 1ms/period = kHz; round up
  const unsigned char *set_speed = my.scripts.SetSpeed;
  int set_speed_len = my.scripts.SetSpeed_len;
  unsigned char buf[4];

  pickit5_uint32_to_array(buf, frq);
  if(pickit5_send_script(pgm, SCR_CMD, set_speed, set_speed_len, buf, 4, 0) < 0)
    return -1;
  if(pickit5_read_response(pgm, "Set UPDI Speed") < 0)
    return -1;
  return 0;
}

static int pickit5_write_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char value) {
  int rc = pickit5_write_array(pgm, p, mem, addr, 1, &value);

  if(rc < 0)
    return rc;
  return 0;
}

static int pickit5_read_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char *value) {
  int rc = pickit5_read_array(pgm, p, mem, addr, 1, value);

  if(rc < 0)
    return rc;
  return 0;
}

// UPDI Specific function providing a reduced overhead when writing a single byte
static int pickit5_updi_write_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char value) {

  if(mem->size < 1 || addr > (unsigned long) mem->size) {
    pmsg_error("address %i out of range for %s [0, %i]\n", (unsigned) addr, mem->desc, mem->size);
    return -1;
  }
  addr += mem->offset;
  pmsg_debug("%s(0x%4X, %i)\n", __func__, (unsigned) addr, value);
  // This script is based on WriteCSreg; reduces overhead by avoiding writing data EP
  const unsigned char h_len = 24;       // 16 + 8
  const unsigned char p_len = 8;
  const unsigned char s_len = 8;
  const unsigned char m_len = h_len + p_len + s_len;

  unsigned char write8_fast[] = {
    0x00, 0x01, 0x00, 0x00,     // [0]  SCR_CMD
    0x00, 0x00, 0x00, 0x00,     // [4]  always 0
    m_len, 0x00, 0x00, 0x00,    // [8]  message length = 16 + 8 + param (8) + script (8) = 40
    0x00, 0x00, 0x00, 0x00,     // [12] keep at 0 to receive the data in the "response"

    p_len, 0x00, 0x00, 0x00,    // [16] param length: 8 bytes
    s_len, 0x00, 0x00, 0x00,    // [20] length of script: 8 bytes

    0x00, 0x00, 0x00, 0x00,     // [24] param: address to write to, will be overwritten
    0x00, 0x00, 0x00, 0x00,     // [28] param: byte to write, will be overwritten

    // Script itself:
    0x91, 0x00,                 // Copy first 4 bytes of param to reg 0
    0x91, 0x01,                 // Copy second 4 bytes of param to reg 1
    0x1E, 0x06, 0x00, 0x01,     // Store to address in reg 0 the byte in reg 1
  };
  write8_fast[24] = (((unsigned char *) &addr)[0]);
  write8_fast[25] = (((unsigned char *) &addr)[1]);
  write8_fast[28] = value;

  serial_send(&pgm->fd, write8_fast, m_len);
  unsigned char *buf = my.rxBuf;

  if(serial_recv(&pgm->fd, buf, 512) >= 0) { // Read response
    if(buf[0] == 0x0D) {
      return 0;
    }
  }
  return -1;
}

// UPDI-specific function providing a reduced overhead when reading a single byte
static int pickit5_updi_read_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char *value) {
  if(mem_is_sram(mem) || mem_is_io(mem)
    || mem_is_lock(mem) || mem_is_in_fuses(mem)) {
    if(mem->size < 1 || addr > (unsigned long) mem->size) {
      pmsg_error("address %i out of range for %s [0, %i]\n", (unsigned) addr, mem->desc, mem->size);
      return -1;
    }
    addr += mem->offset;
    pmsg_debug("%s(0x%4X)\n", __func__, (unsigned int) addr);
    // This script is based on ReadSIB; reduces overhead by avoiding readind data EP
    const unsigned char h_len = 24;     // 16 + 8
    const unsigned char p_len = 4;
    const unsigned char s_len = 6;
    const unsigned char m_len = h_len + p_len + s_len;

    unsigned char read8_fast[] = {
      0x00, 0x01, 0x00, 0x00,   // [0]  SCR_CMD
      0x00, 0x00, 0x00, 0x00,   // [4]  always 0
      m_len, 0x00, 0x00, 0x00,  // [8]  message length = 16 + 8 + param (4) + script (6) = 34
      0x00, 0x00, 0x00, 0x00,   // [12] keep at 0 to receive the data in the "response"

      p_len, 0x00, 0x00, 0x00,  // [16] param length: 4 bytes
      s_len, 0x00, 0x00, 0x00,  // [20] length of script: 6 bytes

      0x00, 0x00, 0x00, 0x00,   // [24] param: address to read from, will be overwritten

      // Script itself:
      0x91, 0x00,               // Copy first 4 bytes of param to reg 0
      0x1E, 0x03, 0x00,         // Load byte from address in reg 0
      0x9F                      // Send data from 0x1E to "response"
    };
    read8_fast[24] = (((unsigned char *) &addr)[0]);
    read8_fast[25] = (((unsigned char *) &addr)[1]);

    serial_send(&pgm->fd, read8_fast, m_len);
    unsigned char *buf = my.rxBuf;

    if(serial_recv(&pgm->fd, buf, 512) >= 0) {  // Read response
      if(buf[0] == 0x0D) {
        if(buf[20] == 0x01) {
          *value = buf[24];
          return 0;
        }
      }
    }
    return -1;
  } else {                      // Fall back to standard function
    int rc = pickit5_read_array(pgm, p, mem, addr, 1, value);

    if(rc < 0)
      return rc;
    return 0;
  }
}

// Return numbers of byte written
static int pickit5_write_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s(%s, 0x%04x, %i)", __func__, mem->desc, (unsigned int) addr, len);

  if(len > mem->size || mem->size < 1) {
    pmsg_error("cannot write to %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  }
  if(addr >= (unsigned long) mem->size) {
    pmsg_error("cannot write to %s %s as address 0x%04lx outside range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size - 1);
    return -1;
  }

  const unsigned char *write_bytes = NULL;
  unsigned int write_bytes_len = 0;

  if((mem_is_in_flash(mem) && (len == mem->page_size))) {
    write_bytes = my.scripts.WriteProgmem;
    write_bytes_len = my.scripts.WriteProgmem_len;
  } else if(mem_is_eeprom(mem)) {
    write_bytes = my.scripts.WriteDataEEmem;
    write_bytes_len = my.scripts.WriteDataEEmem_len;
  } else if(mem_is_user_type(mem)) {
    write_bytes = my.scripts.WriteIDmem;
    write_bytes_len = my.scripts.WriteIDmem_len;
  } else if(mem_is_in_fuses(mem)) {
    write_bytes = my.scripts.WriteConfigmem;
    write_bytes_len = my.scripts.WriteConfigmem_len;
  } else if(!mem_is_readonly(mem)) { // SRAM, IO, LOCK
    if((len == 1) && (pgm->prog_modes == PM_UPDI)) {
      return pickit5_updi_write_byte(pgm, p, mem, addr, value[0]);
    }
    write_bytes = my.scripts.WriteMem8;
    write_bytes_len = my.scripts.WriteMem8_len;
  } else {
    pmsg_error("unsupported memory %s\n", mem->desc);
    return -2;
  }
  addr += mem->offset;

  unsigned char buf[8];

  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);

  if(pickit5_send_script(pgm, SCR_DOWNLOAD, write_bytes, write_bytes_len, buf, 8, len) < 0) {
    pmsg_error("sending script failed\n");
    return -1;
  }

  if(pickit5_read_response(pgm, "Write Bytes") < 0) {
    pmsg_error("reading script response failed\n");
    return -1;
  }
  if(usbdev_data_send(&pgm->fd, value, len) < 0) {
    pmsg_error("failed when sending data\n");
    return -1;
  }
  if(pickit5_get_status(pgm, CHECK_ERROR) < 0) {
    pmsg_error("error check failed\n");
    return -1;
  }
  if(pickit5_send_script_done(pgm, "Write Bytes") < 0) {
    pmsg_error("sending script done message failed\n");
    return -1;
  }
  return len;
}

// Return numbers of byte read
static int pickit5_read_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s(%u, %u)\n", __func__, (unsigned) addr, len);

  if(len > mem->size || mem->size < 1) {
    pmsg_error("cannot read from %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  }
  if(addr >= (unsigned long) mem->size) {
    pmsg_error("cannot read from %s %s as address 0x%04lx outside range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size - 1);
    return -1;
  }

  const unsigned char *read_bytes = NULL;
  unsigned int read_bytes_len = 0;

  if(mem_is_in_flash(mem)) {
    read_bytes = my.scripts.ReadProgmem;
    read_bytes_len = my.scripts.ReadProgmem_len;
  } else if(mem_is_eeprom(mem)) {
    read_bytes = my.scripts.ReadDataEEmem;
    read_bytes_len = my.scripts.ReadDataEEmem_len;
  } else if(mem_is_user_type(mem)) {
    read_bytes = my.scripts.ReadIDmem;
    read_bytes_len = my.scripts.ReadIDmem_len;
  } else if(mem_is_sib(mem)) {
    if(len == 1) {
      *value = my.sib_string[addr];
      return 0;
    } else if(len == 32) {
      memcpy(value, my.sib_string, 32);
      return 32;
    }
    return -1;
  } else if(mem_is_signature(mem)) {
    if(len == 1) {
      *value = my.devID[addr];
      return 0;
    }
    return -1;
  } else if(mem_is_in_sigrow(mem) || mem_is_user_type(mem) || mem_is_a_fuse(mem)) {
    read_bytes = my.scripts.ReadConfigmem;
    read_bytes_len = my.scripts.ReadConfigmem_len;
  } else if(!mem_is_readonly(mem)) { // SRAM, IO, LOCK, USERROW
    if((len == 1) && (pgm->prog_modes == PM_UPDI)) {
      return pickit5_updi_read_byte(pgm, p, mem, addr, value);
    }
    read_bytes = my.scripts.ReadMem8;
    read_bytes_len = my.scripts.ReadMem8_len;
  } else {
    pmsg_error("unsupported memory %s\n", mem->desc);
    return -2;
  }

  addr += mem->offset;
  unsigned char buf[8];

  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);

  if(pickit5_send_script(pgm, SCR_UPLOAD, read_bytes, read_bytes_len, buf, 8, len) < 0) {
    pmsg_error("sending script failed\n");
    return -1;
  }
  if(pickit5_read_response(pgm, "Read Bytes") < 0) {
    pmsg_error("unexpected read response\n");
    return -1;
  }
  if(usbdev_data_recv(&pgm->fd, value, len) < 0) {
    pmsg_error("reading data memory failed\n");
    return -1;
  }
  if(pickit5_send_script_done(pgm, "Read Bytes") < 0) {
    pmsg_error("sending script done message failed\n");
    return -1;
  }
  return len;
}

static int pickit5_read_dev_id(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *read_id = my.scripts.GetDeviceID; // Defaults
  unsigned int read_id_len = my.scripts.GetDeviceID_len;

  if(my.nvm_version >= '0' && my.nvm_version <= '9') {
    read_id = get_devid_script_by_nvm_ver(my.nvm_version); // Only address changes, not length
  }

  if(pickit5_send_script(pgm, SCR_CMD, read_id, read_id_len, NULL, 0, 0) < 0)
    return -1;
  if(pickit5_read_response(pgm, "Read Device ID") >= 0) {
    if(my.rxBuf[0] == 0x0D) {
      if(my.rxBuf[20] == 0x04) {
        memcpy(my.devID, &my.rxBuf[24], 4);
        return 0;
      } else {
        if(my.hvupdi_enabled && p->hvupdi_variant == HV_UPDI_VARIANT_2) {
          pmsg_info("failed to get DeviceID with activated HV Pulse on RST\n");
          pmsg_info("if the wiring is correct, try connecting a 16 V, 1 uF cap between RST and GND\n");
        }
      }
    }
  }
  return -1;
}

static int pickit5_read_sib(const PROGRAMMER *pgm, const AVRPART *p, char *sib) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *read_sib = my.scripts.ReadSIB;
  unsigned int read_sib_len = my.scripts.ReadSIB_len;

  if(pickit5_send_script(pgm, SCR_CMD, read_sib, read_sib_len, NULL, 0, 0) < 0)
    return -1;
  if(pickit5_read_response(pgm, "Read SIB") < 0)
    return -1;
  unsigned int ret_len = pickit5_array_to_uint32(&(my.rxBuf[20]));

  if(ret_len == 32) {
    memcpy(sib, &my.rxBuf[24], 32);
    sib[31] = 0x00;             // Known zero-terminator
    my.nvm_version = sib[10];
    return 0;
  }
  my.nvm_version = 0xFF;
  return -1;
}

static int pickit5_read_chip_rev(const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev) {
  pmsg_debug("%s()\n", __func__);
  *chip_rev = my.devID[3];
  return 0;
}

static int pickit5_write_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char value) {
  pmsg_debug("%s(%u, %i)", __func__, addr, value);
  const unsigned char *write_cs = my.scripts.WriteCSreg;
  unsigned int write_cs_len = my.scripts.WriteCSreg_len;

  if(addr > 0x0C) {
    pmsg_error("CS reg %i out of range [0x00, 0x0C], addr\n", addr);
    return -1;
  }

  unsigned char buf[2];

  buf[0] = addr;
  buf[1] = value;

  if(pickit5_send_script(pgm, SCR_CMD, write_cs, write_cs_len, buf, 2, 0) < 0) {
    pmsg_error("sending script failed\n");
    return -1;
  }

  if(pickit5_read_response(pgm, "Write CS reg") < 0) {
    pmsg_error("reading script response failed\n");
    return -1;
  }
  return 1;
}

static int pickit5_read_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char *value) {
  pmsg_debug("%s(%i)\n", __func__, addr);
  const unsigned char *read_cs = my.scripts.ReadCSreg;
  unsigned int read_cs_len = my.scripts.ReadCSreg_len;

  if(addr > 0x0C) {
    pmsg_error("CS reg %i out of range [0x00, 0x0C], addr\n", addr);
    return -1;
  }

  unsigned char buf[1];

  buf[0] = addr;

  if(pickit5_send_script(pgm, SCR_UPLOAD, read_cs, read_cs_len, buf, 1, 1) < 0) {
    pmsg_error("sending script failed\n");
    return -1;
  }
  if(pickit5_read_response(pgm, "Read CS") < 0) {
    pmsg_error("unexpected read response\n");
    return -1;
  }
  if(usbdev_data_recv(&pgm->fd, value, 1) < 0) {
    pmsg_error("reading CS memory failed\n");
    return -1;
  }
  if(pickit5_send_script_done(pgm, "Read CS") < 0) {
    pmsg_error("sending script done message failed\n");
    return -1;
  }
  return 0;
}

static int pickit5_get_fw_info(const PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  unsigned char *buf = my.rxBuf;
  const unsigned char get_fw[] = { 0xE1 };

  if(serial_send(&pgm->fd, get_fw, 1) < 0) {
    pmsg_error("sending command via serial_send() failed\n");
    return -1;
  }

  if(serial_recv(&pgm->fd, buf, 512) < 0) {
    pmsg_error("receiving FW response failed\n");
    return -1;
  }

  if(buf[0] != 0xE1) {
    pmsg_error("unexpected device response for get firmware info command\n");
    return -1;
  }

  memcpy(my.app_version, &(my.rxBuf[3]), 3);
  memcpy(my.fw_info, &(my.rxBuf[7]), 16);
  memcpy(my.sernum_string, &(my.rxBuf[32]), 20);
  my.sernum_string[19] = 0;    // Known zero-terminator
  return 0;
}

static int pickit5_set_vtarget(const PROGRAMMER *pgm, double v) {
  if(my.pgm_type >= PGM_TYPE_SNAP)  // SNAP can't supply power, ignore
    return 0;

  unsigned char set_vtarget[] = {
    0x40,
    0x00, 0x00, 0x00, 0x00,     // Vdd
    0x00, 0x00, 0x00, 0x00,     // Vpp
    0x00, 0x00, 0x00, 0x00,     // Vpp_op
    0x42, 0x43,
  };
  unsigned char power_source[] = {
    0x46, 0x00, 0x00, 0x00, 0x00,
  };
  unsigned char disable_power[] = {
    0x44
  };

  if(v < 1.0) {                 // Anything below 1 V equals disabling Power
    pmsg_debug("%s(disable)\n", __func__);
    if(pickit5_send_script(pgm, SCR_CMD, power_source, 5, NULL, 0, 0) < 0)
      return -1;
    if(pickit5_read_response(pgm, "Select external power source") < 0)
      return -1;

    if(pickit5_send_script(pgm, SCR_CMD, disable_power, 1, NULL, 0, 0) < 0)
      return -1;
    if(pickit5_read_response(pgm, "Disabling Power") < 0)
      return -1;
    usleep(50000);              // There might be some caps, let them discharge
  } else {
    pmsg_debug("%s(%1.2f V)\n", __func__, v);
    power_source[1] = 0x01;
    if(pickit5_send_script(pgm, SCR_CMD, power_source, 5, NULL, 0, 0) < 0)
      return -1;
    if(pickit5_read_response(pgm, "Select internal power source") < 0)
      return -1;

    int vtarg = (int) (v * 1000.0);

    pickit5_uint32_to_array(&set_vtarget[1], vtarg);
    pickit5_uint32_to_array(&set_vtarget[5], vtarg);
    pickit5_uint32_to_array(&set_vtarget[9], vtarg);

    if(pickit5_send_script(pgm, SCR_CMD, set_vtarget, 15, NULL, 0, 0) < 0)
      return -1;

    if(pickit5_read_response(pgm, "set_vtarget") < 0)
      return -1;
  }
  return 0;
}

static int pickit5_get_vtarget(const PROGRAMMER *pgm, double *v) {
  const unsigned char get_vtarget[] = { 0x47, };
  unsigned char *buf = my.rxBuf;

  pmsg_debug("%s()\n", __func__);

  if(pickit5_send_script(pgm, SCR_CMD, get_vtarget, 1, NULL, 0, 0) < 0)
    return -1;

  if(pickit5_read_response(pgm, "get_vtarget") < 0)
    return -1;

  // 24 - internal Vdd [mV]
  // 28 - target Vdd [mV]
  // 48 - Vdd Current Sense [mA]
  my.measured_vcc = pickit5_array_to_uint32(&buf[28]) / 1000.0;
  my.measured_current = pickit5_array_to_uint32(&buf[48]);

  pmsg_notice("target Vdd: %1.2f V, target current: %u mA\n", my.measured_vcc, my.measured_current);

  if(v != NULL)
    *v = my.measured_vcc;
  return 0;
}

static int pickit5_set_ptg_mode(const PROGRAMMER *pgm) {
  if(my.pgm_type >= PGM_TYPE_SNAP)  // Don't bother if Programmer doesn't support PTG
    return 0;                       // Side note: Bitmask would be probably better in the future

  unsigned char ptg_mode[] = {
    0x5E, 0x00, 0x00, 0x00, 0x00,
  };
  unsigned char buf[8];

  pmsg_debug("%s()\n", __func__);

  if(pickit5_send_script(pgm, SCR_UPLOAD, ptg_mode, 5, NULL, 0, 4) < 0)
    return -1;
  if(pickit5_read_response(pgm, "Set PTG mode") < 0)
    return -1;

  if(usbdev_data_recv(&pgm->fd, buf, 4) < 0)
    return -1;
  if(pickit5_send_script_done(pgm, "Set PTG Mode") < 0)
    return -1;
  return 0;
}

static int pickit5_get_status(const PROGRAMMER *pgm, unsigned char status) {
  unsigned char *buf = my.txBuf;
  const unsigned int type = 0x0105;
  unsigned int key_len = 0;

  if(CHECK_ERROR == status) {
    key_len = strlen("ERROR_STATUS_KEY") + 1;
    memcpy(&buf[16], "ERROR_STATUS_KEY", key_len);
  } else if(BIST_TEST == status) {
    key_len = strlen("BIST Tested") + 1;
    memcpy(&buf[16], "BIST Tested", key_len);
  } else if(BIST_RESULT == status) {
    key_len = strlen("BIST Results") + 1;
    memcpy(&buf[16], "BIST Results", key_len);
  }
  if(0 == key_len) {
    pmsg_error("unknown key type %d passed to %s()", status, __func__);
    return -1;
  }
  unsigned int msg_len = 16 + key_len;

  pickit5_create_payload_header(buf, type, msg_len, 0);
  serial_send(&pgm->fd, buf, msg_len);
  serial_recv(&pgm->fd, my.rxBuf, 512);
  if(pickit5_check_ret_status(pgm) < 0) {
    return -1;
  }
  unsigned int status_len = pickit5_array_to_uint32(&(my.rxBuf[8]));

  if(status_len > 64)
    status_len = 64;
  my.rxBuf[16 + status_len] = 0x00;    // Known zero-terminator
  if(str_starts((const char *) &(my.rxBuf[16]), "NONE") == 0) {
    pmsg_error("PICkit error status report: %s", buf);
    return -1;
  }
  return 0;
}

inline static int pickit5_check_ret_status(const PROGRAMMER *pgm) {
  unsigned char ret = my.rxBuf[0];

  if(0x0D != ret) {
    pmsg_error("PICkit5 bad response %i", ret);
    return -1;
  }
  return 0;
}

void pickit5_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "pickit5");

  // Mandatory functions
  pgm->initialize = pickit5_initialize;
  pgm->parseextparams = pickit5_parseextparms;
  pgm->display = pickit5_display;
  pgm->enable = pickit5_enable;
  pgm->disable = pickit5_disable;
  pgm->program_enable = pickit5_program_enable;
  pgm->chip_erase = pickit5_chip_erase;
  pgm->cmd = pickit5_cmd;
  pgm->open = pickit5_open;
  pgm->close = pickit5_close;
  pgm->write_byte = pickit5_write_byte;
  pgm->read_byte = pickit5_read_byte;

  // Optional functions
  pgm->paged_write = pickit5_paged_write;
  pgm->paged_load = pickit5_paged_load;
  pgm->setup = pickit5_setup;
  pgm->teardown = pickit5_teardown;
  pgm->set_sck_period = pickit5_set_sck_period;
  pgm->end_programming = pickit5_program_disable;
  pgm->read_sib = pickit5_read_sib;
  pgm->read_chip_rev = pickit5_read_chip_rev;
  pgm->set_vtarget = pickit5_set_vtarget;
  pgm->get_vtarget = pickit5_get_vtarget;
  pgm->print_parms = pickit5_print_parms;

}

#if defined(HAVE_USB_H)
static int usb_fill_buf(const union filedescriptor *fd, int maxsize, int ep, int use_interrupt_xfer) {
  int rv = (use_interrupt_xfer? usb_interrupt_read: usb_bulk_read)(fd->usb.handle, ep, cx->usb_buf,
    maxsize, 10000);

  if(rv < 0) {
    pmsg_notice2("%s(): usb_%s_read() error: %s\n", __func__,
      use_interrupt_xfer? "interrupt": "bulk", usb_strerror());
    return -1;
  }

  cx->usb_buflen = rv;
  cx->usb_bufptr = 0;

  return 0;
}

static int usbdev_data_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes) {
  int i, amnt;
  unsigned char *p = buf;

  if(fd->usb.handle == NULL)
    return -1;

  for(i = 0; nbytes > 0;) {
    if(cx->usb_buflen <= cx->usb_bufptr) {
      if(usb_fill_buf(fd, fd->usb.max_xfer, USB_PK5_DATA_READ_EP, fd->usb.use_interrupt_xfer) < 0)
        return -1;
    }
    amnt = cx->usb_buflen - cx->usb_bufptr > (int) nbytes? (int) nbytes: cx->usb_buflen - cx->usb_bufptr;
    memcpy(buf + i, cx->usb_buf + cx->usb_bufptr, amnt);
    cx->usb_bufptr += amnt;
    nbytes -= amnt;
    i += amnt;
  }

  if(verbose > 4)
    trace_buffer(__func__, p, i);

  return 0;
}

static int usbdev_data_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen) {
  int rv;
  int i = mlen;
  const unsigned char *p = bp;
  int tx_size;

  if(fd->usb.handle == NULL)
    return -1;

  /*
   * Split the frame into multiple packets.  It's important to make sure we
   * finish with a short packet, or else the device won't know the frame is
   * finished.  For example, if we need to send 64 bytes, we must send a packet
   * of length 64 followed by a packet of length 0.
   */
  do {
    tx_size = ((int) mlen < fd->usb.max_xfer)? (int) mlen: fd->usb.max_xfer;
    if(fd->usb.use_interrupt_xfer)
      rv = usb_interrupt_write(fd->usb.handle, USB_PK5_DATA_WRITE_EP, (char *) bp, tx_size, 10000);
    else
      rv = usb_bulk_write(fd->usb.handle, USB_PK5_DATA_WRITE_EP, (char *) bp, tx_size, 10000);
    if(rv != tx_size) {
      pmsg_error("wrote %d out of %d bytes, err = %s\n", rv, tx_size, usb_strerror());
      return -1;
    }
    bp += tx_size;
    mlen -= tx_size;
  } while(mlen > 0);

  if(verbose > 3)
    trace_buffer(__func__, p, i);
  return 0;
}
#else

/*
 * Allow compiling without throwing dozen errors
 * We need libusb so we can access specific Endpoints
 * This does not seem to be possible with usbhid
 */

static int usbdev_data_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes) {
  return -1;
}

static int usbdev_data_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen) {
  return -1;
}
#endif

#else                           // defined(HAVE_LIBUSB) || defined(HAVE_LIBUSB_1_0)
static int pickit5_nousb_open(PROGRAMMER *pgm, const char *name);

static int pickit5_nousb_open(PROGRAMMER *pgm, const char *name) {
  pmsg_error("no usb support; please compile again with libusb installed\n");

  return -1;
}

void pickit5_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "pickit5");

  pgm->open = pickit5_nousb_open;
}
#endif                          // defined(HAVE_USB_H) || defined(HAVE_LUSB0_USB_H)

const char pickit5_desc[] = "Microchip's PICkit 5 Programmer/Debugger";
