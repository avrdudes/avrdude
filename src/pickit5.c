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
 */

/* $Id$ */


#include <ac_cfg.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#if defined(HAVE_USB_H)
#  include <usb.h>
#elif defined(HAVE_LUSB0_USB_H)
#  include <lusb0_usb.h>
#else
#  error "libusb needs either <usb.h> or <lusb0_usb.h>"
#endif

#include "avrdude.h"
#include "libavrdude.h"

#include "pickit5.h"
#include "scripts_lut.h"
#include "usbdevs.h"



#if defined(HAVE_LIBUSB) || defined(HAVE_LIBUSB_1_0)

#ifdef HAVE_LIBUSB_1_0
# define USE_LIBUSB_1_0
#endif

#if defined(USE_LIBUSB_1_0)
# if defined(HAVE_LIBUSB_1_0_LIBUSB_H)
#  include <libusb-1.0/libusb.h>
# else
#  include <libusb.h>
# endif
#else
# if defined(HAVE_USB_H)
#  include <usb.h>
# elif defined(HAVE_LUSB0_USB_H)
#  include <lusb0_usb.h>
# else
#  error "libusb needs either <usb.h> or <lusb0_usb.h>"
# endif
#endif


#define USB_PK5_CMD_READ_EP   0x81
#define USB_PK5_CMD_WRITE_EP  0x02
#define USB_PK5_DATA_READ_EP  0x83
#define USB_PK5_DATA_WRITE_EP 0x04

#define USB_PK5_MAX_XFER      512


// Private data for this programmer
struct pdata {
#ifdef USE_LIBUSB_1_0
  libusb_device_handle *usbhandle;
#else
  usb_dev_handle *usbhandle;
#endif
  unsigned char pk_op_mode;     // 0 - init, 1: pk found, 2: pk operating, 3: pk programming
  unsigned char power_source;   // external / from PICkit / ignore check
  unsigned int  target_voltage; // voltage to supply to target
  unsigned int capabilities;
  unsigned char txBuf[1024];
  unsigned char rxBuf[1024];
  struct avr_script_lut* scripts;

#ifdef USE_LIBUSB_1_0
  libusb_context *ctx;
  char msg[30];                 // Used in errstr()
#endif
  int USB_init;                 // Used in both usbOpenDevice() variants
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))


/* Prototypes */
// interface - management
static void pickit5_setup(PROGRAMMER *pgm);
static void pickit5_teardown(PROGRAMMER *pgm);
static int pickit5_parseextparms(const PROGRAMMER *pgm, const LISTID extparms);

// interface - prog.
static int pickit5_open(PROGRAMMER *pgm, const char *port);
static void pickit5_close(PROGRAMMER *pgm);
// dummy functions
static void pickit5_disable(const PROGRAMMER *pgm);
static void pickit5_enable(PROGRAMMER *pgm, const AVRPART *p);
static void pickit5_display(const PROGRAMMER *pgm, const char *p);
// universal functions
static int pickit5_initialize(const PROGRAMMER *pgm, const AVRPART *p);
// UPDI specific functions
static int pickit5_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res);
static int pickit5_program_enable(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_program_disable(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                 unsigned int page_size,
                                 unsigned int addr, unsigned int n_bytes);
static int pickit5_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                  unsigned int page_size,
                                  unsigned int addr, unsigned int n_bytes);
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod);
static int pickit5_write_byte     (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned long addr, unsigned char value);
static int pickit5_read_byte (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned long addr, unsigned char *value);
static int pickit5_read_sig_bytes (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m);
static int pickit5_read_sib (const PROGRAMMER *pgm, const AVRPART *p, char *sib);
static int pickit5_read_chip_rev  (const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev);
// Extra Functions
static int pickit5_get_fw_info(const PROGRAMMER *pgm);
static int pickit5_set_vtarget (const PROGRAMMER *pgm, double v);
static int pickit5_get_vtarget (const PROGRAMMER *pgm, double *v);
static int pickit5_set_ptg_mode(const PROGRAMMER *pgm);


// Internal Functions
inline static void pickit5_uint32_to_array(unsigned char* buf, uint32_t num);
inline static unsigned int pickit5_array_to_uint32(unsigned char* buf);
static int pickit5_transfer_cmd(const PROGRAMMER *pgm, const char *op_name,
			        const unsigned char *cmd, unsigned char cmd_len,
              unsigned char *receive);
static int pickit5_send_script(const PROGRAMMER *pgm, unsigned int script_type,
                        const unsigned char* script, unsigned int script_len,
                        const unsigned char* param,  unsigned int param_len,
                        unsigned int payload_len);
static int pickit5_send_script_done(const PROGRAMMER *pgm);
static int pickit5_read_response(const PROGRAMMER *pgm, char* fn_name);

// Extra USB related Functions, because we need more then 2 Endpoints
static int usbdev_data_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes);
static int usbdev_data_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen);


inline static void pickit5_uint32_to_array(unsigned char* buf, uint32_t num) {
  *(buf++) = (uint8_t)  num;
  *(buf++) = (uint8_t) (num >> 8);
  *(buf++) = (uint8_t) (num >> 16);
  *(buf)   = (uint8_t) (num >> 24);
}

inline static unsigned int pickit5_array_to_uint32(unsigned char* buf) {
  unsigned int retval = 0;
  retval |= *(buf++);
  retval |= *(buf++) << 8;
  retval |= *(buf++) << 16;
  retval |= *(buf++) << 24;
  return retval;
}

/* Interface - management */
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

  for (ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if (str_starts(extended_param, "voltage=")) {
      int voltage = -1;
      if (sscanf(extended_param, "voltage=%i", &voltage) != 1) {

        pmsg_error("invalid voltage paramter '%s'\n", extended_param);
        rv = -1;
        continue;
      }
      if (voltage == 0) {
        PDATA(pgm)->power_source = 2; // Voltage check disabled
      } else if (voltage < 1800 || voltage > 5500) {
        pmsg_error("Voltage %imV outside of valid range [1800~5500]\n", voltage);
        rv = -1;
        continue;
      }
      PDATA(pgm)->power_source = 1; // PK supplies power
      PDATA(pgm)->target_voltage = voltage;
      continue;
    }

    if (str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      msg_error("  -xvoltage=<arg> Enables power output. <arg> must be in range 1800~5500[mV]\n");
      msg_error("  -xhelp            Show this help menu and exit\n");
      return LIBAVRDUDE_EXIT;;
    }

    pmsg_error("invalid extended parameter '%s'\n", extended_param);
    rv = -1;
  }
  return rv;
}


/* Internal functions */


static int pickit5_transfer_cmd(const PROGRAMMER *pgm, const char *op_name,
			        const unsigned char *cmd, unsigned char cmd_len,
              unsigned char *receive) {

  if (PDATA(pgm) == NULL) {
    return -1;
  }

  if (serial_send(&pgm->fd, cmd, cmd_len) < 0) {
    pmsg_info("Failed to send command\n");
    return -1;
  }

  // PICkit didn't like receive transfers smaller 1024
  if (serial_recv(&pgm->fd, receive, 1024) < 0) {
    pmsg_info("Failed to receive %s response\n", op_name);
    return -1;
  }

  return 0;
}

static int pickit5_get_fw_info(const PROGRAMMER *pgm) {
  if (PDATA(pgm) == NULL)
    return -1;
  unsigned char* buf = PDATA(pgm)->rxBuf;
  const unsigned char get_fw = 0xE1;

  if (pickit5_transfer_cmd(pgm, "FW", &get_fw, 1, buf) < 0)
    return -1;

  if (buf[0] != 0xE1) {
    pmsg_info("Unexpected Device Response on \"Get Firmware Info\"\n");
    return -1;
  }

  unsigned char str_num [32];
  memcpy(str_num, &buf[32], 18);
  str_num[18] = 0;  // Known Zero terminator
  pmsg_info("PICkit5, FW Ver: %02x.%02x.%02x, SerialNumber: %s\n", buf[3], buf[4], buf[5], str_num);
  return 0;
}

// type can be CMD, UPLOAD or DOWNLOAD. Might enum be better?
#define SCR_CMD 0x0100
#define SCR_UPLOAD 0x80000102
#define SCR_DOWNLAOD 0x0C0000101


static int pickit5_send_script(const PROGRAMMER *pgm, unsigned int script_type,
                        const unsigned char* script, unsigned int script_len,
                        const unsigned char* param,  unsigned int param_len,
                        unsigned int payload_len) {
  if (PDATA(pgm) == NULL)
    return -1;
  
  if (script == NULL)   // a valid pointer to a script is needed
    return -1;
  
  unsigned int header_len = 16 + 8;  // header info + script header
  unsigned int preamble_len = header_len + param_len;
  unsigned int message_len = preamble_len + script_len;

  if (message_len > 1023) // required memory will exceed buffer size, abort
    return -1;            // 1kB should be enough for everything
  
  unsigned char buf [1024];

  pickit5_uint32_to_array(&buf[0], script_type);
  pickit5_uint32_to_array(&buf[4], 0);
  pickit5_uint32_to_array(&buf[8], message_len);
  pickit5_uint32_to_array(&buf[12], payload_len);
  pickit5_uint32_to_array(&buf[16], param_len);
  pickit5_uint32_to_array(&buf[20], script_len);

  if (param != NULL) // param can be NULL, make sure not to touch it then
    memcpy(&buf[24], param, param_len);

  memcpy(&buf[preamble_len], script, script_len);

  return serial_send(&pgm->fd, buf, message_len);
}

static int pickit5_read_response(const PROGRAMMER *pgm, char* fn_name) {
  unsigned char *buf = PDATA(pgm)->rxBuf;
  if (serial_recv(&pgm->fd, buf, 1024) < 0) {
    pmsg_error("Reading from PICkit failed");
    return -1;
  }
  unsigned int status = pickit5_array_to_uint32(&buf[0]);
  if (status != 0x0D) {
    pmsg_error("Error on response in function \"%s\"", fn_name);
    return -1;
  }
  return 0;
}

static int pickit5_send_script_done(const PROGRAMMER *pgm) {
  unsigned char script_done [16];
  unsigned int script_done_type = 0x0103;
  pickit5_uint32_to_array(&script_done[0], script_done_type);
  pickit5_uint32_to_array(&script_done[4], 0);
  pickit5_uint32_to_array(&script_done[8], 16);
  pickit5_uint32_to_array(&script_done[12], 0);
  return serial_send(&pgm->fd, script_done, 16);
}


/* Interface - prog. */
static int pickit5_open(PROGRAMMER *pgm, const char *port) {
  pmsg_debug("pickit5_open(\"%s\")\n", port);

  union pinfo pinfo;
  LNODEID usbpid;
  int rv = -1;

#if !defined(HAVE_LIBUSB) && !defined(HAVE_LIBHIDAPI)
  pmsg_error("was compiled without USB or HIDAPI support\n");
  return -1;
#endif

  // If the config entry did not specify a USB PID, insert the default one.
  if (lfirst(pgm->usbpid) == NULL)
    ladd(pgm->usbpid, (void *)USB_DEVICE_PICKIT5);

  pinfo.usbinfo.vid = pgm->usbvid? pgm->usbvid: USB_VENDOR_MICROCHIP;

#if defined(HAVE_LIBHIDAPI)
  // Try HIDAPI first. LibUSB is more generic, but might
  // cause trouble for HID-class devices in some OSes
  serdev = &usbhid_serdev;
  for (usbpid = lfirst(pgm->usbpid); rv < 0 && usbpid != NULL; usbpid = lnext(usbpid)) {
    pinfo.usbinfo.flags = PINFO_FL_SILENT;
    pinfo.usbinfo.pid = *(int *)(ldata(usbpid));
    pgm->fd.usb.max_xfer = USB_PK5_MAX_XFER;
    pgm->fd.usb.rep = 0x81;   // command read
    pgm->fd.usb.wep = 0x02;   // command write
    pgm->fd.usb.eep = 0x00;   // data read

    pgm->port = port;
    rv = serial_open(port, pinfo, &pgm->fd);
  }
  if (rv < 0) {
#endif    /* HAVE_LIBHIDAPI */
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev_frame;
    for (usbpid = lfirst(pgm->usbpid); rv < 0 && usbpid != NULL; usbpid = lnext(usbpid)) {
      pinfo.usbinfo.flags = PINFO_FL_SILENT;
      pinfo.usbinfo.pid = *(int *)(ldata(usbpid));
      pgm->fd.usb.max_xfer = USB_PK5_MAX_XFER;
      pgm->fd.usb.rep = 0x81;   // command read
      pgm->fd.usb.wep = 0x02;   // command write
      pgm->fd.usb.eep = 0x00;   // data read

      pgm->port = port;
      rv = serial_open(port, pinfo, &pgm->fd);
    }
#endif    /* HAVE_LIBUSB */
#if defined(HAVE_LIBHIDAPI)
  }
#endif

  // Make USB serial number available to programmer
  if (serdev && serdev->usbsn) {
    pgm->usbsn = serdev->usbsn;
    PDATA(pgm)->pk_op_mode = 0x01;
  }

  return rv;
}

static void pickit5_close(PROGRAMMER * pgm) {
  pmsg_debug("pickit5_close()\n");
  if (PDATA(pgm)->pk_op_mode == 3)
    pickit5_program_disable(pgm, NULL); // Disable Programming mode if still enabled
  
  if (PDATA(pgm)->power_source != 0)
    pickit5_set_vtarget(pgm, 0.0);  // Switches off PICkit Voltage regulator, if enabled

  if (PDATA(pgm)->usbhandle!=NULL) {
    unsigned char temp[4];
    memset(temp, 0, sizeof(temp));
    serial_close(&pgm->fd);
  }
}

static void pickit5_disable(const PROGRAMMER *pgm) {
  return;
}

static void pickit5_enable(PROGRAMMER *pgm, const AVRPART *p) {
  return;
}

static void pickit5_display(const PROGRAMMER *pgm, const char *p) {
  return;
}

static int pickit5_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  if (PDATA(pgm) == NULL)
    return -1;

  PDATA(pgm)->scripts = get_pickit_script(p->desc);
  if (PDATA(pgm)->scripts == NULL) {
    pmsg_info("Failed to match scripts to %s\n", p->desc);
    return -1;
  }
  pmsg_info("Scripts for %s found and loaded\n", p->desc);

  if (pickit5_get_fw_info(pgm) < 0) // PK responds, we can try to enable voltage
    return -1;

  pickit5_set_ptg_mode(pgm);
  double v_target;
  pickit5_get_vtarget(pgm, &v_target);  // see if we have to supply power, or we get it externally
  if (v_target < 1800.0) {
    if (PDATA(pgm)->power_source == 2) {
      pmsg_info("No external Voltage detected, but continuing anyway");
    } else if (PDATA(pgm)->power_source == 1) { 
      pmsg_info("No extenal Voltage detected, will try to supply from PICkit\n");
      if (pickit5_set_vtarget(pgm, (double)PDATA(pgm)->target_voltage) < 0) return -1;
      if (pickit5_get_vtarget(pgm, &v_target) < 0) return -1;    // Verify Voltage
      if (v_target < PDATA(pgm)->target_voltage - 300.0 
        || v_target > PDATA(pgm)->target_voltage + 300.0) {
        pmsg_error("Target Voltage out of requested range, Aborting");
        return -1;
      }
      pickit5_program_enable(pgm, p);
    } else {
      pmsg_error("No external Voltage detected, aborting. Overwrite this check with -xvoltage=0");
      return -1;
    }
  } else {
    PDATA(pgm)->power_source = 0;
    pmsg_info("External Voltage detected, won't supply power\n");
  }
  return 0;
}


static int pickit5_cmd(const PROGRAMMER *pgm, const unsigned char *cmd,
                   unsigned char *res) {
  return 0;
}

static int pickit5_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_notice("pickit5_enter_prog_mode()\n");
  const unsigned char* enter_prog = PDATA(pgm)->scripts->EnterProgMode;
  int enter_prog_len = PDATA(pgm)->scripts->EnterProgMode_len;
  if (PDATA(pgm)->pk_op_mode != 3) {
    if (pickit5_send_script(pgm, SCR_CMD, enter_prog, enter_prog_len, NULL, 0, 0) < 0)
      return -1;

    if (pickit5_read_response(pgm, "Enter Programming Mode") < 0)
      return -1;
     PDATA(pgm)->pk_op_mode = 3;
  }
  return 0;
}

static int pickit5_program_disable(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_notice("pickit5_exit_prog_mode()\n");
  const unsigned char* enter_prog = PDATA(pgm)->scripts->ExitProgMode;
  int enter_prog_len = PDATA(pgm)->scripts->ExitProgMode_len;
    if (PDATA(pgm)->pk_op_mode == 3) {
    if (pickit5_send_script(pgm, SCR_CMD, enter_prog, enter_prog_len, NULL, 0, 0) < 0)
      return -1;

    if (pickit5_read_response(pgm, "Exit Programming Mode") < 0)
      return -1;
    PDATA(pgm)->pk_op_mode = 2;
  }
  return 0;
}

static int pickit5_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  return 0;
}

static int pickit5_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {
  return 0;
}

static int pickit5_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {

  return 0;
}





/*
 * Set sck period (in seconds)
 * Find next possible sck period and write it to the programmer.
 */
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod) {
  
  return 0;
}


static int pickit5_write_byte (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned long addr, unsigned char value) {
    return 0;
}

static int pickit5_read_byte (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned long addr, unsigned char *value) {
    return 0;
}

static int pickit5_read_sig_bytes (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m) {
  pmsg_info("pickit5_read_sig_bytes()\n");
  const unsigned char* read_id = PDATA(pgm)->scripts->GetDeviceID;
  int read_id_len = PDATA(pgm)->scripts->GetDeviceID_len;
  if(pickit5_send_script(pgm, SCR_CMD, read_id, read_id_len, NULL, 0, 0) < 0)
    return -1;
  if (pickit5_read_response(pgm, "Read Device ID") < 0)
    return -1;
  m->buf[0] = PDATA(pgm)->rxBuf[24];
  m->buf[1] = PDATA(pgm)->rxBuf[25];
  m->buf[2] = PDATA(pgm)->rxBuf[26];
  //m->buf[3] = PDATA(pgm)->rxBuf[27];  // Chip revision
  return 3;
}


static int pickit5_read_sib (const PROGRAMMER *pgm, const AVRPART *p, char *sib) {
  // Will read out the first 8 bytes only
  pmsg_info("pickit5_read_sib()\n");
  const unsigned char* read_sib = PDATA(pgm)->scripts->ReadSIB;
  int read_sib_len = PDATA(pgm)->scripts->ReadSIB_len;

  if (pickit5_send_script(pgm, SCR_CMD, read_sib, read_sib_len, NULL, 0, 0) < 0)
    return -1;
  if (pickit5_read_response(pgm, "Read SIB") < 0)
    return -1;
  
  return -1;
}

static int pickit5_read_chip_rev (const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev) {
    return 0;
}

static int pickit5_set_vtarget (const PROGRAMMER *pgm, double v) {
  unsigned char set_vtarget [] = {
    0x40, 
    0x00, 0x00, 0x00, 0x00, // Vdd
    0x00, 0x00, 0x00, 0x00, // Vpp
    0x00, 0x00, 0x00, 0x00, // Vpp_op
    0x42, 0x43,
  };
  unsigned char power_source [] = {
    0x46, 0x00, 0x00, 0x00, 0x00,
  };
  unsigned char disable_power [] = {
    0x44
  };

  if (v < 1.0) {  // make sure not to trip on float's "inaccuracy"
    if (pickit5_send_script(pgm, SCR_CMD, power_source, 5, NULL, 0, 0) < 0)
      return -1;
    if (pickit5_read_response(pgm, "Select external power source") < 0)
      return -1;
    
    if (pickit5_send_script(pgm, SCR_CMD, disable_power, 1, NULL, 0, 0) < 0)
      return -1;
    if (pickit5_read_response(pgm, "Disabling Power") < 0)
      return -1;
  } else {
    power_source[1] = 0x01;
    if (pickit5_send_script(pgm, SCR_CMD, power_source, 5, NULL, 0, 0) < 0)
      return -1;
    if (pickit5_read_response(pgm, "Select internal power source") < 0)
      return -1;

    pickit5_uint32_to_array(&set_vtarget[1], PDATA(pgm)->target_voltage);
    pickit5_uint32_to_array(&set_vtarget[5], PDATA(pgm)->target_voltage);
    pickit5_uint32_to_array(&set_vtarget[9], PDATA(pgm)->target_voltage);
    
    if (pickit5_send_script(pgm, SCR_CMD, set_vtarget, 15, NULL, 0, 0) < 0)
      return -1;
    
    if (pickit5_read_response(pgm, "set_vtarget") < 0)
      return -1;
  }
  return 0;
}

static int pickit5_get_vtarget (const PROGRAMMER *pgm, double *v) {
  const unsigned char get_vtarget [] = {0x47, };
  unsigned char* buf = PDATA(pgm)->rxBuf;

  if (pickit5_send_script(pgm, SCR_CMD, get_vtarget, 1, NULL, 0, 0) < 0)
    return -1;

  if (pickit5_read_response(pgm, "get_vtarget") < 0)
    return -1;
  
  // 24 - internal Vdd [mV]
  // 28 - target Vdd [mV]
  // 32 - Target Vpp [mV]
  // 36 - Internal Vpp [mV]
  // 48 - Vdd Current Sense [mA] 
  int vtarget = pickit5_array_to_uint32(&buf[28]);
  pmsg_info("Target Vdd: %umV\n", vtarget);
  *v = (double)vtarget;
  return 0;
}

static int pickit5_set_ptg_mode(const PROGRAMMER *pgm) {
  unsigned char ptg_mode [] = {
    0x5E, 0x00, 0x00, 0x00, 0x00,
  };
  unsigned char buf[8];
  
  if (pickit5_send_script(pgm, SCR_UPLOAD, ptg_mode, 5, NULL, 0, 4) < 0)
    return -1;
  if (pickit5_read_response(pgm, "Set PTG mode") < 0)
    return -1;
  
  if (usbdev_data_recv(&pgm->fd, buf, 4) < 0)
    return -1;
  if (pickit5_send_script_done(pgm) < 0)
    return -1;
  if (pickit5_read_response(pgm, "Set PTG mode") < 0)
    return -1;
  return 0;
}





void pickit5_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "pickit5");

  /*
   * mandatory functions
   */

  pgm->initialize     = pickit5_initialize;
  pgm->parseextparams = pickit5_parseextparms;
  pgm->display        = pickit5_display;
  pgm->enable         = pickit5_enable;
  pgm->disable        = pickit5_disable;
  pgm->program_enable = pickit5_program_enable;
  pgm->chip_erase     = pickit5_chip_erase;
  pgm->cmd            = pickit5_cmd;
  pgm->open           = pickit5_open;
  pgm->close          = pickit5_close;
  pgm->write_byte     = pickit5_write_byte;
  pgm->read_byte      = pickit5_read_byte;
  
  
  /*
   * optional functions
   */

  pgm->paged_write    = pickit5_paged_write;
  pgm->paged_load     = pickit5_paged_load;
  pgm->setup          = pickit5_setup;
  pgm->teardown       = pickit5_teardown;
  pgm->set_sck_period = pickit5_set_sck_period;
  pgm->end_programming = pickit5_program_disable;
  pgm->read_sig_bytes = pickit5_read_sig_bytes;
  pgm->read_sib       = pickit5_read_sib;
  pgm->read_chip_rev  = pickit5_read_chip_rev;
  pgm->set_vtarget    = pickit5_set_vtarget;
  pgm->get_vtarget    = pickit5_get_vtarget;

}

static int usb_fill_buf(usb_dev_handle *udev, int maxsize, int ep, int use_interrupt_xfer);
static int usb_fill_buf(usb_dev_handle *udev, int maxsize, int ep, int use_interrupt_xfer) {
  int rv;

  if (use_interrupt_xfer)
    rv = usb_interrupt_read(udev, ep, cx->usb_buf, maxsize, 10000);
  else
    rv = usb_bulk_read(udev, ep, cx->usb_buf, maxsize, 10000);
  if (rv < 0)
    {
      pmsg_notice2("usb_fill_buf(): usb_%s_read() error: %s\n",
        use_interrupt_xfer? "interrupt": "bulk", usb_strerror());
      return -1;
    }

  cx->usb_buflen = rv;
  cx->usb_bufptr = 0;

  return 0;
}

static int usbdev_data_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes) {
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;
  int i, amnt;
  unsigned char * p = buf;

  if (udev == NULL)
    return -1;

  for (i = 0; nbytes > 0;)
    {
      if (cx->usb_buflen <= cx->usb_bufptr)
	{
	  if (usb_fill_buf(udev, fd->usb.max_xfer, USB_PK5_DATA_READ_EP, fd->usb.use_interrupt_xfer) < 0)
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
  usb_dev_handle *udev = (usb_dev_handle *)fd->usb.handle;
  int rv;
  int i = mlen;
  const unsigned char * p = bp;
  int tx_size;

  if (udev == NULL)
    return -1;

  /*
   * Split the frame into multiple packets.  It's important to make
   * sure we finish with a short packet, or else the device won't know
   * the frame is finished.  For example, if we need to send 64 bytes,
   * we must send a packet of length 64 followed by a packet of length
   * 0.
   */
  do {
    tx_size = ((int) mlen < fd->usb.max_xfer)? (int) mlen: fd->usb.max_xfer;
    if (fd->usb.use_interrupt_xfer)
      rv = usb_interrupt_write(udev, USB_PK5_DATA_WRITE_EP, (char *)bp, tx_size, 10000);
    else
      rv = usb_bulk_write(udev, USB_PK5_DATA_WRITE_EP, (char *)bp, tx_size, 10000);
    if (rv != tx_size)
    {
        pmsg_error("wrote %d out of %d bytes, err = %s\n", rv, tx_size, usb_strerror());
        return -1;
    }
    bp += tx_size;
    mlen -= tx_size;
  } while (mlen > 0);

  if(verbose > 3)
    trace_buffer(__func__, p, i);
  return 0;
}


#else /* HAVE_LIBUSB */
static int pickit5_nousb_open(PROGRAMMER *pgm, const char *name);

static int pickit5_nousb_open(PROGRAMMER *pgm, const char *name) {
  pmsg_error("no usb support; please compile again with libusb installed\n");

  return -1;
}

void pickit5_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "pickit5");

  pgm->open           = pickit5_nousb_open;
}

#endif  /* HAVE_LIBUSB */

const char pickit5_desc[] = "Microchip's PICkit 5 Programmer/Debugger";
