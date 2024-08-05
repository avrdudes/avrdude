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



#if defined(HAVE_LIBUSB) || defined(HAVE_LIBUSB_1_0)

#if defined(HAVE_USB_H)
  #include <usb.h>
#elif defined(HAVE_LIBUSB_1_0_LIBUSB_H)
  #  include <libusb-1.0/libusb.h>
#elif defined(HAVE_LIBUSB_H)
  #  include <libusb.h>
#else
  #  error "libusb needs either <usb.h>, <lusb0_usb.h> or <libusb-1.0/libusb.h>"
#endif

#define USE_LIBUSB_1_0

#define USB_PK5_CMD_READ_EP   0x81
#define USB_PK5_CMD_WRITE_EP  0x02
#define USB_PK5_DATA_READ_EP  0x83
#define USB_PK5_DATA_WRITE_EP 0x04

#define USB_PK5_MAX_XFER      512

#define CHECK_ERROR           0x01
#define BIST_TEST             0x02
#define BIST_RESULT           0x03


// Private data for this programmer
struct pdata {
  unsigned char pk_op_mode;     // 0: init / 1: pk found / 2: pk operating / 3: pk programming
  unsigned char power_source;   // external / from PICkit / ignore check
  unsigned char hvupdi_enabled; // 0: no HV / 1: HV generation enabled
  unsigned int  target_voltage; // voltage to supply to target
  unsigned char signature[4];   // as we get DeviceID and Chip revision in one go, buffer them
  unsigned char txBuf[1024];
  unsigned char rxBuf[1024];
  struct avr_script_lut *scripts;
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
static int pickit5_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes);
static int pickit5_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes);
static int pickit5_write_byte     (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char value);
static int pickit5_read_byte (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, 
  unsigned long addr, unsigned char *value);
static int pickit5_read_sig_bytes (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem);
static int pickit5_read_sib (const PROGRAMMER *pgm, const AVRPART *p, char *sib);
static int pickit5_read_chip_rev  (const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev);
static int pickit5_read_array (const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);
static int pickit5_write_array(const PROGRAMMER *pgm, const AVRPART *p, 
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);
// Extra Functions
static int pickit5_get_fw_info(const PROGRAMMER *pgm);
static int pickit5_set_vtarget (const PROGRAMMER *pgm, double v);
static int pickit5_get_vtarget (const PROGRAMMER *pgm, double *v);
static int pickit5_set_ptg_mode(const PROGRAMMER *pgm);
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod);
static int pickit5_write_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char value);
static int pickit5_read_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char* value);


// Internal Functions
inline static void pickit5_uint32_to_array(unsigned char *buf, uint32_t num);
inline static unsigned int pickit5_array_to_uint32(unsigned char *buf);
inline static void pickit5_create_payload_header(unsigned char *buf, unsigned int type, 
  unsigned int msg_len, unsigned int trans_len);
inline static void pickit5_create_script_header(unsigned char *buf, unsigned int arg_len,
  unsigned int script_len);
inline static int pickit5_check_ret_status(const PROGRAMMER *pgm);
static int pickit5_get_Status(const PROGRAMMER *pgm, unsigned char status);

static int pickit5_send_script(const PROGRAMMER *pgm, unsigned int script_type,
  const unsigned char *script, unsigned int script_len,
  const unsigned char *param,  unsigned int param_len, unsigned int payload_len);
static int pickit5_send_script_done(const PROGRAMMER *pgm, char *func);
static int pickit5_read_response(const PROGRAMMER *pgm, char *fn_name);


// Extra USB related Functions, because we need more then 2 Endpoints
static int usbdev_data_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes);
static int usbdev_data_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen);


inline static void pickit5_uint32_to_array(unsigned char *buf, uint32_t num) {
  *(buf++) = (uint8_t)  num;
  *(buf++) = (uint8_t) (num >> 8);
  *(buf++) = (uint8_t) (num >> 16);
  *(buf)   = (uint8_t) (num >> 24);
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

inline static void pickit5_create_script_header(unsigned char *buf, unsigned int arg_len,
  unsigned int script_len) {
    pickit5_uint32_to_array(&buf[0], arg_len);
    pickit5_uint32_to_array(&buf[4], script_len);
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
    if (str_starts(extended_param, "hvupdi")) {
      PDATA(pgm)->hvupdi_enabled = 1;
      continue;
    }

    if (str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      msg_error("  -xvoltage=<arg> Enables power output. <arg> must be in range 1800~5500[mV]\n");
      msg_error("  -xhvupdi        Enable high-voltage UPDI initialization\n");
      msg_error("  -xhelp            Show this help menu and exit\n");
      return LIBAVRDUDE_EXIT;
    }

    pmsg_error("invalid extended parameter '%s'\n", extended_param);
    rv = -1;
  }
  return rv;
}


/* Internal functions */

// type can be CMD, UPLOAD or DOWNLOAD. Might enum be better?
#define SCR_CMD 0x0100
#define SCR_UPLOAD 0x80000102
#define SCR_DOWNLAOD 0x0C0000101


static int pickit5_send_script(const PROGRAMMER *pgm, unsigned int script_type,
                        const unsigned char *script, unsigned int script_len,
                        const unsigned char *param,  unsigned int param_len,
                        unsigned int payload_len) {
  if (script == NULL)   // a valid pointer to a script is needed
    return -1;
  
  unsigned int header_len = 16 + 8;  // header info + script header
  unsigned int preamble_len = header_len + param_len;
  unsigned int message_len = preamble_len + script_len;

  if (message_len > 1023) // required memory will exceed buffer size, abort
    return -1;            // 1kB should be enough for everything
  
  unsigned char *buf = PDATA(pgm)->txBuf;

  pickit5_create_payload_header(&buf[0], script_type, message_len, payload_len);
  pickit5_create_script_header(&buf[16], param_len, script_len);

  if (param != NULL) // param can be NULL, make sure not to touch it then
    memcpy(&buf[24], param, param_len);

  memcpy(&buf[preamble_len], script, script_len);

  return serial_send(&pgm->fd, buf, message_len);
}

static int pickit5_read_response(const PROGRAMMER *pgm, char *fn_name) {
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

static int pickit5_send_script_done(const PROGRAMMER *pgm, char *func) {
  unsigned char script_done [16];
  unsigned int script_done_type = 0x0103;
  pickit5_create_payload_header(script_done, script_done_type, 16, 0);
  if (serial_send(&pgm->fd, script_done, 16) >= 0)
    return pickit5_read_response(pgm, func);
  pmsg_error("Failed to send \"script done\"");
  return -1;
}


/* Interface - prog. */
static int pickit5_open(PROGRAMMER *pgm, const char *port) {
  if (PDATA(pgm) == NULL)   // Make sure we do have a pointer to our data structure
    return -1;
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

  // PICkit 5 doesn't have support for HID, so no need to support it
#if defined(HAVE_LIBUSB_1_0)
  serdev = &usb_serdev_frame;
  for (usbpid = lfirst(pgm->usbpid); rv < 0 && usbpid != NULL; usbpid = lnext(usbpid)) {
    pinfo.usbinfo.flags = PINFO_FL_SILENT;
    pinfo.usbinfo.pid = *(int *)(ldata(usbpid));
    pgm->fd.usb.max_xfer = USB_PK5_MAX_XFER;
    pgm->fd.usb.rep = USB_PK5_CMD_READ_EP;   // command read
    pgm->fd.usb.wep = USB_PK5_CMD_WRITE_EP;  // command write
    pgm->fd.usb.eep = 0x00;

    pgm->port = port;
    rv = serial_open(port, pinfo, &pgm->fd);
  }
#endif    /* HAVE_LIBUSB */

  // Make USB serial number available to programmer
  if (serdev && serdev->usbsn) {
    pgm->usbsn = serdev->usbsn;
    PDATA(pgm)->pk_op_mode = 0x01;
  }

  return rv;
}

static void pickit5_close(PROGRAMMER *pgm) {
  pmsg_debug("pickit5_close()\n");
  if (PDATA(pgm)->pk_op_mode == 3)
    pickit5_program_disable(pgm, NULL); // Disable Programming mode if still enabled
  
  pickit5_set_vtarget(pgm, 0.0);  // Switches off PICkit Voltage regulator, if enabled

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
}

static void pickit5_display(const PROGRAMMER *pgm, const char *p) {
  return;
}

static int pickit5_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("pickit5_initialize()");
  if (PDATA(pgm) == NULL)
    return -1;

  PDATA(pgm)->scripts = get_pickit_script(p->desc);
  if (PDATA(pgm)->scripts == NULL) {
    pmsg_error("Failed to match scripts to %s\n", p->desc);
    return -1;
  }
  pmsg_info("Scripts for %s found and loaded\n", p->desc);

  if (PDATA(pgm)->hvupdi_enabled > 0) {
    if (p->hvupdi_variant == 0)
      pmsg_info("High Voltage SYSCFG0 override on UPDI Pin enabled\n");
    if (p->hvupdi_variant == 2)
      pmsg_info("High Voltage SYSCFG0 override on RST Pin enabled\n");
  }

  if (pickit5_get_fw_info(pgm) < 0) // PK responds, we can try to enable voltage
    return -1;

  pickit5_set_ptg_mode(pgm);
  pickit5_set_vtarget(pgm, 0.0);  // avoid the edge case when avrdude was CTRL+C'd but still provides power

  double v_target;
  pickit5_get_vtarget(pgm, &v_target);  // see if we have to supply power, or we get it externally
  if (v_target < 1800.0) {
    if (PDATA(pgm)->power_source == 2) {
      pmsg_warning("No external Voltage detected, but continuing anyway\n");
    } else if (PDATA(pgm)->power_source == 1) { 
      pmsg_info("No extenal Voltage detected, will try to supply from PICkit\n");
      if (pickit5_set_vtarget(pgm, (double)PDATA(pgm)->target_voltage) < 0) return -1;
      if (pickit5_get_vtarget(pgm, &v_target) < 0) return -1;    // Verify Voltage
      if (v_target < PDATA(pgm)->target_voltage - 250.0     // Voltage supply is not accurate, allow some room
        || v_target > PDATA(pgm)->target_voltage + 150.0) {
        pmsg_error("Target Voltage out of range, Aborting.\n");
        return -1;
      }

      pickit5_program_enable(pgm, p);
      unsigned int baud = pgm->baudrate;
      if (baud < 300) {   // Better be safe than sorry
        pmsg_warning("UPDI needs a higher clock for operation, limiting UPDI to 300Hz\n");
        baud = 300;
      }
      if (baud > 225000) {
        if (v_target < 2900.0) {
          pmsg_warning("UPDI needs a higher Voltage for a faster Baudrate, limiting UPDI to 225kHz\n");
          baud = 225000;
        } else {
          // ToDo: Datasheet recommends setting BOD to highest level, but the BODs address is either 0x00A0 or 0x0080
          // So best would be to figure out a way to deduct the correct address
          
          if (baud > 900000) {
            pmsg_warning("Requested clock too high. Limiting UPDI to 900kHz\n");
            baud = 900000;
          }
          pickit5_set_sck_period(pgm, 1.0/100000);  // Start with 200kHz
          pickit5_write_cs_reg(pgm, UPDI_ASI_CTRLA, 0x01);    // Change UPDI Clock
          unsigned char ret_val = 0;
          pickit5_read_cs_reg(pgm, UPDI_ASI_CTRLA, &ret_val);
          if (ret_val != 0x01) {
            pmsg_warning("Failed to change UPDI Clock, falling back to 225kHz");
            baud = 225000;
          }
        }
      }
      if (pickit5_set_sck_period(pgm, 1.0/baud) >= 0) {
        pmsg_info("UPDI Speed set to %ikHz\n", baud / 1000);
      } else {
        pmsg_warning("Failed to set UPDI speed, continuing...\n");
      }
      pickit5_program_enable(pgm, p);
    } else {
      pmsg_error("No external Voltage detected, aborting. Overwrite this check with -xvoltage=0\n");
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
  return -2;
}

static int pickit5_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("pickit5_enter_prog_mode()\n");
  const unsigned char *enter_prog = PDATA(pgm)->scripts->EnterProgMode;
  unsigned int enter_prog_len = PDATA(pgm)->scripts->EnterProgMode_len;
  if (PDATA(pgm)->hvupdi_enabled) {
    if (p->hvupdi_variant == HV_UPDI_VARIANT_0) { // High Voltage Generation on UPDI line
      enter_prog = PDATA(pgm)->scripts->EnterProgModeHvSp;
      enter_prog_len = PDATA(pgm)->scripts->EnterProgModeHvSp_len;
    } else if (p->hvupdi_variant == HV_UPDI_VARIANT_2) {  // High Voltage Generation on RST line
      enter_prog = PDATA(pgm)->scripts->EnterProgModeHvSpRst;
      enter_prog_len = PDATA(pgm)->scripts->EnterProgModeHvSpRst_len;
    }
  }
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
  pmsg_debug("pickit5_exit_prog_mode()\n");
  const unsigned char *enter_prog = PDATA(pgm)->scripts->ExitProgMode;
  unsigned int enter_prog_len = PDATA(pgm)->scripts->ExitProgMode_len;
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
  pmsg_debug("pickit5_chip_erase()\n");
  if (PDATA(pgm)->pk_op_mode == 3) {  // we have to be in Prog-Mode
    const unsigned char *chip_erase = PDATA(pgm)->scripts->EraseChip;
    unsigned int chip_erase_len = PDATA(pgm)->scripts->EraseChip_len;

    if (pickit5_send_script(pgm, SCR_CMD, chip_erase, chip_erase_len, NULL, 0, 0) >= 0){
      if (pickit5_read_response(pgm, "Erase Chip") >= 0) {
        if (pickit5_array_to_uint32(&(PDATA(pgm)->rxBuf[16])) == 0x00) {
          pmsg_info("Target successfully erased");
          PDATA(pgm)->pk_op_mode = 2;
          return 0;
        }
      }
    }
  }
  pmsg_error("Chip Erase failed\n");
  return -1;
}

static int pickit5_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {

  pickit5_program_enable(pgm, p);
  return pickit5_read_array(pgm, p, mem, address, n_bytes, &mem->buf[address]);
}

static int pickit5_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {
  
  pickit5_program_enable(pgm, p);
  return pickit5_write_array(pgm, p, mem, address, n_bytes, &mem->buf[address]);
}


// Sets UPDI Frequency in kHz
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod) {
  pmsg_debug("pickit5_set_sck_period()\n");
  double frq = (0.001 / sckperiod) + 0.5; // 1ms/period = kHz; round up
  const unsigned char *set_speed = PDATA(pgm)->scripts->SetSpeed;
  int set_speed_len = PDATA(pgm)->scripts->SetSpeed_len;
  unsigned char buf[4];
  pickit5_uint32_to_array(buf, frq);
  if(pickit5_send_script(pgm, SCR_CMD, set_speed, set_speed_len, buf, 4, 0) < 0)
    return -1;
  if (pickit5_read_response(pgm, "Set UPDI Speed") < 0)
    return -1;
  return 0;
}

static int pickit5_write_byte (const PROGRAMMER *pgm, const AVRPART *p, 
  const AVRMEM *mem, unsigned long addr, unsigned char value) {
    if (mem_is_sram(mem) || mem_is_io(mem)) {
      if (mem->size < 1 || addr > (unsigned long)mem->size) {
        pmsg_error("address %i out of range for %s [0, %i]\n", (unsigned int) addr, mem->desc, mem->size);
        return -1;
      }
      addr += mem->offset;
      pmsg_debug("pickit5_write_byte(0x%4X, %i)\n", (unsigned int)addr, value);
      // This script is based on WriteCSreg. Reduces overhead by avoiding writing data EP
      const unsigned char h_len = 24; // 16 + 8
      const unsigned char p_len = 8;
      const unsigned char s_len = 8;
      const unsigned char m_len = h_len + p_len + s_len; 
      unsigned char write8_fast [] = {
        0x00, 0x01, 0x00, 0x00,   // [0]  SCR_CMD
        0x00, 0x00, 0x00, 0x00,   // [4]  always 0
        m_len, 0x00, 0x00, 0x00,  // [8]  message length = 16 + 8 + param (8) + script (8) = 40
        0x00, 0x00, 0x00, 0x00,   // [12] keep at 0 to receive the data in the "response"
        
        p_len, 0x00, 0x00, 0x00,  // [16] param length: 8 bytes
        s_len, 0x00, 0x00, 0x00,  // [20] length of script: 8 bytes

        0x00, 0x00, 0x00, 0x00,   // [24] param: address to write to, will be overwritten
        0x00, 0x00, 0x00, 0x00,   // [28] param: byte to write, will be overwritten
        
        // Script itself:
        0x91, 0x00,               // copy first 4 bytes of param to reg 0
        0x91, 0x01,               // copy second 4 bytes of param to reg 1
        0x1E, 0x06, 0x00, 0x01,   // store to address in reg 0 the byte in reg 1
      };
      write8_fast[24] = (((unsigned char *)&addr)[0]);
      write8_fast[25] = (((unsigned char *)&addr)[1]);
      write8_fast[28] = value;

      serial_send(&pgm->fd, write8_fast, m_len);
      unsigned char *buf = PDATA(pgm)->rxBuf;
      if (serial_recv(&pgm->fd, buf, 1024) >= 0) {  // read response
        if (buf[0] == 0x0D) {
          return 0;
        }
      }
      return -1;
    }
  int rc = pickit5_write_array(pgm, p, mem, addr, 1, &value);
  if (rc < 0)
    return rc;
  return 0; 
}

static int pickit5_read_byte (const PROGRAMMER *pgm, const AVRPART *p, 
  const AVRMEM *mem, unsigned long addr, unsigned char *value) {
    if (mem_is_sram(mem) || mem_is_io(mem) 
     || mem_is_lock(mem) || mem_is_in_fuses(mem)) {
      if (mem->size < 1 || addr > (unsigned long)mem->size) {
        pmsg_error("address %i out of range for %s [0, %i]\n", (unsigned int) addr, mem->desc, mem->size);
        return -1;
      }
      addr += mem->offset;
      pmsg_debug("pickit5_read_byte(0x%4X)\n", (unsigned int)addr);
      // This script is based on ReadSIB. Reduces overhead by avoiding readind data EP
      const unsigned char h_len = 24; // 16 + 8
      const unsigned char p_len = 4;
      const unsigned char s_len = 6;
      const unsigned char m_len = h_len + p_len + s_len; 
      unsigned char read8_fast [] = {
        0x00, 0x01, 0x00, 0x00,   // [0]  SCR_CMD
        0x00, 0x00, 0x00, 0x00,   // [4]  always 0
        m_len, 0x00, 0x00, 0x00,  // [8]  message length = 16 + 8 + param (4) + script (6) = 34
        0x00, 0x00, 0x00, 0x00,   // [12] keep at 0 to receive the data in the "response"
        
        p_len, 0x00, 0x00, 0x00,  // [16] param length: 4 bytes
        s_len, 0x00, 0x00, 0x00,  // [20] length of script: 6 bytes

        0x00, 0x00, 0x00, 0x00,   // [24] param: address to read from, will be overwritten
        
        // Script itself:
        0x91, 0x00,               // copy first 4 bytes of param to reg 0
        0x1E, 0x03, 0x00,         // load byte from address in reg 0
        0x9F                      // send data from 0x1E to "response"
      };
      read8_fast[24] = (((unsigned char *)&addr)[0]);
      read8_fast[25] = (((unsigned char *)&addr)[1]);

      serial_send(&pgm->fd, read8_fast, m_len);
      unsigned char *buf = PDATA(pgm)->rxBuf;
      if (serial_recv(&pgm->fd, buf, 1024) >= 0) {  // read response
        if (buf[0] == 0x0D) {
          if (buf[20] == 0x01) {
            *value = buf[24];
            return 0;
          }
        }
      }
      return -1;
    }
    int rc = pickit5_read_array(pgm, p, mem, addr, 1, value);
    if (rc < 0)
      return rc;
    return 0;
}


// return numbers of byte written 
static int pickit5_write_array(const PROGRAMMER *pgm, const AVRPART *p, 
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("pickit5_write_array(%s, 0x%04x, %i)", mem->desc, (unsigned int)addr, len);

  if (len > mem->size || mem->size < 1) {
    pmsg_error("cannot write byte to %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  }
  if (addr >= (unsigned long) mem->size) {
    pmsg_error("cannot write byte to %s %s as address 0x%04lx outside range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size-1);
    return -1;
  }
  
  const unsigned char *write_bytes = NULL;
  unsigned int write_bytes_len = 0;
  if (mem_is_flash(mem) && (len == mem->page_size)) {
    write_bytes = PDATA(pgm)->scripts->WriteProgmem;
    write_bytes_len = PDATA(pgm)->scripts->WriteProgmem_len;
  } else if (mem_is_eeprom(mem)) {
    write_bytes = PDATA(pgm)->scripts->WriteDataEEmem;
    write_bytes_len = PDATA(pgm)->scripts->WriteDataEEmem_len;
  } else if (mem_is_in_fuses(mem)) {
    write_bytes = PDATA(pgm)->scripts->WriteConfigmem;
    write_bytes_len = PDATA(pgm)->scripts->WriteConfigmem_len;
  } else if (mem_is_io(mem) || mem_is_sram(mem)) {
    write_bytes = PDATA(pgm)->scripts->WriteMem8;
    write_bytes_len = PDATA(pgm)->scripts->WriteMem8_len;
  } else {
    pmsg_error("Unsupported memory type: %s\n", mem->desc);
    return -2;
  }
  addr += mem->offset;

  unsigned char buf [8];
  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);

  if (pickit5_send_script(pgm, SCR_DOWNLAOD, write_bytes, write_bytes_len, buf, 8, len) < 0) {
    pmsg_error("Sending Script failed\n");
    return -1;
  }
    
  if (pickit5_read_response(pgm, "Write Bytes") < 0) {
    pmsg_error("Reading Script Response\n");
    return -1;
  }
  if (usbdev_data_send(&pgm->fd, value, len) < 0) {
    pmsg_error("Failed to send data\n");
    return -1;
  }
  if (pickit5_get_Status(pgm, CHECK_ERROR) < 0) {
    pmsg_error("Error Check failed\n");
    return -1;
  }
  if (pickit5_send_script_done(pgm, "Write Bytes") < 0) {
    pmsg_error("Sending Script Done failed\n");
    return -1;
  }
  return len;
}

// return numbers of byte read 
static int pickit5_read_array (const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("pickit5_read_array(%i)\n", len);

  if (len > mem->size || mem->size < 1) {
    pmsg_error("cannot read byte from %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  }
  if (addr >= (unsigned long) mem->size) {
    pmsg_error("cannot read byte from %s %s as address 0x%04lx outside range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size-1);
    return -1;
  }

  const unsigned char *read_bytes = NULL;
  unsigned int read_bytes_len = 0;
  if (mem_is_flash(mem)) {
    read_bytes = PDATA(pgm)->scripts->ReadProgmem;
    read_bytes_len = PDATA(pgm)->scripts->ReadProgmem_len;
  } else if (mem_is_eeprom(mem)) {
    read_bytes = PDATA(pgm)->scripts->ReadDataEEmem;
    read_bytes_len = PDATA(pgm)->scripts->ReadDataEEmem_len;
  } else if (mem_is_in_fuses(mem) || mem_is_prodsig(mem) || mem_is_sigrow(mem)
          || mem_is_sernum(mem) || mem_is_tempsense(mem)) {
    read_bytes = PDATA(pgm)->scripts->ReadConfigmem;
    read_bytes_len = PDATA(pgm)->scripts->ReadConfigmem_len;
  } else if (mem_is_io(mem) || mem_is_sram(mem) || mem_is_lock(mem)) {
    read_bytes = PDATA(pgm)->scripts->ReadMem8;
    read_bytes_len = PDATA(pgm)->scripts->ReadMem8_len;
  } else if (mem_is_sib(mem)) {
    return pickit5_read_sib(pgm, p, (char *)value); // silence pointer sign warning
  } else if (mem_is_signature(mem)) {
    if (addr == 0) {                        // Buffer signature bytes on first read
      pickit5_read_sig_bytes(pgm, p, NULL);
    }
    if (addr < 4) {
      *value =  PDATA(pgm)->signature[addr];
      return 0;
    } else {
      return -1;
    }
  } else {
    pmsg_error("Unsupported memory type: %s\n", mem->desc);
    return -2;
  }
  addr += mem->offset;

  unsigned char buf [8];
  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);

  if (pickit5_send_script(pgm, SCR_UPLOAD, read_bytes, read_bytes_len, buf, 8, len) < 0) {
    pmsg_error("Sending Script failed\n");
    return -1;
  }
  if (pickit5_read_response(pgm, "Read Bytes") < 0) {
    pmsg_error("Read Response Failed\n");
    return -1;
  }
  if (usbdev_data_recv(&pgm->fd, value, len) < 0) {
    pmsg_error("Reading Data Memory failed\n");
    return -1;
  }
  if (pickit5_send_script_done(pgm, "Read Bytes") < 0) {
    pmsg_error("Sending Script Done failed\n");
    return -1;
  }
  return len;
}

static int pickit5_read_sig_bytes (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem) {
  pmsg_debug("pickit5_read_sig_bytes()\n");
  const unsigned char *read_id = PDATA(pgm)->scripts->GetDeviceID;
  unsigned int read_id_len = PDATA(pgm)->scripts->GetDeviceID_len;

  if(pickit5_send_script(pgm, SCR_CMD, read_id, read_id_len, NULL, 0, 0) < 0)
    return -1;
  if (pickit5_read_response(pgm, "Read Device ID") >= 0) {
    if (PDATA(pgm)->rxBuf[0] == 0x0D) {
      if (PDATA(pgm)->rxBuf[20] == 0x04) {
        memcpy(PDATA(pgm)->signature, &PDATA(pgm)->rxBuf[24], 4);
        if (mem != NULL) {
          mem->buf[0] = PDATA(pgm)->rxBuf[24];
          mem->buf[1] = PDATA(pgm)->rxBuf[25];
          mem->buf[2] = PDATA(pgm)->rxBuf[26];
        }
        return 3;
      } else {
        if (PDATA(pgm)->hvupdi_enabled && p->hvupdi_variant == HV_UPDI_VARIANT_2) {
          pmsg_info("Failed to get DeviceID with activated HV Pulse on RST.\n");
          pmsg_info("If your wiring is correct, try connecting a 16V 1u cap between RST and GND.\n");
        }
      }
    }
  }
  return -1;
}

static int pickit5_read_sib (const PROGRAMMER *pgm, const AVRPART *p, char *sib) {
  pmsg_debug("pickit5_read_sib()\n");
  const unsigned char *read_sib = PDATA(pgm)->scripts->ReadSIB;
  unsigned int read_sib_len = PDATA(pgm)->scripts->ReadSIB_len;

  if (pickit5_send_script(pgm, SCR_CMD, read_sib, read_sib_len, NULL, 0, 0) < 0)
    return -1;
  if (pickit5_read_response(pgm, "Read SIB") < 0)
    return -1;
  unsigned int ret_len = pickit5_array_to_uint32(&(PDATA(pgm)->rxBuf[20]));
  if (ret_len == 0x20) {  // 0x20 bytes == 32 bytes
    memcpy(sib, &PDATA(pgm)->rxBuf[24], 32);
  } else {
    return -1;
  }
  return 0;
}

static int pickit5_read_chip_rev (const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev) {
  pmsg_debug("pickit5_read_chip_rev()\n");
  *chip_rev = PDATA(pgm)->signature[3];
  return 0;
}

static int pickit5_write_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char value)  {
  pmsg_debug("pickit5_write_cs_reg(*%i = %i)", addr, value);
  const unsigned char *write_cs = PDATA(pgm)->scripts->WriteCSreg;
  unsigned int write_cs_len = PDATA(pgm)->scripts->WriteCSreg_len;

  if (addr > 0x0C) {
    pmsg_error("CS reg %i out of range [0x00, 0x0C], addr\n", addr);
    return -1;
  }

  unsigned char buf [2];
  buf[0] = addr;
  buf[1] = value;

  if (pickit5_send_script(pgm, SCR_CMD, write_cs, write_cs_len, buf, 2, 0) < 0) {
    pmsg_error("Sending Script failed\n");
    return -1;
  }
    
  if (pickit5_read_response(pgm, "Write CS reg") < 0) {
    pmsg_error("Reading Script Response\n");
    return -1;
  }
  return 1;
}
  
static int pickit5_read_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char* value) {
  pmsg_debug("pickit5_read_cs_reg(%i)\n", addr);
  const unsigned char *read_cs = PDATA(pgm)->scripts->ReadCSreg;
  unsigned int read_cs_len = PDATA(pgm)->scripts->ReadCSreg_len;

  if (addr > 0x0C) {
    pmsg_error("CS reg %i out of range [0x00, 0x0C], addr\n", addr);
    return -1;
  }

  unsigned char buf [1];
  buf[0] = addr;

  if (pickit5_send_script(pgm, SCR_UPLOAD, read_cs, read_cs_len, buf, 1, 1) < 0) {
    pmsg_error("Sending Script failed\n");
    return -1;
  }
  if (pickit5_read_response(pgm, "Read CS") < 0) {
    pmsg_error("Read Response Failed\n");
    return -1;
  }
  if (usbdev_data_recv(&pgm->fd, value, 1) < 0) {
    pmsg_error("Reading CS Memory failed\n");
    return -1;
  }
  if (pickit5_send_script_done(pgm, "Read CS") < 0) {
    pmsg_error("Sending Script Done failed\n");
    return -1;
  }
  return 0;
}


static int pickit5_get_fw_info(const PROGRAMMER *pgm) {
  pmsg_debug("pickit5_get_fw_info()");
  unsigned char *buf = PDATA(pgm)->rxBuf;
  const unsigned char get_fw [] = {0xE1};

  if (serial_send(&pgm->fd, get_fw, 1) < 0) {
    pmsg_error("Failed to send command\n");
    return -1;
  }

  // PICkit didn't like receive transfers smaller 1024
  if (serial_recv(&pgm->fd, buf, 1024) < 0) {
    pmsg_error("Failed to receive FW response\n");
    return -1;
  }

  if (buf[0] != 0xE1) {
    pmsg_error("Unexpected Device Response on \"Get Firmware Info\"\n");
    return -1;
  }

  unsigned char str_num [32];
  memcpy(str_num, &buf[32], 18);
  str_num[18] = 0;  // Known Zero terminator
  pmsg_info("PICkit5, FW Ver: %02x.%02x.%02x, SerialNumber: %s\n", buf[3], buf[4], buf[5], str_num);
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

  pmsg_debug("pickit5_set_vtarget(%4.1fmV)\n", v);

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
  unsigned char *buf = PDATA(pgm)->rxBuf;
  pmsg_debug("pickit5_get_vtarget()\n");

  if (pickit5_send_script(pgm, SCR_CMD, get_vtarget, 1, NULL, 0, 0) < 0)
    return -1;

  if (pickit5_read_response(pgm, "get_vtarget") < 0)
    return -1;
  
  // 24 - internal Vdd [mV]
  // 28 - target Vdd [mV]
  // 32 - Target Vpp [mV] (Reset Pin Voltage)
  // 36 - Internal Vpp [mV]
  // 48 - Vdd Current Sense [mA] 
  int vtarget = pickit5_array_to_uint32(&buf[28]);
  int itarget = pickit5_array_to_uint32(&buf[48]);
  pmsg_info("Target Vdd: %umV, Target current: %umA\n", vtarget, itarget);
  *v = (double)vtarget;
  return 0;
}


static int pickit5_set_ptg_mode(const PROGRAMMER *pgm) {
  unsigned char ptg_mode [] = {
    0x5E, 0x00, 0x00, 0x00, 0x00,
  };
  unsigned char buf[8];
  pmsg_debug("pickit5_set_ptg_mode()\n");
  
  if (pickit5_send_script(pgm, SCR_UPLOAD, ptg_mode, 5, NULL, 0, 4) < 0)
    return -1;
  if (pickit5_read_response(pgm, "Set PTG mode") < 0)
    return -1;
  
  if (usbdev_data_recv(&pgm->fd, buf, 4) < 0)
    return -1;
  if (pickit5_send_script_done(pgm, "Set PTG Mode") < 0)
    return -1;
  return 0;
}

static int pickit5_get_Status(const PROGRAMMER *pgm, unsigned char status) {
  unsigned char* buf = PDATA(pgm)->txBuf;
  const unsigned int type = 0x0105;
  unsigned int key_len = 0;

  if (CHECK_ERROR == status) {
    key_len = strlen("ERROR_STATUS_KEY") + 1;
    memcpy (&buf[16], "ERROR_STATUS_KEY", key_len);
  } else if (BIST_TEST == status) {
    key_len = strlen("BIST Tested") + 1;
    memcpy (&buf[16], "BIST Tested", key_len);
  } else if (BIST_RESULT == status) {
    key_len = strlen("BIST Results") + 1;
    memcpy (&buf[16], "BIST Results", key_len);
  }
  if (0 == key_len) {
    pmsg_error("Uknown key typed passed");
    return -1;
  }
  unsigned int msg_len = 16 + key_len;
  pickit5_create_payload_header(&buf[0], type, msg_len, 0);
  serial_send(&pgm->fd, buf, msg_len);
  serial_recv(&pgm->fd, PDATA(pgm)->rxBuf, 1024);
  if (pickit5_check_ret_status(pgm) < 0) {
    return -1;
  }
  unsigned int status_len = pickit5_array_to_uint32(&(PDATA(pgm)->rxBuf[8]));
  if (status_len > 64)
    status_len = 64;
  PDATA(pgm)->rxBuf[16+status_len] = 0x00; // Known Zero Terminator
  if (str_starts((const char*)&(PDATA(pgm)->rxBuf[16]), "NONE") == 0) { // cast to remove warning
    pmsg_error("PICkit Error Status report: %s", buf);
    return -1;
  }
  return 0;
}

inline static int pickit5_check_ret_status(const PROGRAMMER *pgm) {
  unsigned char ret = PDATA(pgm)->rxBuf[0];
  if (0x0D != ret) {
    pmsg_error("PICkit5 Bad Response: %i", ret);
    return -1;
  }
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


#if defined(HAVE_USB_H)
static int usb_fill_buf(const union filedescriptor *fd, int maxsize, int ep, int use_interrupt_xfer);
static int usb_fill_buf(const union filedescriptor *fd, int maxsize, int ep, int use_interrupt_xfer) {
  int rv;

  if (use_interrupt_xfer)
    rv = usb_interrupt_read(fd->usb.handle, ep, cx->usb_buf, maxsize, 10000);
  else
    rv = usb_bulk_read(fd->usb.handle, ep, cx->usb_buf, maxsize, 10000);
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
  int i, amnt;
  unsigned char  *p = buf;

  if (fd->usb.handle == NULL)
    return -1;

  for (i = 0; nbytes > 0;)
    {
      if (cx->usb_buflen <= cx->usb_bufptr)
	{
	  if (usb_fill_buf(fd, fd->usb.max_xfer, USB_PK5_DATA_READ_EP, fd->usb.use_interrupt_xfer) < 0)
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
  const unsigned char  *p = bp;
  int tx_size;

  if (fd->usb.handle == NULL)
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
      rv = usb_interrupt_write(fd->usb.handle, USB_PK5_DATA_WRITE_EP, (char *)bp, tx_size, 10000);
    else
      rv = usb_bulk_write(fd->usb.handle, USB_PK5_DATA_WRITE_EP, (char *)bp, tx_size, 10000);
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
#else 
 
// Allow compiling without throwing dozen errors
// We need libusb so we can access specific Endpoints.
// This does not seem to be possible with usbhid

static int usbdev_data_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes) {
  return -1;
}

static int usbdev_data_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen) {
  return -1;
}

#endif


#else /* defined(HAVE_LIBUSB) || defined(HAVE_LIBUSB_1_0) */
static int pickit5_nousb_open(PROGRAMMER *pgm, const char *name);

static int pickit5_nousb_open(PROGRAMMER *pgm, const char *name) {
  pmsg_error("no usb support; please compile again with libusb installed\n");

  return -1;
}

void pickit5_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "pickit5");

  pgm->open           = pickit5_nousb_open;
}

#endif /* defined(HAVE_LIBUSB) || defined(HAVE_LIBUSB_1_0) */

const char pickit5_desc[] = "Microchip's PICkit 5 Programmer/Debugger";
