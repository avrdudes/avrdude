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
#include <ctype.h>

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

#if defined(HAVE_USB_H) || defined(HAVE_LUSB0_USB_H)
#define USB_PK5_CMD_READ_EP   0x81
#define USB_PK5_CMD_WRITE_EP  0x02
#define USB_PK5_DATA_READ_EP  0x83
#define USB_PK5_DATA_WRITE_EP 0x04

#define USB_PK5_MAX_XFER    512 // That's the size Pickit reports

#define CHECK_ERROR        0x01
#define BIST_TEST          0x02
#define BIST_RESULT        0x03

#define PK_OP_NONE         0x00 // Init
#define PK_OP_FOUND        0x01 // PK is connected to USB
#define PK_OP_RESPONDS     0x02 // Responds to get_fw() requests
#define PK_OP_READY        0x03 // Voltage Set, Clock Set

#define POWER_SOURCE_EXT   0x00
#define POWER_SOURCE_INT   0x01
#define POWER_SOURCE_NONE  0x02

#define ERROR_USB_SEND              (LIBAVRDUDE_BEYOND_ERRS - 1) // Start at 16 to avoid collisions
#define ERROR_USB_RECV              (LIBAVRDUDE_BEYOND_ERRS - 2)
#define ERROR_SCRIPT_PARAM_SIZE     (LIBAVRDUDE_BEYOND_ERRS - 3)
#define ERROR_BAD_RESPONSE          (LIBAVRDUDE_BEYOND_ERRS - 4)
#define ERROR_SCRIPT_DEVICE_LOCKED  (LIBAVRDUDE_BEYOND_ERRS - 5)
#define ERROR_SCRIPT_EXECUTION      (LIBAVRDUDE_BEYOND_ERRS - 6)


#define can_power_target(pgm) (!!(pgm->extra_features & HAS_VTARG_ADJ))
#define can_gen_hv_pulse(pgm) (!!(pgm->extra_features & HAS_VTARG_ADJ))  // Currently, with the 4
#define can_do_ptg(pgm)       (can_power_target(pgm)) //  supported ICDs, it is enough to figure this out

// Private data for this programmer
struct pdata {
  unsigned char pk_op_mode;     // See PK_OP_ defines
  unsigned char power_source;   // 0: external / 1: from PICkit / 2: ignore check
  unsigned char hvupdi_enabled; // 0: no HV / 1: HV generation enabled
  unsigned char keep_power;     // 0: No power on exit / 1: Keeps supplying power on exit
  double target_voltage;        // Voltage to supply to target

  double measured_vcc;          // This and below for print_params()
  unsigned int measured_current;
  unsigned int actual_pgm_clk;

  unsigned char nvm_version;    // Used to determine the offset for SIGROW/DevID on UPDI

  unsigned char dW_switched_isp;  // For debugWIRE: Flag to indicate we switch to ISP
  unsigned char target_locked;    // Avoid additional "program_enable" when doing chip erase

  unsigned char devID[4];       // Last byte has the Chip Revision of the target
  unsigned char app_version[3]; // Buffer for display() sent by get_fw()
  unsigned char fw_info[16];    // Buffer for display() sent by get_fw()
  unsigned char sernum_string[20]; // Buffer for display() sent by get_fw()
  char sib_string[32];
  unsigned char prodsig[256];   // Buffer for Prodsig that contains more then one memory
  unsigned int prod_sig_len;    // Length of read prodsig (to know if it got  filled)
  unsigned char txBuf[2048];    // Buffer for transfers
  unsigned char rxBuf[2048];    // 2048 because of WriteEEmem_dw with 1728 bytes length
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
static int pickit5_read_chip_rev(const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev);
static int pickit5_read_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);
static int pickit5_write_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);

// UPDI-specific functions
static int pickit5_updi_init(const PROGRAMMER *pgm, const AVRPART *p, double v_target);
static int pickit5_updi_write_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char value);
static int pickit5_updi_read_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char *value);
static int pickit5_updi_read_sib(const PROGRAMMER *pgm, const AVRPART *p, char *sib);
static int pickit5_updi_write_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char value);
static int pickit5_updi_read_cs_reg(const PROGRAMMER *pgm, const AVRPART *p, unsigned int addr, unsigned char *value);

// ISP-specific
static int pickit5_isp_write_fuse(const PROGRAMMER *pgm, const AVRMEM *mem, unsigned char value);
static int pickit5_isp_read_fuse(const PROGRAMMER *pgm, const AVRMEM *mem, unsigned long addr, unsigned char *value);

// debugWIRE-specific
static int pickit5_dw_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char value);
static int pickit5_dw_read_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char *value);
static void pickit5_dw_switch_to_isp(const PROGRAMMER *pgm, const AVRPART *p);
static void pickit5_isp_switch_to_dw(const PROGRAMMER *pgm, const AVRPART *p);

// TPI-specific
static int pickit5_tpi_read(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);
static int pickit5_tpi_write(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);

// JTAG-Specific
static int pickit5_jtag_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char value);
static int pickit5_jtag_read_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char *value);

// PDI-Specific
static int pickit5_pdi_flash_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, int len, unsigned char *value);

// Extra functions
static int pickit5_get_fw_info(const PROGRAMMER *pgm);
static int pickit5_set_vtarget(const PROGRAMMER *pgm, double v);
static int pickit5_get_vtarget(const PROGRAMMER *pgm, double *v);
static int pickit5_set_ptg_mode(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod);


// Internal functions
inline static void pickit5_uint32_to_array(unsigned char *buf, uint32_t num);
inline static unsigned int pickit5_array_to_uint32(unsigned char *buf);
inline static void pickit5_create_payload_header(unsigned char *buf, unsigned int type,
  unsigned int msg_len, unsigned int transfer_len);
inline static void pickit5_create_script_header(unsigned char *buf, unsigned int arg_len, unsigned int script_len);
static const char *pickit5_error_to_str(unsigned int error_code);

static int pickit5_read_prodsig(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);

static int pickit5_get_status(const PROGRAMMER *pgm, unsigned char status);
static int pickit5_send_script(const PROGRAMMER *pgm, unsigned int script_type,
  const unsigned char *script, unsigned int script_len,
  const unsigned char *param, unsigned int param_len, unsigned int payload_len);
static int pickit5_send_script_done(const PROGRAMMER *pgm);
static int pickit5_read_response(const PROGRAMMER *pgm);
static int pickit5_send_script_cmd(const PROGRAMMER *pgm, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len);
static int pickit5_upload_data(const PROGRAMMER *pgm, const AVRPART *p, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len, unsigned char *recv_buf, unsigned int recv_len);
static int pickit5_download_data(const PROGRAMMER *pgm, const AVRPART *p, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len, unsigned char *send_buf, unsigned int send_len);


// Extra-USB related functions, because we need more then 2 endpoints
static int usbdev_bulk_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes);
static int usbdev_bulk_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen);
// Check if the device exists
static int usbdev_check_connected(unsigned int vid, unsigned int pid);

const char *pickit5_error_to_str(unsigned int error_code) {
  switch(error_code) {          // These strings were found in the depths of the IDE files (HEX Editor ftw!)
  case   0: return "NO_ERROR";
  case  16: return "DW_PHY_ERROR";
  case  17: return "JTAGM_INIT_ERROR";
  case  18: return "JTAGM_ERROR";
  case  19: return "JTAG_ERROR";
  case  20: return "JTAGM_VERSION";
  case  21: return "JTAGM_TIMEOUT";
  case  22: return "JTAG_BIT_BANGER_TIMEOUT";
  case  23: return "PARITY_ERROR";
  case  24: return "EB_ERROR";
  case  25: return "PDI_TIMEOUT";
  case  26: return "COLLISION";
  case  27: return "PDI_ENABLE";
  case  28: return "FRAMING_ERROR";
  case  29: return "DMA_ERROR";
  case  32: return "NO_DEVICE_FOUND";
  case  33: return "CLOCK_ERROR";
  case  34: return "NO_TARGET_POWER";
  case  35: return "NOT_ATTACHED";
  case  36: return "DAISY_CHAIN_TOO_LONG";
  case  37: return "DAISY_CHAIN_CONFIG";
  case  49: return "INVALID_PHYSICAL_STATE";
  case  50: return "ILLEGAL_STATE";
  case  51: return "INVALID_CONFIG";
  case  52: return "INVALID_MEMTYPE";
  case  53: return "INVALID_SIZE";
  case  54: return "INVALID_ADDRESS";
  case  55: return "INVALID_ALIGNMENT";
  case  56: return "ILLEGAL_MEMORY_RANGE";
  case  57: return "ILLEGAL_VALUE";
  case  58: return "ILLEGAL_ID";
  case  59: return "INVALID_CLOCK_SPEED";
  case  60: return "TIMEOUT";
  case  61: return "ILLEGAL_OCD_STATUS";
  case  64: return "NVM_ENABLE";
  case  65: return "NVM_DISABLE";
  case  66: return "CS_ERROR";
  case  67: return "CRC_FAILURE";
  case  68: return "OCD_LOCKED";
  case  69: return "KEY_ERROR";
  case  70: return "BOOT_ERROR";
  case  71: return "ACK_ERROR";
  case  80: return "NO_OCD_CONTROL";
  // case 81: return "PLEASE_TOGGLE_POWER";  // Original, but the one below might help better
  case  81: return "NO_RESPONSE_CHECK_CONNECTIONS";
  case  82: return "NO_VOUT_SET";
  case  83: return "VOUT_ERROR";
  case  84: return "VTG_TOO_LOW_FOR_FEATURE";
  case  96: return "PC_READ_FAILED";
  case  97: return "REGISTER_READ_FAILED";
  case 112: return "READ_ERROR";
  case 113: return "WRITE_ERROR";
  case 114: return "WRITE_TIMEOUT";
  case 144: return "NOT_SUPPORTED";
  case 145: return "NOT_IMPLEMENTED";
  default:  return "UNKNOWN";
  }
}

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
  unsigned int msg_len, unsigned int transfer_len) {
  pickit5_uint32_to_array(&buf[0], type);
  pickit5_uint32_to_array(&buf[4], 0);
  pickit5_uint32_to_array(&buf[8], msg_len);
  pickit5_uint32_to_array(&buf[12], transfer_len);
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

static int pickit5_parseexitspecs(PROGRAMMER *pgm, const char *sp) {
  char *cp, *s, *str = mmt_strdup(sp);
  int rv = 0;
  bool help = false;

  s = str;
  while((cp = strtok(s, ","))) {
    s = NULL;
    if(str_eq(cp, "vcc")) {
      if(!can_power_target(pgm)) {
        pmsg_warning("-E vcc setting detected but programmer can not provide power, continuing\n");
        continue;
      }
      my.keep_power = 0x01;
      continue;
    }
    if(str_eq(cp, "help")) {
      help = true;
      rv = LIBAVRDUDE_EXIT_OK;
    }

    if(!help) {
      pmsg_error("invalid exitspec parameter -E %s\n", cp);
      rv = -1;
    }
    msg_error("%s -c %s exitspec parameter options:\n", progname, pgmid);
    if(can_power_target(pgm))
      msg_error("  -E vcc     Programmer will continue to provide power after the session ended\n");
    msg_error("  -E help    Show this help menu and exit\n");
    mmt_free(str);
    return rv;
  }

  mmt_free(str);
  return rv;
}

static int pickit5_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  LNODEID ln;
  const char *extended_param;
  int rv = 0;

  for(ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if(str_starts(extended_param, "vtarg=")) {
      double voltage = -1.0;
      if(!can_power_target(pgm)) {
        pmsg_warning("-x vtarg setting detected but programmer can not provide power, continuing\n");
        continue;
      }
      if(sscanf(extended_param, "vtarg=%lf", &voltage) != 1) {
        pmsg_error("invalid voltage parameter %s\n", extended_param);
        rv = -1;
        continue;
      }
      if(voltage < 0.1 && voltage > -1.0) {
        my.power_source = POWER_SOURCE_NONE; // Voltage check disabled
        continue;
      }
      if(voltage < 1.8 || voltage > 5.5) {
        pmsg_error("voltage %1.1lf V outside valid range [1.8 V, 5.5 V]\n", voltage);
        rv = -1;
        continue;
      }
      my.power_source = POWER_SOURCE_INT;  // PK supplies power
      my.target_voltage = voltage;
      continue;
    }
    if(str_starts(extended_param, "hvupdi")) {
      if(can_gen_hv_pulse(pgm))
        for(LNODEID ln = lfirst(pgm->hvupdi_support); ln; ln = lnext(ln)) {
          my.hvupdi_enabled |= 1 << *(unsigned char *)ldata(ln);
        }
      else
        msg_warning("HV pulse requested but programmer doesn't support it, continuing\n");
      continue;
    }

    if(str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      if(can_power_target(pgm))
        msg_error("  -x vtarg=<dbl>  Enable power output; <dbl> must be in [1.8, 5.5] V\n");
      if(can_gen_hv_pulse(pgm))
        msg_error("  -x hvupdi       Enable high-voltage UPDI initialization\n");
      msg_error("  -x help         Show this help menu and exit\n");
      return LIBAVRDUDE_EXIT_OK;
    }

    pmsg_error("invalid extended parameter: %s\n", extended_param);
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

  if(script == NULL) {
    pmsg_error("invalid script pointer passed\n");
    return LIBAVRDUDE_EXIT_FAIL; // If script is NULL there is a significant problem
  }

  unsigned int header_len = 16 + 8;     // Header info + script header
  unsigned int preamble_len = header_len + param_len;
  unsigned int message_len = preamble_len + script_len;
  pmsg_debug("%s(scr_len: %u, param_len: %u, data_len: %u)\n", __func__, script_len, param_len, payload_len);

  if(message_len >= 2048) {     // Required memory will exceed buffer size, abort
    pmsg_error("requested message size (%u) too large\n", message_len);
    return ERROR_SCRIPT_PARAM_SIZE;     // 2 kB should be enough for everything
  }
  unsigned char *buf = my.txBuf;

  pickit5_create_payload_header(&buf[0], script_type, message_len, payload_len);
  pickit5_create_script_header(&buf[16], param_len, script_len);

  if(param != NULL)
    memcpy(&buf[24], param, param_len);

  memcpy(&buf[preamble_len], script, script_len);

  if(serial_send(&pgm->fd, buf, message_len) < 0) {
    return LIBAVRDUDE_GENERAL_FAILURE;
  }
  return LIBAVRDUDE_SUCCESS;
}

static int pickit5_read_response(const PROGRAMMER *pgm) {
  unsigned char *buf = my.rxBuf;

  if(serial_recv(&pgm->fd, buf, 512) < 0) {
    pmsg_error("reading from programmer failed\n");
    return ERROR_USB_RECV;
  }
  unsigned int status = pickit5_array_to_uint32(&buf[0]);
  unsigned int error_code = pickit5_array_to_uint32(&buf[16]);

  if(status != 0x0D) {
    pmsg_error("unexpected read response: 0x%2X\n", status);
    return ERROR_BAD_RESPONSE;
  }

  int rc = LIBAVRDUDE_SUCCESS;
  if(error_code == 0x44) {
    my.target_locked = 0x01;
    rc = LIBAVRDUDE_DEVICE_LOCKED;
  } else if(error_code == 0x51) {
    if(is_updi(pgm))
      pmsg_error("failed to start session; reason might be: no power, bad connection or missing HV pulse.");
    else
      pmsg_error("failed to start session; reason might be: no power or bad connection");
    rc = LIBAVRDUDE_GENERAL_FAILURE;
  } else if(error_code != 0x00) {
    pmsg_error("script error returned: 0x%2X - %s\n", error_code, pickit5_error_to_str(error_code));
    rc = ERROR_SCRIPT_EXECUTION;
  }
  return rc;
}

/*
  Terminates the data stream over the data endpoint
*/
static int pickit5_send_script_done(const PROGRAMMER *pgm) {
  unsigned char script_done[16];
  unsigned int script_done_type = 0x0103;

  pickit5_create_payload_header(script_done, script_done_type, 16, 0);
  if(serial_send(&pgm->fd, script_done, 16) < 0) {
    pmsg_error("failed sending script done message\n");
    return ERROR_USB_SEND;
  }
  return pickit5_read_response(pgm);
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
    pmsg_error("unknown key type %d passed to %s()\n", status, __func__);
    return -1;
  }
  unsigned int msg_len = 16 + key_len;

  pickit5_create_payload_header(buf, type, msg_len, 0);
  serial_send(&pgm->fd, buf, msg_len);
  serial_recv(&pgm->fd, my.rxBuf, 512);

  if(0x0D != my.rxBuf[0]) {
    pmsg_error("bad response from programmer: %i\n", my.rxBuf[0]);
    return -1;
  }

  unsigned int status_len = pickit5_array_to_uint32(&(my.rxBuf[8]));

  if(status_len > 64)
    status_len = 64;
  my.rxBuf[16 + status_len] = 0x00;    // Known zero-terminator
  if(str_starts((const char *) &(my.rxBuf[16]), "NONE") == 0) {
    pmsg_error("programmer's status report: %s\n", buf);
    return -1;
  }
  return 0;
}


/*
  Sends a script without any data on the data endpoint
*/
static int pickit5_send_script_cmd(const PROGRAMMER *pgm, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len) {
  pmsg_debug("%s()\n", __func__);
  int rc = pickit5_send_script(pgm, SCR_CMD, scr, scr_len, param, param_len, 0);
  if(rc == LIBAVRDUDE_SUCCESS) {
    return pickit5_read_response(pgm);
  }
  return rc;
}

/*
  Sends a script and sends the send_buf array over the data endpoint
*/
static int pickit5_download_data(const PROGRAMMER *pgm, const AVRPART *p, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len, unsigned char *send_buf, unsigned int send_len) {

  if(pickit5_send_script(pgm, SCR_DOWNLOAD, scr, scr_len, param, param_len, send_len) < 0) {
    pmsg_error("sending script with download failed\n");
    return -1;
  }
  if(pickit5_read_response(pgm) < 0)
    return -2;
  if(usbdev_bulk_send(&pgm->fd, send_buf, send_len) < 0) {
    pmsg_error("transmission failed on the data channel\n");
    if(pickit5_send_script_done(pgm) < 0) {
      pmsg_error("failed to abort download mode, please power-cycle the programmer and part\n");
      return -3;
    }
    pmsg_notice("attempting to recover from transmission error\n");
    if(pickit5_program_disable(pgm, p) < 0) {
      pmsg_error("failed to disable programming mode, please power-cycle the programmer and part\n");
      return -3;
    }
    if(pickit5_program_enable(pgm, p) < 0) {
      pmsg_error("failed to re-enable programming mode, please power-cycle the programmer and part\n");
      return -3;
    }
    pmsg_notice("successfully recovered from transmission error, please retry the previous operation\n");
    return -3;
  }
  if(pickit5_get_status(pgm, CHECK_ERROR) < 0) {
    pmsg_error("status check not 'NONE' on download\n");

    if(pickit5_send_script_done(pgm) < 0)
      pmsg_error("failed to abort download mode, please power-cycle the programmer and part\n");
    return -4;
  }
  if(pickit5_send_script_done(pgm) < 0) {
    pmsg_error("sending script done message failed\n");
    return -5;
  }
  return 0;
}

/*
  Sends a script and reads data from the data channel into recv_buf.
*/
static int pickit5_upload_data(const PROGRAMMER *pgm, const AVRPART *p, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len, unsigned char *recv_buf, unsigned int recv_len) {

  if(pickit5_send_script(pgm, SCR_UPLOAD, scr, scr_len, param, param_len, recv_len) < 0) {
    pmsg_error("sending script with upload failed\n");
    return -1;
  }
  if(pickit5_read_response(pgm) < 0) {
    if(pickit5_send_script_done(pgm) < 0) {
      pmsg_error("failed to abort upload mode, please power-cycle the programmer and part\n");
      return -2;
    }
    if(pickit5_program_disable(pgm, NULL) < 0) {
      pmsg_error("failed to disable programming mode, please power-cycle the programmer and part\n");
      return -2;
    }
    if(pickit5_program_enable(pgm, NULL) < 0) {
      pmsg_error("failed to re-enable programming mode, please power-cycle the programmer and part\n");
      return -2;
    }
    return -2;
  }
  if(usbdev_bulk_recv(&pgm->fd, recv_buf, recv_len) < 0) {
    pmsg_error("reading data memory failed\n");
    // return -3; // Do not abort here, try to send script done
  }
  if(pickit5_send_script_done(pgm) < 0) {
    pmsg_error("sending script done message failed\n");
    return -4;
  }
  return 0;
}

static int pickit5_open(PROGRAMMER *pgm, const char *port) {
  if(!pgm->cookie)              // Sanity
    return LIBAVRDUDE_GENERAL_FAILURE;
  pmsg_debug("%s(\"%s\")\n", __func__, port);
  union pinfo pinfo;
  LNODEID usbpid;
  int rv = LIBAVRDUDE_GENERAL_FAILURE;
  size_t serial_num_len = 0;

#if !defined(HAVE_LIBUSB)
  pmsg_error("need to be compiled with USB or HIDAPI support\n");
  return LIBAVRDUDE_GENERAL_FAILURE;
#endif

  if(!str_starts(port, "usb:") && !str_eq(port, "usb")) {
    pmsg_error("invalid -P %s; drop -P option or else use -P usb:<vid>:<pid> or -P usb:<serialno>\n", port);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }
  unsigned int new_vid = 0, new_pid = 0, setids = 0;
  const char *vidp, *pidp;

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
      setids = 1;
      if(vidp != pidp) {        // User specified an VID
        // First: Handle VID input
        if(sscanf(vidp, "%x", &new_vid) != 1) {
          pmsg_error("failed to parse -P VID input %s: expected hexadecimal number\n", vidp);
          return LIBAVRDUDE_GENERAL_FAILURE;
        }
      } else {                  // VID space empty: default to Microchip
        new_vid = USB_VENDOR_MICROCHIP;
      }

      // Now handle PID input
      if(sscanf(pidp + 1, "%x", &new_pid) != 1) {
        pmsg_error("failed to parse -P PID input %s: expected hexadecimal number\n", pidp+1);
        return LIBAVRDUDE_GENERAL_FAILURE;
      }

      pmsg_notice("overwriting VID:PID to %04x:%04x\n", new_vid, new_pid);
      port = "usb";             // Overwrite the string to avoid confusing the libusb
    } else {                    // pidp == NULL means vidp points to serial number
      serial_num_len = strlen(vidp);
    }
  }                             // vidp == NULL means dropped -P option or -P usb

  // If the config entry did not specify a USB PID, insert the default one
  if(lfirst(pgm->usbpid) == NULL)
    ladd(pgm->usbpid, (void *) USB_DEVICE_PICKIT5);

  pinfo.usbinfo.vid = pgm->usbvid? pgm->usbvid: USB_VENDOR_MICROCHIP;

  // PICkit 5 does not have support for HID, so no need to support it
  serdev = &usb_serdev;
  if(setids) {                  // In case a specific VID/PID was specified
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
  }

  if(rv >= 0)                   // If a programmer in PIC mode found, we're done
    return rv;

  // No known PID found, try to figure out if the device is connected in the wrong mode

  // The following piece of code is for PK4 and SNAP only, don't try to look for them
  // when we don't expect them, like with the Basic, that uses the id pickit_basic...
  const char *id = lget(pgm->id);
  if(str_starts(id, "pickit5")) {
    pinfo.usbinfo.vid = USB_VENDOR_ATMEL;
    pinfo.usbinfo.pid = USB_DEVICE_SNAP_AVR_MODE;

    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_3;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_3;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_3;
    pgm->fd.usb.eep = USBDEV_EVT_EP_READ_3;

    const char *pgm_suffix = strchr(pgmid, '_')? strchr(pgmid, '_'): "";
    char part_option[128] = {0};

    if(partdesc)
      snprintf(part_option, sizeof(part_option), "-p %s ", partdesc);

  // Use LIBHIDAPI to connect to SNAP/PICkit4 if present.
  // For some reason, chances are smaller to get
  // permission denied errors when using LIBHIDAPI
  #if defined(HAVE_LIBHIDAPI)
    serdev = &usbhid_serdev;
    pgm->fd.usb.eep = 0;
  #endif

    rv = serial_open(port, pinfo, &pgm->fd);  // Try SNAP PID

    if(rv >= 0) {
      msg_error("\n");
      cx->usb_access_error = 0;

      pmsg_error("MPLAB SNAP in AVR mode detected; to switch into MPLAB mode try\n");
      imsg_error("$ %s -c snap%s %s-P %s -x mode=mplab\n", progname, pgm_suffix, part_option, port);
      imsg_error("or use the programmer in AVR mode with the following command:\n");
      imsg_error("$ %s -c snap%s %s-P %s\n", progname, pgm_suffix, part_option, port);

      serial_close(&pgm->fd);
      return LIBAVRDUDE_EXIT_FAIL;
    }
    pinfo.usbinfo.pid = USB_DEVICE_PICKIT4_AVR_MODE;
    rv = serial_open(port, pinfo, &pgm->fd);  // Try PICkit4 PID

    if(rv >= 0) {
      msg_error("\n");
      cx->usb_access_error = 0;

      pmsg_error("PICkit 4 in AVR mode detected; to switch into MPLAB mode try\n");
      imsg_error("$ %s -c pickit4%s %s-P %s -x mode=mplab\n", progname, pgm_suffix, part_option, port);
      imsg_error("or use the programmer in AVR mode with the following command:\n");
      imsg_error("$ %s -c pickit4%s %s-P %s\n", progname, pgm_suffix, part_option, port);

      serial_close(&pgm->fd);
      return LIBAVRDUDE_EXIT_FAIL;
    }
    if(serial_num_len) {
      pmsg_error("no device found matching the specified serial number %s", vidp);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    pmsg_error("no device found matching VID 0x%04x and PID list: 0x%04x, 0x%04x, 0x%04x\n", USB_VENDOR_MICROCHIP,
      USB_DEVICE_PICKIT5, USB_DEVICE_PICKIT4_PIC_MODE, USB_DEVICE_SNAP_PIC_MODE);
    imsg_error("nor VID 0x%04x with PID list: 0x%04x, 0x%04x\n", USB_VENDOR_ATMEL, USB_DEVICE_PICKIT4_AVR_MODE, USB_DEVICE_SNAP_AVR_MODE);
    return LIBAVRDUDE_EXIT_FAIL;
  }

  if(str_starts(id, "pickit_basic")) {
    // Check if the Basic is in Bootloader Mode or CMSIS-DAP, should help trouble-shooting.
    rv = usbdev_check_connected(USB_VENDOR_MICROCHIP, USB_DEVICE_PICKIT_BASIC_CIMSIS_CDC);
    if(rv >= 0) {
      pmsg_error("PICkit Basic in CMSIS-DAP mode detected;\n");
      imsg_error("please use a Microchip tool to switch the firmware to \"mplab\"\n");
      imsg_error("in order to use the programmer with avrdude\n");
      return LIBAVRDUDE_EXIT_FAIL;
    }

    rv = usbdev_check_connected(USB_VENDOR_MICROCHIP, USB_DEVICE_PICKIT_BASIC_BL);
    if(rv >= 0) {
      pmsg_error("PICkit Basic in Bootloader mode detected;\n");
      imsg_error("please use a Microchip tool to load the \"mplab\" firmware\n");
      imsg_error("in order to use the programmer with avrdude\n");
      return LIBAVRDUDE_EXIT_FAIL;
    }
  }

  // Fall-back in case the user adds a custom programmer
  pmsg_error("no device found matching VID 0x%04x and PID list: ", (unsigned) pinfo.usbinfo.vid);
  int notfirst = 0;

  for(usbpid = lfirst(pgm->usbpid); usbpid; usbpid = lnext(usbpid)) {
    if(notfirst)
      msg_error(", ");
    msg_error("0x%04x", (unsigned int) (*(int *) (ldata(usbpid))));
    notfirst = 1;
  }
  return LIBAVRDUDE_EXIT_FAIL;
}

static void pickit5_close(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  if(!my.keep_power)            // Switches off PICkit voltage regulator if enabled
    pickit5_set_vtarget(pgm, 0.0);

  serial_close(&pgm->fd);
}

static void pickit5_disable(const PROGRAMMER *pgm) {
  return;
}

static void pickit5_enable(PROGRAMMER *pgm, const AVRPART *p) {
  // Overwrite page sizes so that avrdude uses pages read/writes
  // This will reduce overhead and increase speed
  AVRMEM *mem;

  if(is_updi(pgm)) {
    if((mem = avr_locate_sram(p)))
      mem->page_size = mem->size < 256? mem->size : 256;
    if((mem = avr_locate_eeprom(p)))
      mem->page_size = mem->size < 32? mem->size : 32;
    if((mem = avr_locate_sib(p))) { // This is mandatory as PICkit is reading all 32 bytes at once
      mem->page_size = 32;
      mem->readsize = 32;
    }
  }
  if(is_debugwire(pgm)) {
    if((mem = avr_locate_flash(p))) {
      mem->page_size = mem->size < 1024? mem->size : 1024; // The Flash Write function on DW needs 1600 bytes.
      mem->readsize = mem->size < 1024? mem->size : 1024;  // This reduces overhead and speeds things up
    }
  }
  if(is_isp(pgm)) {
    if((mem = avr_locate_flash(p))) {
      if(mem->mode != 0x04) {   // Don't change default flash settings on old AVRs
        mem->page_size = mem->size < 1024? mem->size : 1024;
        mem->readsize = mem->size < 1024? mem->size : 1024;
      } else {
        mem->page_size = 256;
        mem->readsize = 256;
        mem->blocksize = 256;
      }
    }
    if((mem = avr_locate_eeprom(p))) {
      if(mem->mode == 0x04) {   // Increasing minimal write/read length so that the old AVRs work with PK5
        mem->page_size = 0x04;
        mem->readsize = 0x04;
        mem->blocksize = 0x04;
      }
    }
    if((mem = avr_locate_calibration(p))) {
      if(mem->size == 1)    // Any 1 byte wide calibration is also in prodsig
        mem->offset = 1;    // Add an offset to profit of the prodsig buffering
    }
  }
  if(both_jtag(pgm, p)) {
    if((mem = avr_locate_flash(p))) {
      mem->page_size = mem->size < 512? mem->size : 512;
      mem->readsize = mem->size < 512? mem->size : 512;
    }
  }
  if(both_xmegajtag(pgm, p)) {    // True Page size is needed for a PDI fix, so don't increase them
    if((mem = avr_locate_flash(p))) {
      mem->page_size = mem->size < 1024? mem->size : 1024;
      mem->readsize = mem->size < 1024? mem->size : 1024;
    }
    if((mem = avr_locate_application(p))) {
      mem->page_size = mem->size < 1024? mem->size : 1024;
      mem->readsize = mem->size < 1024? mem->size : 1024;
    }
    if((mem = avr_locate_apptable(p))) {
      mem->page_size = mem->size < 1024? mem->size : 1024;
      mem->readsize = mem->size < 1024? mem->size : 1024;
    }
    if((mem = avr_locate_boot(p))) {
      mem->page_size = mem->size < 1024? mem->size : 1024;
      mem->readsize = mem->size < 1024? mem->size : 1024;
    }
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
  fmsg_out(fp, "Target programming clk: %u kHz\n", my.actual_pgm_clk / 1000);
  fmsg_out(fp, "Target vcc            : %1.2f V\n", my.measured_vcc);
  fmsg_out(fp, "Target current        : %3u mA\n", my.measured_current);
}

static int pickit5_updi_init(const PROGRAMMER *pgm, const AVRPART *p, double v_target) {
  int rc = pickit5_program_enable(pgm, p);
  if(rc < LIBAVRDUDE_SUCCESS) {
    return (rc < LIBAVRDUDE_BEYOND_ERRS)? LIBAVRDUDE_GENERAL_FAILURE : rc;
  }
  // Get SIB so we can get the NVM Version
  if(pickit5_updi_read_sib(pgm, p, my.sib_string) < 0) {
    pmsg_error("failed to obtain System Info Block\n");
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  if(pickit5_read_dev_id(pgm, p) < 0) {
    pmsg_error("failed to obtain device ID\n");
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  if(!(pgm->extra_features & HAS_BITCLOCK_ADJ))
    pmsg_warning("setting bitclock despite HAS_BITCLOCK_ADJ missing in pgm->extra_features\n");
  unsigned int baud = my.actual_pgm_clk;
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
      pickit5_set_sck_period(pgm, 1.0 / 100000); // Start with 100 kHz
      pickit5_updi_write_cs_reg(pgm, UPDI_ASI_CTRLA, 0x01); // Change UPDI clock to 16 MHz

      unsigned char ret_val = 0;
      pickit5_updi_read_cs_reg(pgm, p, UPDI_ASI_CTRLA, &ret_val);
      if(ret_val != 0x01) {
        pmsg_warning("failed to change UPDI clock, falling back to 225 kHz\n");
        baud = 225000;
      }
      // Possible speed optimization: Reduce Guard Time Value and maybe response signature?
    }
  }

  if(pickit5_set_sck_period(pgm, 1.0 / baud) >= 0) {
    pmsg_notice("UPDI speed set to %i kHz\n", baud / 1000);
    my.actual_pgm_clk = baud;
  } else {
    pmsg_warning("failed to set UPDI speed, continuing\n");
  }
  return LIBAVRDUDE_SUCCESS;
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
  unsigned int default_baud;

  if(both_debugwire(pgm, p)) {
    rc = get_pickit_dw_script(&(my.scripts), p->desc);
    default_baud = 125000;      // debugWIRE does not allow to select speed, this is for ISP mode
  } else if(both_isp(pgm, p)) {
    rc = get_pickit_isp_script(&(my.scripts), p->desc);
    default_baud = 125000;
  } else if(both_jtag(pgm, p) || both_xmegajtag(pgm, p)) {
    rc = get_pickit_jtag_script(&(my.scripts), p->desc);
    default_baud = 500000;
  } else if(both_updi(pgm, p)) {
    rc = get_pickit_updi_script(&(my.scripts), p->desc);
    default_baud = 200000;
  } else if(both_tpi(pgm, p)) {
    rc = get_pickit_tpi_script(&(my.scripts), p->desc);
    default_baud = 125000;
  } else if(both_pdi(pgm, p)) {
    rc = get_pickit_pdi_script(&(my.scripts), p->desc);
    default_baud = 500000;
  }

  if(rc == -1) {
    pmsg_error("no matching prog_mode found, aborting\n");
    return -1;
  }
  if(rc == -2) {
    pmsg_error("failed to match scripts to %s, aborting\n", p->desc);
    return -1;
  }
  pmsg_debug("found scripts at namepos %d", rc);

  my.target_locked = 0;

  if(my.hvupdi_enabled > 0) {
    if(p->hvupdi_variant == UPDI_ENABLE_HV_UPDI)
      pmsg_notice("high-voltage SYSCFG0 override on UPDI pin enabled\n");
    if(p->hvupdi_variant == UPDI_ENABLE_HV_RESET)
      pmsg_notice("high-voltage SYSCFG0 override on RST pin enabled\n");
  }

  if(my.pk_op_mode < PK_OP_RESPONDS) {
    if(pickit5_get_fw_info(pgm) < 0) // PK responds: we can try to enable voltage
      return -1;
    my.pk_op_mode = PK_OP_RESPONDS;
  }

  pickit5_set_ptg_mode(pgm, p);
  pickit5_set_vtarget(pgm, 0.0); // Avoid the edge case when avrdude was CTRL+C'd but still provides power

  // Now we try to figure out if we have to supply power from PICkit
  double v_target = 3.30; // Placeholder in case no VTARG Read

  if(pgm->extra_features & HAS_VTARG_READ) { // If not supported (PK Basic), use a place
    pickit5_get_vtarget(pgm, &v_target);
    if(v_target < 1.8) {
      if(my.power_source == POWER_SOURCE_NONE) {
        pmsg_warning("no external voltage detected but continuing anyway\n");
      } else if(my.power_source == POWER_SOURCE_INT) {
        pmsg_notice("no extenal voltage detected; trying to supply from programmer\n");
        if(both_xmegajtag(pgm, p) || both_pdi(pgm, p)) {
          if(my.target_voltage > 3.49) {
            pmsg_error("xMega part selected but requested voltage is over 3.49V, aborting");
            return -1;
          }
        }

        if(pickit5_set_vtarget(pgm, my.target_voltage) < 0)
          return -1;            // Set requested voltage

        if(pickit5_get_vtarget(pgm, &v_target) < 0)
          return -1;            // Verify voltage

        // Make sure the voltage is in our requested range. Due to voltage drop on
        // the LDO and on USB itself, the lower limit is capped at 4.4V
        double upper_limit = my.target_voltage + 0.2;
        double lower_limit = my.target_voltage - 0.3;
        if(lower_limit > 4.4)
          lower_limit = 4.4;
        if((v_target < lower_limit) || (v_target > upper_limit)) {
          pmsg_error("target voltage (%1.2fV) is outside of allowed range, aborting\n", v_target);
          return -1;
        }
      } else {
        pmsg_error("no external voltage detected, aborting; overwrite this check with -x vtarg=0\n");
        return -1;
      }
    } else {
      my.power_source = POWER_SOURCE_EXT; // Overwrite user input
      pmsg_notice("external voltage detected: will not supply power\n");
    }
  }

  my.pk_op_mode = PK_OP_READY;
  my.dW_switched_isp = 0;

  if(pgm->baudrate && pgm->bitclock)
    pmsg_warning("both -b baudrate and -B bitclock given; using -b setting\n");

  if(!(pgm->extra_features & HAS_BITCLOCK_ADJ))
    pmsg_warning("setting bitclock despite HAS_BITCLOCK_ADJ missing in pgm->extra_features\n");

  my.actual_pgm_clk =
    pgm->baudrate? (unsigned int) pgm->baudrate:
    pgm->bitclock? (unsigned int) (1.0 / pgm->bitclock): // pgm->bitclock in seconds
    default_baud;

  if(is_updi(pgm)) {            // UPDI got it's own init as it is well enough documented to select
    return pickit5_updi_init(pgm, p, v_target); // the CLKDIV based on the voltage and requested baud
  }

  // JTAG __requires__ setting the speed before program enable
  pickit5_set_sck_period(pgm, 1.0 / my.actual_pgm_clk);
  if(pickit5_program_enable(pgm, p) < 0) {
    pmsg_error("failed to enable programming mode\n");
    return -1;
  }
  if(pickit5_read_dev_id(pgm, p) < 0) {
    pmsg_error("failed to obtain device ID\n");
    return -1;
  }

  return 0;
}

static int pickit5_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  return -2;
}

static int pickit5_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  (void) p; // p is unused but required by libavrdude.h
  pmsg_debug("%s()\n", __func__);
  const unsigned char *enter_prog = my.scripts.EnterProgMode;
  unsigned int enter_prog_len = my.scripts.EnterProgMode_len;

  if(my.hvupdi_enabled && can_gen_hv_pulse(pgm)) { // SNAP and Basic have no HV generation
    if(p->hvupdi_variant == UPDI_ENABLE_HV_UPDI) { // High voltage generation on UPDI line
      enter_prog = my.scripts.EnterProgModeHvSp;
      enter_prog_len = my.scripts.EnterProgModeHvSp_len;
    } else if(p->hvupdi_variant == UPDI_ENABLE_HV_RESET ||  // High voltage generation on RST line
              p->hvupdi_variant == UPDI_ENABLE_RESET_HS) {  // Handshake for SD-Family
      enter_prog = my.scripts.EnterProgModeHvSpRst;
      enter_prog_len = my.scripts.EnterProgModeHvSpRst_len;
    }
  }
  if(my.pk_op_mode == PK_OP_READY) {
    return pickit5_send_script_cmd(pgm, enter_prog, enter_prog_len, NULL, 0);
  }
  return LIBAVRDUDE_SUCCESS;
}

static int pickit5_program_disable(const PROGRAMMER *pgm, const AVRPART *p) {
  (void) p; // p is unused but required by libavrdude.h
  pmsg_debug("%s()\n", __func__);
  const unsigned char *exit_prog = my.scripts.ExitProgMode;
  unsigned int exit_prog_len = my.scripts.ExitProgMode_len;

  if(my.pk_op_mode == PK_OP_READY) {
    return pickit5_send_script_cmd(pgm, exit_prog, exit_prog_len, NULL, 0);
  }
  return 0;
}

static int pickit5_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);

  if(!my.target_locked)             // Avoid function call to avoid error printing (might be confusing)
    pickit5_program_enable(pgm, p); // Ignore any errors that might come up

  if(is_debugwire(pgm))             // debugWire Chip erase doesn't seem to be working, use ISP
    pickit5_dw_switch_to_isp(pgm, p);

  const unsigned char *chip_erase = my.scripts.EraseChip;
  unsigned int chip_erase_len = my.scripts.EraseChip_len;

  if(pickit5_send_script_cmd(pgm, chip_erase, chip_erase_len, NULL, 0) >= 0) {
    if(pickit5_array_to_uint32(&(my.rxBuf[16])) == 0x00) {
      pmsg_info("target successfully erased\n");
      my.pk_op_mode = PK_OP_READY;
      pickit5_program_enable(pgm, p);
      return LIBAVRDUDE_SUCCESS;
    }
  }

  pmsg_error("chip erase failed\n");
  return LIBAVRDUDE_GENERAL_FAILURE;
}

static int pickit5_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {

  return pickit5_read_array(pgm, p, mem, address, n_bytes, &mem->buf[address]);
}

static int pickit5_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {

  return pickit5_write_array(pgm, p, mem, address, n_bytes, &mem->buf[address]);
}

// Sets Frequency in kHz
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod) {
  pmsg_debug("%s()\n", __func__);
  double frq = (0.001 / sckperiod) + 0.5; // 1 ms/period = kHz; round up
  const unsigned char *set_speed = my.scripts.SetSpeed;
  unsigned int set_speed_len = my.scripts.SetSpeed_len;
  unsigned char buf[4];
  if(set_speed == NULL)         // debugWIRE has no set speed, just return success
    return 0;

  pickit5_uint32_to_array(buf, frq);

  int rc = pickit5_send_script_cmd(pgm, set_speed, set_speed_len, buf, 4);
  if(rc != LIBAVRDUDE_SUCCESS)
    pmsg_error("failed to set speed\n");
  return rc;
}

static int pickit5_write_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char value) {
  int rc = 0;
  if(mem_is_a_fuse(mem)) {
    if(is_isp(pgm))
      rc = pickit5_isp_write_fuse(pgm, mem, value);
    else if(is_debugwire(pgm))
      rc = pickit5_dw_write_fuse(pgm, p, mem, value);
    else if(both_jtag(pgm, p))
      rc = pickit5_jtag_write_fuse(pgm, p, mem, value);
  }
  if(rc == 0)
    rc = pickit5_write_array(pgm, p, mem, addr, 1, &value);

  if(rc < 0)
    return rc;
  return 0;
}

static int pickit5_read_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char *value) {
  int rc = 0;
  if(mem_is_signature(mem)) {
    if(addr == 0)
      pickit5_read_dev_id(pgm, p);
    if(addr < 4) {
      *value = my.devID[addr];
      rc = 1;
    } else {
      rc = -1;
    }
  } else if(mem_is_a_fuse(mem)) {
    if(is_isp(pgm))
      rc = pickit5_isp_read_fuse(pgm, mem, addr, value);
    else if(is_debugwire(pgm))
      rc = pickit5_dw_read_fuse(pgm, p, mem, value);
    else if(both_jtag(pgm, p))
      rc = pickit5_jtag_read_fuse(pgm, p, mem, value);
  } else if(mem_is_in_sigrow(mem) || mem_is_calibration(mem)) { // For some weird reason this OR is needed?
    rc = pickit5_read_prodsig(pgm, p, mem, addr, 1, value);
  }
  if(rc == 0)
    rc = pickit5_read_array(pgm, p, mem, addr, 1, value);

  return rc < 0? rc: 0;
}

// UPDI Specific function providing a reduced overhead when writing a single byte
static int pickit5_updi_write_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char value) {

  if(mem->size < 1 || addr > (unsigned long) mem->size) {
    pmsg_error("address %i out of range for %s [0, %i]\n", (unsigned) addr, mem->desc, mem->size);
    return -1;
  }
  addr += mem->offset;
  pmsg_debug("%s(addr: 0x%4X, value: %i)\n", __func__, (unsigned) addr, value);

  // This script is based on WriteCSreg; reduces overhead by avoiding writing data EP
  unsigned char write8_fast[] = {
    0x90, 0x00, addr, (addr >> 8), 0x00, 0x00, // Place address in r0
    0x9B, 0x01, value,                         // Place value in r1
    0x1E, 0x06, 0x00, 0x01,                    // Store to address in reg 0 the byte in reg 1
  };

  int rc = pickit5_send_script_cmd(pgm, write8_fast, sizeof(write8_fast), NULL, 0);

  return rc < 0? -1: 1;
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
    pmsg_debug("%s(addr: 0x%4X)\n", __func__, (unsigned int) addr);

    unsigned char read8_fast[] = {
      0x90, 0x00, addr, (addr >> 8), 0x00, 0x00, // Load address (only 16-bit wide)
      0x1E, 0x03, 0x00,                          // Load byte from address in reg 0
      0x9F                                       // Send data from 0x1E to "response"
    };

    int rc = pickit5_send_script_cmd(pgm, read8_fast, sizeof(read8_fast), NULL, 0);
    if(rc < 0)
      return -1;
    *value = my.rxBuf[24];
    return 1;
  }
  return 0;
  /*else {                      // Fall back to standard function
    int rc = pickit5_read_array(pgm, p, mem, addr, 1, value);

    return rc < 0? rc: 1;
  }
  */
}

/* a little workaround function to write application and boot from one function */
static int pickit5_pdi_flash_write(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s\n", __func__);
  unsigned short page_size = mem->page_size;
  if(len % page_size != 0) { // Sanity
    pmsg_error("length %i is not a multiple of page size %i, aborting.\n", len, page_size);
    return -1;
  }

  unsigned char flash_cmd[] = {
    0x91, 0x00,                          // Load script paramter to r00
    0x91, 0x01,                          // Load script paramter to r01
    0x90, 0x04, 0xCA, 0x01, 0x00, 0x01,  // Set r04 to value 0x10001CA  (NVM CMD)
    0x90, 0x05, 0xC4, 0x01, 0x00, 0x01,  // Set r05 to value 0x10001C4  (NVM Data)
    0x90, 0x06, 0xCF, 0x01, 0x00, 0x01,  // Set r06 to value 0x10001CF  (NVM Status)
    0x9B, 0x07, 0x23,                    // Set r07 to value 0x23       (NVM Load Page Buffer)
    0x9B, 0x08, 0x2F,                    // Set r08 to value 0x2F       (NVM Erase and write flash page)
    0x9B, 0x09, 0xFF,                    // Set r09 to value 0xFF       (Dummy Flash write value)
    0x9C, 0x0A, page_size, (page_size >> 8),  // Load word (page size) to r09

    0x1E, 0x03, 0x04,                    // Load byte from NVM Command register (r04)
    0x6C, 0x0B,                          // Move temp_reg to r11
    0x1E, 0x03, 0x05,                    // Load byte from NVM Data register (r05)
    0x6C, 0x0C,                          // Move temp_reg to r12

    0x60, 0x03, 0x01,                    // Copy r01 to r03
    0x93, 0x03, page_size, (page_size >> 8),  // Integer divide r03 by page size
    0xAD, 0x03,                          // while (r03 --) {

    0x1E, 0x06, 0x04, 0x07,              // Load "load page command" to NVM Cmd Reg
    0x1E, 0x09, 0x00,                    // Set pointer for indirect addressing to r00
    0x1E, 0x10, 0x0A,                    // Set repeat counter to number in r09
    0x1E, 0x0A, 0x0A,                    // Read from data stream and send it to the device

    0x1E, 0x06, 0x04, 0x08,              // Load "Erase and write flash page" command into NVM Cmd buffer
    0x1E, 0x06, 0x00, 0x09,              // Triger NVM Cmd by writing to the first address (r0, 0xFF)
    0xA2,                                // Do {
    0x1E, 0x03, 0x06,                    // Check status reg (r06)
    0xA5, 0x80, 0x00, 0x00, 0x00,        // } while ((status reg & 0x80 != 0x00))
    0x00, 0x00, 0x00, 0x00, 0x64, 0x00,  // for at most 100 times
    0xAE,                                // } (End of Loop)

    0x1E, 0x06, 0x05, 0x0C,              // Store byte in r12 to SRAM Address in r05
    0x1E, 0x06, 0x04, 0x0B,              // Store byte in r11 to SRAM Address in r04
    0x92, 0x00, page_size, (page_size >> 8), 0x00, 0x00,  // Increment r00 by page size
    0x5A,                                // Set Error Status
  };

  addr += mem->offset;

  unsigned char param[8];
  pickit5_uint32_to_array(&param[0], addr);
  pickit5_uint32_to_array(&param[4], len);

  int rc = pickit5_download_data(pgm, p, flash_cmd, sizeof(flash_cmd), param, sizeof(param), value, len);
  return rc < 0? LIBAVRDUDE_EXIT_FAIL: rc;
}

// Return numbers of byte written
static int pickit5_write_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s(%s, addr: 0x%04x, offset: %i, len: %i)\n", __func__, mem->desc, (unsigned int) addr, mem->offset, len);

  if(len > mem->size || mem->size < 1) {
    pmsg_error("cannot write to %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  }
  if(addr >= (unsigned long) mem->size) {
    pmsg_error("cannot write to %s %s as address 0x%04lx is not in range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size - 1);
    return -1;
  }

  if(is_debugwire(pgm) && !mem_is_in_flash(mem)) // For flash programming, stay in ISP mode
    pickit5_isp_switch_to_dw(pgm, p);
  if(is_tpi(pgm))
    return pickit5_tpi_write(pgm, p, mem, addr, len, value);
  if(is_pdi(pgm) && mem_is_in_flash(mem))
    return pickit5_pdi_flash_write(pgm, p, mem, addr, len, value);

  const unsigned char *write_bytes = NULL;
  unsigned int write_bytes_len = 0;

  if((mem_is_in_flash(mem) && (len == mem->page_size))) {
    write_bytes     = my.scripts.WriteProgmem;
    write_bytes_len = my.scripts.WriteProgmem_len;
  } else if(mem_is_boot(mem) && my.scripts.WriteBootMem != NULL) {
    write_bytes     = my.scripts.WriteBootMem;
    write_bytes_len = my.scripts.WriteBootMem_len;
  } else if(mem_is_eeprom(mem) && my.scripts.WriteDataEEmem != NULL) {
    write_bytes     = my.scripts.WriteDataEEmem;
    write_bytes_len = my.scripts.WriteDataEEmem_len;
  } else if((mem_is_a_fuse(mem) || mem_is_in_fuses(mem)) && my.scripts.WriteConfigmemFuse != NULL) {
    write_bytes     = my.scripts.WriteConfigmemFuse;
    write_bytes_len = my.scripts.WriteConfigmemFuse_len;
  } else if(mem_is_lock(mem) && my.scripts.WriteConfigmemLock != NULL) {
    write_bytes     = my.scripts.WriteConfigmemLock;
    write_bytes_len = my.scripts.WriteConfigmemLock_len;
  } else if(mem_is_user_type(mem) && my.scripts.WriteIDmem != NULL) {
    write_bytes     = my.scripts.WriteIDmem;
    write_bytes_len = my.scripts.WriteIDmem_len;
  } else if(!mem_is_readonly(mem)) { // SRAM, IO, LOCK
    if((len == 1) && is_updi(pgm))
      return pickit5_updi_write_byte(pgm, p, mem, addr, value[0]);
    write_bytes = my.scripts.WriteMem8;
    write_bytes_len = my.scripts.WriteMem8_len;
  } else {
    pmsg_error("unsupported memory %s\n", mem->desc);
    return -2;
  }


  addr += mem->offset;
  if(both_jtag(pgm, p) && mem_is_in_flash(mem))
    addr /= 2;

  unsigned char param[8];
  pickit5_uint32_to_array(&param[0], addr);
  pickit5_uint32_to_array(&param[4], len);

  int rc = pickit5_download_data(pgm, p, write_bytes, write_bytes_len, param, 8, value, len);
  if(rc < 0)                    // Any error here means that a write fail occured, so restart
    return LIBAVRDUDE_EXIT_FAIL;
  return len;
}

// Return numbers of byte read
static int pickit5_read_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s(%s, addr: 0x%04x, offset: %i, len: %i)\n", __func__, mem->desc, (unsigned int) addr, mem->offset, len);


  if(len > mem->size || mem->size < 1) {
    pmsg_error("cannot read from %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  }
  if(addr >= (unsigned long) mem->size) {
    pmsg_error("cannot read from %s %s as address 0x%04lx is not in range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size - 1);
    return -1;
  }

  if(mem_is_signature(mem)) { // DeviceID is read only once and buffered
    if(addr == 0)
      pickit5_read_dev_id(pgm, p);
    if(len == 1) {
      *value = my.devID[addr];
      return 0;
    }
    return -1;
  }

  if(is_debugwire(pgm))
    pickit5_isp_switch_to_dw(pgm, p);
  if(is_tpi(pgm))
    return pickit5_tpi_read(pgm, p, mem, addr, len, value);

  const unsigned char *read_bytes = NULL;
  unsigned int read_bytes_len = 0;

  if(mem_is_in_flash(mem)) {
    read_bytes      = my.scripts.ReadProgmem;
    read_bytes_len  = my.scripts.ReadProgmem_len;
  } else if(mem_is_boot(mem) && my.scripts.ReadBootMem != NULL) {
    read_bytes     = my.scripts.ReadBootMem;
    read_bytes_len = my.scripts.ReadBootMem_len;
  } else if(mem_is_calibration(mem) && my.scripts.ReadCalibrationByte != NULL) {
    read_bytes      = my.scripts.ReadCalibrationByte;
    read_bytes_len  = my.scripts.ReadCalibrationByte_len;
  } else if(mem_is_eeprom(mem) && my.scripts.ReadDataEEmem != NULL) {
    read_bytes      = my.scripts.ReadDataEEmem;
    read_bytes_len  = my.scripts.ReadDataEEmem_len;
  } else if((mem_is_a_fuse(mem) || mem_is_in_fuses(mem)) && my.scripts.ReadConfigmemFuse != NULL) {
    read_bytes      = my.scripts.ReadConfigmemFuse;
    read_bytes_len  = my.scripts.ReadConfigmemFuse_len;
  } else if(mem_is_lock(mem) && my.scripts.ReadConfigmemLock != NULL) {
    read_bytes      = my.scripts.ReadConfigmemLock;
    read_bytes_len  = my.scripts.ReadConfigmemLock_len;
  } else if(mem_is_user_type(mem) && my.scripts.ReadIDmem != NULL) {
    read_bytes      = my.scripts.ReadIDmem;
    read_bytes_len  = my.scripts.ReadIDmem_len;
  } else if(mem_is_sib(mem)) {
    if(len == 1) {
      *value = my.sib_string[addr];
      return 0;
    }
    if(len == 32) {
      memcpy(value, my.sib_string, 32);
      return 32;
    }
    return -1;
  } else if((mem_is_in_sigrow(mem) || mem_is_user_type(mem)) && my.scripts.ReadConfigmem != NULL) {
    read_bytes      = my.scripts.ReadConfigmem;
    read_bytes_len  = my.scripts.ReadConfigmem_len;
  } else if(!mem_is_readonly(mem)) { // SRAM, IO, LOCK, USERROW
    if((len == 1) && is_updi(pgm)) {
      if(pickit5_updi_read_byte(pgm, p, mem, addr, value) < 0)
        return -1;
      return 0;
    }
    read_bytes      = my.scripts.ReadMem8;
    read_bytes_len  = my.scripts.ReadMem8_len;
  } else {
    pmsg_error("unsupported memory %s\n", mem->desc);
    return -2;
  }

  addr += mem->offset;
  if(both_jtag(pgm, p) && mem_is_in_flash(mem))
    addr /= 2;
  unsigned char param[8];

  pickit5_uint32_to_array(&param[0], addr);
  pickit5_uint32_to_array(&param[4], len);

  int rc = pickit5_upload_data(pgm, p, read_bytes, read_bytes_len, param, 8, value, len);
  if(rc < 0)                    // Any error here means that a read fail occured, better restart
    return LIBAVRDUDE_EXIT_FAIL;
  return len;
}

static int pickit5_read_dev_id(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *read_id = my.scripts.GetDeviceID; // Defaults
  unsigned int read_id_len = my.scripts.GetDeviceID_len;

  if(is_updi(pgm)) {
    if(my.nvm_version >= '0' && my.nvm_version <= '9')
      read_id = get_devid_script_by_nvm_ver(my.nvm_version); // Only address changes, not length
  } else if(is_debugwire(pgm)) {
    unsigned char scr [] = {0x7D, 0x00, 0x00, 0x00};  // Not sure what this does
    unsigned int scr_len = sizeof(scr);
    pickit5_send_script_cmd(pgm, scr, scr_len, NULL, 0);
    pickit5_program_enable(pgm, p);
    if(my.rxBuf[17] == 0x0E) {  // Errors figured out during 6 hours of failing to get it to work
      if(my.rxBuf[16] == 0x10 || my.rxBuf[16] == 58) { // Serial/bootloader auto-reset circuit on Arduino board
        pmsg_error("debugWIRE transmission error, aborting"
                   " (ensure reset has a pullup >= 10 kOhm and no capacitance)\n");
      } else {
        pmsg_error("%d\n", my.rxBuf[16]);
      }
      return -1;
    }
    const unsigned char get_sig [] = {    // *screams* why was this function not in the scripts??
      0x90, 0x0C, 0x03, 0x00, 0x00, 0x00, // Set reg to 0x03
      0x1e, 0x45, 0x0C,                   // Send 0xF0 + reg and receive 2 bytes (found by trial and error)
      0x9D,                               // Place word into status response
    };
    int rc = pickit5_send_script_cmd(pgm, get_sig, sizeof(get_sig), NULL, 0);
    if(rc >= LIBAVRDUDE_SUCCESS) {
      unsigned char len = my.rxBuf[20];
      if(len == 0x02) {         // if debugWIRE
        my.devID[0] = 0x1E;     // debugWIRE doesn't send the first byte, fill it in
        my.devID[1] = my.rxBuf[25]; // Flip byte order
        my.devID[2] = my.rxBuf[24];
      }
    }
    return rc;
  }

  if(pickit5_send_script_cmd(pgm, read_id, read_id_len, NULL, 0) < 0)
    return -1;

  if(my.rxBuf[0] == 0x0D) {
    unsigned char len = my.rxBuf[20];
    if(len == 0x03 || len == 0x04) {  // Just DevId or UPDI with revision
      memcpy(my.devID, &my.rxBuf[24], len);
    } else {
      if(my.hvupdi_enabled &&
      (p->hvupdi_variant == UPDI_ENABLE_HV_RESET || p->hvupdi_variant == UPDI_ENABLE_RESET_HS)) {
        pmsg_info("failed to get DeviceID with activated HV Pulse on RST\n");
         msg_info("if the wiring is correct, try connecting a 16 V, 1 uF cap between RST and GND\n");
      } else {
        pmsg_error("length (%u) mismatch of returned Device ID\n", len);
      }
      return -1;
    }
  }
  return 0;
}

static int pickit5_updi_read_sib(const PROGRAMMER *pgm, const AVRPART *p, char *sib) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *read_sib = my.scripts.ReadSIB;
  unsigned int read_sib_len = my.scripts.ReadSIB_len;

  if(pickit5_send_script_cmd(pgm, read_sib, read_sib_len, NULL, 0) < 0)
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
  // On UPDI Devices the chip revision is sent as the 4th byte; the rest needs more research
  *chip_rev = is_updi(pgm)? my.devID[3]: 0x00;
  return 0;
}

static int pickit5_updi_write_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char value) {
  pmsg_debug("%s(CS Addr: %u, Value:%i)\n", __func__, addr, value);
  const unsigned char *write_cs = my.scripts.WriteCSreg;
  unsigned int write_cs_len = my.scripts.WriteCSreg_len;

  if(addr > 0x0C) {
    pmsg_error("CS reg %i out of range [0x00, 0x0C]\n", addr);
    return -1;
  }

  unsigned char buf[2];

  buf[0] = addr;
  buf[1] = value;

  if(pickit5_send_script_cmd(pgm, write_cs, write_cs_len, buf, 2) < 0) {
    pmsg_error("CS reg write failed\n");
    return -1;
  }

  return 1;
}

static int pickit5_updi_read_cs_reg(const PROGRAMMER *pgm, const AVRPART *p, unsigned int addr, unsigned char *value) {
  pmsg_debug("%s(CS Addr: %u)\n", __func__, addr);
  const unsigned char *read_cs = my.scripts.ReadCSreg;
  unsigned int read_cs_len = my.scripts.ReadCSreg_len;

  if(addr > 0x0C) {
    pmsg_error("CS reg %i out of range [0x00, 0x0C], addr\n", addr);
    return -1;
  }

  unsigned char buf[1];

  buf[0] = addr;
  int ret_val = pickit5_upload_data(pgm, p, read_cs, read_cs_len, buf, 1, value, 1);

  switch(ret_val) {
  case -1:
    pmsg_error("sending script failed\n");
    return -1;
  case -2:
    pmsg_error("unexpected read response\n");
    return -1;
  case -3:
    pmsg_error("reading CS reg failed\n");
    return -1;
  case -4:
    pmsg_error("sending script done message failed\n");
    return -1;
  }
  return 0;
}


static void pickit5_dw_switch_to_isp(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s(%u)\n", __func__, my.dW_switched_isp);
  if(my.dW_switched_isp == 0) {
    if(pickit5_send_script_cmd(pgm, my.scripts.switchtoISP, my.scripts.switchtoISP_len, NULL, 0) >= 0) {
      my.dW_switched_isp = 1;
      pickit5_program_disable(pgm, p);
      if(get_pickit_isp_script(&(my.scripts), p->desc) < 0) {
        pmsg_error("failed switching scripts, aborting\n");
        return;
      }
      pmsg_notice("switched to ISP mode\n");
      pickit5_set_sck_period(pgm, 1.0 / my.actual_pgm_clk);
      pickit5_program_enable(pgm, p);
    }
  }
}

static void pickit5_isp_switch_to_dw(const PROGRAMMER *pgm, const AVRPART *p) {
  if(my.dW_switched_isp) {
    /* dW_switched_isp is set when accessing fuses where dW has to be switched to ISP
     * we have to power cycle to switch back to dW so that the scripts work.
     * For now, only support power-cycling through PICkit.
     * Maybe in the future: monitor voltages and wait for voltage falling and rising
     */
    if(my.power_source == POWER_SOURCE_INT) {
      pickit5_program_disable(pgm, p);
      pickit5_set_vtarget(pgm, 0.0); // Has a little delay already built in
      if(get_pickit_dw_script(&(my.scripts), p->desc) < 0) {
        pmsg_error("failed switching scripts, aborting\n");
        return;
      }
      pickit5_set_vtarget(pgm, my.target_voltage);
      pickit5_program_enable(pgm, p);
      my.dW_switched_isp = 0;
    } else {
      pmsg_error("programmer switched the part to ISP mode when writing fuses;\n");
       msg_error("to continue, the part has to be power cycled and the operation restarted\n");
    }
  }
}


/* Original Script only supports doing all three bytes at once,
 * doing a custom script felt easier to integrate into avrdude,
 * especially as we already have all the programming commands in the .conf file
 */
static int pickit5_isp_write_fuse(const PROGRAMMER *pgm, const AVRMEM *mem, unsigned char value) {
  pmsg_debug("%s(offset: %i, val: %i)\n", __func__, mem->offset, value);

  unsigned int cmd;
  avr_set_bits(mem->op[AVR_OP_WRITE], (unsigned char *)&cmd);
  avr_set_addr(mem->op[AVR_OP_WRITE], (unsigned char *)&cmd, mem_fuse_offset(mem));
  avr_set_input(mem->op[AVR_OP_WRITE], (unsigned char *)&cmd, value);

  unsigned char write_fuse_isp [] = {
    0x90, 0x00, 0x32, 0x00, 0x00, 0x00, // Load 0x32 to r00
    0x1E, 0x37, 0x00,                   // Enable Programming?
    0x9F,                               // Send status byte from temp_reg to host
    0xA8, 0x00, 0x00, 0x00, 0x00,       // ???
    0x90, 0x01, (cmd >> 24), (cmd >> 16), (cmd >> 8), cmd, // Load programming command to r01 (swapped bitorder)
    0x1E, 0x34, 0x01,                   // Execute write command placed in r01
  };
  unsigned int write_fuse_isp_len = sizeof(write_fuse_isp);

/*
  write_fuse_isp[14] = (uint8_t) cmd;   // Swap byte order and fill array
  write_fuse_isp[13] = (uint8_t) (cmd >> 8);
  write_fuse_isp[12] = (uint8_t) (cmd >> 16);
  write_fuse_isp[11] = (uint8_t) (cmd >> 24);
*/
  if(pickit5_send_script_cmd(pgm, write_fuse_isp, write_fuse_isp_len, NULL, 0) < 0) {
    pmsg_error("write fuse script failed\n");
    return -1;
  }
  if(0x01 != my.rxBuf[20]) {    // Length
    pmsg_error("write fuse script did not receive a status response\n");
    return -1;
  }
  if(0x00 != my.rxBuf[24]) {
    pmsg_error("failed to start fuse write operation(%d)\n", my.rxBuf[24]);
    return -1;
  }

  // Support slow AVRs without write status polling (won't affect performance)
  int delay = mem->min_write_delay;
  if(delay > 0)
    usleep(delay);

  return 1;
}

static int pickit5_isp_read_fuse(const PROGRAMMER *pgm, const AVRMEM *mem, unsigned long addr, unsigned char *value) {
  pmsg_debug("%s(offset: %i)\n", __func__, mem->offset);

  unsigned int cmd;
  avr_set_bits(mem->op[AVR_OP_READ], (unsigned char *)&cmd);
  avr_set_addr(mem->op[AVR_OP_READ], (unsigned char *)&cmd, addr + mem->offset);


  unsigned char read_fuse_isp [] = {    // As we pull the command from avrdude's conf file, this isn't limited to fuses
    0x90, 0x00, 0x32, 0x00, 0x00, 0x00, // Load 0x32 to r00
    0x1E, 0x37, 0x00,                   // Enable Programming?
    0x9F,                               // Send status from temp_reg to host
    0xA8, 0x00, 0x00, 0x00, 0x00,       // ???
    0x90, 0x01, (cmd >> 24), (cmd >> 16), (cmd >> 8), cmd, // Load programming command to r01 (swapped bitorder)
    0x9B, 0x02, 0x03,                   // Load 0x03 to r02
    0x9B, 0x03, 0x00,                   // Load 0x00 to r03
    0x1E, 0x35, 0x01, 0x02, 0x03,       // Execute Command placed in r01
    0x9F                                // Send data from temp_reg to host
  };
  unsigned int read_fuse_isp_len = sizeof(read_fuse_isp);

/*
  read_fuse_isp[14] = (uint8_t) cmd;    // Swap byte order and fill array
  read_fuse_isp[13] = (uint8_t) (cmd >> 8);
  read_fuse_isp[12] = (uint8_t) (cmd >> 16);
  read_fuse_isp[11] = (uint8_t) (cmd >> 24);
*/
  if(pickit5_send_script_cmd(pgm, read_fuse_isp, read_fuse_isp_len, NULL, 0) < 0) {
    pmsg_error("read fuse script failed\n");
    return -1;
  }
  if(0x02 != my.rxBuf[20]) {    // Length
    pmsg_error("unexpected amount (%d) of bytes returned\n", my.rxBuf[20]);
    return -1;
  }
  if(0x00 != my.rxBuf[24]) {
    pmsg_error("failed to start fuse read operation (%d)\n", my.rxBuf[24]);
    return -1;
  }
  *value = my.rxBuf[25];        // Return value
  return 1;
}

/* debugWIRE cannot write nor read fuses, have to change to ISP for that.
 * Luckily, there is a custom script doing fuse access on ISP anyway,
 * so no need to switch between script sets
 */

static int pickit5_dw_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char value) {
  pickit5_dw_switch_to_isp(pgm, p);
  return pickit5_isp_write_fuse(pgm, mem, value);
}

static int pickit5_dw_read_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char *value) {
  pickit5_dw_switch_to_isp(pgm, p);
  return pickit5_isp_read_fuse(pgm, mem, 0, value);
}


// Gave JTAG also a custom script to make integration into avrdude easier.
// Also encodes all data in script itself instead of using paramters
static int pickit5_jtag_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char value) {
  pmsg_debug("%s(offset: %i, val: %i)\n", __func__, mem->offset, value);
  unsigned char fuse_cmd = 0x33;  // Value for lfuse
  unsigned char fuse_poll = 0x33; // Value for lfuse
  if(mem_is_hfuse(mem)) {
    fuse_cmd = 0x37;
    fuse_poll = 0x37;
  } else if(mem_is_efuse(mem)) {
    fuse_cmd = 0x3B;
    fuse_poll = 0x37;
  }

  unsigned char write_fuse_jtag [] = {
    0x9C, 0x00, 0x00,   fuse_cmd,         // Write fuse write command A to r00
    0x9C, 0x06, 0x00,  (fuse_cmd & 0xFD), // Write fuse write command B to r06
    0x9C, 0x07, 0x00,  fuse_poll,         // Write fuse poll command to r07
    0x9C, 0x01, value, 0x13,              // Write new fuse value plus load command (0x13) into r01
    0x9b, 0x02, 0x0F,                     // Set r02 to 0x0F
    0x9b, 0x03, 0x05,                     // Set r03 to 0x05    (PROG_COMMANDS)
    0x1e, 0x66, 0x03,                     // JTAG Write to Instruction Reg the value in r03
    0x90, 0x04, 0x40, 0x23, 0x00, 0x00,   // Set r04 to 0x2340  (Enter Fuse Write)
    0x1e, 0x67, 0x04, 0x02,               // JTAG: Write to Data Reg the value in r04 with a length in r02(16)
    0x1e, 0x67, 0x01, 0x02,               // JTAG: Write to Data Reg the value in r01 with a length in r02(16)

    0x1e, 0x67, 0x00, 0x02,               // JTAG: Write to Data Reg the value in r00 with a length in r02(16)
    0x1e, 0x67, 0x06, 0x02,               // JTAG: Write to Data Reg the value in r06 with a length in r02(16)
    0x1e, 0x67, 0x00, 0x02,               // JTAG: Write to Data Reg the value in r00 with a length in r02(16)
    0x1e, 0x67, 0x00, 0x02,               // JTAG: Write to Data Reg the value in r00 with a length in r02(16)

    0xa2,                                 // Do
    0x1e, 0x6b, 0x07, 0x02,               // JTAG: Write/read Data Reg the value in r07 with a length in r02(16)
    0xa5, 0x00, 0x02, 0x00, 0x00,         // while((temp_reg & 0x200) != 0x200)
    0x00, 0x02, 0x00, 0x00, 0x0a, 0x00,
  };
  unsigned int write_fuse_isp_len = sizeof(write_fuse_jtag);

  if(pickit5_send_script_cmd(pgm, write_fuse_jtag, write_fuse_isp_len, NULL, 0) < 0) {
    pmsg_error("write fuse script failed\n");
    return -1;
  }
  return 1;
}

static int pickit5_jtag_read_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char *value) {
  pmsg_debug("%s(offset: %i)\n", __func__, mem->offset);

  unsigned char fuse_cmd = 0x33; // Value for lfuse
  if(mem_is_hfuse(mem))
    fuse_cmd = 0x3F;
  else if(mem_is_efuse(mem))
    fuse_cmd = 0x3B;

  unsigned char read_fuse_jtag [] = {
    0x9C, 0x00, 0x00,  fuse_cmd,         // Load fuse read command A in r00
    0x9C, 0x01, 0x00, (fuse_cmd & 0xFE), // Load fuse read command B in r01
    0x9b, 0x02, 0x0f,                    // Set r02 to 0x07
    0x9b, 0x03, 0x05,                    // Set r03 to 0x05 (PROG COMMANDS)
    0x1e, 0x66, 0x03,                    // Write JTAG Instruction in r03
    0x9C, 0x04, 0x04, 0x23,              // Set r04 to 0x2304 (Enter Fuse Bit Read)
    0x1e, 0x67, 0x04, 0x02,              // Write JTAG instruction in r04 with length in r02 (7 bits)
    0x1e, 0x67, 0x01, 0x02,              // Write JTAG instruction in r01 with length in r02 (7 bits)
    0x1E, 0x6B, 0x00, 0x02,              // Write JTAG instruction in r00 with length in r02 (7 bits) and shift data in
    0x9F,                                // Send temp-reg to return status
  };

  unsigned int read_fuse_jtag_len = sizeof(read_fuse_jtag);

  if(pickit5_send_script_cmd(pgm, read_fuse_jtag, read_fuse_jtag_len, NULL, 0) < 0) {
    pmsg_error("read fuse script failed\n");
    return -1;
  }
  if(0x01 != my.rxBuf[20])      // Length
    return -1;
  *value = my.rxBuf[24];        // Return value
  return 1;
}


/* TPI has an unified memory space, meaning that any memory (even SRAM)
 * can be accessed by the same command, meaning that we don't need the
 * decision tree found in the "read/write array" functions
 */
static int pickit5_tpi_write(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s(%s, addr: 0x%04x, offset: %i, len: %i)\n", __func__, mem->desc, (unsigned int) addr, mem->offset, len);

  const unsigned char *write_bytes = my.scripts.WriteProgmem;
  unsigned int write_bytes_len = my.scripts.WriteProgmem_len;
  addr += mem->offset;

  unsigned char buf[8];

  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);

  int rc = pickit5_download_data(pgm, p, write_bytes, write_bytes_len, buf, 8, value, len);

  return rc < 0? -1: len;
}

static int pickit5_tpi_read(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s(%s, addr: 0x%04x, offset: %i, len: %i)\n", __func__, mem->desc, (unsigned int) addr, mem->offset, len);

  const unsigned char *read_bytes = my.scripts.ReadProgmem;
  unsigned int read_bytes_len = my.scripts.ReadProgmem_len;

  addr += mem->offset;
  unsigned char buf[8];
  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);
  int rc = pickit5_upload_data(pgm, p, read_bytes, read_bytes_len, buf, 8, value, len);

  return rc < 0? -1: len;
}


// There are often multiple memories located in prodsig, we try to read it once.
// Handle all further requests through a buffer as it doesn't change.
static int pickit5_read_prodsig(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s(%s, addr: 0x%04x, offset: %i, len: %i)\n", __func__, mem->desc, (unsigned int) addr, mem->offset, len);
  int rc = 0;

  AVRMEM *prodsig = avr_locate_prodsig(p);
  if(prodsig == NULL)           // No prodsig on this device, try again in read_array
    return 0;
  if(mem->offset < prodsig->offset ||
    (mem->offset + mem->size) > (prodsig->offset) + (prodsig->size)) {
    return 0;                   // Requested memory not in prodsig, try again in read_array
  }

  int max_mem_len = sizeof(my.prodsig);  // Current devices have less then 128 bytes
  unsigned mem_len = (prodsig->size < max_mem_len)? prodsig->size: max_mem_len;

  if((addr + len) > mem_len) {
    pmsg_warning("requested memory is outside of the progsig on the device\n");
    return 0;
  }

  unsigned int prod_addr = addr + mem->offset - prodsig->offset; // Adjust offset

  if(prod_addr == 0x00 || (my.prod_sig_len == 0x00)) { // Update buffer
    if(my.scripts.ReadConfigmem != NULL) {
      unsigned char param_buf[8];
      pickit5_uint32_to_array(&param_buf[0], prodsig->offset);
      pickit5_uint32_to_array(&param_buf[4], mem_len);
      rc = pickit5_upload_data(pgm, p, my.scripts.ReadConfigmem, my.scripts.ReadConfigmem_len, param_buf, 8, my.prodsig, mem_len);
    } else if(mem->op[AVR_OP_READ] != NULL) {
      if(both_jtag(pgm, p)) {
        const unsigned char read_prodsigmem_jtag [] = {
          0x90, 0x00, 0x00, 0x03, 0x00, 0x00, // Set r00 to 0x0300 (Load Address byte command (0x3bb))
          0x9b, 0x01, 0x0f,                   // Set r01 to 0x0F
          0x9b, 0x02, 0x05,                   // Set r02 to 0x05 (PROG COMMANDS)
          0x90, 0x03, 0x08, 0x23, 0x00, 0x00, // Set r03 to 0x2308 (Enter Signature Read)
          0x90, 0x05, 0x00, 0x32, 0x00, 0x00, // Set r05 to 0x3200 (Read Signature byte I)
          0x90, 0x06, 0x00, 0x33, 0x00, 0x00, // Set r06 to 0x3300 (Read Signature byte II)

          0xAC, mem_len, 0x00,                // Loop for mem length
          0x1e, 0x66, 0x02,                   // Write JTAG Instruction in r03 (PROG COMMANDS)
          0x1e, 0x67, 0x03, 0x01,             // Write JTAG instruction in r02 with length in r01 (15 bits)
          0x1e, 0x67, 0x00, 0x01,             // Write JTAG instruction in r00 with length in r01 (15 bits)
          0x1e, 0x67, 0x05, 0x01,             // Write JTAG instruction in r05 with length in r01 (15 bits)
          0x1E, 0x6B, 0x06, 0x01,             // Write JTAG instruction in r06 with length in r01 (15 bits) and shift data in
          0x9F,                               // Send temp-reg to return status
          0x92, 0x00, 0x01, 0x00, 0x00, 0x00, // Increase address (r00) by 1
          0xA4,                               // End of for loop
        };
        rc = pickit5_upload_data(pgm, p, read_prodsigmem_jtag, sizeof(read_prodsigmem_jtag), NULL, 0, my.prodsig, mem_len);
      } else if(is_isp(pgm)) {
        // Ok, this one is tricky due to the LSB being on another position compared to the rest,
        // The solution is to read two bytes in one while-loop and toggle the LSB
        const unsigned char read_prodsig_isp [] = {
          0x90, 0x00, 0x32, 0x00, 0x00, 0x00, // Load 0x32 to r00
          0x90, 0x01, 0x00, 0x00, 0x00, 0x30, // Load programming command to r01 (the same on all)
          0x9B, 0x02, 0x03,                   // Load 0x03 to r02
          0x9B, 0x03, 0x00,                   // Load 0x00 to r03
          0x1E, 0x37, 0x00,                   // Enable Programming?
          0xAC, (mem_len / 2), 0x00,          // Loop for half the mem length
          0x1E, 0x35, 0x01, 0x02, 0x03,       // Execute ISP Read command in r01
          0x9F,                               // Send Data back to USB
          0x92, 0x01, 0x00, 0x00, 0x00, 0x08, // Set LSB of prodsig address
          0x1E, 0x35, 0x01, 0x02, 0x03,       // Execute ISP Read command in r01
          0x9F,                               // Send Data back to USB
          0x69, 0x01, 0x00, 0x00, 0x00, 0x08, // Clear LSB of prodsig address
          0x92, 0x01, 0x00, 0x01, 0x00, 0x00, // Increase address by "2"
          0xA4,                               // End of for loop
        };
        rc = pickit5_upload_data(pgm, p, read_prodsig_isp, sizeof(read_prodsig_isp), NULL, 0, my.prodsig, mem_len);
      } else {                  // debugWIRE
        return 0;
      }
    } else {                    // Part has no prodsig nor ReadConfigmem
      return 0;
    }
  }
  if(rc >= 0) {                 // No errors, copy data
    my.prod_sig_len = mem_len;
    if(len == 1)
      *value = my.prodsig[prod_addr];
    else
      memcpy(value, &my.prodsig[prod_addr], len);
    return 1;                   // Success
  }
  return rc;                    // Error Code from transfer functions
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
  my.sernum_string[19] = 0;     // Known zero-terminator
  return 0;
}

static int pickit5_set_vtarget(const PROGRAMMER *pgm, double v) {
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

  if(!can_power_target(pgm))    // SNAP and Basic can't supply power, ignore
    return 0;

  if(v < 1.0) {                 // Anything below 1 V equals disabling Power
    pmsg_debug("%s(disable)\n", __func__);
    if(pickit5_send_script_cmd(pgm, power_source, 5, NULL, 0) < 0)
      return -1;

    if(pickit5_send_script_cmd(pgm, disable_power, 1, NULL, 0) < 0)
      return -1;
    usleep(50000);              // There might be some caps, let them discharge
  } else {
    pmsg_debug("%s(%1.2f V)\n", __func__, v);
    power_source[1] = 0x01;
    if(pickit5_send_script_cmd(pgm, power_source, 5, NULL, 0) < 0)
      return -1;

    int vtarg = (int) (v * 1000.0);

    pickit5_uint32_to_array(&set_vtarget[1], vtarg);
    pickit5_uint32_to_array(&set_vtarget[5], vtarg);
    pickit5_uint32_to_array(&set_vtarget[9], vtarg);

    if(pickit5_send_script_cmd(pgm, set_vtarget, 15, NULL, 0) < 0)
      return -1;
  }
  return 0;
}

static int pickit5_get_vtarget(const PROGRAMMER *pgm, double *v) {
  const unsigned char get_vtarget[] = { 0x47, };
  unsigned char *buf = my.rxBuf;

  pmsg_debug("%s()\n", __func__);

  if(pickit5_send_script_cmd(pgm, get_vtarget, 1, NULL, 0) < 0)
      return -1;

  // 24 - internal Vdd [mV]
  // 28 - target Vdd [mV]
  // 48 - Vdd Current Sense [mA]
  my.measured_vcc = pickit5_array_to_uint32(&buf[28]) / 1000.0;
  my.measured_current = pickit5_array_to_uint32(&buf[48]);

  if(pgm->extra_features & HAS_VTARG_READ) // If not supported (PK Basic), don't print placeholder value
    pmsg_notice("target Vdd: %1.2f V, target current: %u mA\n", my.measured_vcc, my.measured_current);

  if(v != NULL)
    *v = my.measured_vcc;
  return 0;
}

static int pickit5_set_ptg_mode(const PROGRAMMER *pgm, const AVRPART *p) {
  if(!can_do_ptg(pgm))          // Don't bother if Programmer doesn't support PTG
    return 0;

  unsigned char ptg_mode[] = {
    0x5E, 0x00, 0x00, 0x00, 0x00,
  };
  unsigned char buf[8];

  pmsg_debug("%s()\n", __func__);

  if(pickit5_upload_data(pgm, p, ptg_mode, 5, NULL, 0, buf, 4))
    return -1;
  return 0;
}

/*
 * Found sw reset command in basic firmware switcher.
 * other commands are
 * Enter Boot Mode: 0xEB
 * jump to app: 0xEC
 * erase flash: 0xE2
 * erase application: 0xFA
 * write page: 0xE3
 * read crc32: 0x5E
 */

/* currently unused, thus uncommented, but maybe useful in the future
static int pickit5_software_reset(const PROGRAMMER *pgm) {
  unsigned char sw_reset[] = {
    0xED,
  };

  pmsg_debug("%s()\n", __func__);

  if(pickit5_send_script_cmd(pgm, sw_reset, 1, NULL, 0))
    return -1;
  return 0;
}
*/

void pickit5_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "pickit5");

  // Mandatory functions
  pgm->initialize = pickit5_initialize;
  pgm->parseextparams = pickit5_parseextparms;
  pgm->parseexitspecs = pickit5_parseexitspecs;
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
  pgm->read_sib = pickit5_updi_read_sib;
  pgm->read_chip_rev = pickit5_read_chip_rev;
  pgm->set_vtarget = pickit5_set_vtarget;
  pgm->get_vtarget = pickit5_get_vtarget;
  pgm->print_parms = pickit5_print_parms;

}



#if defined(HAVE_USB_H)
/*
 * In order to make it easier for users, we try to figure out if a specific VID/PID is connected
 * to the computer. For that, going through the list of usb devices is more straightforward
 * compared to what serial_open / usbdev_open does.
 */
static int usbdev_check_connected(unsigned int vid, unsigned int pid) {
  struct usb_bus *bus;
  struct usb_device *dev;

  usb_init();

  usb_find_busses();
  usb_find_devices();

  for(bus = usb_get_busses(); bus; bus = bus->next) {
    for(dev = bus->devices; dev; dev = dev->next) {
      if(dev->descriptor.idVendor == vid &&
         dev->descriptor.idProduct == pid) {
          return 0;
         }
    }
  }
  return -1;
}

/*
  The following functions are required to handle the data read and write Endpoints of the Pickit.
  This functions are hardcoded on the Pickit endpoint numbers
*/
static int usbdev_bulk_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes) {
  int i, amnt;
  unsigned char *p = buf;

  if(fd->usb.handle == NULL)
    return -1;

  for(i = 0; nbytes > 0;) {
    if(cx->usb_buflen <= cx->usb_bufptr) {
      int rv = usb_bulk_read(fd->usb.handle, USB_PK5_DATA_READ_EP, cx->usb_buf, fd->usb.max_xfer, 10000);

      if(rv < 0) {
        pmsg_notice2("%s(): usb_bulk_read() error: %s\n", __func__, usb_strerror());
        return -1;
      }

      cx->usb_buflen = rv;
      cx->usb_bufptr = 0;
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

static int usbdev_bulk_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen) {
  int rv;
  const unsigned char *p = bp;
  int tx_size, i = mlen;

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

    rv = usb_bulk_write(fd->usb.handle, USB_PK5_DATA_WRITE_EP, (char *) bp, tx_size, 10000);
    if(rv != tx_size) {
      pmsg_error("wrote %d out of %d bytes, err = %s\n", rv, tx_size, usb_strerror());
      return -1;
    }
    bp += tx_size;
    mlen -= tx_size;
    i += tx_size;
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

static int usbdev_check_connected(unsigned int vid, unsigned int pid) {
  return -1;
}

static int usbdev_bulk_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes) {
  return -1;
}

static int usbdev_bulk_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen) {
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

