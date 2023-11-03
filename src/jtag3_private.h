/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2012 Joerg Wunsch <j@uriah.heep.sax.de>
 *
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
 * JTAGICE3 definitions
 * Reverse-engineered from various USB traces.
 */

#if !defined(JTAG3_PRIVATE_EXPORTED)
/*
 * Communication with the JTAGICE3 uses three data endpoints:
 *
 * Endpoint 0x01 (OUT) and 0x82 (IN) are the usual conversation
 * endpoints, with a maximal packet size of 512 octets.  The
 * JTAGICE3 does *not* work on older USB 1.1 hubs that would only
 * allow for 64-octet max packet size.
 *
 * Endpoint 0x83 (IN) is also a bulk endpoint, with a max packetsize
 * of 64 octets.  This endpoint is used by the ICE to deliver events
 * from the ICE.
 *
 * The request (host -> ICE, EP 0x01) format is:
 *
 *  +---------------------------------------------
 *  |   0   |  1  |  2 . 3 |  4  |  5  |  6  | ...
 *  |       |     |        |     |     |     |
 *  | token |dummy|serial# |scope| cmd |dummy| optional data
 *  | 0x0e  |  0  |  NNNN  | SS  | CC  |  0  | ...
 *  +---------------------------------------------
 *
 * Both dummy bytes are always 0.  The "scope" identifier appears
 * to distinguish commands (responses, events, parameters) roughly:
 *
 * 0x01 - general scope ("hello", "goodbye", firmware info, target
 *        voltage readout)
 * 0x11 - scope for AVR in ISP mode (basically a wrapper around
 *        the AVRISPmkII commands, as usual)
 * 0x12 - scope for AVR (JTAG, PDI, debugWIRE)
 *
 * The serial number is counted up.
 *
 *
 * The response (ICE -> host, EP 0x82) format is:
 *
 *  +--------------------------------------------------+
 *  |   0   |  1 . 2 |  3  |  4  | ...           |  N  |
 *  |       |        |     |     |               |     |
 *  | token |serial# |scope| rsp | optional data |dummy|
 *  | 0x0e  |  NNNN  | SS  | RR  | ...           |  0  |
 *  +--------------------------------------------------+
 *
 * The response's serial number is mirrored from the request, but the
 * dummy byte before the serial number is left out.  However, another
 * zero dummy byte is always attached to the end of the response data.
 * Response codes are similar to the JTAGICEmkII, 0x80 is a generic
 * "OK" response, other responses above 0x80 indicate various data
 * responses (parameter read, memory read, PC value), and 0xa0 is a
 * generic "failure" response.  It appears the failure response gets
 * another byte appended (probably indicating the reason) after the
 * 0 dummy byte, but there's not enough analysis material so far.
 *
 *
 * The event format (EP 0x83) is:
 *
 *  +----------------------------------------
 *  |   0   |  1  |  2 . 3 |  4  |  5  | ...
 *  |       |     |        |     |     |
 *  | token |dummy|serial# |scope| evt | data
 *  | 0x0e  |  0  |  NNNN  | SS  | EV  | ...
 *  +----------------------------------------
 */
#define TOKEN 0x0e

#endif /* JTAG3_PRIVATE_EXPORTED */

#define SCOPE_INFO                 0x00
#define SCOPE_GENERAL              0x01
#define SCOPE_AVR_ISP              0x11
#define SCOPE_AVR                  0x12
#define SCOPE_AVR32                0x13
#define SCOPE_AVR_TPI              0x14
#define SCOPE_EDBG                 0x20

/* Info scope */
#define CMD3_GET_INFO              0x00

/* byte after GET_INFO is always 0, next is: */
#  define CMD3_INFO_NAME           0x80 /* JTAGICE3 */
#  define CMD3_INFO_SERIAL         0x81 /* J3xxxxxxxxxx */

/* Generic scope */
#define CMD3_SET_PARAMETER         0x01
#define CMD3_GET_PARAMETER         0x02
#define CMD3_SIGN_ON               0x10
#define CMD3_SIGN_OFF              0x11 /* takes one parameter? */
#define CMD3_GET_ID                0x12
#define CMD3_START_DW_DEBUG        0x13
#define CMD3_MONCON_DISABLE        0x17

/* AVR ISP scope: no commands of its own */

/* AVR scope */
//#define CMD3_SET_PARAMETER       0x01
//#define CMD3_GET_PARAMETER       0x02
//#define CMD3_SIGN_ON             0x10 /* an additional signon/-off pair */
//#define CMD3_SIGN_OFF            0x11
#define CMD3_ENTER_PROGMODE        0x15
#define CMD3_LEAVE_PROGMODE        0x16
#define CMD3_ERASE_MEMORY          0x20
#define CMD3_READ_MEMORY           0x21
#define CMD3_WRITE_MEMORY          0x23
#define CMD3_READ_PC               0x35

/* ICE responses */
#define RSP3_OK                    0x80
#define RSP3_INFO                  0x81
#define RSP3_PC                    0x83
#define RSP3_DATA                  0x84
#define RSP3_FAILED                0xA0

#define RSP3_STATUS_MASK           0xE0

/* possible failure codes that could be appended to RSP3_FAILED: */
#  define RSP3_FAIL_DEBUGWIRE           0x10
#  define RSP3_FAIL_PDI                 0x1B
#  define RSP3_FAIL_NO_ANSWER           0x20
#  define RSP3_FAIL_NO_TARGET_POWER     0x22
#  define RSP3_FAIL_WRONG_MODE          0x32 /* progmode vs. non-prog */
#  define RSP3_FAIL_UNSUPP_MEMORY       0x34 /* unsupported memory type */
#  define RSP3_FAIL_WRONG_LENGTH        0x35 /* wrong lenth for mem access */
#  define RSP3_FAIL_CRC_FAILURE         0x43 /* CRC failure in device */
#  define RSP3_FAIL_OCD_LOCKED          0x44 /* device is locked */
#  define RSP3_FAIL_NOT_UNDERSTOOD      0x91

/* ICE events */
#define EVT3_BREAK                 0x40 /* AVR scope */
#define EVT3_SLEEP                 0x11 /* General scope, also wakeup */
#define EVT3_POWER                 0x10 /* General scope */

/* memories */
#define MTYPE_SRAM        0x20	/* target's SRAM or [ext.] IO registers */
#define MTYPE_EEPROM      0x22	/* EEPROM, what way? */
#define MTYPE_SPM         0xA0	/* flash through LPM/SPM */
#define MTYPE_FLASH_PAGE  0xB0	/* flash in programming mode */
#define MTYPE_EEPROM_PAGE 0xB1	/* EEPROM in programming mode */
#define MTYPE_FUSE_BITS   0xB2	/* fuse bits in programming mode */
#define MTYPE_LOCK_BITS   0xB3	/* lock bits in programming mode */
#define MTYPE_SIGN_JTAG   0xB4	/* signature in programming mode */
#define MTYPE_OSCCAL_BYTE 0xB5	/* osccal cells in programming mode */
#define MTYPE_FLASH       0xc0	/* xmega (app.) flash - undocumented in AVR067 */
#define MTYPE_BOOT_FLASH  0xc1	/* xmega boot flash - undocumented in AVR067 */
#define MTYPE_EEPROM_XMEGA 0xc4	/* xmega EEPROM in debug mode - undocumented in AVR067 */
#define MTYPE_USERSIG     0xc5	/* xmega user signature - undocumented in AVR067 */
#define MTYPE_PRODSIG     0xc6	/* xmega production signature - undocumented in AVR067 */
#define MTYPE_SIB         0xD3  /* AVR8X System Information Block */

/*
 * SET and GET context definitions
 */
#define SET_GET_CTXT_CONFIG    0x00 /* Configuration */
#define SET_GET_CTXT_PHYSICAL  0x01 /* Physical interface related */
#define SET_GET_CTXT_DEVICE    0x02 /* Device specific settings */
#define SET_GET_CTXT_OPTIONS   0x03 /* Option-related settings */
#define SET_GET_CTXT_SESSION   0x04 /* Session-related settings */

/*
 * Parameters are divided into sections, where the section number
 * precedes each parameter address.  There are distinct parameter
 * sets for generic and AVR scope.
 */
#define PARM3_HW_VER            0x00  /* section 0, generic scope, 1 byte */
#define PARM3_FW_MAJOR          0x01  /* section 0, generic scope, 1 byte */
#define PARM3_FW_MINOR          0x02  /* section 0, generic scope, 1 byte */
#define PARM3_FW_RELEASE        0x03  /* section 0, generic scope, 1 byte;
                                       * always asked for by Atmel Studio,
                                       * but never displayed there */

#define PARM3_VTARGET           0x00  /* section 1, generic scope, 2 bytes, in millivolts */
#define PARM3_VBUF              0x01  /* section 1, generic scope, 2 bytes, bufferred target voltage reference */
#define PARM3_VUSB              0x02  /* section 1, generic scope, 2 bytes, USB voltage */
#define PARM3_ANALOG_A_CURRENT  0x10  /* section 1, generic scope, 2 bytes, Ch A current in milliamps,  Powerdebugger only */
#define PARM3_ANALOG_A_VOLTAGE  0x11  /* section 1, generic scope, 2 bytes, Ch A voltage in millivolts, Powerdebugger only */
#define PARM3_ANALOG_B_CURRENT  0x12  /* section 1, generic scope, 2 bytes, Ch B current in milliamps,  Powerdebugger only */
#define PARM3_ANALOG_B_VOLTAGE  0x13  /* section 1, generic scope, 2 bytes, Ch V voltage in millivolts, Powerdebugger only */
#define PARM3_TSUP_VOLTAGE_MEAS 0x14  /* section 1, generic scope, 2 bytes, target voltage measurement in millivolts */
#define PARM3_USB_VOLTAGE_MEAS  0x15  /* section 1, generic scope, 2 bytes, USB voltage measurement in millivolts */
#define PARM3_VADJUST           0x20  /* section 1, generic scope, 2 bytes, set voltage in millivolts */
#define PARM3_ANALOG_STATUS     0x30  /* section 1, generic scope, 2 bytes, analog status */

/* mEDBG Xplained Mini / Nano constants */
#define MEDBG_REG_SUFFER_BANK   0x01
#define MEDBG_REG_SUFFER_OFFSET 0x20 /* section 17, EDBG scope, 1 byte, SUFFER register value */

#define PARM3_DEVICEDESC  0x00  /* section 2, memory etc. configuration,
                                 * 31 bytes for tiny/mega AVR, 47 bytes
                                 * for Xmega; is also used in command
                                 * 0x36 in JTAGICEmkII, starting with
                                 * firmware 7.x */

#define PARM3_ARCH        0x00  /* section 0, AVR scope, 1 byte */
#  define PARM3_ARCH_TINY   1   /* also small megaAVR with ISP/DW only */
#  define PARM3_ARCH_MEGA   2
#  define PARM3_ARCH_XMEGA  3
#  define PARM3_ARCH_UPDI   5   /* AVR devices with UPDI i/f */

#define PARM3_SESS_PURPOSE 0x01 /* section 0, AVR scope, 1 byte */
#  define PARM3_SESS_PROGRAMMING 1
#  define PARM3_SESS_DEBUGGING   2

#define PARM3_CONNECTION  0x00  /* section 1, AVR scope, 1 byte */
#  define PARM3_CONN_ISP    1
#  define PARM3_CONN_JTAG   4
#  define PARM3_CONN_DW     5
#  define PARM3_CONN_PDI    6
#  define PARM3_CONN_UPDI   8


#define PARM3_JTAGCHAIN   0x01  /* JTAG chain info, AVR scope (units
                                 * before/after, bits before/after), 4
                                 * bytes */

/*
 * Physical context parameters
 */
#define PARM3_CLK_MEGA_PROG  0x20 /* section 1, AVR scope, 2 bytes (kHz) */
#define PARM3_CLK_MEGA_DEBUG 0x21 /* section 1, AVR scope, 2 bytes (kHz) */
#define PARM3_CLK_XMEGA_JTAG 0x30 /* section 1, AVR scope, 2 bytes (kHz) */
#define PARM3_CLK_XMEGA_PDI  0x31 /* section 1, AVR scope, 2 bytes (kHz) */

/*
 * Options context parameters
 */
#define PARM3_OPT_12V_UPDI_ENABLE      0x06
#define PARM3_OPT_CHIP_ERASE_TO_ENTER  0x07

/*
 * UPDI high-voltage enable modes
 */
#define PARM3_UPDI_HV_NONE              0x00  /* Do not use high-voltage */
#define PARM3_UPDI_HV_SIMPLE_PULSE      0x01  /* Issue a single high-voltage pulse immediately*/
#define PARM3_UPDI_HV_AUTO_POWER_TOGGLE 0x02  /* Toggle power automatically and then apply a high-voltage pulse */
#define PARM3_UPDI_HV_USER_POWER_TOGGLE 0x03  /* The user toggles power, and the tool applies a high-voltage pulse on power-up */

/* Xmega erase memories for CMND_XMEGA_ERASE */
#define XMEGA_ERASE_CHIP        0x00
#define XMEGA_ERASE_APP         0x01
#define XMEGA_ERASE_BOOT        0x02
#define XMEGA_ERASE_EEPROM      0x03
#define XMEGA_ERASE_APP_PAGE    0x04
#define XMEGA_ERASE_BOOT_PAGE   0x05
#define XMEGA_ERASE_EEPROM_PAGE 0x06
#define XMEGA_ERASE_USERSIG     0x07

/* EDBG vendor commands */
#define EDBG_VENDOR_AVR_CMD       0x80
#define EDBG_VENDOR_AVR_RSP       0x81
#define EDBG_VENDOR_AVR_EVT       0x82

#define EDBG_CTXT_CONTROL         0x00
#define EDBG_CONTROL_LED_USAGE    0x00
#define EDBG_CONTROL_EXT_PROG     0x01
#define EDBG_CONTROL_TARGET_POWER 0x10

/* CMSIS-DAP commands */
#define CMSISDAP_CMD_INFO       0x00 /* get info, followed by INFO byte */
#  define CMSISDAP_INFO_VID         0x01 /* vendor ID (string) */
#  define CMSISDAP_INFO_PID         0x02 /* product ID (string) */
#  define CMSISDAP_INFO_SERIAL      0x03 /* serial number (string) */
#  define CMSISDAP_INFO_FIRMWARE    0x04 /* firmware version (string) */
#  define CMSISDAP_INFO_TARGET_VENDOR 0x05 /* target device vendor (string) */
#  define CMSISDAP_INFO_TARGET_NAME   0x06 /* target device name (string) */
#  define CMSISDAP_INFO_CAPABILITIES  0xF0 /* debug unit capabilities (byte) */
#  define CMSISDAP_INFO_PACKET_COUNT  0xFE /* packet count (byte) (which packets, anyway?) */
#  define CMSISDAP_INFO_PACKET_SIZE   0xFF /* packet size (short) */

#define CMSISDAP_CMD_LED        0x01 /* LED control, followed by LED number and on/off byte */
#  define CMSISDAP_LED_CONNECT      0x00 /* connect LED */
#  define CMSISDAP_LED_RUNNING      0x01 /* running LED */

#define CMSISDAP_CMD_CONNECT    0x02 /* connect to target, followed by DAP mode */
#  define CMSISDAP_CONN_DEFAULT     0x00
#  define CMSISDAP_CONN_SWD         0x01 /* serial wire debug */
#  define CMSISDAP_CONN_JTAG        0x02 /* JTAG mode */

#define CMSISDAP_CMD_DISCONNECT 0x03 /* disconnect from target */

#define CMSISDAP_XFR_CONFIGURE  0x04 /* configure transfers; idle cycles (byte);
                                        wait retry (short); match retry (short) */

#define CMSISDAP_CMD_WRITEAPBORT 0x08 /* write to CoreSight ABORT register of target */

#define CMSISDAP_CMD_DELAY      0x09 /* delay for number of microseconds (short) */

#define CMSISDAP_CMD_RESET      0x0A /* reset target */

#define CMSISDAP_CMD_SWJ_CLOCK  0x11 /* SWD/JTAG clock, (word) */

#define CMSISDAP_CMD_SWD_CONFIGURE 0x13 /* configure SWD protocol; (byte) */

#define DEFAULT_MINIMUM_CHARACTERISED_DIV1_VOLTAGE_MV  4500   // Default minimum voltage for 32M => 4.5V
#define DEFAULT_MINIMUM_CHARACTERISED_DIV2_VOLTAGE_MV  2700   // Default minimum voltage for 16M => 2.7V
#define DEFAULT_MINIMUM_CHARACTERISED_DIV4_VOLTAGE_MV  2200   // Default minimum voltage for 8M  => 2.2V
#define DEFAULT_MINIMUM_CHARACTERISED_DIV8_VOLTAGE_MV  1500   // Default minimum voltage for 4M  => 1.5V
#define MAX_FREQUENCY_DEDICATED_UPDI_PIN  1500
#define MAX_FREQUENCY_SHARED_UPDI_PIN     750
#define UPDI_ADDRESS_MODE_16BIT 0
#define UPDI_ADDRESS_MODE_24BIT 1
#define FUSES_SYSCFG0_OFFSET 5

// TPI Protocol commands
#define XPRG_CMD_ENTER_PROGMODE             0x01
#define XPRG_CMD_LEAVE_PROGMODE             0x02
#define XPRG_CMD_ERASE                      0x03
#define XPRG_CMD_WRITE_MEM                  0x04
#define XPRG_CMD_READ_MEM                   0x05
#define XPRG_CMD_CRC                        0x06
#define XPRG_CMD_SET_PARAM                  0x07

// TPI Protocol responses
// Success
#define XPRG_ERR_OK                         0x00
// Errors
#define XPRG_ERR_FAILED                     0x01
#define XPRG_ERR_COLLISION                  0x02
#define XPRG_ERR_TIMEOUT                    0x03
#define XPRG_ERR_ILLEGAL_PARAM              0x04
#define XPRG_ERR_UNKNOWN_COMMAND            0x10

// TPI Memories
#define XPRG_MEM_TYPE_APPL                  0x01
#define XPRG_MEM_TYPE_BOOT                  0x02
#define XPRG_MEM_TYPE_EEPROM                0x03
#define XPRG_MEM_TYPE_FUSE                  0x04
#define XPRG_MEM_TYPE_LOCKBITS              0x05
#define XPRG_MEM_TYPE_USERSIG               0x06
#define XPRG_MEM_TYPE_FACTORY_CALIBRATION   0x07

// TPI Erase modes
#define XPRG_ERASE_CHIP                     0x01
#define XPRG_ERASE_APP                      0x02
#define XPRG_ERASE_BOOT                     0x03
#define XPRG_ERASE_EEPROM                   0x04
#define XPRG_ERASE_APP_PAGE                 0x05
#define XPRG_ERASE_BOOT_PAGE                0x06
#define XPRG_ERASE_EEPROM_PAGE              0x07
#define XPRG_ERASE_USERSIG                  0x08
// Erase types for Tiny XPROG
#define XPRG_ERASE_CONFIG                   0x09

// TPI Parameters
// XPROG parameters of different sizes
// 4-byte address
#define XPRG_PARAM_NVMBASE                  0x01
// 2-byte page size
#define XPRG_PARAM_EEPPAGESIZE              0x02
// tiny TPI, 1-byte address
#define XPRG_PARAM_NVMCMD_ADDR              0x03
#define XPRG_PARAM_NVMCSR_ADDR              0x04

#define TPI_NVMCMD_ADDRESS                  0x33
#define TPI_NVMCSR_ADDRESS                  0x32


#if !defined(JTAG3_PRIVATE_EXPORTED)

struct mega_device_desc {
    unsigned char flash_page_size[2];   // in bytes
    unsigned char flash_size[4];        // in bytes
    unsigned char dummy1[4];            // always 0
    unsigned char boot_address[4];      // maximal (BOOTSZ = 3) bootloader
                                        // address, in 16-bit words (!)
    unsigned char sram_offset[2];       // pointing behind IO registers
    unsigned char eeprom_size[2];
    unsigned char eeprom_page_size;
    unsigned char ocd_revision;         // see XML; basically:
                                        // t13*, t2313*, t4313:        0
                                        // all other DW devices:       1
                                        // ATmega128(A):               1 (!)
                                        // ATmega16*,162,169*,32*,64*: 2
                                        // ATmega2560/2561:            4
                                        // all other megaAVR devices:  3
    unsigned char always_one;           // always = 1
    unsigned char allow_full_page_bitstream; // old AVRs, see XML
    unsigned char dummy2[2];            // always 0
                                        // all IO addresses below are given
                                        // in IO number space (without
                                        // offset 0x20), even though e.g.
                                        // OSCCAL always resides outside
    unsigned char idr_address;          // IDR, aka. OCDR
    unsigned char eearh_address;        // EEPROM access
    unsigned char eearl_address;
    unsigned char eecr_address;
    unsigned char eedr_address;
    unsigned char spmcr_address;
    unsigned char osccal_address;
};


/* Xmega device descriptor */
struct xmega_device_desc {
    unsigned char nvm_app_offset[4];	// NVM offset for application flash
    unsigned char nvm_boot_offset[4];	// NVM offset for boot flash
    unsigned char nvm_eeprom_offset[4]; // NVM offset for EEPROM
    unsigned char nvm_fuse_offset[4];	// NVM offset for fuses
    unsigned char nvm_lock_offset[4];	// NVM offset for lock bits
    unsigned char nvm_user_sig_offset[4]; // NVM offset for user signature row
    unsigned char nvm_prod_sig_offset[4]; // NVM offset for production sign. row
    unsigned char nvm_data_offset[4];	// NVM offset for data memory (SRAM + IO)
    unsigned char app_size[4];		// size of application flash
    unsigned char boot_size[2];		// size of boot flash
    unsigned char flash_page_size[2];	// flash page size
    unsigned char eeprom_size[2];	// size of EEPROM
    unsigned char eeprom_page_size;	// EEPROM page size
    unsigned char nvm_base_addr[2];	// IO space base address of NVM controller
    unsigned char mcu_base_addr[2];	// IO space base address of MCU control
};

/* UPDI device descriptor */
struct updi_device_desc {
    unsigned char prog_base[2];
    unsigned char flash_page_size;
    unsigned char eeprom_page_size;
    unsigned char nvm_base_addr[2];
    unsigned char ocd_base_addr[2];

    // Configuration below, except for "Extended memory support", is only used by kits with
    // embedded debuggers (XPlained, Curiosity, ...).
    unsigned char default_min_div1_voltage[2];  // Default minimum voltage for 32M => 4.5V -> 4500
    unsigned char default_min_div2_voltage[2];  // Default minimum voltage for 16M => 2.7V -> 2700
    unsigned char default_min_div4_voltage[2];  // Default minimum voltage for 8M  => 2.2V -> 2200
    unsigned char default_min_div8_voltage[2];  // Default minimum voltage for 4M  => 1.5V -> 1500

    unsigned char pdi_pad_fmax[2];  // 750

    unsigned char flash_bytes[4];     // Flash size in bytes
    unsigned char eeprom_bytes[2];    // EEPROM size in bytes
    unsigned char user_sig_bytes[2];  // UserSignture size in bytes
    unsigned char fuses_bytes;        // Fuses size in bytes

    unsigned char syscfg_offset;          // Offset of SYSCFG0 within FUSE space
    unsigned char syscfg_write_mask_and;  // AND mask to apply to SYSCFG0 when writing
    unsigned char syscfg_write_mask_or;   // OR mask to apply to SYSCFG0 when writing
    unsigned char syscfg_erase_mask_and;  // AND mask to apply to SYSCFG0 after erase
    unsigned char syscfg_erase_mask_or;   // OR mask to apply to SYSCFG0 after erase

    unsigned char eeprom_base[2];          // Base address for EEPROM memory
    unsigned char user_sig_base[2];  // Base address for UserSignature memory
    unsigned char signature_base[2];       // Base address for Signature memory
    unsigned char fuses_base[2];           // Base address for Fuses memory
    unsigned char lockbits_base[2];        // Base address for Lockbits memory

    unsigned char device_id[2];  // Two last bytes of the device ID

    // Extended memory support. Needed for flash >= 64kb
    unsigned char prog_base_msb;        // Extends prog_base, used in 24-bit mode
    unsigned char flash_page_size_msb;  // Extends flash_page_size, used in 24-bit mode

    unsigned char address_mode;  // 0x00 = 16-bit mode, 0x01 = 24-bit mode

    unsigned char hvupdi_variant; // Indicates the target UPDI HV implementation
};
#endif /* JTAG3_PRIVATE_EXPORTED */
