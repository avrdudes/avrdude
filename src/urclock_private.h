/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2022 Stefan Rueger <stefan.rueger@urclocks.com>
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

#ifndef urclock_private_h
#define urclock_private_h

// EEPROM or flash cache for bytewise access
typedef struct {
  int base, size;
  char *page, *copy;
} Cache;


// STK500v1 protocol constants

#define Resp_STK_OK             0x10
#define Resp_STK_INSYNC         0x14

#define Sync_CRC_EOP            0x20

#define Cmnd_STK_GET_SYNC       0x30
#define Cmnd_STK_ENTER_PROGMODE 0x50
#define Cmnd_STK_LEAVE_PROGMODE 0x51
#define Cmnd_STK_CHIP_ERASE     0x52
#define Cmnd_STK_LOAD_ADDRESS   0x55
#define Cmnd_STK_UNIVERSAL      0x56

#define Cmnd_STK_PROG_PAGE      0x64
#define Cmnd_STK_READ_PAGE      0x74
#define Cmnd_STK_READ_SIGN      0x75


// Urprotocol command extensions to STK500v1

#define Cmnd_UR_PROG_PAGE_EE    0x00
#define Cmnd_UR_READ_PAGE_EE    0x01

#define Cmnd_UR_PROG_PAGE_FL    0x02
#define Cmnd_UR_READ_PAGE_FL    0x03


// STK_UNIVERSAL commands for backward compatibility
#define Subc_STK_UNIVERSAL_LEXT 0x4d000000u // Load extended address
#define Subc_STK_UNIVERSAL_CE   0xac800000u // Chip erase


// Urboot protocol side channel info about MCU id and 5 binary bootloader features

// Number of differnt MCU ids
#define UB_N_MCU           2040 // MCU id 0..2039

// 5 bootloader features
#define UB_RESERVED_1         1
#define UB_RESERVED_2         2
#define UB_READ_FLASH         4 // Bootloader can read flash
#define UB_FLASH_LL_NOR       8 // Bootloader flash programming looks like a NOR memory
#define UB_CHIP_ERASE        16 // Bootloader has a flash-only chip erase that protects itself

#define UB_INFO(ub_features, ub_mcuid) (ub_features*UB_N_MCU + ub_mcuid)
#define UB_FEATURES(ub_info) ((uint16_t)(ub_info)/UB_N_MCU)
#define UB_MCUID(ub_info)    ((uint16_t)(ub_info)%UB_N_MCU)


/*
 * Urboot layout of top six bytes
 *
 * FLASHEND-5: numblpags, only from v7.5: 1 byte number 1..127 of bootloader flash pages
 * FLASHEND-4: vblvecnum, only from v7.5: 1 byte vector number 1..127 for vector bootloader
 * FLASHEND-3: 2 byte rjmp opcode to bootloader pgm_write_page(sram, flash) or ret opcode
 * FLASHEND-1: capability byte of bootloader
 * FLASHEND-0: version number of bootloader: 5 msb = major version, 3 lsb = minor version
 */

// Capability byte of bootloader from version 7.2 onwards
#define UR_PGMWRITEPAGE     128 // pgm_write_page() can be called from application at FLASHEND+1-4
#define UR_AUTOBAUD         128 // Bootloader has autobaud detection (from v7.7)
#define UR_EEPROM            64 // EEPROM read/write support
#define UR_URPROTOCOL        32 // Using urprotocol (v7.6 and v7.7 only)
#define UR_EXPEDITE          32 // Check need to write flash first (from v8.0)
#define UR_DUAL              16 // Dual boot
#define UR_VBLMASK           12 // Vector bootloader bits (up to v7.7 only)
#define UR_VBLPATCHVERIFY    12 // Patch reset/interrupt vectors and show original ones on verify
#define UR_VBLPATCH           8 // Patch reset/interrupt vectors only (expect an error on verify)
#define UR_VBL                4 // Merely start application via interrupt vector instead of reset
#define UR_NO_VBL             0 // Not a vector bootloader, must set fuses to HW bootloader support
#define UR_PROTECTME          2 // Bootloader safeguards against overwriting itself
#define UR_PROTECTRESET       2 // Bootloader safeguards against overwriting itself and reset
#define UR_RESETFLAGS         1 // Load reset flags into register R2 before starting application
#define UR_HAS_CE             1 // Bootloader has Chip Erase (from v7.7)

#define verbyte_cv(capver)      ((uint8_t) ((uint16_t) (capver) >> 8))
#define hascapbyte_cv(capver)   ({ uint8_t _vh = verbyte_cv(capver); _vh >= 072 && _vh != 0xff; })
#define hasextendedv_cv(capver) ({ uint8_t _vh = verbyte_cv(capver); _vh >= 075 && _vh != 0xff; })
#define capabilities_cv(capver) ({ uint16_t _vc = capver; \
  (uint8_t) (hascapbyte_cv(_vc)? _vc&0xff: 0); })
#define vblvecnum_cv(capver)    ({ uint16_t _vc = capver; \
  (uint8_t) (hasextendedv_cv(_vc)? pgm_read_b1(FLASHEND-4): 0); })
#define numblpages_cv(capver)   ({ uint16_t _vc = capver; \
  (uint8_t) (hasextendedv_cv(_vc)? pgm_read_b1(FLASHEND-5): 0); })
#define blurversion_cv(capver)  ({ uint8_t _vh = verbyte_cv(capver); \
  (uint8_t) (_vh >= 072 && _vh != 0xff? _vh: 0); })

#define vercapis(capver, mask)  ({ uint16_t _vi = capver; !!(capabilities_cv(_vi) & (mask)); })
#define isautobaud_cv(capver)        vercapis(capver, UR_AUTOBAUD)     // from v7.7
#define iseeprom_cv(capver)          vercapis(capver, UR_EEPROM)
#define isdual_cv(capver)            vercapis(capver, UR_DUAL)
#define isprotectreset_cv(capver)    vercapis(capver, UR_PROTECTRESET) // from 7.7
#define ishas_ce_cv(capver)          vercapis(capver, UR_HAS_CE)       // from v7.7

// Up to v7.7
#define isurprotocol077_cv(capver)   vercapis(capver, UR_URPROTOCOL)
#define isvectorbl077_cv(capver)     vercapis(capver, UR_VBLMASK)

// Up to v7.6
#define ispgmwritepage076_cv(capver) vercapis(capver, UR_PGMWRITEPAGE)
#define isprotectme076_cv(capver)    vercapis(capver, UR_PROTECTME)
#define isresetflags076_cv(capver)   vercapis(capver, UR_RESETFLAGS)


// Capability bits incl position
#define autobaud_bit_cap(cap)        ((cap) & UR_AUTOBAUD)     // from v7.7
#define eeprom_bit_cap(cap)          ((cap) & UR_EEPROM)
#define dual_bit_cap(cap)            ((cap) & UR_DUAL)
#define protectreset_bit_cap(cap)    ((cap) & UR_PROTECTRESET) // from v7.7
#define has_ce_bit_cap(cap)          ((cap) & UR_HAS_CE)       // from v7.7

// Up to v7.7
#define urprotocol077_bit_cap(cap)   ((cap) & UR_URPROTOCOL)
#define vector077_bits_cap(cap)      ((cap) & UR_VBLMASK))

// Up to v7.6
#define pgmwritepage076_bit_cap(cap) ((cap) & UR_PGMWRITEPAGE)
#define protectme076_bit_cap(cap)    ((cap) & UR_PROTECTME)
#define resetflags076_bit_cap(cap)   ((cap) & UR_RESETFLAGS)


// Boolean capabilities
#define isautobaud_cap(cap)        (!!((cap) & UR_AUTOBAUD))     // from v7.7
#define iseeprom_cap(cap)          (!!((cap) & UR_EEPROM))
#define isdual_cap(cap)            (!!((cap) & UR_DUAL))
#define isprotectreset_cap(cap)    (!!((cap) & UR_PROTECTRESET)) // from v7.7
#define ishas_ce_cap(cap)          (!!((cap) & UR_HAS_CE))       // from v7.7

// Up to v7.7
#define isurprotocol077_cap(cap)   (!!((cap) & UR_URPROTOCOL))
#define isvectorbl077_cap(cap)     (!!((cap) & UR_VBLMASK)))

// Up to v7.6
#define ispgmwritepage076_cap(cap) (!!((cap) & UR_PGMWRITEPAGE))
#define isprotectme076_cap(cap)    (!!((cap) & UR_PROTECTME))
#define isresetflags076_cap(cap)   (!!((cap) & UR_RESETFLAGS))

// Capability levels 0, 1, 2 or 3 (only for bootloaders up to v7.7)
#define vectorbl_level077_cap(cap) (((cap) & UR_VBLMASK)/(UR_VBLMASK & -UR_VBLMASK))

#endif
