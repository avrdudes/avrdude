/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * avrdude is Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
 *
 * This file: Copyright (C) 2005 Colin O'Flynn <coflynn@newae.com>
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


#include <stdio.h>

#include "ac_cfg.h"
#include "avr.h"
#include "pgm.h"
#include "safemode.h"

/* This value from ac_cfg.h */
char * progname = PACKAGE_NAME; 

/* Writes the specified fuse in fusename (can be "lfuse", "hfuse", or "efuse") and verifies it. Will try up to tries
amount of times before giving up */
int safemode_writefuse (unsigned char fuse, char * fusename, PROGRAMMER * pgm, AVRPART * p, int tries, int verbose)
{
  AVRMEM * m;
  unsigned char fuseread;
  int returnvalue = -1;
  
  m = avr_locate_mem(p, fusename);
  if (m == NULL) {
    return -1;
    }
 
  /* Keep trying to write then read back the fuse values */   
  while (tries > 0) {
    avr_write_byte(pgm, p, m, 0, fuse);
    avr_read_byte(pgm, p, m, 0, &fuseread);
        
    /* Report information to user if needed */
    if (verbose > 0) {
      fprintf(stderr, "%s: safemode: Wrote %s to %x, read as %x. %d attempts left\n",
             progname, fusename, fuse, fuseread, tries-1);
    }
    
    /* If fuse wrote OK, no need to keep going */
    if (fuse == fuseread) {
       tries = 0;
       returnvalue = 0;
    }
    tries--;
  }
  
  return returnvalue;
}

/* Reads the fuses three times, checking that all readings are the same. This will ensure that the before values aren't in error! */
int safemode_readfuses (char * lfuse, char * hfuse, char * efuse, PROGRAMMER * pgm, AVRPART * p, int verbose)  
{

  unsigned char value;
  unsigned char fusegood = 0;
  unsigned char safemode_lfuse;
  unsigned char safemode_hfuse;
  unsigned char safemode_efuse;
  AVRMEM * m;
  
  safemode_lfuse = *lfuse;
  safemode_hfuse = *hfuse;
  safemode_efuse = *efuse;

  /* Read lfuse three times */  
  fusegood = 2; /* If AVR device doesn't support this fuse, don't want
                   to generate a verify error */
  m = avr_locate_mem(p, "lfuse");
  if (m != NULL) {
    fusegood = 0; /* By default fuse is a failure */
    avr_read_byte(pgm, p, m, 0, &safemode_lfuse);
    avr_read_byte(pgm, p, m, 0, &value);
    if (value == safemode_lfuse) {
        avr_read_byte(pgm, p, m, 0, &value);
        if (value == safemode_lfuse){
        fusegood = 1; /* Fuse read OK three times */
        }
    }
  }

    if (fusegood == 0)	 {
        fprintf(stderr,
           "%s: safemode: Verify error - unable to read lfuse properly. "
           "Programmer may not be reliable.\n", progname);
        return -1;
    }
    else if ((fusegood == 1) && (verbose > 0)) {
        printf("%s: safemode: lfuse reads as %X\n", progname, safemode_lfuse);
    }

  /* Read hfuse three times */  
  fusegood = 2; /* If AVR device doesn't support this fuse, don't want
                   to generate a verify error */
  m = avr_locate_mem(p, "hfuse");
  if (m != NULL) {
    fusegood = 0; /* By default fuse is a failure */
    avr_read_byte(pgm, p, m, 0, &safemode_hfuse);
    avr_read_byte(pgm, p, m, 0, &value);
    if (value == safemode_hfuse) {
        avr_read_byte(pgm, p, m, 0, &value);
        if (value == safemode_hfuse){
             fusegood = 1; /* Fuse read OK three times */
        }
    }
  }

    if (fusegood == 0)	 {
            fprintf(stderr,
           "%s: safemode: Verify error - unable to read hfuse properly. "
           "Programmer may not be reliable.\n", progname);
       return -2;
    }
    else if ((fusegood == 1) && (verbose > 0)){
        printf("%s: safemode: hfuse reads as %X\n", progname, safemode_hfuse);
    }

  /* Read efuse three times */  
  fusegood = 2; /* If AVR device doesn't support this fuse, don't want
                   to generate a verify error */
  m = avr_locate_mem(p, "efuse");
  if (m != NULL) {
    fusegood = 0; /* By default fuse is a failure */
    avr_read_byte(pgm, p, m, 0, &safemode_efuse);
    avr_read_byte(pgm, p, m, 0, &value);
    if (value == safemode_efuse) {
        avr_read_byte(pgm, p, m, 0, &value);
        if (value == safemode_efuse){
             fusegood = 1; /* Fuse read OK three times */
        }
    }
  }
    
    if (fusegood == 0)	 {
        fprintf(stderr,
           "%s: safemode: Verify error - unable to read efuse properly. "
           "Programmer may not be reliable.\n", progname);
        return -3;
        }
    else if ((fusegood == 1) && (verbose > 0)) {
        printf("%s: safemode: efuse reads as %X\n", progname, safemode_efuse);
        }

  *lfuse = safemode_lfuse;
  *hfuse = safemode_hfuse;
  *efuse = safemode_efuse;

  return 0;
}


/* This routine will store the current values pointed to by lfuse, hfuse, and efuse into an internal buffer in this routine
when save is set to 1. When save is 0 (or not 1 really) it will copy the values from the internal buffer into the locations
pointed to be lfuse, hfuse, and efuse. This allows you to change the fuse bits if needed from another routine (ie: have it so
if user requests fuse bits are changed, the requested value is now verified */
int safemode_memfuses (int save, unsigned char * lfuse, unsigned char * hfuse, unsigned char * efuse)
{
  static unsigned char safemode_lfuse = 0xff;
  static unsigned char safemode_hfuse = 0xff;
  static unsigned char safemode_efuse = 0xff;
  
  switch (save) {
    
    /* Save the fuses as safemode setting */
    case 1:  
        safemode_lfuse = *lfuse;
        safemode_hfuse = *hfuse;
        safemode_efuse = *efuse;

        break;
    /* Read back the fuses */
    default:
        *lfuse = safemode_lfuse;
        *hfuse = safemode_hfuse;
        *efuse = safemode_efuse;
        break;
  }
  
  return 0;
}
