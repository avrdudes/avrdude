/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2023 Stefan Rueger <stefan.rueger@urclocks.com>
 * Copyright (C) 2023 Hans Eirik Bull
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

#include "ac_cfg.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "avrdude.h"
#include "libavrdude.h"

#ifdef HAVE_LIBSERIALPORT

#include <libserialport.h>


// Set new port string freeing any previously set one
static int sa_setport(char **portp, const char *sp_port) {
  if(!sp_port) {
    pmsg_warning("port string to be assigned is NULL\n");
    return -1;
  }

  if(portp) {
    if(*portp)
      free(*portp);
    *portp = cfg_strdup(__func__, sp_port);
  }

  return 0;
}


int find_serialport_adapter(char **portp, const SERIALADAPTER *ser, const char *sernum) {
  int rv = -1;
  /* A pointer to a null-terminated array of pointers to
   * struct sp_port, which will contain the ports found */
  struct sp_port **port_list;

  /* Call sp_list_ports() to get the ports. The port_list
   * pointer will be updated to refer to the array created */
  enum sp_return result = sp_list_ports(&port_list);

  if (result != SP_OK) {
    pmsg_error("sp_list_ports() failed!\n");
    return -1;
  }

  for (int i = 0; port_list[i]; i++) {
    struct sp_port *prt = port_list[i];
    int usb_vid, usb_pid;
    sp_get_port_usb_vid_pid(prt, &usb_vid, &usb_pid);

    // USB VID match
    if(usb_vid == ser->usbvid) {
      for (LNODEID usbpid = lfirst(ser->usbpid); usbpid; usbpid = lnext(usbpid)) {
        // USB PID match
        if (usb_pid == *(int *)(ldata(usbpid))) {
          // SN present
          if (sernum && sernum[0]) {
            char *s = sp_get_port_usb_serial(prt);
            // SN matches
            if(s && str_eq(sernum, s))
              rv = sa_setport(portp, sp_get_port_name(prt));
          }
          // SN not present
          else {
            rv = sa_setport(portp, sp_get_port_name(prt));
          }
        }
      }
    }
  }

  sp_free_port_list(port_list); // Free the array created by sp_list_ports()
  return rv;
}

int find_serialport_vid_pid(char **portp, int vid, int pid, const char *sernum) {
  int rv = -1;
  /* A pointer to a null-terminated array of pointers to
   * struct sp_port, which will contain the ports found */
  struct sp_port **port_list;

  /* Call sp_list_ports() to get the ports. The port_list
   * pointer will be updated to refer to the array created */
  enum sp_return result = sp_list_ports(&port_list);

  if (result != SP_OK) {
    pmsg_error("sp_list_ports() failed!\n");
    return -1;
  }

  for (int i = 0; port_list[i]; i++) {
    struct sp_port *prt = port_list[i];
    int usb_vid, usb_pid;
    sp_get_port_usb_vid_pid(prt, &usb_vid, &usb_pid);

    // USB VID and PIDmatch
    if(usb_vid == vid && usb_pid == pid) {
      // SN present
      if (sernum && sernum[0]) {
        char *s = sp_get_port_usb_serial(prt);
        // SN matches
        if(s && str_eq(sernum, s))
          rv = sa_setport(portp, sp_get_port_name(prt));
      }
      // SN not present
      else {
        rv = sa_setport(portp, sp_get_port_name(prt));
      }
    }
  }

  sp_free_port_list(port_list); // Free the array created by sp_list_ports()
  return rv;
}

#else

int find_serialport_adapter(char **portp, const SERIALADAPTER *ser, const char *sernum) {
  pmsg_error("avrdude built without libserialport support; please compile again with libserialport installed\n");
  return -1;
}

int find_serialport_vid_pid(char **portp, int vid, int pid, const char *sernum) {
  pmsg_error("avrdude built without libserialport support; please compile again with libserialport installed\n");
  return -1;
}

#endif

void list_serialadapters(FILE *fp, const char *prefix, LISTID programmers) {
  LNODEID ln1, ln2, ln3;
  SERIALADAPTER *sea;
  int maxlen=0, len;

  sort_programmers(programmers);

  // Compute max length of serial adapter names
  for(ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    sea = ldata(ln1);
    if(!is_serialadapter(sea))
      continue;
    for(ln2=lfirst(sea->id); ln2; ln2=lnext(ln2)) {
      const char *id = ldata(ln2);
      if(*id == 0 || *id == '.')
        continue;
      if((len = strlen(id)) > maxlen)
        maxlen = len;
    }
  }

  for(ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    sea = ldata(ln1);
    if(!is_serialadapter(sea))
      continue;
    for(ln2=lfirst(sea->id); ln2; ln2=lnext(ln2)) {
      const char *id = ldata(ln2);
      if(*id == 0 || *id == '.')
        continue;
      fprintf(fp, "%s%-*s = [usbvid 0x%04x, usbpid", prefix, maxlen, id, sea->usbvid);
      for(ln3=lfirst(sea->usbpid); ln3; ln3=lnext(ln3))
        fprintf(fp, " 0x%04x", *(int *) ldata(ln3));
      if(sea->usbsn && *sea->usbsn)
        fprintf(fp, ", usbsn %s", sea->usbsn);
      fprintf(fp, "]\n");
    }
  }
}


void serialadapter_not_found(const char *sea_id) {
  msg_error("\v");
  if(sea_id && *sea_id)
    pmsg_error("cannot find serial adapter id %s\n", sea_id);

  msg_error("\nValid serial adapters are:\n");
  list_serialadapters(stderr, "  ", programmers);
  msg_error("\n");
}
