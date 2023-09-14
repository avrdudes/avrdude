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

typedef struct {
  int vid;
  int pid;
  bool match;
  bool unique;
  char *sernum;
  char *port;
} SERPORT;

// Set new port string freeing any previously set one
static int sa_setport(char **portp, const char *sp_port) {
  if (!sp_port) {
    pmsg_warning("port string to be assigned is NULL\n");
    return -1;
  }

  if (portp) {
    if (*portp)
      free(*portp);
    *portp = cfg_strdup(__func__, sp_port);
  }

  return 0;
}

// Is the actual serial number sn matched by the query q?
static int sa_snmatch(const char *sn, const char *q) {
  return sn && (str_starts(sn, q) || (str_starts(q , "...") && str_ends(sn, q+3)));
}

// Get serial port data and store it to a struct that this function returns a pointer to.
// Store the number of serial ports to int pointer n.
static SERPORT *get_libserialport_data(int *n) {
  struct sp_port **port_list;
  enum sp_return result = sp_list_ports(&port_list);

  if (result != SP_OK) {
    pmsg_error("sp_list_ports() failed!\n");
    sp_free_port_list(port_list);
    *n = -1;
    return NULL;
  }

  int i;
  for (i = 0; port_list[i]; i++)
    continue;
  *n = i;
  SERPORT *s = cfg_malloc(__func__, i*sizeof*s);

  // Fill struct with port information
  for (int j = 0; j < i; j++) {
    struct sp_port *prt = port_list[j];
    if (sp_get_port_usb_vid_pid(prt, &s[j].vid, &s[j].pid) != SP_OK)
      s[j].vid = s[j].pid = 0;
    if (sp_get_port_name(prt))
      s[j].port = cfg_strdup(__func__, sp_get_port_name(prt));
    if (sp_get_port_usb_serial(prt))
      s[j].sernum = cfg_strdup(__func__, sp_get_port_usb_serial(prt));
    else
      s[j].sernum = cfg_malloc(__func__, 1);
  }
  // Flag unique serial adapters
  for (int j = 0; j < i; j++) {
    bool unique = true;
    for (int k = 0; k < i; k++) {
      if (j != k) {
        if (s[j].vid == s[k].vid && s[j].pid == s[k].pid) {
          if ((sa_snmatch(s[j].sernum, s[k].sernum) && !s[k].sernum) ||
              (!s[j].sernum && !s[k].sernum)) {
            unique = false;
            }
        }
      }
    }
    s[j].unique = unique;
  }

  sp_free_port_list(port_list);
  return s;
}

static int sa_num_matches_by_sea(const SERIALADAPTER *sea, SERPORT *sp, const char *sernum, int n) {
  int matches = 0;
  for (int i = 0; i < n; i++) {
    if (sp[i].vid == sea->usbvid) {
      for (LNODEID usbpid = lfirst(sea->usbpid); usbpid; usbpid = lnext(usbpid)) {
        if (sp[i].pid == *(int *)(ldata(usbpid))) {
          const char *sn = *sernum? sernum: sea->usbsn;
          if(sa_snmatch(sp[i].sernum, sn)) // also matches NULL
            matches++;
        }
      }
    }
  }
  return matches;
}

static int sa_num_matches_by_vid_pid(int vid, int pid, SERPORT *sp, const char *sernum, int n) {
  int matches = 0;
  for (int i = 0; i < n; i++) {
    if (sp[i].vid == vid) {
      if (sp[i].pid == pid) {
        if(sa_snmatch(sp[i].sernum, sernum)) // also matches NULL
          matches++;
      }
    }
  }
  return matches;
}

int setport_from_serialadapter(char **portp, const SERIALADAPTER *ser, const char *sernum) {
  int rv = -1;
  int n;
  SERPORT *sp = get_libserialport_data(&n);
  if (n < 0)
    return rv;

  // Non-unique serial adapter specified
  if (1 < sa_num_matches_by_sea(ser, sp, sernum, n)) {
    pmsg_warning("-P %s is not unique; consider one of the below\n", *portp);
    for (int i = 0; i < n; i++) {
      if (sp[i].unique && sp[i].sernum[0])
        msg_warning("-P %s or -P %s:%s\n", sp[i].port, *portp, sp[i].sernum);
      else
        msg_warning("-P %s (via %s serial adapter)\n", sp[i].port, *portp);
    }
    rv = -2;
  }
  // Unique serial adapter specified
  else {
    for (int j = 0; j < n; j++) {
      if (sa_num_matches_by_sea(ser, &sp[j], sernum, 1))
        rv = sa_setport(portp, sp[j].port);
    }
  }

  for (int k = 0; k < n; k++) {
    free(sp[k].sernum);
    free(sp[k].port);
  }
  free(sp);
  return rv;
}

int setport_from_vid_pid(char **portp, int vid, int pid, const char *sernum) {
  int rv = -1;
  int n;
  SERPORT *sp = get_libserialport_data(&n);
  if (n < 0)
    return rv;

  // Non-unique serial adapter specified
  if (1 < sa_num_matches_by_vid_pid(vid, pid, sp, sernum, n)) {
    pmsg_warning("-P %s is not unique; consider one of the below\n", *portp);
    for (int i = 0; i < n; i++) {
      if (sp[i].unique && sp[i].sernum[0])
        msg_warning("-P %s or -P %s:%s\n", sp[i].port, *portp, sp[i].sernum);
      else
        msg_warning("-P %s (via %s serial adapter)\n", sp[i].port, *portp);
    }
    rv = -2;
  }
  // Unique serial adapter specified
  else {
    for (int j = 0; j < n; j++) {
      if (sa_num_matches_by_vid_pid(vid, pid, &sp[j], sernum, 1))
        rv = sa_setport(portp, sp[j].port);
    }
  }

  for (int k = 0; k < n; k++) {
    free(sp[k].sernum);
    free(sp[k].port);
  }
  free(sp);
  return rv;
}

int print_available_serialports(LISTID programmers) {
  // Get serial port information from libserialport
  int n;
  SERPORT *sp = get_libserialport_data(&n);
  if (n < 0)
    return -1;

  // Flag non-unique serial adapters
  for (int j = 0; j < n; j++) {
    for (int k = j+1; k < n; k++) {
      if ((sp[j].vid == sp[k].vid && sp[j].pid == sp[k].pid))
        sp[j].match = sp[k].match = true;
    }
  }

  if (sp[0].port) {
    msg_info("Possible candidate serial ports are:\n");
    for (int j = 0; j < n; j++) {
      msg_info("-P %s", sp[j].port);
      if (sp[j].vid && sp[j].pid) {
        bool serid = false;
        LNODEID ln1, ln2, ln3;
        // Loop though all programmers
        for (ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
          SERIALADAPTER *sea = ldata(ln1);
          if (!is_serialadapter(sea))
            continue;
          // Loop though USB pid list
          for (ln2 = lfirst(sea->usbpid); ln2; ln2=lnext(ln2)) {
            // Serial adapter USB VID and PID matches
            if (sp[j].vid == sea->usbvid && sp[j].pid == *(int *)ldata(ln2)) {
              // Loop though IDs
              for (ln3 = lfirst(sea->id); ln3; ln3=lnext(ln3)) {
                // Multiple matches
                if (sp[j].match) {
                  // SN present and matches
                  if (sa_snmatch(sp[j].sernum, sea->usbsn) && (sp[j].sernum[0] || sea->usbsn[0])) {
                    serid = true;
                    msg_info(", -P %s:%s", ldata(ln3), sp[j].sernum);
                  }
                  // SN not present or no match
                  else {
                    serid = true;
                    msg_info(" (via %s serial adapter)", ldata(ln3));
                    goto end;
                  }
                } else if (sa_snmatch(sp[j].sernum, sea->usbsn) || (!sp[j].sernum[0] && !sea->usbsn[0])) {
                  serid = true;
                  msg_info(", -P %s", ldata(ln3));
                }
              }
            }
          }
        }
        // Print USB vid/pid/sn if serial adapter is not known
        if (!serid) {
          if (!sp[j].match) {
            msg_info(" or -P usb:%04x:%04x", sp[j].vid, sp[j].pid);
            if (sp[j].sernum[0])
              msg_info(":%s", sp[j].sernum);
          }
          else
            msg_info(" via usb:%04x:%04x", sp[j].vid, sp[j].pid);
          msg_info(" (serial adapter unknown to avrdude.conf)");
        }
      }
      end:
      msg_info("\n");
    }
    msg_info("Note that above ports may not necessarily be connected to a target board or an AVR programmer.\n");
    msg_info("Also note there may be other direct serial ports not listed above.\n");
  }

  for (int k = 0; k < n; k++) {
    free(sp[k].sernum);
    free(sp[k].port);
  }
  free(sp);
  return 0;
}

#else

int setport_from_serialadapter(char **portp, const SERIALADAPTER *ser, const char *sernum) {
  pmsg_error("avrdude built without libserialport support; please compile again with libserialport installed\n");
  return -1;
}

int setport_from_vid_pid(char **portp, int vid, int pid, const char *sernum) {
  pmsg_error("avrdude built without libserialport support; please compile again with libserialport installed\n");
  return -1;
}

int print_available_serialports(LISTID programmers) {
  return -1;
}

#endif

void list_serialadapters(FILE *fp, const char *prefix, LISTID programmers) {
  LNODEID ln1, ln2, ln3;
  SERIALADAPTER *sea;
  int maxlen=0, len;

  sort_programmers(programmers);

  // Compute max length of serial adapter names
  for (ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    sea = ldata(ln1);
    if (!is_serialadapter(sea))
      continue;
    for (ln2=lfirst(sea->id); ln2; ln2=lnext(ln2)) {
      const char *id = ldata(ln2);
      if (*id == 0 || *id == '.')
        continue;
      if ((len = strlen(id)) > maxlen)
        maxlen = len;
    }
  }

  for (ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    sea = ldata(ln1);
    if (!is_serialadapter(sea))
      continue;
    for (ln2=lfirst(sea->id); ln2; ln2=lnext(ln2)) {
      const char *id = ldata(ln2);
      if (*id == 0 || *id == '.')
        continue;
      fprintf(fp, "%s%-*s = [usbvid 0x%04x, usbpid", prefix, maxlen, id, sea->usbvid);
      for (ln3=lfirst(sea->usbpid); ln3; ln3=lnext(ln3))
        fprintf(fp, " 0x%04x", *(int *) ldata(ln3));
      if (sea->usbsn && *sea->usbsn)
        fprintf(fp, ", usbsn %s", sea->usbsn);
      fprintf(fp, "]\n");
    }
  }
}


void serialadapter_not_found(const char *sea_id) {
  msg_error("\v");
  if (sea_id && *sea_id)
    pmsg_error("cannot find serial adapter id %s\n", sea_id);

  msg_error("\nValid serial adapters are:\n");
  list_serialadapters(stderr, "  ", programmers);
  msg_error("\n");
}
