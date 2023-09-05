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

// Returns how many connected devices are matched by vid, pid and serial number
static int sa_num_matches_by_ids(int vid, int pid, const char *snq, SERPORT *sp, int n) {
  int nm = 0;
  for(int i = 0; i < n; i++)
    nm += sp && sp[i].vid == vid && sp[i].pid == pid && sa_snmatch(sp[i].sernum, snq);

  return nm;
}

// Returns how many connected devices are matched by the avrdude.conf serial adapter sea
static int sa_num_matches_by_sea(const SERIALADAPTER *sea, SERPORT *sp, int n) {
  int nm = 0;
  if(sea)
    for(LNODEID pid = lfirst(sea->usbpid); pid; pid = lnext(pid)) // Assume pid not twice in list
      nm += sa_num_matches_by_ids(sea->usbvid, *(int *) ldata(pid), sea->usbsn, sp, n);

  return nm;
}

// Print " or -P ..." for all avrdude.conf serial adapters uniquely matching the target and return number of matches
static int list_matching_serialadapters(SERPORT *target, SERPORT *sp, int n) {
  LNODEID ln1, ln2, ln3;
  SERIALADAPTER *sea;
  int nm = 0;

  for(ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    sea = ldata(ln1);
    if(!is_serialadapter(sea))
      continue;
    for(ln2=lfirst(sea->id); ln2; ln2=lnext(ln2)) {
      const char *id = ldata(ln2);
      if (*id == 0 || *id == '.')
        continue;
      for(ln3=lfirst(sea->usbpid); ln3; ln3=lnext(ln3))
        if(sa_num_matches_by_ids(sea->usbvid, *(int *) ldata(ln3), sea->usbsn, target, 1))
          if(1 == sa_num_matches_by_ids(sea->usbvid, *(int *) ldata(ln3), sea->usbsn, sp, n)) {
            msg_info(" or -P %s", id);
            if(*sea->usbsn)
              msg_info(":%s", sea->usbsn);
            nm++;
          }
    }
  }

  return nm;
}

int setport_from_serialadapter(char **portp, const SERIALADAPTER *ser, const char *sernum) {
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

  // Count the number of available ports and allocate space according to the needed size
  int n;
  for (n = 0; port_list[n]; n++)
    continue;
  SERPORT *sp = cfg_malloc(__func__, n*sizeof*sp);

  int i;
  for (i = 0; i < n; i++) {
    struct sp_port *prt = port_list[i];

    // Fill sp struct with port information
    if (sp_get_port_usb_vid_pid(prt, &sp[i].vid, &sp[i].pid) != SP_OK)
      sp[i].vid = sp[i].pid = 0;
    if (sp_get_port_name(prt))
      sp[i].port = cfg_strdup(__func__, sp_get_port_name(prt));
    if (sp_get_port_usb_serial(prt))
      sp[i].sernum = cfg_strdup(__func__, sp_get_port_usb_serial(prt));

    // Check for USB VID/PID/SN match
    if (sp[i].vid == ser->usbvid) {
      for (LNODEID usbpid = lfirst(ser->usbpid); usbpid; usbpid = lnext(usbpid)) {
        // USB PID match
        if (sp[i].pid == *(int *)(ldata(usbpid))) {
          sp[i].match = true;
          // SN present
          if ((sernum && sernum[0]) || ser->usbsn[0]) {
            // SN matches
            if ((sernum[0] && sp[i].sernum && str_starts(sp[i].sernum, sernum)) ||
               (sernum[0] && str_starts(sernum , "...") && str_ends(sp[i].sernum, sernum+3)) ||
               (sp[i].sernum && str_eq(sp[i].sernum, sernum && sernum[0]? sernum: ser->usbsn)))
              sp[i].match = true;
            // SN does not match
            else
              sp[i].match = false;
          }
          // SN present in avrduderc but did not match
          else if (!sernum[0] && ser->usbsn[0])
            sp[i].match = false;
        }
      }
    }
    else
      sp[i].match = false;
  }

  bool is_unique = true;
  for (int j = 0; j < i; j++) {
    static bool m;
    if (m && sp[j].match) {
      pmsg_warning("-P %s is not unique; consider one of the below\n", *portp);
      for (int k = 0; k < i; k++) {
        if (sp[k].match) {
          int l = k;
          for (; l < i; l++) {
            if (!sp[k].sernum || (!sp[k].sernum[0] && str_eq(sp[k].sernum, sp[l].sernum)))
              break;
          }
          // SN is unique
          if (l == i && sp[k].sernum[0])
            msg_warning("-P %s or -P %s:%s\n", sp[k].port, *portp, sp[k].sernum);
          // SN is not present or not unique for the part
          else
            msg_warning("-P %s (via %s serial adapter)\n", sp[k].port, *portp);
        }
      }
      is_unique = false;
      rv = -2;
      break;
    }
    if (sp[j].match)
      m = true;
  }

  // Overwrite portp is passed port is unique
  if (is_unique) {
    for (int k = 0; k < i; k++) {
      if (sp[k].match)
        rv = sa_setport(portp, sp[k].port);
    }
  }

  for (int l = 0; l < n; l++) {
    free(sp[l].sernum);
    free(sp[l].port);
  }
  free(sp);
  sp_free_port_list(port_list); // Free the array created by sp_list_ports()
  return rv;
}

int setport_from_vid_pid(char **portp, int vid, int pid, const char *sernum) {
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

  // Count the number of available ports and allocate space according to the needed size
  int n;
  for (n = 0; port_list[n]; n++)
    continue;
  SERPORT *sp = cfg_malloc(__func__, n*sizeof*sp);
  int i;
  for (i = 0; i < n; i++) {
    struct sp_port *prt = port_list[i];

    // Fill sp struct with port information
    if (sp_get_port_usb_vid_pid(prt, &sp[i].vid, &sp[i].pid) != SP_OK)
      sp[i].vid = sp[i].pid = 0;
    if (sp_get_port_name(prt))
      sp[i].port = cfg_strdup(__func__, sp_get_port_name(prt));
    if (sp_get_port_usb_serial(prt))
      sp[i].sernum = cfg_strdup(__func__, sp_get_port_usb_serial(prt));

    // Check for USB VID/PID/SN match
    if (sp[i].vid == vid && sp[i].pid == pid) {
      sp[i].match = true;
      // SN present
      if (sernum && sernum[0]) {
        // SN matches
        if ((sp[i].sernum && str_starts(sp[i].sernum, sernum)) ||
            (str_starts(sernum , "...") && str_ends(sp[i].sernum, sernum+3)))
          sp[i].match = true;
        // SN does not match
        else
          sp[i].match = false;
      }
    }
    else
      sp[i].match = false;
  }

  bool is_unique = true;
  for (int j = 0; j < i; j++) {
    static bool m;
    if (m && sp[j].match) {
      pmsg_warning("-P %s is not unique; consider one of the below\n", *portp);
      for (int k = 0; k < i; k++) {
        if (sp[k].match) {
          int l = k;
          for (; l < i; l++) {
            if (!sp[k].sernum || (!sp[k].sernum[0] && str_eq(sp[k].sernum, sp[l].sernum)))
              break;
          }
          // SN is unique
          if (l == i && sp[k].sernum[0])
            msg_warning("-P %s or -P %s:%s\n", sp[k].port, *portp, sp[k].sernum);
          // SN is not present or not unique for the part
          else
            msg_warning("-P %s (via %s serial adapter)\n", sp[k].port, *portp);
        }
      }
      is_unique = false;
      rv = -2;
      break;
    }
    if (sp[j].match)
      m = true;
  }

  // Overwrite portp is passed port is unique
  if (is_unique) {
    for (int k = 0; k < i; k++) {
      if (sp[k].match)
        rv = sa_setport(portp, sp[k].port);
    }
  }

  for (int l = 0; l < n; l++) {
    free(sp[l].sernum);
    free(sp[l].port);
  }
  free(sp);
  sp_free_port_list(port_list); // Free the array created by sp_list_ports()
  return rv;
}

int print_available_serialports(LISTID programmers) {
  struct sp_port **port_list;

  /* Call sp_list_ports() to get the ports. The port_list
   * pointer will be updated to refer to the array created */
  enum sp_return result = sp_list_ports(&port_list);

  if (result != SP_OK) {
    pmsg_error("sp_list_ports() failed!\n");
    return -1;
  }

  // Count the number of available ports and allocate space according to the needed size
  int n;
  for (n = 0; port_list[n]; n++)
    continue;
  SERPORT *sp = cfg_malloc(__func__, n*sizeof*sp);

  for (int i = 0; i < n; i++) {
    struct sp_port *prt = port_list[i];
    // Fill sp struct with port information
    if (sp_get_port_usb_vid_pid(prt, &sp[i].vid, &sp[i].pid) != SP_OK)
      sp[i].vid = sp[i].pid = 0;
    if (sp_get_port_name(prt))
      sp[i].port = cfg_strdup(__func__, sp_get_port_name(prt));
    if (sp_get_port_usb_serial(prt))
      sp[i].sernum = cfg_strdup(__func__, sp_get_port_usb_serial(prt));
    else
      sp[i].sernum = cfg_malloc(__func__, 1);
  }

  // Flag non-unique serial adapters
  for (int j = 0; j < n; j++) {
    for (int k = j+1; k < n; k++) {
      if ((sp[j].vid == sp[k].vid && sp[j].pid == sp[k].pid) &&
         ((!sp[j].sernum[0] && !sp[k].sernum[0]) || str_eq(sp[j].sernum, sp[k].sernum)))
          sp[j].match = sp[k].match = true;
    }
  }

  if (sp[0].port) {
    msg_info("Possible candidate serial ports are:\n");
    for (int j = 0; j < n; j++) {
      msg_info("-P %s", sp[j].port);
      if (sp[j].vid && sp[j].pid) {
        char *serid = NULL;
        // Loop though all programmers
        for (LNODEID ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
          SERIALADAPTER *sea = ldata(ln1);
          if (!is_serialadapter(sea))
            continue;
          // Loop though the USB pid list
          for (LNODEID ln2 = lfirst(sea->usbpid); ln2; ln2=lnext(ln2)) {
            // Serial adapter USB VID and PID matches
            if (sp[j].vid == sea->usbvid && sp[j].pid == *(int *)ldata(ln2))
              serid = lfirst(lfirst(sea->id));
          }
        }
        if (serid) {
          if (!sp[j].match)
            msg_info(" or -P %s", serid);
          else
            msg_info(" (via %s serial adapter)", serid);
        }
        else {
          if (!sp[j].match) {
            msg_info(" or -P usb:%04x:%04x", sp[j].vid, sp[j].pid);
            if (sp[j].sernum && sp[j].sernum[0])
              msg_info(":%s", sp[j].sernum);
          }
          else
            msg_info(" via usb:%04x:%04x", sp[j].vid, sp[j].pid);
        }
        if (!serid)
          msg_info(" (serial adapter unknown to avrdude.conf)");
      }
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
  sp_free_port_list(port_list); // Free the array created by sp_list_ports()
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
