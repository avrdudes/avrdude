#include <libavrdude.h>

#include "ac_cfg.h"

static
const char *const libavrdude_buildinfo[] = {
  AVRDUDE_FULL_VERSION,
  "buildsystem: " AVRDUDE_BUILDSYSTEM,
#ifdef HAVE_LIBELF
  "libelf",
#endif
#ifdef HAVE_LIBUSB
  "libusb",
#endif
#ifdef HAVE_LIBUSB_1_0
  "libusb_1_0",
#endif
#ifdef HAVE_LIBHIDAPI
  "libhidapi",
#endif
#ifdef HAVE_LIBHID
  "libhid",
#endif
#ifdef HAVE_LIBFTDI
  "libftdi",
#endif
#ifdef HAVE_LIBFTDI1
  "libftdi1",
#endif
#ifdef HAVE_LIBREADLINE
  "libreadline",
#endif
#ifdef HAVE_LIBSERIALPORT
  "libserialport",
#endif
#ifdef HAVE_PARPORT
  "parport",
#endif
#ifdef HAVE_LINUXGPIO
  "linuxgpio",
#endif
#ifdef HAVE_LINUXSPI
  "linuxspi",
#endif
  NULL
};

const char *const *avr_get_buildinfo(void)
{
  return libavrdude_buildinfo;
}
