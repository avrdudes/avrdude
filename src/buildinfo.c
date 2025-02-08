#include <libavrdude.h>

#include <ac_cfg.h>


const avr_buildinfo libavrdude_buildinfo = {
  "libavrdude", AVRDUDE_FULL_VERSION,
  {
    {"buildsystem", AVRDUDE_BUILDSYSTEM},

#include <buildinfo-include.c>

    {"libelf",
#ifdef HAVE_LIBELF
     "yes"
#else
     NULL
#endif
    },

    {"libusb",
#ifdef HAVE_LIBUSB
     "yes"
#else
     NULL
#endif
    },

    {"libusb_1_0",
#ifdef HAVE_LIBUSB_1_0
     "yes"
#else
     NULL
#endif
    },

    {"libhidapi",
#ifdef HAVE_LIBHIDAPI
     "yes"
#else
     NULL
#endif
    },

    {"libhid",
#ifdef HAVE_LIBHID
     "yes"
#else
     NULL
#endif
    },

    {"libftdi",
#ifdef HAVE_LIBFTDI
     "yes"
#else
     NULL
#endif
    },

    {"libftdi1",
#ifdef HAVE_LIBFTDI1
     "yes"
#else
     NULL
#endif
    },

    {"libreadline",
#ifdef HAVE_LIBREADLINE
     "yes"
#else
     NULL
#endif
    },

    {"libserialport",
#ifdef HAVE_LIBSERIALPORT
    "yes"
#else
    NULL
#endif
    },

    {"parport",
#ifdef HAVE_PARPORT
     "yes"
#else
    NULL
#endif
    },

    {"linuxgpio",
#ifdef HAVE_LINUXGPIO
     "yes"
#else
    NULL
#endif
    },

    {"linuxspi",
#ifdef HAVE_LINUXSPI
     "yes"
#else
    NULL
#endif
    },

    {NULL, NULL},
  },
};
