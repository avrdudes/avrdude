#pragma once
#include "ac_cfg.h"

#include <stdint.h>

#if defined(HAVE_LIBFTDI1) && defined(HAVE_LIBUSB_1_0)
# if defined(HAVE_LIBUSB_1_0_LIBUSB_H)
#  include <libusb-1.0/libusb.h>
# else
#  include <libusb.h>
# endif
# include <libftdi1/ftdi.h>
# undef HAVE_LIBFTDI_TYPE_232H
# define HAVE_LIBFTDI_TYPE_232H 1
#elif defined(HAVE_LIBFTDI)
#include <ftdi.h>
#else

#ifdef _MSC_VER
#pragma message("No libftdi or libusb support. Install libftdi1/libusb-1.0 or libftdi/libusb and run configure/make again.")
#else
#warning No libftdi or libusb support. Install libftdi1/libusb-1.0 or libftdi/libusb and run configure/make again.
#endif
#define DO_NOT_BUILD_AVRFTDI
#endif

#ifndef DO_NOT_BUILD_AVRFTDI

enum { ERR, WARN, INFO, DEBUG, TRACE };

#define __log(lvl, fmt, ...)                                  \
  do {                                                        \
    avrftdi_log(lvl, __func__, __LINE__, fmt, ##__VA_ARGS__); \
  } while(0)


#define log_err(fmt, ...)   __log(ERR, fmt, ##__VA_ARGS__)
#define log_warn(fmt, ...)  __log(WARN,  fmt, ##__VA_ARGS__)
#define log_info(fmt, ...)  __log(INFO,  fmt, ##__VA_ARGS__)
#define log_debug(fmt, ...) __log(DEBUG, fmt, ##__VA_ARGS__)
#define log_trace(fmt, ...) __log(TRACE, fmt, ##__VA_ARGS__)

#define E(x, ftdi)                                                  \
  do {                                                              \
    if ((x))                                                        \
    {                                                               \
      msg_error("%s:%d %s() %s: %s (%d)\n\t%s\n",                   \
          __FILE__, __LINE__, __FUNCTION__,                         \
          #x, strerror(errno), errno, ftdi_get_error_string(ftdi)); \
      return -1;                                                    \
    }                                                               \
  } while(0)

#define E_VOID(x, ftdi)                                             \
  do {                                                              \
    if ((x))                                                        \
    {                                                               \
      msg_error("%s:%d %s() %s: %s (%d)\n\t%s\n",                   \
          __FILE__, __LINE__, __FUNCTION__,                         \
          #x, strerror(errno), errno, ftdi_get_error_string(ftdi)); \
    }                                                               \
  } while(0)


#define to_pdata(pgm) \
  ((avrftdi_t *)((pgm)->cookie))

typedef struct avrftdi_s {
  /* pointer to struct maintained by libftdi to identify the device */
  struct ftdi_context* ftdic;
  /* bitmask of values for pins. bit 0 represents pin 0 ([A|B]DBUS0) */
  uint16_t pin_value;
  /* bitmask of pin direction. a '1' make a pin an output.
   * bit 0 corresponds to pin 0. */
  uint16_t pin_direction;
  /* don't know. not useful. someone put it in. */
  uint16_t led_mask;
  /* total number of pins supported by a programmer. varies with FTDI chips */
  int pin_limit;
  /* internal RX buffer of the device. needed for INOUT transfers */
  int rx_buffer_size;
  int tx_buffer_size;
  /* use bitbanging instead of mpsse spi */
  bool use_bitbanging;
  /* bits 16-23 of extended 24-bit word flash address for parts with flash > 128k */
  uint8_t lext_byte;
} avrftdi_t;

void avrftdi_log(int level, const char * func, int line, const char * fmt, ...);

#endif /* DO_NOT_BUILD_AVRFDTI */

