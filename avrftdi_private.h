#pragma once
#include "ac_cfg.h"

#include <stdint.h>

#ifdef HAVE_LIBUSB_1_0
#ifdef HAVE_LIBFTDI1

#include <libusb-1.0/libusb.h>
#include <libftdi1/ftdi.h>

#include "pgm.h"
#include "pindefs.h"

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
			fprintf(stderr, "%s:%d %s() %s: %s (%d)\n\t%s\n",             \
					__FILE__, __LINE__, __FUNCTION__,                         \
					#x, strerror(errno), errno, ftdi_get_error_string(ftdi)); \
			return -1;                                                    \
		}                                                               \
	} while(0)

#define E_VOID(x, ftdi)                                             \
	do {                                                              \
		if ((x))                                                        \
		{                                                               \
			fprintf(stderr, "%s:%d %s() %s: %s (%d)\n\t%s\n",             \
					__FILE__, __LINE__, __FUNCTION__,                         \
	 			 #x, strerror(errno), errno, ftdi_get_error_string(ftdi));  \
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
	/* pin checklist. */
 struct pin_checklist_t pin_checklist[N_PINS - 1];
} avrftdi_t;

void avrftdi_log(int level, const char * func, int line, const char * fmt, ...);

#else /* HAVE_LIBFTDI1 */

#warning "libftdi1 required for programmer avrftdi."

#endif  /* HAVE_LIBFTDI1 */

#else /* HAVE_LIBUSB_1_0 */

#warning "libusb-1.0 required for programmer avrftdi."

#endif /* HAVE_LIBUSB_1_0 */

