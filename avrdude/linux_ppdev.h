#ifndef __linux_ppdev_h__
#define __linux_ppdev_h__

#define OBSOLETE__IOW _IOW

#include <sys/ioctl.h>
#include <linux/parport.h>
#include <linux/ppdev.h>

#include <stdlib.h>

#define PPISDATA    PPWDATA
#define PPIGDATA    PPRDATA

#define PPISCTRL    PPWCONTROL
#define PPIGCTRL    PPRCONTROL

#define PPISSTATUS  PPWSTATUS
#define PPIGSTATUS  PPRSTATUS

#define ppi_claim(pgm)                                       \
  if (ioctl(pgm->fd, PPCLAIM)) {                             \
    fprintf(stderr, "%s: can't claim device \"%s\": %s\n\n", \
            progname, port, strerror(errno));                \
    close(pgm->fd);                                          \
    exit(1);                                                 \
  } 

#define ppi_release(pgm)                                     \
  if (ioctl(pgm->fd, PPRELEASE)) {                           \
    fprintf(stderr, "%s: can't release device: %s\n\n",      \
            progname, strerror(errno));                      \
    exit(1);                                                 \
  }


#endif /* __linux_ppdev_h__ */
