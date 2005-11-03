#ifndef __linux_ppdev_h__
#define __linux_ppdev_h__

#define OBSOLETE__IOW _IOW

#include <sys/ioctl.h>
#include <linux/parport.h>
#include <linux/ppdev.h>

#include <stdlib.h>

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

#define DO_PPI_READ(fd, reg, valp) \
	(void)ioctl(fd, \
		(reg) == PPIDATA? PPRDATA: ((reg) == PPICTRL? PPRCONTROL: PPRSTATUS), \
		    valp)
#define DO_PPI_WRITE(fd, reg, valp) \
	(void)ioctl(fd, \
		(reg) == PPIDATA? PPWDATA: ((reg) == PPICTRL? PPWCONTROL: PPWSTATUS), \
		    valp)

#endif /* __linux_ppdev_h__ */
