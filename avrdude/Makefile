#------------------------------------------------------------------------
#
# $Id$
#


TARGET      = avrprog

PREFIX      ?= /usr/local
BINDIR       = ${PREFIX}/bin
MANDIR       = ${PREFIX}/man/man1
MANUAL       = avrprog.1
DOCDIR       = ${PREFIX}/share/doc/avrprog
CONFIGDIR    = ${PREFIX}/etc

DIRS         = ${BINDIR} ${MANDIR} ${DOCDIR} ${CONFIGDIR}

INSTALL      = /usr/bin/install -c -o root -g wheel

CFLAGS       = -g -Wall --pedantic -DCONFIG_DIR=\"${CONFIGDIR}\"

LDFLAGS      =  

INSTALL_PROGRAM = ${INSTALL} -m 555 -s
INSTALL_DATA    = ${INSTALL} -m 444
INSTALL_MANUAL  = ${INSTALL_DATA}


LIBS = -lreadline

.include "Makefile.inc"

all :
	make depend
	make ${TARGET}

${TARGET} : ${OBJS}
	${CC} ${LDFLAGS} -o ${TARGET} ${OBJS} ${LIBS}

clean :
	rm -f *~ *.core ${TARGET} *.o

install : dirs                  \
	  ${BINDIR}/${TARGET}   \
	  ${MANDIR}/${MANUAL}   \
	  ${DOCDIR}/avrprog.pdf \
	  ${CONFIGDIR}/avrprog.conf.sample


dirs :
	@for dir in ${DIRS}; do \
	  if [ ! -d $$dir ]; then \
	    echo "creating directory $$dir"; \
	    mkdir -p $$dir; \
	  fi \
	done

${BINDIR}/${TARGET} : ${TARGET}
	${INSTALL_PROGRAM} ${TARGET} $@

${MANDIR}/${MANUAL} : ${MANUAL}
	${INSTALL_MANUAL} ${MANUAL} $@

${DOCDIR}/avrprog.pdf : avrprog.pdf
	${INSTALL_DATA} avrprog.pdf $@

${CONFIGDIR}/avrprog.conf.sample : avrprog.conf.sample
	${INSTALL_DATA} avrprog.conf.sample $@

