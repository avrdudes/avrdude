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

CFLAGS       += -g -Wall --pedantic -DCONFIG_DIR=\"${CONFIGDIR}\" ${YYDEF}
LDFLAGS      =  
YFLAGS       = -t -d -v

INSTALL_PROGRAM = ${INSTALL} -m 555 -s
INSTALL_DATA    = ${INSTALL} -m 444
INSTALL_MANUAL  = ${INSTALL_DATA}


LIBS       = -lreadline

YYDEF  = -DYYSTYPE="struct token_t *"


.include "Makefile.inc"

EXTRA_OBJS = config_gram.o lexer.o
OBJECTS = ${EXTRA_OBJS} ${OBJS} 

all :
	@if [ ! -f y.tab.h ]; then touch y.tab.h; fi
	make depend
	make ${TARGET}

${TARGET} : ${OBJECTS}
	${CC} ${LDFLAGS} -o ${TARGET} ${OBJECTS} ${LIBS}

clean :
	rm -f *.o lexer.c ${TARGET} *~ *.core y.tab.c y.tab.h
	touch y.tab.h

install : dirs                             \
	  ${BINDIR}/${TARGET}              \
	  ${MANDIR}/${MANUAL}              \
	  ${DOCDIR}/avrprog.pdf            \
	  ${CONFIGDIR}/avrprog.conf.sample \
	  ${CONFIGDIR}/avrprog.conf

config_gram.o : avr.h config.h lists.h pindefs.h


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

${CONFIGDIR}/avrprog.conf : avrprog.conf.sample
	@if [ -f ${CONFIGDIR}/avrprog.conf ]; then                       \
	  export TS=`date '+%Y%m%d%H%M%S'`;                              \
	  echo "NOTE: backing up ${CONFIGDIR}/avrprog.conf to ${CONFIGDIR}/avrprog.conf.$${TS}"; \
	  cp -p ${CONFIGDIR}/avrprog.conf ${CONFIGDIR}/avrprog.conf.$${TS}; \
	fi
	${INSTALL_DATA} avrprog.conf.sample $@

