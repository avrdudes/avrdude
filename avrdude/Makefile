#
# avrdude - A Downloader/Uploader for AVR device programmers
# Copyright (C) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#

#
# $Id$
#

TARGET       = avrdude

PREFIX      ?= /usr/local
BINDIR       = ${PREFIX}/bin
MANDIR       = ${PREFIX}/man/man1
MANUAL       = avrdude.1
DOCDIR       = ${PREFIX}/share/doc/avrdude
CONFIGDIR    = ${PREFIX}/etc

DIRS         = ${BINDIR} ${MANDIR} ${DOCDIR} ${CONFIGDIR}

YACC         = byacc
INSTALL      = /usr/bin/install -c -o root -g wheel

CFLAGS       += -Wall -DCONFIG_DIR=\"${CONFIGDIR}\" ${YYDEF}
#CFLAGS       = -g -Wall -DCONFIG_DIR=\"${CONFIGDIR}\" ${YYDEF}
LDFLAGS      =  
YFLAGS       = -t -d -v

INSTALL_PROGRAM = ${INSTALL} -m 555 -s
INSTALL_DATA    = ${INSTALL} -m 444
INSTALL_MANUAL  = ${INSTALL_DATA}

LIBS       = -lreadline

YYDEF  = -DYYSTYPE="struct token_t *"

SRCS = config_gram.c avr.c config.c fileio.c lexer.c lists.c main.c pgm.c \
	ppi.c stk500.c term.c

OBJS = config_gram.o avr.o config.o fileio.o lexer.o lists.o main.o pgm.o \
	ppi.o stk500.o term.o

all : depend ${TARGET}


${TARGET} : ${OBJS}
	${CC} ${LDFLAGS} -o ${TARGET} ${OBJS} ${LIBS}

clean :
	rm -f *.o config_gram.c lexer.c ${TARGET} *~ *.core y.tab.h y.output

install : dirs                             \
	  ${BINDIR}/${TARGET}              \
	  ${MANDIR}/${MANUAL}              \
	  ${DOCDIR}/avrdude.pdf            \
	  ${CONFIGDIR}/avrdude.conf.sample \
	  ${CONFIGDIR}/avrdude.conf

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

${DOCDIR}/avrdude.pdf : avrdude.pdf
	${INSTALL_DATA} avrdude.pdf $@

${CONFIGDIR}/avrdude.conf.sample : avrdude.conf.sample
	${INSTALL_DATA} avrdude.conf.sample $@

${CONFIGDIR}/avrdude.conf : avrdude.conf.sample
	@if [ -f ${CONFIGDIR}/avrdude.conf ]; then                       \
	  export TS=`date '+%Y%m%d%H%M%S'`;                              \
	  echo "NOTE: backing up ${CONFIGDIR}/avrdude.conf to ${CONFIGDIR}/avrdude.conf.$${TS}"; \
	  cp -p ${CONFIGDIR}/avrdude.conf ${CONFIGDIR}/avrdude.conf.$${TS}; \
	fi
	${INSTALL_DATA} avrdude.conf.sample $@

.SUFFIXES: .c .y .l

.y.c:
	${YACC} ${YFLAGS} $<
	mv -f y.tab.c $@

.l.c:
	${LEX} -t $< > $@

depend :
	@if [ ! -f y.tab.h ]; then touch y.tab.h; fi
	@${MAKE} config_gram.c lexer.c
	@${CC} ${CFLAGS} -MM ${SRCS} > .depend

-include .depend

