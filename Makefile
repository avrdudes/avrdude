#------------------------------------------------------------------------
#
# $Id$
#
# Makefile
#

TARGET      = avrprog

PREFIX      ?= /usr/local
BINDIR       = ${PREFIX}/bin

INSTALL      = /usr/bin/install -c -o root -g wheel

CFLAGS += -Wall --pedantic

INSTALL_PROGRAM = ${INSTALL} -m 555 -s
INSTALL_DATA    = ${INSTALL} -m 444
INSTALL_MANUAL  = ${INSTALL_DATA}


all : $(TARGET)

$(TARGET) : avrprog.c
	$(CC) $(CFLAGS) -o $(TARGET) $<

clean :
	rm -f *~ *.core $(TARGET)

install : ${BINDIR}/$(TARGET)

${BINDIR}/$(TARGET) : $(TARGET)
	${INSTALL_PROGRAM} $(TARGET) ${BINDIR}

