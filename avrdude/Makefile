#------------------------------------------------------------------------
#
# $Id$
#


TARGET      = avrprog

PREFIX      ?= /usr/local
BINDIR       = ${PREFIX}/bin

INSTALL      = /usr/bin/install -c -o root -g wheel

CFLAGS      += -Wall --pedantic

LDFLAGS      =  

INSTALL_PROGRAM = ${INSTALL} -m 555 -s
INSTALL_DATA    = ${INSTALL} -m 444
INSTALL_MANUAL  = ${INSTALL_DATA}


OBJS = avr.o fileio.o main.o ppi.o term.o
LIBS = -lreadline

all : $(TARGET)

$(TARGET) : $(OBJS)
	$(CC) $(LDFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

main.o   : avr.h fileio.h ppi.h term.h
avr.o    : avr.h ppi.h
fileio.o : fileio.h avr.h
ppi.o    : ppi.h
term.o   : term.h avr.h

clean :
	rm -f *~ *.core $(TARGET) *.o

install : ${BINDIR}/$(TARGET)

${BINDIR}/$(TARGET) : $(TARGET)
	${INSTALL_PROGRAM} $(TARGET) ${BINDIR}

