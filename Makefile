#
# $Id$
#

TARGET = avrprog

all : ${TARGET}

CFLAGS = -Wall

${TARGET} : avrprog.c
	${CC} ${CFLAGS} -o ${TARGET} avrprog.c

clean :
	rm -f *.o ${TARGET} *~

