#
# $Id$
#

TARGET = avrprog

DEST   = ${HOME}/bin/0.${ARCH}

all : ${TARGET}

CFLAGS = -Wall --pedantic

${TARGET} : avrprog.c
	${CC} ${CFLAGS} -o ${TARGET} avrprog.c

clean :
	rm -f *.o ${TARGET} *~

install : ${DEST}/${TARGET}

${DEST}/${TARGET} : ${TARGET}
	cp -p ${TARGET} $@

