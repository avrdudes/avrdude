
.SUFFIXES: .o .c .l .y

first_rule : all

.y.o:
	${YACC} ${YFLAGS} ${.IMPSRC}
	${CC} ${CFLAGS} -c y.tab.c
	rm -f y.tab.c
	mv y.tab.o ${.TARGET}


.depend : .PHONY
	@echo "# dependencies generated `date +'%D %T'`"        >  .depend
	@echo                                                   >> .depend
	@echo "# SUBDIR=`pwd`"                                  >> .depend
	@echo                                                   >> .depend
	@echo                                                   >> .depend
	@echo "SRCS = \\"                                       >> .depend
	@for i in *.S *.c; do                           \
	  if [ -f "$$i" ]; then                         \
	    echo "      $$i \\" >> .depend;             \
	  fi ;                                          \
	done
	@echo                                                   >> .depend
	@echo "OBJS = \\"                                       >> .depend
	@for i in *.S *.c; do                           \
	  if [ -f "$$i" ]; then                         \
	    echo "      $${i%%.*}.o \\" >> .depend;     \
	  fi ;                                          \
	done
	@echo                                                   >> .depend
	@for i in *.S *.c; do                           \
	  if [ -f "$$i" ]; then                         \
	    $(CC) -E $(CFLAGS) -M $$i >> .depend;       \
	  fi ;                                          \
	done
	@echo                                                   >> .depend

.if exists(.depend)
.include ".depend"
.endif

depend : .depend

