
.SUFFIXES: .o .c

first_rule : all


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

