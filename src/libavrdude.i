%define DOCSTRING
"SWIG wrapper and helper around libavrdude

The module defines all things necessary to perform the
same operations as the CLI AVRDUDE program, and exports
it to Python.

Invoke like

import swig_avrdude as ad

The following global variables are available in `ad.cvar`:

`ad.cvar.version`  - str, read-only
`ad.cvar.progname` - str, for messages
`ad.cvar.progbuf`  - str, spaces same length as `progname`
`ad.cvar.verbose`  - int, message verbosity (0 ... 5)
`ad.cvar.quell_progress` - int, message supression (0 ... 2)
`ad.cvar.ovsigck`  - int, override signature and some other checks (0 ... 1)"
%enddef
%module (docstring=DOCSTRING) swig_avrdude
%feature("autodoc", "1");
%{
#include "ac_cfg.h"
#include "libavrdude.h"

// global variables referenced by library
char * version  = AVRDUDE_FULL_VERSION;
char * progname = "avrdude";
char * progbuf = "       ";
int verbose;
int quell_progress;
int ovsigck;
const char *partdesc = "";
const char *pgmid = "";

static PyObject *msg_cb = NULL;
static PyObject *progress_cb = NULL;
static void swig_progress(int percent, double etime, const char *hdr, int finish);

void set_msg_callback(PyObject *PyFunc) {
  if (PyFunc == Py_None) {
    if (msg_cb)
      Py_XDECREF(msg_cb);      // Remove reference to previous callback
    msg_cb = NULL;
  } else {
    if (msg_cb)
      Py_XDECREF(msg_cb);      // Remove reference to previous callback
    Py_XINCREF(PyFunc);         // Add reference to new callback
    msg_cb = PyFunc;
  }
}

void set_progress_callback(PyObject *PyFunc) {
  if (PyFunc == Py_None) {
    if (progress_cb)
      Py_XDECREF(progress_cb);  // Remove reference to previous callback
    update_progress = NULL;
    progress_cb = NULL;
  } else {
    if (progress_cb)
      Py_XDECREF(progress_cb);  // Remove reference to previous callback
    progress_cb = PyFunc;
    Py_XINCREF(PyFunc);         // Add reference to new callback
    update_progress = swig_progress;
  }
}

// We cannot pass va_args to Python, so pre-process the message here
// in C code.
int avrdude_message2(FILE *fp, int lno, const char *file,
                     const char *func, int msgmode, int msglvl,
                     const char *format, ...)
{
    int rc = 0;
    va_list ap;
    PyObject *backslash_v = Py_False;

    const char *target = fp == stderr? "stderr": "stdout";

    if (msglvl <= MSG_ERROR)     // Serious error? Free progress bars (if any)
      report_progress(1, -1, NULL);

    if (msgmode & MSG2_FLUSH) {
      // ignored
    }

    // Vertical tab at start of format string is a conditional new line.
    // We pass this information down to the callee.
    if (*format == '\v') {
      backslash_v = Py_True;
      format++;
    }

    // Determine required size first
    va_start(ap, format);
    rc = vsnprintf(NULL, 0, format, ap);
    va_end(ap);

    if (rc < 0)              // Some errror?
      return 0;

    rc++;                   // Accommodate terminating nul
    char *p = cfg_malloc(__func__, rc);
    va_start(ap, format);
    rc = vsnprintf(p, rc, format, ap);
    va_end(ap);

    if (rc < 0) {
      free(p);
      return 0;
    }

    if (*p) {
      if (msg_cb) {
        PyObject *result =
          PyObject_CallFunction(msg_cb, "(sissiisO)",
                                target, lno, file, func, msgmode, msglvl, p, backslash_v);
        Py_XDECREF(result);
      }
      free(p);
    }

    return rc;
}

static void swig_progress(int percent, double etime, const char *hdr, int finish)
{
  if (progress_cb) {
    PyObject *result =
      PyObject_CallFunction(progress_cb, "(idsi)", percent, etime, hdr, finish);
    Py_XDECREF(result);
  }
}

PROGRAMMER *ldata_programmer(LNODEID p) {
  return (PROGRAMMER *)ldata(p);
}

AVRPART *ldata_avrpart(LNODEID p) {
  return (AVRPART *)ldata(p);
}

AVRMEM *ldata_avrmem(LNODEID p) {
  return (AVRMEM *)ldata(p);
}

const char *ldata_string(LNODEID p) {
  return (const char *)ldata(p);
}

%}

// globals from above are mapped to Python
%immutable;
char * version;
%mutable;
char * progname;
char * progbuf;
int verbose;
int quell_progress;
int ovsigck;
%immutable;
const char *partdesc;
const char *pgmid;
%mutable;

typedef void * LNODEID;
typedef void * LISTID;
typedef struct avrmem AVRMEM;
typedef struct programmer_t PROGRAMMER;
typedef void pgm_initpgm(PROGRAMMER*);

enum msglvl {
  MSG_EXT_ERROR = (-3), // OS-type error, no -v option, can be suppressed with -qqqqq
  MSG_ERROR = (-2),     // Avrdude error, no -v option, can be suppressed with -qqqq
  MSG_WARNING = (-1),   // Warning, no -v option, can be suppressed with -qqq
  MSG_INFO = 0,         // Commentary, no -v option, can be suppressed with -qq
  MSG_NOTICE = 1,       // Displayed with -v
  MSG_NOTICE2 = 2,      // Displayed with -vv
  MSG_DEBUG = 3,        // Displayed with -vvv
  MSG_TRACE = 4,        // Displayed with -vvvv, show trace communication
  MSG_TRACE2 = 5,       // Displayed with -vvvvv
};

enum msgmode {
  MSG2_PROGNAME = 1,    // Start by printing progname
  MSG2_FUNCTION = 2,    // Print calling function (1st arg) after progname if >= notice
  MSG2_FILELINE = 4,    // Print source file and line number after function if >= debug
  MSG2_TYPE = 8,        // Print message type after function or progname
  MSG2_INDENT1 = 16,    // Start by printing indentation of progname+1 blanks
  MSG2_INDENT2 = 32,    // Start by printing indentation of progname+2 blanks
  MSG2_FLUSH = 64,      // Flush before and after printing
};

// Functions to record a callback
%typemap(in) PyObject *PyFunc {
  // calling with None removes previous callback
  if ($input != Py_None && !PyCallable_Check($input)) {
      PyErr_SetString(PyExc_TypeError, "Need a callable object!");
      return NULL;
  }
  $1 = $input;
}
void set_msg_callback(PyObject *PyFunc);
void set_progress_callback(PyObject *PyFunc);

// These things are read from config file(s), and must be considered
// read-only by any program. Most internals are only relevant for
// libavrdude itself, so only map things to the Python level that are
// needed there.

%typemap(out) unsigned char signature[3] {
  $result = PyBytes_FromStringAndSize((const char *)$1, 3);
 }

%immutable;
typedef struct avrpart {
  const char  * desc;               /* long part name */
  const char  * id;                 /* short part name */
  LISTID        variants;           /* String with variant name and chip properties */
  const char  * parent_id;          /* Used by developer options */
  const char  * family_id;          /* family id in the SIB (avr8x) */
  int           prog_modes;         /* Programming interfaces, see #define PM_... */
  unsigned char signature[3];       /* expected value of signature bytes */
  unsigned short usbpid;            /* USB DFU product ID (0 = none) */
  LISTID        mem;            /* avr memory definitions */
  LISTID        mem_alias;      /* memory alias definitions */
  const char  * config_file;    /* config file where defined */
  int           lineno;         /* config file line number */
} AVRPART;
%mutable;

typedef unsigned int memtype_t;
typedef struct avrmem {
  %immutable;
  const char *desc;           /* memory description ("flash", "eeprom", etc) */
  memtype_t type;             /* internally used type, cannot be set in conf files */
  bool paged;                  /* 16-bit page addressed, e.g., ATmega flash but not EEPROM */
  int size;                   /* total memory size in bytes */
  int page_size;              /* size of memory page (if page addressed) */
  int num_pages;              /* number of pages (if page addressed) */
  int initval;                /* factory setting of fuses and lock bits */
  int bitmask;                /* bits used in fuses and lock bits */
  %mutable;
  unsigned char * buf;        /* pointer to memory buffer */
} AVRMEM;

%extend avrmem {
%feature("autodoc", "m.get(len: int, offset: int = 0) => return bytes; Read from memory buffer") get;
  PyObject *get(unsigned int len, unsigned int offset = 0) {
    if ($self->buf == NULL)
      // missing avr_initmem()?
      return Py_None;
    if (offset >= (unsigned)$self->size)
      return Py_None;
    if (offset + len > (unsigned)$self->size)
      len = $self->size - offset;
    return PyBytes_FromStringAndSize((char *)($self->buf + offset), len);
  }
}

int avr_initmem(const AVRPART *p);

%extend avrmem {
  %typemap(in) (unsigned char *in, unsigned int len) {
    Py_ssize_t len;
    PyBytes_AsStringAndSize($input, (char **)&$1, &len);
    $2 = len;
  }
%feature("autodoc", "m.put(in: bytes, len: int, offset: int = 0) => return len; Copy to memory buffer, set ALLOCATED tag") put;
  int put(unsigned char *in, unsigned int len, unsigned int offset = 0) {
    if ($self->buf == NULL)
      // missing avr_initmem()?
      return 0;
    if (offset >= (unsigned)$self->size)
      return 0;
    if (offset + len > (unsigned)$self->size)
      len = $self->size - offset;
    memcpy($self->buf + offset, in, len);
    memset($self->tags + offset, TAG_ALLOCATED, len);
    return len;
  }
}

%extend avrmem {
%feature("autodoc", "m.clear(len, offset, value=0xFF) => return len; Clear memory buffer range, and ALLOCATED tag") clear;
  int clear(unsigned int len, unsigned int offset = 0, unsigned char value = 0xFF) {
    if ($self->buf == NULL)
      // missing avr_initmem()?
      return 0;
    if (offset >= (unsigned)$self->size)
      return 0;
    if (offset + len > (unsigned)$self->size)
      len = $self->size - offset;
    memset($self->buf + offset, value, len);
    memset($self->tags + offset, 0, len);
    return len;
  }
}

%feature("autodoc", "1");

typedef enum {
  CONNTYPE_PARALLEL,
  CONNTYPE_SERIAL,
  CONNTYPE_USB,
  CONNTYPE_SPI,
  CONNTYPE_LINUXGPIO
} conntype_t;

// https://stackoverflow.com/questions/11023940/how-do-i-get-swig-to-automatically-wrap-an-emulated-this-pointer-to-a-c-struct/11029809#11029809
// Make sure the wrapped function doesn't expect an input for this:
%typemap(in, numinputs=0) struct programmer_t *pgm "$1=NULL;"
// Slightly abuse check typemap, but it needs to happen after the rest of the arguments have been set:
%typemap(check) struct programmer_t *pgm {
  $1 = arg1;
}

%typemap(in, numinputs=0) char *sib (char temp[AVR_SIBLEN + 1]) {
  $1 = temp;
 }
%typemap(argout) char *sib {
  if ($result == NULL)
    $result = Py_None;
  else if (PyLong_Check($result) && PyLong_AsLong($result) != 0)
    $result = Py_None;
  else
    $result = PyBytes_FromStringAndSize((const char *)$1, AVR_SIBLEN);
 }
%typemap(in, numinputs=0) (double *vtarg_out) (double temp) {
  $1 = &temp;
 }
%typemap(argout) (double *vtarg_out) {
  if ($result == NULL)
    $result = Py_None;
  else if (PyLong_Check($result) && PyLong_AsLong($result) != 0)
    $result = Py_None;
  else
    $result = PyFloat_FromDouble(*$1);
 }
%typemap(in, numinputs=0) (double *varef_out) (double temp) {
  $1 = &temp;
 }
%typemap(argout) (double *varef_out) {
  if ($result == NULL)
    $result = Py_None;
  else if (PyLong_Check($result) && PyLong_AsLong($result) != 0)
    $result = Py_None;
  else
    $result = PyFloat_FromDouble(*$1);
 }
%typemap(in, numinputs=0) (double *fosc_out) (double temp) {
  $1 = &temp;
 }
%typemap(argout) (double *fosc_out) {
  if ($result == NULL)
    $result = Py_None;
  else if (PyLong_Check($result) && PyLong_AsLong($result) != 0)
    $result = Py_None;
  else
    $result = PyFloat_FromDouble(*$1);
 }
%typemap(in, numinputs=0) (double *sck_out) (double temp) {
  $1 = &temp;
 }
%typemap(argout) (double *sck_out) {
  if ($result == NULL)
    $result = Py_None;
  else if (PyLong_Check($result) && PyLong_AsLong($result) != 0)
    $result = Py_None;
  else
    $result = PyFloat_FromDouble(*$1);
 }
// abuse check typemap to check for methods not being NULL
// it must be ensured that each argument type/name applies to just one method
%typemap(check) (char *sib) {
  if ((arg1)->read_sib == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->read_sib is NULL");
  }
}
%typemap(check) (FILE *f_parms) {
  if ((arg1)->print_parms == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->print_parms is NULL");
  }
}
%typemap(check) (double vtarg_in) {
  if ((arg1)->set_vtarget == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->set_vtarget is NULL");
  }
}
%typemap(check) (double *vtarg_out) {
  if ((arg1)->get_vtarget == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->get_vtarget is NULL");
  }
}
%typemap(check) (double varef_in) {
  if ((arg1)->set_varef == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->set_varef is NULL");
  }
}
%typemap(check) (double *varef_out) {
  if ((arg1)->get_varef == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->get_varef is NULL");
  }
}
%typemap(check) (double fosc_in) {
  if ((arg1)->set_fosc == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->set_fosc is NULL");
  }
}
%typemap(check) (double *fosc_out) {
  if ((arg1)->get_fosc == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->get_fosc is NULL");
  }
}
%typemap(check) (double sck_in) {
  if ((arg1)->set_sck_period == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->set_sck_period is NULL");
  }
}
%typemap(check) (double *sck_out) {
  if ((arg1)->get_sck_period == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->get_sck_period is NULL");
  }
}
%typemap(check) (struct programmer_t *pgm_setup) {
  if ((arg1)->setup == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->setup is NULL");
  }
}
%typemap(check) (struct programmer_t *pgm_teardown) {
  if ((arg1)->teardown == NULL) {
    SWIG_exception_fail(SWIG_RuntimeError, "pgm->teardown is NULL");
  }
}

%immutable;
typedef struct programmer_t {
  LISTID id;
  const char *desc;
  int prog_modes;               // Programming interfaces, see #define PM_...
  conntype_t conntype;
  int baudrate;
  int usbvid;
  LISTID usbpid;
  int ispdelay;                 // ISP clock delay
  double bitclock;              // JTAG ICE clock period in microseconds
  char type[PGM_TYPELEN];

  // methods; they must *not* be declares as pointers
  void initpgm        (struct programmer_t *pgm); // Sets up the AVRDUDE programmer
  int  initialize     (const struct programmer_t *pgm, const AVRPART *p); // Sets up the physical programmer
  void setup          (struct programmer_t *pgm);
  void teardown       (struct programmer_t *pgm);
  int  parseextparams (const struct programmer_t *pgm, const LISTID xparams);
  int  parseexitspecs (struct programmer_t *pgm, const char *s);
  int  open           (struct programmer_t *pgm, const char *port);
  void close          (struct programmer_t *pgm);
  void enable         (struct programmer_t *pgm, const AVRPART *p);
  void disable        (const struct programmer_t *pgm);
  int  read_sib       (const struct programmer_t *pgm, const AVRPART *p, char *sib);
  void powerup        (const struct programmer_t *pgm);
  void powerdown      (const struct programmer_t *pgm);

  int  chip_erase     (const struct programmer_t *pgm, const AVRPART *p);
  int  term_keep_alive(const struct programmer_t *pgm, const AVRPART *p);
  int  end_programming(const struct programmer_t *pgm, const AVRPART *p);

  void print_parms    (const struct programmer_t *pgm, FILE *f_parms);
  int  set_vtarget    (const struct programmer_t *pgm, double vtarg_in);
  int  get_vtarget    (const struct programmer_t *pgm, double *vtarg_out);
  int  set_varef      (const struct programmer_t *pgm, unsigned int chan, double varef_in);
  int  get_varef      (const struct programmer_t *pgm, unsigned int chan, double *varef_out);
  int  set_fosc       (const struct programmer_t *pgm, double fosc_in);
  int  get_fosc       (const struct programmer_t *pgm, double *fosc_out);
  int  set_sck_period (const struct programmer_t *pgm, double sck_in);
  int  get_sck_period (const struct programmer_t *pgm, double *sck_out);
  // Cached r/w API for terminal reads/writes
  int write_byte_cached(const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                        unsigned long addr, unsigned char value);
  int read_byte_cached(const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                        unsigned long addr, unsigned char *value);
  int chip_erase_cached(const struct programmer_t *pgm, const AVRPART *p);
  int page_erase_cached(const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                        unsigned int baseaddr);
  int readonly        (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                        unsigned int addr);
  int flush_cache     (const struct programmer_t *pgm, const AVRPART *p);
  int reset_cache     (const struct programmer_t *pgm, const AVRPART *p);

} PROGRAMMER;
%mutable;
%clear struct programmer_t *pgm;

// Config file handling
int init_config(void);

void cleanup_config(void);

int read_config(const char * file);

// Lists must not be manipulated from the Python level, so
// only map access functions.
extern LISTID      part_list;
extern LISTID      programmers;

LNODEID    lfirst ( LISTID  ); /* head of the list */
LNODEID    llast  ( LISTID  ); /* tail of the list */
LNODEID    lnext  ( LNODEID ); /* next item in the list */
LNODEID    lprev  ( LNODEID ); /* previous item in the list */
void     * ldata  ( LNODEID ); /* data at the current position */
int        lsize  ( LISTID  ); /* number of elements in the list */

// Typecast helpers to interpret LNODEID
%feature("autodoc", "LNODEID -> PROGRAMMER*") ldata_programmer;
PROGRAMMER *ldata_programmer(LNODEID);

%feature("autodoc", "LNODEID -> AVRPART*") ldata_avrpart;
AVRPART *ldata_avrpart(LNODEID);

%feature("autodoc", "LNODEID -> AVRMEM*") ldata_avrmem;
AVRMEM *ldata_avrmem(LNODEID);

%feature("autodoc", "LNODEID -> str") ldata_string;
const char *ldata_string(LNODEID);

// AVRMEM and AVRPART handling
AVRMEM * avr_locate_mem(const AVRPART *p, const char *desc);
AVRPART * locate_part(const LISTID parts, const char *partdesc);

// Programming modes for parts and programmers: reflect changes in lexer.l, developer_opts.c and config.c
#define PM_SPM                1 // Bootloaders, self-programming with SPM opcodes or NVM Controllers
#define PM_TPI                2 // Tiny Programming Interface (t4, t5, t9, t10, t20, t40, t102, t104)
#define PM_ISP                4 // SPI programming for In-System Programming (almost all classic parts)
#define PM_PDI                8 // Program and Debug Interface (xmega parts)
#define PM_UPDI              16 // Unified Program and Debug Interface
#define PM_HVSP              32 // High Voltage Serial Programming (some classic parts)
#define PM_HVPP              64 // High Voltage Parallel Programming (most non-HVSP classic parts)
#define PM_debugWIRE        128 // Simpler alternative to JTAG (a subset of HVPP/HVSP parts)
#define PM_JTAG             256 // Joint Test Action Group standard (some classic parts)
#define PM_JTAGmkI          512 // Subset of PM_JTAG, older parts, Atmel ICE mkI
#define PM_XMEGAJTAG       1024 // JTAG, some XMEGA parts
#define PM_AVR32JTAG       2048 // JTAG for 32-bit AVRs
#define PM_aWire           4096 // For 32-bit AVRs
#define PM_ALL           0x1fff // All programming interfaces

// map Python bytes() to sig+sigsize
%typemap(in) (unsigned char *sig, int sigsize) {
  Py_ssize_t len;
  PyBytes_AsStringAndSize($input, (char **)&$1, &len);
  $2 = (int)len;
}
AVRPART * locate_part_by_signature(const LISTID parts, unsigned char *sig, int sigsize);
AVRPART * locate_part_by_signature_pm(const LISTID parts, unsigned char *sig, int sigsize, int prog_modes);
const char *avr_prog_modes_str(int pm);

PROGRAMMER *locate_programmer_set(const LISTID programmers, const char *id, const char **setid);
PROGRAMMER *locate_programmer_starts_set(const LISTID programmers, const char *id, const char **setid, AVRPART *prt);
PROGRAMMER *locate_programmer(const LISTID programmers, const char *configid);

// Abuse the check typemaps to inject some code into the wrapper

// avr_read_mem() and avr_write_mem() do not initialize progress
// reporting, since this is left to the caller. The idea is that in
// some situations (e.g. signature readout), no progress reporting is
// desired. Thus, we inject the respective progress reporting
// initialization here into the wrapper functions.
%typemap(check) AVRMEM *mem {
  report_progress(0, 1, "Reading");
}
%feature("autodoc", "avr_read_mem(PROGRAMMER pgm, AVRPART p, AVRMEM mem, AVRPART v=None -> int; v: verify against") avr_read_mem;
int avr_read_mem(const PROGRAMMER * pgm, const AVRPART *p, const AVRMEM *mem, const AVRPART *v = NULL);
%clear AVRMEM *mem;

%typemap(check) AVRMEM *mem {
  report_progress(0, 1, "Writing");
}
%feature("autodoc", "avr_write_mem(PROGRAMMER pgm, AVRPART p, AVRMEM mem, int size, int auto_erase) -> int; write entire memory region from `mem` buffer") avr_write_mem;
int avr_write_mem(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                  int size, int auto_erase = false);
%clear AVRMEM *mem;

%feature("autodoc", "avr_write_byte(PROGRAMMER pgm, AVRPART p, AVRMEM mem, int addr, byte data) -> int") avr_write_byte;
int avr_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                   unsigned long addr, unsigned char data);
typedef enum {
  FMT_ERROR = -1,
  FMT_AUTO,
  FMT_SREC,
  FMT_IHEX,
  FMT_RBIN,
  FMT_IMM,
  FMT_EEGG,
  FMT_HEX,
  FMT_DEC,
  FMT_OCT,
  FMT_BIN,
  FMT_ELF,
  FMT_IHXC,
} FILEFMT;

typedef enum {
  FIO_READ,
  FIO_WRITE,
  FIO_READ_FOR_VERIFY,
} FIO_OP;


%feature("autodoc", "fileio_format(c: char) -> FILEFMT") fileio_format;
FILEFMT fileio_format(char c);

%feature("autodoc", "fileio_fmtstr(f: FILEFMT) -> str") fileio_fmtstr;
char *fileio_fmtstr(FILEFMT format);

%feature("autodoc", "fileio_fmtchr(f: FILEFMT) -> char") fileio_fmtchr;
int fileio_fmtchr(FILEFMT format);

%feature("autodoc", "fileio_fmt_autodetect(fname: str) -> FILEFMT") fileio_fmt_autodetect;
int fileio_fmt_autodetect(const char *fname);

%feature("autodoc", "fileio(oprwv: FIO_OP, filename: str, format: FILEFMT, p: AVRPART, memstr: str, size: int) -> int [size, or -1 on error]") fileio;
int fileio(int oprwv, const char *filename, FILEFMT format,
  const AVRPART *p, const char *memstr, int size);

%feature("autodoc", "1");
