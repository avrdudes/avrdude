%module swig_avrdude
%{
#include "ac_cfg.h"
#include "libavrdude.h"

// global variables referenced by library
char * version      = AVRDUDE_FULL_VERSION;
char * progname = "avrdude";
char   progbuf[] = "       ";
int verbose;
int quell_progress;
int ovsigck;
const char *partdesc = "";
const char *pgmid = "";

int avrdude_message(int msglvl, const char *format, ...)
{
    int rc = 0;
    va_list ap;
    if (verbose >= msglvl) {
        va_start(ap, format);
        rc = vfprintf(stderr, format, ap);
        va_end(ap);
    }
    return rc;
}

int avrdude_message2(FILE *fp, int lno, const char *file, const char *func, int msgmode, int msglvl, const char *format, ...) {
  printf("avrdude_message2() called\n");
  return 0;
}

PROGRAMMER *cast_programmer(LNODEID p) {
  return (PROGRAMMER *)p;
}

AVRPART *cast_avrpart(LNODEID p) {
  return (AVRPART *)p;
}

AVRMEM *cast_avrmem(LNODEID p) {
  return (AVRMEM *)p;
}

const char *cast_string(LNODEID p) {
  return (const char *)p;
}

%}

// globals from above are mapped to Python
char * version;
char * progname;
char   progbuf[];
int verbose;
int quell_progress;
int ovsigck;
const char *partdesc;
const char *pgmid;

typedef void * LNODEID;
typedef void * LISTID;
typedef struct avrmem AVRMEM;

// These things are read from config file(s), and must be considered
// read-only by any program. Most internals are only relevant for
// libavrdude itself, so only map things to the Python level that are
// needed there.

%typemap(out) AVRPART* {
  if ($1 == NULL) {
    $result = Py_None;
  } else {
    PyObject* dict = PyDict_New();
    PyDict_SetItem(dict, PyUnicode_FromString("desc"),
                   PyUnicode_FromString($1->desc));
    PyDict_SetItem(dict, PyUnicode_FromString("id"),
                   PyUnicode_FromString($1->id));
    PyDict_SetItem(dict, PyUnicode_FromString("signature"),
                   PyBytes_FromStringAndSize((const char *)($1->signature), 3));
    PyDict_SetItem(dict, PyUnicode_FromString("prog_modes"),
                   PyLong_FromLong($1->prog_modes));
    PyDict_SetItem(dict, PyUnicode_FromString("mem"),
                   SWIG_NewPointerObj($1->mem, SWIGTYPE_p_avrmem, 0));
    $result = dict;
  }
}

%typemap(out) AVRMEM* {
  if ($1 == NULL) {
    $result = Py_None;
  } else {
    PyObject* dict = PyDict_New();
    PyDict_SetItem(dict, PyUnicode_FromString("desc"),
                   PyUnicode_FromString($1->desc));
    PyDict_SetItem(dict, PyUnicode_FromString("paged"),
                   PyBool_FromLong($1->paged));
    PyDict_SetItem(dict, PyUnicode_FromString("size"),
                   PyLong_FromLong($1->size));
    PyDict_SetItem(dict, PyUnicode_FromString("page_size"),
                   PyLong_FromLong($1->page_size));
    $result = dict;
  }
}

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
PROGRAMMER *cast_programmer(LNODEID);

AVRPART *cast_avrpart(LNODEID);

AVRMEM *cast_avrmem(LNODEID);

const char *cast_string(LNODEID);

// AVRMEM and AVRPART handling
AVRMEM * avr_locate_mem(const AVRPART *p, const char *desc);
AVRPART * locate_part(const LISTID parts, const char *partdesc);

enum prog_modes {
  PM_SPM       =    1, // Bootloaders, self-programming with SPM opcodes or NVM Controllers
  PM_TPI       =    2, // Tiny Programming Interface (t4, t5, t9, t10, t20, t40, t102, t104)
  PM_ISP       =    4, // SPI programming for In-System Programming (almost all classic parts)
  PM_PDI       =    8, // Program and Debug Interface (xmega parts)
  PM_UPDI      =   16, // Unified Program and Debug Interface
  PM_HVSP      =   32, // High Voltage Serial Programming (some classic parts)
  PM_HVPP      =   64, // High Voltage Parallel Programming (most non-HVSP classic parts)
  PM_debugWIRE =  128, // Simpler alternative to JTAG (a subset of HVPP/HVSP parts)
  PM_JTAG      =  256, // Joint Test Action Group standard (some classic parts)
  PM_JTAGmkI   =  512, // Subset of PM_JTAG, older parts, Atmel ICE mkI
  PM_XMEGAJTAG = 1024, // JTAG, some XMEGA parts
  PM_AVR32JTAG = 2048, // JTAG for 32-bit AVRs
  PM_aWire     = 4096, // For 32-bit AVRs
  PM_ALL      = 0x1fff // All programming interfaces
};
// map Python bytes() to sig+sigsize
%typemap(in) (unsigned char *sig, int sigsize) {
  Py_ssize_t len;
  PyBytes_AsStringAndSize($input, (char **)&$1, &len);
  $2 = (int)len;
}
AVRPART * locate_part_by_signature(const LISTID parts, unsigned char *sig, int sigsize);
AVRPART * locate_part_by_signature_pm(const LISTID parts, unsigned char *sig, int sigsize, int prog_modes);
const char *avr_prog_modes_str(int pm);
