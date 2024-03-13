%module (docstring="SWIG wrapper and helper around libavrdude") swig_avrdude
%feature("autodoc", "1");
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


// This is the version from main.c
// Should be handled by something overridable from Python
int avrdude_message2(FILE *fp, int lno, const char *file,
                     const char *func, int msgmode, int msglvl,
                     const char *format, ...)
{
    int rc = 0;
    va_list ap;

    static struct {             // Memorise whether last print ended at beginning of line
      FILE *fp;
      int bol;                  // Are we at the beginning of a line for this fp stream?
    } bols[5+1];                // Cater for up to 5 different FILE pointers plus one catch-all

    size_t bi = 0;              // bi is index to bols[] array
    for(bi=0; bi < sizeof bols/sizeof*bols -1; bi++) { // Note the -1, so bi is valid after loop
      if(!bols[bi].fp) {        // First free space
        bols[bi].fp = fp;       // Insert fp in first free space
        bols[bi].bol = 1;       // Assume beginning of line on first use
      }
      if(bols[bi].fp == fp)
        break;
    }

    if(msglvl <= MSG_ERROR)     // Serious error? Free progress bars (if any)
      report_progress(1, -1, NULL);

    if(msgmode & MSG2_FLUSH) {
        fflush(stdout);
        fflush(stderr);
    }

    // Reduce effective verbosity level by number of -q above one when printing to stderr
    if ((quell_progress < 2 || fp != stderr? verbose: verbose+1-quell_progress) >= msglvl) {
        if(msgmode & MSG2_PROGNAME) {
          if(!bols[bi].bol)
            fprintf(fp, "\n");
          fprintf(fp, "%s", progname);
          if(verbose >= MSG_NOTICE && (msgmode & MSG2_FUNCTION))
            fprintf(fp, " %s()", func);
          if(verbose >= MSG_DEBUG && (msgmode & MSG2_FILELINE)) {
            const char *pr = strrchr(file, '/'); // Only print basename
#if defined (WIN32)
            if(!pr)
              pr =  strrchr(file, '\\');
#endif
            pr = pr? pr+1: file;
            fprintf(fp, " [%s:%d]", pr, lno);
          }
          if(msgmode & MSG2_TYPE)
            fprintf(fp, " level %d", msglvl);
          fprintf(fp, ": ");
          bols[bi].bol = 0;
        } else if(msgmode & MSG2_INDENT1) {
          fprintf(fp, "%*s", (int) strlen(progname)+1, "");
          bols[bi].bol = 0;
        } else if(msgmode & MSG2_INDENT2) {
          fprintf(fp, "%*s", (int) strlen(progname)+2, "");
          bols[bi].bol = 0;
        }

        // Vertical tab at start of format string is a conditional new line
        if(*format == '\v') {
          format++;
          if(!bols[bi].bol) {
            fprintf(fp, "\n");
            bols[bi].bol = 1;
          }
        }

        // Figure out whether this print will leave us at beginning of line

        // Determine required size first
        va_start(ap, format);
        rc = vsnprintf(NULL, 0, format, ap);
        va_end(ap);

        if(rc < 0)              // Some errror?
          return 0;

        rc++;                   // Accommodate terminating nul
        char *p = cfg_malloc(__func__, rc);
        va_start(ap, format);
        rc = vsnprintf(p, rc, format, ap);
        va_end(ap);

        if(rc < 0) {
          free(p);
          return 0;
        }

        if(*p) {
          fprintf(fp, "%s", p); // Finally: print!
          bols[bi].bol = p[strlen(p)-1] == '\n';
        }
        free(p);
    }

    if(msgmode & MSG2_FLUSH)
        fflush(fp);

    return rc;
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

typedef unsigned int memtype_t;
typedef struct avrmem {
  const char *desc;           /* memory description ("flash", "eeprom", etc) */
  memtype_t type;             /* internally used type, cannot be set in conf files */
  bool paged;                  /* 16-bit page addressed, e.g., ATmega flash but not EEPROM */
  int size;                   /* total memory size in bytes */
  int page_size;              /* size of memory page (if page addressed) */
  int num_pages;              /* number of pages (if page addressed) */
  int initval;                /* factory setting of fuses and lock bits */
  int bitmask;                /* bits used in fuses and lock bits */
  unsigned char * buf;        /* pointer to memory buffer */
} AVRMEM;

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
