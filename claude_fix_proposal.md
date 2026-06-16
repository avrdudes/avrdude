# Patch Proposal: Fix `libavrdude` shared library build on Windows (issue #1414)

## Root cause

`libavrdude` calls `avrdude_message2()` and references global variables
(`verbose`, `progname`, `ovsigck`, `partdesc`, `pgmid`, `quell_progress`, `cx`)
that are currently **defined in `main.c`** (the executable).

On Linux/macOS this accidentally works via `-Wl,-undefined,dynamic_lookup` or
flat-namespace linking, but **Windows DLLs require all symbols to be fully
resolved at library link time**. This causes `BUILD_SHARED_LIBS=ON` to fail
with hundreds of `undefined reference` errors on both MinGW and MSVC.

## Fix

Move all affected definitions into a new file `src/avrdude_msg.c`, compiled
into `libavrdude`. The changes to `main.c` are **purely deletions** — all
existing assignments (`verbose = 0`, `pgmid = optarg`, etc.) remain unchanged,
they simply now write into variables owned by the library.

Moving `cx` (`libavrdude_context *`) here is consistent with the existing
comment in `libavrdude.h`:
> *"Global and static variables should go here; the only remaining static
> variables ought to be read-only tables."*

**No behaviour change. Zero logic modifications.**

---

## Files changed

| File | Change |
|---|---|
| `src/avrdude_msg.c` | **New file** — receives globals + `avrdude_message2()` cut from `main.c` |
| `src/main.c` | Deletions only |
| `src/CMakeLists.txt` | Add `avrdude_msg.c` to `SOURCES`; add `LIBAVRDUDE_BUILD_DLL` for Windows shared builds |
| `src/avrdude.h` | Add `LIBAVRDUDE_API` macro; annotate the six `extern` declarations and `avrdude_message2()` |
| `src/libavrdude.h` | Fix stale comment on `avrdude_message2()` |

Root `CMakeLists.txt` is **not changed**.

---

## 1. New file: `src/avrdude_msg.c`

```c
/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2005 Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) Joerg Wunsch <j@uriah.heep.sax.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Global variables and avrdude_message2() moved here from main.c so that
 * libavrdude is fully self-contained and can be built as a shared library
 * (.dll / .so / .dylib) on all platforms including Windows.
 *
 * Previously these symbols were defined in main.c (the executable) and
 * referenced by libavrdude, creating a circular dependency that Windows
 * DLL linking strictly forbids.  Moving them into the library eliminates
 * that inversion: the executable now depends on the library only, not
 * vice versa.
 *
 * This also moves cx (libavrdude_context *) here from main.c, consistent
 * with the stated direction in libavrdude.h that cx should eventually be
 * the only remaining global variable.
 *
 * See https://github.com/avrdudes/avrdude/issues/1414
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#include "avrdude.h"
#include "libavrdude.h"

// ---------------------------------------------------------------------------
// Global variables formerly defined in main.c
// ---------------------------------------------------------------------------

libavrdude_context *cx;         // Context pointer, eventually the only global variable

char       *progname       = "avrdude";
int         verbose        = 0;
int         quell_progress = 0;
int         ovsigck        = 0;
const char *partdesc       = NULL;
const char *pgmid          = NULL;

// ---------------------------------------------------------------------------
// avrdude_message2() and its private helper — body identical to main.c
// ---------------------------------------------------------------------------

static const char *avrdude_message_type(int msglvl) {
  switch(msglvl) {
  case MSG_EXT_ERROR: return "OS error";
  case MSG_ERROR:     return "error";
  case MSG_WARNING:   return "warning";
  case MSG_INFO:      return "info";
  case MSG_NOTICE:    return "notice";
  case MSG_NOTICE2:   return "notice2";
  case MSG_DEBUG:     return "debug";
  case MSG_TRACE:     return "trace";
  case MSG_TRACE2:    return "trace2";
  default:            return "unknown msglvl";
  }
}

/*
 * Core messaging routine for msg_xyz(), [pli]msg_xyz() and term_out()
 * See #define lines in avrdude.h for how it is normally called.
 *
 * Named that way as there used to be a now gone different avrdude_message()
 */
int avrdude_message2(FILE *fp, int lno, const char *file, const char *func,
  int msgmode, int msglvl, const char *format, ...) {

  int rc = 0;
  va_list ap;

  static struct {               // Memorise whether last print ended at beginning of line
    FILE *fp;
    int bol;                    // Are we at the beginning of a line for this fp stream?
  } bols[5 + 1];                // Cater for up to 5 different FILE pointers plus one catch-all

  size_t bi = 0;

  for(bi = 0; bi < sizeof bols/sizeof *bols - 1; bi++) {
    if(!bols[bi].fp) {
      bols[bi].fp = fp;
      bols[bi].bol = 1;
    }
    if(bols[bi].fp == fp)
      break;
  }

  if(msglvl <= MSG_ERROR)       // Serious error? Free progress bars (if any)
    report_progress(1, -1, NULL);

  if(msgmode & MSG2_FLUSH) {
    fflush(stdout);
    fflush(stderr);
  }

  // Reduce effective verbosity level by number of -q above one when printing to stderr
  if((fp == stderr? verblevel: verbose) >= msglvl) {
    if(msgmode & MSG2_LEFT_MARGIN && !bols[bi].bol) {
      fprintf(fp, "\n");
      bols[bi].bol = 1;
    }
    // Keep vertical tab at start of format string as conditional new line
    if(*format == '\v') {
      format++;
      if(!bols[bi].bol) {
        fprintf(fp, "\n");
        bols[bi].bol = 1;
      }
    }

    if(msgmode & (MSG2_PROGNAME | MSG2_TYPE)) {
      if(msgmode & MSG2_PROGNAME) {
        fprintf(fp, "%s", progname);
        bols[bi].bol = 0;
      }
      if(msgmode & MSG2_TYPE) {
        const char *mt = avrdude_message_type(msglvl);

        if(bols[bi].bol)
          fprintf(fp, "%c%s", msgmode & (MSG2_UCFIRST)? toupper(*mt & 0xff): *mt, mt + 1);
        else
          fprintf(fp, " %s", mt);
        bols[bi].bol = 0;
      }
      if(verblevel >= MSG_NOTICE2) {
        const char *bfname = strrchr(file, '/');

#if defined(WIN32)
        if(!bfname)
          bfname = strrchr(file, '\\');
#endif

        bfname = bfname? bfname + 1: file;
        if(msgmode & MSG2_FUNCTION)
          fprintf(fp, " %s()", func);
        if(msgmode & MSG2_FILELINE)
          fprintf(fp, " %s %d", bfname, lno);
      }
      fprintf(fp, ": ");
    } else if(msgmode & MSG2_INDENT1) {
      fprintf(fp, "%*s", (int) strlen(progname) + 1, "");
      bols[bi].bol = 0;
    } else if(msgmode & MSG2_INDENT2) {
      fprintf(fp, "%*s", (int) strlen(progname) + 2, "");
      bols[bi].bol = 0;
    }

    va_start(ap, format);
    rc = vsnprintf(NULL, 0, format, ap);
    va_end(ap);

    if(rc < 0)
      return 0;

    rc++;
    char *p = mmt_malloc(rc);

    va_start(ap, format);
    rc = vsnprintf(p, rc, format, ap);
    va_end(ap);

    if(rc < 0) {
      mmt_free(p);
      return 0;
    }

    if(*p) {
      if(bols[bi].bol && (msgmode & MSG2_UCFIRST))
        fprintf(fp, "%c%s", toupper(*p & 0xff), p + 1);
      else
        fprintf(fp, "%s", p);
      bols[bi].bol = p[strlen(p) - 1] == '\n';
    }
    mmt_free(p);
  }

  if(msgmode & MSG2_FLUSH)
    fflush(fp);

  return rc;
}
```

---

## 2. `src/main.c` — deletions only

Remove the following items. Nothing else in `main.c` changes.

```c
// DELETE: file-scope definition (line ~53)
char *progname = "avrdude";

// DELETE: entire static helper function
static const char *avrdude_message_type(int msglvl) { ... }

// DELETE: entire function
int avrdude_message2(FILE *fp, int lno, const char *file, const char *func,
  int msgmode, int msglvl, const char *format, ...) { ... }

// DELETE: near struct list_walk_cookie
libavrdude_context *cx;

// DELETE: Global options block
int verbose;
int quell_progress;
int ovsigck;
const char *partdesc;
const char *pgmid;
```

---

## 3. `src/CMakeLists.txt` — two additions

**Addition 1:** add `avrdude_msg.c` to the `SOURCES` list (alphabetical position between `arduino.c` and `avr.c`):

```diff
 set(SOURCES
     ${CMAKE_CURRENT_BINARY_DIR}/ac_cfg.h
     arduino.h
     arduino.c
+    avrdude_msg.c
     avr.c
```

**Addition 2:** add after the `set_target_properties(libavrdude ...)` block:

```diff
 set_target_properties(libavrdude PROPERTIES
     PREFIX ""
     PUBLIC_HEADER "libavrdude.h;libavrdude-avrintel.h"
     VERSION 4.0.0
     SOVERSION 4
     )
+
+# Export DLL symbols on Windows when building as a shared library.
+# avrdude_message2(), progname, verbose etc. formerly lived in the
+# executable (main.c), causing undefined-reference link failures on
+# Windows DLLs. They now live in libavrdude via avrdude_msg.c.
+if(BUILD_SHARED_LIBS AND WIN32)
+    target_compile_definitions(libavrdude PRIVATE LIBAVRDUDE_BUILD_DLL)
+endif()
```

Note: the `swig_avrdude` target uses `${SOURCES}` directly and will pick up
`avrdude_msg.c` automatically — no further changes needed there.

---

## 4. `src/avrdude.h` — add `LIBAVRDUDE_API` macro and annotate declarations

Add the macro block after the `#include <stdlib.h>` line and before
`#define SYSTEM_CONF_FILE`:

```c
// DLL import/export annotation for Windows shared library builds.
// When compiling libavrdude itself (LIBAVRDUDE_BUILD_DLL set by CMake),
// symbols are exported. When compiling a consumer (main.c, external app),
// symbols are imported. On non-Windows the macro expands to nothing,
// except on GCC >= 4 where it sets default visibility explicitly.
#if defined(_WIN32) && defined(LIBAVRDUDE_BUILD_DLL)
#  define LIBAVRDUDE_API __declspec(dllexport)
#elif defined(_WIN32)
#  define LIBAVRDUDE_API __declspec(dllimport)
#else
#  if defined(__GNUC__) && __GNUC__ >= 4
#    define LIBAVRDUDE_API __attribute__((visibility("default")))
#  else
#    define LIBAVRDUDE_API
#  endif
#endif
```

Then annotate the existing `extern` declarations and `avrdude_message2`:

```diff
-extern char *progname;
-extern int ovsigck;
-extern int verbose;
-extern int quell_progress;
-extern const char *partdesc;
-extern const char *pgmid;
+LIBAVRDUDE_API extern char       *progname;
+LIBAVRDUDE_API extern int         ovsigck;
+LIBAVRDUDE_API extern int         verbose;
+LIBAVRDUDE_API extern int         quell_progress;
+LIBAVRDUDE_API extern const char *partdesc;
+LIBAVRDUDE_API extern const char *pgmid;
```

```diff
-int avrdude_message2(FILE *fp, int lno, const char *file, const char *func, int msgmode, int msglvl, const char *format, ...)
+LIBAVRDUDE_API int avrdude_message2(FILE *fp, int lno, const char *file, const char *func, int msgmode, int msglvl, const char *format, ...)
 #if defined(__GNUC__)
   __attribute__((format(printf, 7, 8)))
 #endif
   ;
```

---

## 5. `src/libavrdude.h` — fix stale comment

```diff
-// This functions is supposed to be supplied by the application
-int avrdude_message2(FILE *fp, int lno, const char *file, const char *func,
-  int msgmode, int msglvl, const char *format, ...);
+// Core messaging function, defined in avrdude_msg.c (compiled into libavrdude)
+int avrdude_message2(FILE *fp, int lno, const char *file, const char *func,
+  int msgmode, int msglvl, const char *format, ...);
```

---

## Verification

To test the shared build on Linux (sanity check before Windows):

```sh
cmake -S . -B build/shared -DBUILD_SHARED_LIBS=ON
cmake --build build/shared
# Confirm no dangling undefined symbols in the .so:
nm -u build/shared/src/libavrdude.so | grep -v UND
```

To test on Windows (MinGW):

```sh
cmake -S . -B build/shared -DBUILD_SHARED_LIBS=ON
cmake --build build/shared
# Should produce libavrdude.dll and libavrdude.dll.a with no link errors
```

Previously this would fail with hundreds of `undefined reference to avrdude_message2`
and `undefined reference to verbose` etc. errors at the DLL link step.
