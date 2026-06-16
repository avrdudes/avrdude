Fixes #1414 — Unable to build libavrdude dynamic library under Windows

Root cause
----------
libavrdude called avrdude_message2() and referenced global variables
(verbose, progname, ovsigck, partdesc, pgmid, quell_progress, cx) that
were defined in main.c (the executable). On Linux/macOS this accidentally
works via -Wl,-undefined,dynamic_lookup or flat namespace linking, but
Windows DLLs require all symbols to be resolved at library link time,
making BUILD_SHARED_LIBS=ON fail with hundreds of "undefined reference"
errors on both MinGW and MSVC.

Fix
---
Move the above definitions into a new file src/avrdude_msg.c, which is
compiled into libavrdude. main.c is left with only deletions — all its
assignments to these variables (verbose = 0, pgmid = optarg, etc.)
remain unchanged, they simply now write into variables owned by the
library.

The move of cx (libavrdude_context *) is consistent with the comment
in libavrdude.h that cx should "eventually be the only global variable".

The LIBAVRDUDE_BUILD_DLL compile definition and LIBAVRDUDE_API annotation
are added so that MSVC generates a proper .lib import library when
building the shared DLL.

No behaviour change. Zero logic modifications.

Files changed
-------------
src/avrdude_msg.c       new file — receives globals + avrdude_message2()
src/main.c              deletions only
src/CMakeLists.txt      add avrdude_msg.c to SOURCES; add LIBAVRDUDE_BUILD_DLL
src/avrdude.h           add LIBAVRDUDE_API macro; annotate externs
src/libavrdude.h        fix stale comment on avrdude_message2()

Tested
------
[ ] Linux static (existing CI)
[ ] Linux shared (BUILD_SHARED_LIBS=ON)
[ ] Windows MinGW shared
[ ] Windows MSVC shared
