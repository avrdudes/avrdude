#ifndef strutil_h
#define strutil_h

#include <string.h>

int str_starts(const char *str, const char *starts);
int str_eq(const char *str1, const char *str2);
int str_contains(const char *str, const char *substr);
int str_ends(const char *str, const char *ends);

#endif
