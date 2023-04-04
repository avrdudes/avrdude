
#include "ac_cfg.h"
#include <string.h>
#include "libavrdude.h"

int str_starts(const char *str, const char *starts) {
  return strncmp(str, starts, strlen(starts)) == 0;
}

int str_eq(const char *str1, const char *str2) {
  return strcmp(str1, str2) == 0;
}

int str_contains(const char *str, const char *substr) {
  return !!strstr(str, substr);
}

int str_ends(const char *str, const char *ends) {
  size_t str_len  = strlen(str);
  size_t ends_len = strlen(ends);
  
  if (ends_len > str_len)
    return 0;
  
  for (size_t i = 0; i < ends_len; i++) {
    if (str[str_len - i] != ends[ends_len - i])
      return 0;
  }
  
  return 1;
}
