#ifndef __UTILS_H__
#define __UTILS_H__

#include <cstdio>

#ifndef DEBUG_FLAG
#define DEBUG_FLAG 0
#endif

#define DEBUG(X) do { \
        if (DEBUG_FLAG == 1) {X;} \
    } while (0)

#define dbg_print(format, ...) \
  fprintf (stderr, "Error at %s:%d " format, __FILE__, __LINE__, ##__VA_ARGS__)

#endif