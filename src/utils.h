#ifndef __UTILS_H__
#define __UTILS_H__

#include <cstdio>
#include <cassert>

#ifndef DEBUG_FLAG
#define DEBUG_FLAG 0
#endif

#define DEBUG(X) do { \
        if (DEBUG_FLAG == 1) {X;} \
    } while (0)

#define dbg_print(format, ...) \
  fprintf (stderr, format, ##__VA_ARGS__)

#define dbg_print_line(format, ...) \
  fprintf (stderr, "Error at %s:%d " format, __FILE__, __LINE__, ##__VA_ARGS__)

const float F_INF = 1e9;
const int INF = 0x3f3f3f3f;

#endif