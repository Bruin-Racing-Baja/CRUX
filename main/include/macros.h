#ifndef MACROS_H
#define MACROS_H

#include <cstdint>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(x, low, high) (MIN(MAX(x, low), high))

inline float map_int_to_float(int32_t x, int32_t in_min, int32_t in_max, float out_min,
                              float out_max) {
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

#endif