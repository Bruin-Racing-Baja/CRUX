#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H
#include <stdint.h>
#include <string.h>
#include <malloc.h>
class MedianFilter {
public:
  MedianFilter(uint32_t window_size_);
  ~MedianFilter();

  float update(float new_value);
  float get();

private:
  uint32_t window_size;
  float *window;
  float *sorted_window;
  uint32_t head;
};

#endif