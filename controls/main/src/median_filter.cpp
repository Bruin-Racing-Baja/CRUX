#include <filters/median_filter.h>

MedianFilter::MedianFilter(uint32_t window_size_) {
  window_size = window_size_;
  if (window_size % 2 == 0) {
    window_size++;
  }
  window = (float *)calloc(window_size, sizeof(float));
  sorted_window = (float *)calloc(window_size, sizeof(float));
  head = 0;
}

MedianFilter::~MedianFilter() {
  free(window);
  free(sorted_window);
}

float MedianFilter::update(float new_value) {
  window[head] = new_value;
  head = (head + 1) % window_size;

  // Copy window to sorted_window
  memcpy(sorted_window, window, window_size * sizeof(float));

  // Sort sorted_window
  for (size_t i = 0; i < window_size - 1; i++) {
    for (size_t j = i + 1; j < window_size; j++) {
      if (sorted_window[i] > sorted_window[j]) {
        float temp = sorted_window[i];
        sorted_window[i] = sorted_window[j];
        sorted_window[j] = temp;
      }
    }
  }

  return sorted_window[window_size / 2];
}

float MedianFilter::get() { return sorted_window[window_size / 2]; }