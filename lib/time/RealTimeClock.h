#pragma once

#include <functional>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <vector>

class RealTimeClock {
 public:
  RealTimeClock();
  void setup();
  void add_on_time_sync_callback(std::function<void()> callback);
  struct tm now();
  bool isValidTime(const struct tm& timeinfo);
  std::string get_current_time_str();
  void synchronize_epoch_(uint32_t epoch);

 private:
  void apply_timezone_();
  std::string timezone_;
  std::vector<std::function<void()>> time_sync_callbacks_;
};