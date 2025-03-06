#include "RealTimeClock.h"
#include <esp_sntp.h>
#include <Arduino.h>

RealTimeClock::RealTimeClock() = default;

void RealTimeClock::setup() {
  Serial.println(F("Initializing SNTP..."));
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
  Serial.println(F("SNTP initialized. Waiting for time synchronization..."));

  // Wait for time to be set
  time_t now = 0;
  struct tm timeinfo;
  while (timeinfo.tm_year < (2016 - 1900)) {
    delay(1000);
    time(&now);
    localtime_r(&now, &timeinfo);
    Serial.print(F("."));
  }
  Serial.println(F("\nTime synchronized."));
}

void RealTimeClock::add_on_time_sync_callback(std::function<void()> callback) {
  this->time_sync_callbacks_.push_back(callback);
}

struct tm RealTimeClock::now() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  return timeinfo;
}

std::string RealTimeClock::get_current_time_str() {
  struct tm timeinfo = this->now();
  char buffer[64];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return std::string(buffer);
}

void RealTimeClock::synchronize_epoch_(uint32_t epoch) {
  struct timeval tv = { .tv_sec = static_cast<time_t>(epoch), .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  this->apply_timezone_();
  for (auto &callback : this->time_sync_callbacks_) {
    callback();
  }
}

void RealTimeClock::apply_timezone_() {
  // Set the timezone here if needed
}