#pragma once

#include <functional>
#include <vector>

// namespace esphome {

template<typename... Ts>
class Condition {
 public:
  virtual bool check(Ts... x) = 0;
};

template<typename T>
class Parented {
 public:
  void set_parent(T *parent) { this->parent_ = parent; }
  T *get_parent() const { return this->parent_; }

 protected:
  T *parent_{nullptr};
};


template<typename... Ts>
class Action {
 public:
  virtual void play(Ts... x) = 0;
};
// }  // namespace esphome