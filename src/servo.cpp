#include <iostream>
#include "servo.hpp"

bool Servo::begin() {
  pwm_.begin();
  pwm_.write_period(20000000);
  pwm_.write_duty(1500000);
  return true;
}

bool Servo::write(float val) {
  if (val > 1.0) {
    val = 1.0;
  } else if (val < -1.0) {
    val = -1.0;
  }
  float normalized = (val + 1.0) / 2.0;
  float newrange = static_cast<float>(high_ - low_);
  std::uint64_t ival = static_cast<std::uint64_t>(normalized * newrange) + low_;
  return pwm_.write_duty(ival);
}
