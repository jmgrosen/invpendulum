#include <iostream>
#include "servo.hpp"

#define DEAD_ZONE (50000)

bool Servo::begin() {
  negative_.begin();
  // positive_.begin();

  negative_.write_duty(1500000);
  // positive_.write_duty(1500000 + DEAD_ZONE);

  return true;
}

bool Servo::write_raw(std::uint64_t val) {
  return negative_.write_duty(val);
}

std::int64_t sign(float val) {
  return (val >= 0.0f) ? 1 : -1;
}

bool Servo::write(float val) {
  val *= -1;

  // float normalized = (val + 1.0) / 2.0;
  float normalized = val;
  float newrange = static_cast<float>(high_ - 1500000);
  std::int64_t ival = static_cast<std::int64_t>(normalized * newrange);

  if (abs(ival) < 1000) {
    ival = ival * 50;
  } else {
    ival = sign(val) * 50000 + ival;
  }
  // if (ival >= 1500000) {
  //   negative_.write_duty(1500000 - DEAD_ZONE);
  //   positive_.write_duty(ival + DEAD_ZONE);
  // } else {
  //   negative_.write_duty(ival - DEAD_ZONE);
  //   positive_.write_duty(1500000 + DEAD_ZONE);
  // }

  // std::cout << "ival is " << ival << std::endl;

  std::uint64_t out = static_cast<std::uint64_t>(ival + 1500000);

  if (out > high_) {
    out = high_;
  } else if (out < low_) {
    out = low_;
  }

  negative_.write_duty(out);

  return true;
}
