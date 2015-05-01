#include <iostream>
#include <cmath>

#include "motor.hpp"

bool Motor::begin() {
  std::cout << "dir_.begin() = " << dir_.begin() << std::endl;;

  speed_.begin();
  speed_.write_period(100000); // period = 100us
  speed_.write_duty(0);

  return true;
}

bool Motor::write(float val) {
  if (val > cap_) {
    val = cap_;
  } else if (val < -cap_) {
    val = -cap_;
  }

  // std::cout << "motor writing " << val << std::endl;

  std::uint64_t duty = static_cast<std::uint64_t>(std::abs(val) * 100000.0f);

  if (val >= 0.0f) {
    dir_.write(true);
  } else {
    dir_.write(false);
  }

  return speed_.write_duty(duty);
}
