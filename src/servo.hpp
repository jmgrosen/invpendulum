#ifndef SERVO_HPP
#define SERVO_HPP

#include "pins.hpp"
#include "pwm.hpp"

class Servo {
public:
  Servo(Pin negative, Pin positive, std::uint64_t low, std::uint64_t high)
    : negative_(negative), positive_(positive), low_(low), high_(high) { }
  bool begin();
  bool write(float val);
  bool write_raw(std::uint64_t val);

private:
  PWM negative_;
  PWM positive_;
  std::uint64_t low_;
  std::uint64_t high_;
};
#endif
