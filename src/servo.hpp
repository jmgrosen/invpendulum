#ifndef SERVO_HPP
#define SERVO_HPP

#include "pins.hpp"
#include "pwm.hpp"

class Servo {
public:
  Servo(Pin pin, std::uint64_t low, std::uint64_t high) : pwm_(pin), low_(low), high_(high) { }
  bool begin();
  bool write(float val);

private:
  PWM pwm_;
  std::uint64_t low_;
  std::uint64_t high_;
};
#endif
