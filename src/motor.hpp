#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "pins.hpp"
#include "pwm.hpp"
#include "output.hpp"

class Motor {
public:
  Motor(Pin dir_pin, Pin speed_pin, float cap) : dir_(dir_pin), speed_(speed_pin), cap_(cap) { }
  bool begin();
  bool write(float val);

private:
  Output dir_;
  PWM speed_;
  float cap_;
};

#endif
