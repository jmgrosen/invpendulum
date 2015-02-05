#ifndef PWM_HPP
#define PWM_HPP

#include <fstream>
#include <string>
#include <cstdint>

#include "pins.hpp"

class PWM {
public:
  PWM(Pin pin) : pin_(pin) { }
  bool write_freq(float freq);
  bool write_polarity(int polarity);
  bool write(float val);
  bool write_period(std::uint64_t period);
  bool write_duty(std::uint64_t duty);
  bool begin(float duty_cycle=0.0, float frequency=2000.0, int polarity=1);

private:
  Pin pin_;
  std::uint64_t duty_;
  std::uint64_t period_ns_;
  std::fstream period_f_;
  std::fstream duty_f_;
  std::fstream polarity_f_;
};

#endif
