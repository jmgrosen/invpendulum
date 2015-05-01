#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>

#include "output.hpp"
#include "pins.hpp"

bool Output::begin() {
  int gpio = pin_to_gpio(pin_);

  std::cout << "gpio is " << gpio << std::endl;

  struct stat stat_result;

  if (stat(("/sys/class/gpio/gpio" + std::to_string(gpio)).c_str(), &stat_result) != 0) {
    std::fstream export_f("/sys/class/gpio/export", std::ios::out);
    export_f << gpio;
    export_f.flush();
    if (!export_f.good()) {
      std::cout << "exporting not good" << std::endl;
      return false;
    }
  }

  std::fstream dir_f("/sys/class/gpio/gpio" + std::to_string(gpio) + "/direction");
  dir_f << "out";
  if (!dir_f.good()) {
    return false;
  }

  value_f_.open("/sys/class/gpio/gpio" + std::to_string(gpio) + "/value");
  return value_f_.good();
}

bool Output::write(bool value) {
  value_f_ << (value ? "1" : "0");
  value_f_.flush();
  return value_f_.good();
}
