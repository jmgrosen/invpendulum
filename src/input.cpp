#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>

#include "input.hpp"
#include "pins.hpp"

bool Input::begin() {
  gpio_ = pin_to_gpio(pin_);

  std::cout << "gpio_ is " << gpio_ << std::endl;

  struct stat stat_result;

  if (stat(("/sys/class/gpio/gpio" + std::to_string(gpio_)).c_str(), &stat_result) != 0) {
    std::fstream export_f("/sys/class/gpio/export", std::ios::out);
    export_f << gpio_;
    export_f.flush();
    if (!export_f.good()) {
      std::cout << "exporting not good" << std::endl;
      return false;
    }
  }

  std::fstream dir_f("/sys/class/gpio/gpio" + std::to_string(gpio_) + "/direction");
  dir_f << "input";
  if (!dir_f.good()) {
    return false;
  }

  value_f_.open("/sys/class/gpio/gpio" + std::to_string(gpio_) + "/value");
  return value_f_.good();
}

bool Input::setEdge(std::string edge) {
  std::fstream edge_f("/sys/class/gpio/gpio" + std::to_string(gpio_) + "/edge");
  edge_f << edge;
  edge_f.flush();
  return edge_f.good();
}

int Input::read() {
  int value;
  value_f_.seekg(0);
  value_f_ >> value;
  return value_f_.good() ? value : -1;
}
