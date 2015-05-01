#ifndef OUTPUT_HPP
#define OUTPUT_HPP

#include <fstream>
#include "pins.hpp"

class Output {
public:
  Output(Pin pin) : pin_(pin) { }
  bool begin();
  bool write(bool value);

private:
  Pin pin_;
  std::fstream value_f_;
};

#endif
