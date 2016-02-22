#ifndef INPUT_HPP
#define INPUT_HPP

#include <fstream>
#include "pins.hpp"

class Input {
public:
  Input(Pin pin) : pin_(pin) { }
  bool begin();
  bool setEdge(std::string edge);
  int read();

private:
  Pin pin_;
  int gpio_;
  std::fstream value_f_;
};

#endif
