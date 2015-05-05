#include <iostream>
#include <string>

#include "pwm.hpp"
#include "common.h"

// I know, I know, I hate globals too... will fix this later
static bool pwm_initted = false;
static std::string ocp_dir = "/sys/devices/ocp.3";

static int count = 12;

bool PWM::begin(float duty_cycle, float freq, int polarity) {
  if (!pwm_initted) {
    if (!load_device_tree("am33xx_pwm")) {
      std::cout << "successfully loaded device tree for am33xx_pwm" << std::endl;
      pwm_initted = true;
    } else {
      std::cout << "could not load device tree for am33xx_pwm" << std::endl;
      return false;
    }
  }

  auto key = pin_to_key(pin_);
  std::string pwm_test_path;

  if (!load_device_tree(("bone_pwm_" + key).c_str())) {
    std::cout << "loading device tree for bone_pwm_" + key + " failed" << std::endl;
    return false;
  }
  pwm_test_path = "/sys/devices/ocp.3/" + pin_to_pwm(pin_) + "." + std::to_string(count);
  std::cout << "pwm_test_path is " << pwm_test_path << std::endl;

  count++;
  // if (!BuildPath(ocp_dir, "pwm_test_" + key, pwm_test_path)) {
  //   return false;
  // }

  period_f_.open(pwm_test_path + "/period");
  if (!period_f_.good()) {
    return false;
  }

  duty_f_.open(pwm_test_path + "/duty");
  if (!duty_f_.good()) {
    period_f_.close();
    return false;
  }

  polarity_f_.open(pwm_test_path + "/polarity");
  if (!polarity_f_.good()) {
    period_f_.close();
    duty_f_.close();
    return false;
  }

  // if (!write_freq(freq)) {
  //   return false;
  // }
  if (!write_polarity(polarity)) {
    return false;
  }
  // if (!write(duty_cycle)) {
  //   return false;
  // }

  return true;
}

bool PWM::write_freq(float freq) {
  if (freq <= 0.0) {
    return false;
  }

  std::uint32_t period_ns = static_cast<std::uint32_t>(1e9 / freq);

  if (period_ns != period_ns_) {
    period_ns_ = period_ns;

    period_f_ << period_ns;
    period_f_.flush();
    if (!period_f_.good()) {
      return false;
    }
  }

  return true;
}

bool PWM::write_period(std::uint64_t period) {
  if (period != period_ns_) {
    period_ns_ = period;

    period_f_ << period;
    period_f_.flush();
    if (!period_f_.good()) {
      return false;
    }
  }

  return true;
}

bool PWM::write_polarity(int polarity) {
  polarity_f_ << polarity;
  polarity_f_.flush();

  return polarity_f_.good();
}

bool PWM::write_duty(std::uint64_t duty) {
  duty_ = duty;
  duty_f_ << duty;
  duty_f_.flush();
  return duty_f_.good();
}

bool PWM::write(float duty) {
  if (duty < 0.0 || duty > 100.0) {
    return false;
  }

  duty_ = static_cast<std::uint32_t>(period_ns_ * (duty / 100.0));

  duty_f_ << duty_;
  duty_f_.flush();
  return duty_f_.good();
}
