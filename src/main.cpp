#include <time.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <sched.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>

#include "eqep.h"
#include "pwm.hpp"
// #include "servo.hpp"
#include "motor.hpp"

#include "readerwriterqueue/readerwriterqueue.h"

#define MY_PRIORITY (80)
#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC (1000000000)

#define INTERVAL (2000000) /* 10ms, i.e. 10000000ns */

struct LogEntry {
  LogEntry() : time_({.tv_sec = 0, .tv_nsec = 0}), u_cart_(0.0f), x_cart_(0.0f), desired_(0.0f), theta_pend_(0.0f) { }

  LogEntry(struct timespec time, float u_cart, float x_cart, float dx_cart, float desired, float theta_pend, float dtheta_pend) :
    time_(time), u_cart_(u_cart), x_cart_(x_cart), dx_cart_(dx_cart), desired_(desired), theta_pend_(theta_pend), dtheta_pend_(dtheta_pend) { }

  struct timespec time_;
  float u_cart_;
  float x_cart_;
  float dx_cart_;
  float desired_;
  float theta_pend_;
  float dtheta_pend_;
};

class VelEstimator {
public:
  VelEstimator(float initial) : last_last_pos_(initial), last_pos_(initial) { }

  float step(float new_pos) {
    float new_vel = b1_ * last_vel_ + b2_ * last_last_vel_
      + b3_ * new_pos + b4_ * last_last_pos_;

    last_last_pos_ = last_pos_;
    last_pos_ = new_pos;
    last_last_vel_ = last_vel_;
    last_vel_ = new_vel;

    return new_vel;
  }

private:
  float last_last_pos_;
  float last_pos_;
  float last_last_vel_;
  float last_vel_;

  const float b1_ = 1.636;
  const float b2_ = -0.6694;
  const float b3_ = 8.264;
  const float b4_ = -8.264;
};

moodycamel::ReaderWriterQueue<LogEntry, 1024> log_queue(1024);

void stack_prefault(void) {
  unsigned char dummy[MAX_SAFE_STACK];
  memset(dummy, 0, sizeof(dummy));
}

float sat(float val, float bound) {
  if (val > bound)
    return 0.2;
  else if (val < -bound)
    return -0.2;
  else
    return val;
}

std::vector<LogEntry> playback;

enum controller_state {
  SWING_UP_LEFT,
  SWING_UP_RIGHT,
  MAINTAIN,
};

float func(float time) {
  time = fmodf(time, 4.0f);
  if (time < 2.0f) {
    return 0.6f;
  } else {
    return -0.6f;
  }
}

float wrap_radians(float ang) {
  if (ang > 0) {
    return fmodf(ang + M_PI, 2.0*M_PI) - M_PI;
  } else {
    return fmodf(ang - M_PI, 2.0*M_PI) + M_PI;
  }
}

void *body(void *param) {
  struct timespec t;
  struct timespec actual_time;
  struct sched_param sparam;
  volatile eqep *position_encoder;
  volatile eqep *angle_encoder;
  int32_t position_initial;
  int32_t angle_initial;
  // Servo motor(SERVO_P9_14, SERVO_P9_16, 1400000, 1600000);
  Motor motor(P9_15, P9_14, 0.6);
  // Output out(P9_15);
  enum controller_state state = MAINTAIN;

  const float Ts = ((float) INTERVAL) / 1000000000.0;
  float Kp_cart = -4.3;
  float Kd_cart = -0.0; // .37
  float Kp_pend = 0.0;
  float Kd_pend = 0.0;
  const float f = 1.0;
  
  VelEstimator cart_estimator(0.0);
  VelEstimator pend_estimator(M_PI);
  float last_filtered_u_cart = 0.0;
  float last_u_cart = 0.0;
  float last_x_cart = 0.0;
  float last_error_x = 0.0;
  float last_control = 0.0;
  float last_cart_vel = 0.0;
  float last_theta_pend = M_PI;

  // out.begin();
  // out.write(true);

  sparam.sched_priority = MY_PRIORITY;
  if (sched_setscheduler(0, SCHED_FIFO, &sparam) == -1) {
    perror("sched_setscheduler failed");
    exit(1);
  }

  if (eqep_setup() < 0) {
    perror("eqep_setup failed");
    exit(1);
  }

  if (eqep_init("bone_eqep1", EQEP1, 1000000, &position_encoder) < 0) {
    perror("eqep_init failed");
    exit(1);
  }

  if (eqep_init("bone_eqep2b", EQEP2, 1000000, &angle_encoder) < 0) {
    perror("eqep_init failed");
    exit(1);
  }

  position_initial = position_encoder->count;
  angle_initial = angle_encoder->count;

  stack_prefault();

  motor.begin();

  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_nsec += INTERVAL;
  float time_initial = ((float) t.tv_sec) + ((float) t.tv_nsec) / 1000000000;

  const float Dswing = 0.1;

  printf("Kp_cart = %f\n", Kp_cart);

  float desired = 0.0;

  //  for (auto entry : playback) {
  for (int i = 0; i < 100000; i++) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    /* MAIN LOOP START */

    int32_t raw_position = position_encoder->count;
    // std::cout << "raw_position is " << raw_position << std::endl;
    int32_t raw_angle = angle_encoder->count;

    int32_t position_diff = raw_position - position_initial;
    int32_t angle_diff = raw_angle - angle_initial;

    float x_cart = ((float) position_diff) * (2.0f * M_PI / 4096.0) * (30.0 / 34.0) * (0.0541 / 2.0);
    float theta_pend = ((float) (angle_diff + 2048)) * (2.0f * M_PI / 4096.0);
    // std::cout << theta_pend << std::endl;
    float cart_velocity = (x_cart - last_x_cart) / Ts;
    float pend_velocity = (theta_pend - last_theta_pend) / Ts;

    clock_gettime(CLOCK_MONOTONIC, &actual_time);
    float time = (((float) actual_time.tv_sec) + ((float) actual_time.tv_nsec) / 1000000000) - time_initial;
    // float desired = 2 * sin(M_PI * time);
    // float desired = func(time);

    switch (state) {
    case SWING_UP_LEFT:
      desired = -Dswing;
      if (time > 0.5f && theta_pend < last_theta_pend) {
        state = SWING_UP_RIGHT;
      }
      // if (fmodf(time, 6.0f) > 3.0f) {
      //   state = SWING_UP_RIGHT;
      // }
      break;
    case SWING_UP_RIGHT:
      desired = Dswing;
      if (time > 0.5f && theta_pend > last_theta_pend) {
        state = SWING_UP_LEFT;
      }
      // if (fmodf(time, 6.0f) < 3.0f) {
      //   state = SWING_UP_LEFT;
      // }
      break;
    case MAINTAIN:
      // Kp_cart = 0.0;
      Kp_pend = 7.3;
      Kd_pend = 0.7;
      break;
    default:
      break;
    }

    float error_x = desired - x_cart;
    float derror_x = cart_estimator.step(x_cart);
    float error_th = 0 - theta_pend;
    float error_th_wrap = wrap_radians(error_th);
    float derror_th = -pend_estimator.step(theta_pend);

    // std::cout << "error_x = " << error_x << ", derror_x = " << derror_x << std::endl;
    // std::cout << "error_th_wrap = " << error_th_wrap << ", derror_th = " << derror_th << std::endl;

    if (state != MAINTAIN && (fabs(error_th_wrap) < 0.4f && fabs(derror_th) > 0.5)) {
      std::cout << "inverted the pendulum at " << actual_time.tv_sec << "s, " << actual_time.tv_nsec << "ns" << std::endl;
      // motor.write(0.0f);
      state = MAINTAIN;
    }

    if (x_cart > 0.2 || x_cart < -0.2) {
      motor.write(0.0f);
      std::cout << "cart too close to edges, aborting" << std::endl;
      break;
    }

    // float to_write = Kp_cart * error + (Ki * Ts - Kp_cart) * last_error + last_control;
    float u_cart = Kp_cart * error_x + Kd_cart * derror_x
      + Kp_pend * error_th_wrap + Kd_pend * derror_th;
    // float u_cart = Kp_cart * error_x + Kd_cart * derror_th;
    if (u_cart > 25.0) {
      u_cart = 25.0;
    } else if (u_cart < -25.0) {
      u_cart = -25.0;
    }

    const float a = 0.95;

    const float filtered_u_cart = a * last_filtered_u_cart + ((1 - a) / 2) * (u_cart + last_u_cart);

    if (time > 8.0) {
      motor.write(u_cart);
    }
    // if (x_cart <= 0.90) {
    //   motor.write_raw(1450000);
    // } else {
    //   motor.write_raw(1500000);
    // }

    log_queue.try_enqueue(LogEntry(actual_time, filtered_u_cart, x_cart, derror_x, desired, theta_pend, derror_th));

    last_filtered_u_cart = filtered_u_cart;
    last_u_cart = u_cart;
    last_x_cart = x_cart;
    last_error_x = error_x;
    last_control = u_cart;
    last_cart_vel = cart_velocity;
    last_theta_pend = theta_pend;

    /* MAIN LOOP END */

    t.tv_nsec += INTERVAL;
    /* adjust for when tv_nsec overflows (logically) */
    while (t.tv_nsec >= NSEC_PER_SEC) {
      t.tv_nsec -= NSEC_PER_SEC;
      t.tv_sec++;
    }
  }
  sleep(1);
}

void *logthread(void *data) {
  FILE *logf = fopen("log.csv", "w");
  fputs("times,timens,u_cart,x_cart,dx_cart,desired,theta_pend,dtheta_pend\n", logf);

  while (1) {
    LogEntry entry;

    while (log_queue.try_dequeue(entry)) {
      fprintf(logf, "%ld,%lu,%f,%f,%f,%f,%f,%f\n", entry.time_.tv_sec, entry.time_.tv_nsec,
              entry.u_cart_, entry.x_cart_, entry.dx_cart_, entry.desired_, entry.theta_pend_, entry.dtheta_pend_);
    }

    fflush(logf);
    sleep(1);
  }
}

LogEntry read_val(FILE *f) {
  LogEntry entry;

  // fscanf(f, "%ld,%lu,%f,%f,%d\n", &entry.time_.tv_sec, &entry.time_.tv_nsec, &entry.written_, &entry.pos_val_, &entry.angle_val_);

  return entry;
}

std::vector<LogEntry> read_all(void) {
  std::vector<LogEntry> entries;
  FILE *f = fopen("log.csv", "r");
  char buf[512];

  fgets(buf, 512, f);

  while (!feof(f)) {
    entries.push_back(read_val(f));
  }

  return entries;
}

int main(int argc, char *argv[]) {
  pthread_t realtime_thread;
  pthread_t log_thread;

  // playback = read_all();

  /* first, prevent memory from swapping (unlikely, but might as well) */
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    perror("mlockall failed");
    exit(1);
  }

  /* start real-time thread */
  if (pthread_create(&realtime_thread, NULL, body, NULL) != 0) {
    perror("pthread_create failed");
    exit(2);
  }

  pthread_create(&log_thread, NULL, logthread, NULL);

  if (pthread_join(realtime_thread, NULL) != 0) {
    perror("pthread_join failed");
    exit(3);
  }

  sleep(1);

  return 0;
}
