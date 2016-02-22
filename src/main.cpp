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

#include "common.h"
#include "eqep.h"
#include "pwm.hpp"
// #include "servo.hpp"
#include "motor.hpp"
#include "input.hpp"

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
  CENTER_AND_STABILIZE,
  CENTER,
};

float func(float time) {
  time = fmodf(time, 3.0f);
  if (time < 1.5f) {
    return 0.1;
  } else {
    return -0.1;
  }
}

static int sign(float f) {
  return f >= 0 ? 1 : -1;
}

static void add_nsec_to_time(struct timespec *t, long nsec) {
  t->tv_nsec += nsec;
  while (t->tv_nsec >= NSEC_PER_SEC) {
    t->tv_nsec -= NSEC_PER_SEC;
    t->tv_sec++;
  }
}

float wrap_radians(float ang) {
  if (ang > 0) {
    return fmodf(ang + M_PI, 2.0*M_PI) - M_PI;
  } else {
    return fmodf(ang - M_PI, 2.0*M_PI) + M_PI;
  }
}

Motor motor(P9_15, P9_14, 0.6); // global for the atexit

bool routine(int32_t position_end, int32_t angle_initial,
             volatile eqep *position_encoder, volatile eqep *angle_encoder,
             Input& trigger) {
  struct timespec t;
  struct timespec actual_time;

  int32_t position_center = position_end + 6145; // 6145 = ticks from end to center (I think)

  const float Ts = ((float) INTERVAL) / 1e9;
  const float Dswing = 0.1;
  const float filter_a = 0.95;
  enum controller_state state = SWING_UP_LEFT;
  float Kp_cart = -3.8;
  float Kd_cart = 0.0;
  float Kp_pend = 0.0;
  float Kd_pend = 0.0;

  float last_filtered_u_cart = 0.0;
  float last_u_cart = 0.0;
  float last_x_cart = 0.0;
  float last_theta_pend = M_PI;

  clock_gettime(CLOCK_MONOTONIC, &t);
  add_nsec_to_time(&t, INTERVAL);
  float time_initial = ((float) t.tv_sec) + (((float) t.tv_nsec) / 1e9);
  float time_maintained = 0.0;
  float last_state_change = 0.0;

  std::cout << "state is " << state << std::endl;

  while (1) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    int32_t raw_position = position_encoder->count;
    int32_t raw_angle = angle_encoder->count;

    int32_t position_diff = raw_position - position_center;
    int32_t angle_diff = raw_angle - angle_initial;

    float x_cart = ((float) position_diff) * (2.0 * M_PI / 4096.0) * (30.0 / 34.0) * (0.0541 / 2.0);
    float theta_pend = ((float) (angle_diff + 2048)) * (2.0 * M_PI / 4096.0);

    float cart_velocity = (x_cart - last_x_cart) / Ts;
    float pend_velocity = (theta_pend - last_theta_pend) / Ts;

    clock_gettime(CLOCK_MONOTONIC, &actual_time);
    float time = (((float) actual_time.tv_sec) + ((float) actual_time.tv_nsec) / 1000000000) - time_initial;
    float desired = 0.0;
    float desired_pend = 0.0;

    switch (state) {
    case SWING_UP_LEFT:
      desired = -Dswing;
      if (time > 0.5f && theta_pend < last_theta_pend) {
        state = SWING_UP_RIGHT;
      }
      break;
    case SWING_UP_RIGHT:
      desired = Dswing;
      if (time > 0.5f && theta_pend > last_theta_pend) {
        state = SWING_UP_LEFT;
      }
      break;
    case MAINTAIN:
      desired = 0.0;
      if (time - time_maintained > 1.0f) {
        // desired = 0.1 * sin(((time - time_maintained) - 1.0) * 4.0);
      }
      Kp_cart = 3.8;
      Kd_cart = 2.4;
      Kp_pend = 5.7;
      Kd_pend = 0.8;
      break;
    case CENTER:
      Kp_cart = -2.0;
      Kd_cart = 0.0;
      Kp_pend = 0.0;
      Kd_pend = 0.0;
      desired = 0.0;
      break;
    case CENTER_AND_STABILIZE:
      if (time - last_state_change > 15.0) {
        Kp_cart = -4.3;
        Kd_cart = -2.0;
        Kp_pend = -4.0;
        Kd_pend = 0.01;
        desired = 0.0;
        desired_pend = M_PI;
      } else {
        Kp_cart = 0.0;
        Kd_cart = 0.0;
        Kp_pend = 0.0;
        Kd_pend = 0.0;
        desired = 0.0;
        desired_pend = M_PI;
      }
      break;
    default:
      break;
    }

    float error_x = desired - x_cart;
    float derror_x = -(x_cart - last_x_cart) / Ts;
    float error_th = desired_pend - theta_pend;
    float error_th_wrap = wrap_radians(error_th);
    float derror_th = -(theta_pend - last_theta_pend) / Ts;

    if (state == CENTER_AND_STABILIZE && time - last_state_change > 15.0) {
      std::cout << "error_th = " << error_th << ", error_th_wrap = " << error_th_wrap << std::endl;
    }

    if (state == CENTER && fabs(x_cart) < 0.03 && fabs(error_th_wrap) > (M_PI - 0.1) && fabs(derror_th) < 0.2) {
      std::cout << "stopped centering" << std::endl;
      return false;
    }

    if (state != MAINTAIN && state != CENTER && state != CENTER_AND_STABILIZE && (fabs(error_th_wrap) < 0.2f && fabs(derror_th) > 0.5 && fabs(derror_th) < 3.0)
        //        && sign(derror_th) == 1
        ) {
      std::cout << "inverted the pendulum at " << actual_time.tv_sec << "s, " << actual_time.tv_nsec << "ns" << std::endl;
      std::cout << "derror_th was " << derror_th << std::endl;
      state = MAINTAIN;
      time_maintained = time;
    }

    if (state != CENTER && time > 0.5 && (x_cart > 0.2 || x_cart < -0.2)) {
      motor.write(0.0);
      // std::cout << "cart too close to edges, STOPPING" << std::endl;
      // return false;
      std::cout << "cart too close to edges, centering then stopping" << std::endl;
      state = CENTER;
      continue;
    }

    int trigger_res;
    if ((trigger_res = trigger.read()) != 1) {
//    if (state != CENTER_AND_STABILIZE && time > 30.0) {
      motor.write(0.0);
      std::cout << "inside loop, trigger_res = " << trigger_res << std::endl;
      std::cout << "trigger either failed or went low, stopping" << std::endl;
      // state = CENTER_AND_STABILIZE;
      // last_state_change = time;
      // continue;
      return true;
    }

    float u_cart = Kp_cart * error_x + Kd_cart * derror_x
      + Kp_pend * error_th_wrap + Kd_pend * derror_th;

    const float filtered_u_cart = filter_a * last_filtered_u_cart + ((1 - filter_a) / 2) * (u_cart + last_u_cart);
    motor.write(filtered_u_cart);
    // no logging in production
    // log_queue.try_enqueue(LogEntry(actual_time, filtered_u_cart, x_cart, derror_x, desired, theta_pend, derror_th));

    last_filtered_u_cart = filtered_u_cart;
    last_u_cart = u_cart;
    last_x_cart = x_cart;
    last_theta_pend = theta_pend;

    add_nsec_to_time(&t, INTERVAL);
  }
  return true; // :)
}

void clear_motor() {
  motor.write(0.0);
}

void *body(void *param) {
  struct timespec t;
  struct sched_param sparam;
  volatile eqep *position_encoder;
  volatile eqep *angle_encoder;
  int32_t position_end;
  int32_t angle_initial;
  Input limit_switch(P8_13);
  Input trigger(P8_8);

  sparam.sched_priority = MY_PRIORITY;
  if (sched_setscheduler(0, SCHED_FIFO, &sparam) == -1) {
    perror("sched_setscheduler failed");
    exit(1);
  }

  stack_prefault();

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

  load_device_tree("limit-switch");
  if (!limit_switch.begin()) {
    perror("limit_switch.begin() failed");
    exit(1);
  }

  if (!trigger.begin()) {
    perror("trigger.begin() failed");
    exit(1);
  }

  angle_initial = angle_encoder->count;
  motor.begin();
  std::atexit(clear_motor);

  clock_gettime(CLOCK_MONOTONIC, &t);
  add_nsec_to_time(&t, INTERVAL);

  int switch_res;
  while ((switch_res = limit_switch.read()) == 1) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    motor.write(0.15);
    add_nsec_to_time(&t, INTERVAL);
  }

  motor.write(0.0);
  position_end = position_encoder->count;
  std::cout << "position_end is " << position_end << std::endl;

  if (switch_res == -1) {
    perror("reading from limit switch failed");
    exit(1);
  }

  while (1) {
    // wait for signal
    // trigger.setEdge("both");
    // interrupts aren't working, use polling instead
    clock_gettime(CLOCK_MONOTONIC, &t);
    add_nsec_to_time(&t, 200000000);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    int trigger_res = trigger.read();
    std::cout << "outside loop, trigger_res = " << trigger_res << std::endl;
    if (trigger_res == 0) {
      continue;
    } else if (trigger_res == -1) {
      // uhoh, a gpio error?
      std::cout << "gpio error, exiting" << std::endl;
      break;
    }

    if (!routine(position_end, angle_initial, position_encoder, angle_encoder, trigger)) {
      sleep(2);
    }
    motor.write(0.0);
    // std::cout << "just doing once" << std::endl;
    // break;
  }
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

  // no logging in production
  // pthread_create(&log_thread, NULL, logthread, NULL);

  if (pthread_join(realtime_thread, NULL) != 0) {
    perror("pthread_join failed");
    exit(3);
  }

  sleep(1);

  return 0;
}
