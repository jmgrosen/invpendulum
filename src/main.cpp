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
#include "servo.hpp"

#include "readerwriterqueue/readerwriterqueue.h"

#define MY_PRIORITY (80)
#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC (1000000000)

#define INTERVAL (10000000) /* 10ms, i.e. 10000000ns */

struct LogEntry {
  LogEntry() : time_({.tv_sec = 0, .tv_nsec = 0}), written_(0.0f), pos_val_(0), angle_val_(0) { }

  LogEntry(struct timespec time, float written, float pos_val, float desired, float error, std::int32_t angle_val) :
    time_(time), written_(written), pos_val_(pos_val), desired_(desired), error_(error), angle_val_(angle_val) { }

  struct timespec time_;
  float written_;
  float pos_val_;
  float desired_;
  float error_;
  std::int32_t angle_val_;
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

void *body(void *param) {
  struct timespec t;
  struct timespec actual_time;
  struct sched_param sparam;
  volatile eqep *position_encoder;
  volatile eqep *angle_encoder;
  int32_t position_initial;
  int32_t angle_initial;
  Servo motor(P9_14, 1300000, 1700000);

  const float Ts = ((float) INTERVAL) / 1000000000.0;
  const float Kp = 0.4;
  const float Ki = 4.0;

  float last_error = 0.0;
  float last_control = 0.0;

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

  printf("Kp = %f, (Ki * Ts - Kp) = %f\n", Kp, Ki * Ts - Kp);

  //  for (auto entry : playback) {
  for (int i = 0; i < 10000; i++) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    /* MAIN LOOP START */

    int32_t raw_position = position_encoder->count;
    int32_t angle = angle_encoder->count;

    int32_t position_diff = raw_position - position_initial;
    // position_diff = 0.0;

    float position = ((float) position_diff) * (2.0f * M_PI / 4096.0) * (30.0 / 34.0) * 7.0;

    clock_gettime(CLOCK_MONOTONIC, &actual_time);
    float time = (((float) actual_time.tv_sec) + ((float) actual_time.tv_nsec) / 1000000000) - time_initial;
    float desired = 2 * sin(M_PI * time);
    float error = desired + position;

    float to_write = Kp * error + (Ki * Ts - Kp) * last_error + last_control;
    if (to_write > 25.0) {
      to_write = 25.0;
    } else if (to_write < -25.0) {
      to_write = -25.0;
    }
    motor.write(to_write);

    log_queue.try_enqueue(LogEntry(actual_time, to_write, position, desired, error, angle));

    last_error = error;
    last_control = to_write;

    /* MAIN LOOP END */

    t.tv_nsec += INTERVAL;
    /* adjust for when tv_nsec overflows (logically) */
    while (t.tv_nsec >= NSEC_PER_SEC) {
      t.tv_nsec -= NSEC_PER_SEC;
      t.tv_sec++;
    }
  }
}

void *logthread(void *data) {
  FILE *logf = fopen("log.csv", "w");
  fputs("times,timens,written,pos,desired,error,angle\n", logf);

  while (1) {
    LogEntry entry;

    while (log_queue.try_dequeue(entry)) {
      fprintf(logf, "%ld,%lu,%f,%f,%f,%f,%d\n", entry.time_.tv_sec,
              entry.time_.tv_nsec, entry.written_, entry.pos_val_, entry.desired_, entry.error_, entry.angle_val_);
    }

    fflush(logf);
    sleep(1);
  }
}

LogEntry read_val(FILE *f) {
  LogEntry entry;
  
  fscanf(f, "%ld,%lu,%f,%f,%d\n", &entry.time_.tv_sec, &entry.time_.tv_nsec, &entry.written_, &entry.pos_val_, &entry.angle_val_);

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
