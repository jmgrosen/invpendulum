#include <time.h>
#include <sched.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <pthread.h>
#include <string.h>

#include "eqep.h"

#define MY_PRIORITY (80)
#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC (1000000000)

#define INTERVAL (100000000) /* 100ms, i.e. 100000000ns */

void stack_prefault(void) {
  unsigned char dummy[MAX_SAFE_STACK];
  memset(dummy, 0, sizeof(dummy));
}

void *body(void *param) {
  struct timespec t;
  struct timespec actual_time;
  struct sched_param sparam;
  volatile int32_t *encoder1;

  sparam.sched_priority = MY_PRIORITY;
  if (sched_setscheduler(0, SCHED_FIFO, &sparam) == -1) {
    perror("sched_setscheduler failed");
    exit(1);
  }

  if (eqep_setup() < 0) {
    perror("eqep_setup failed");
    exit(1);
  }

  if (eqep_init(EQEP0, 1000000, &encoder1) < 0) {
    perror("eqep_init failed");
    exit(1);
  }

  printf("encoder1 at 0x%x\n", encoder1);

  stack_prefault();

  clock_gettime(CLOCK_MONOTONIC, &t);
  /* start after a second */
  t.tv_sec++;

  while (1) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    int32_t encoder_val = *encoder1;
    clock_gettime(CLOCK_MONOTONIC, &actual_time);
    printf("enc = %d, sec = %lu, nsec = %u\n", encoder_val, actual_time.tv_sec, actual_time.tv_nsec);

    t.tv_nsec += INTERVAL;

    /* adjust for when tv_nsec overflows (logically) */
    while (t.tv_nsec >= NSEC_PER_SEC) {
      t.tv_nsec -= NSEC_PER_SEC;
      t.tv_sec++;
    }
  }
}

int main(int argc, char *argv[]) {
  pthread_t realtime_thread;

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

  if (pthread_join(realtime_thread, NULL) != 0) {
    perror("pthread_join failed");
    exit(3);
  }

  return 0;
}
