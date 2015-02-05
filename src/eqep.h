#ifndef EQEP_H
#define EQEP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  EQEP0 = 0x48300180,
  EQEP1 = 0x48302180,
  EQEP2 = 0x48304180
};

int eqep_setup(void);
int eqep_teardown(void);

int eqep_init(uint32_t addr, uint32_t period, volatile int32_t **mapping);
int eqep_destroy(volatile int32_t **mapping);

#ifdef __cplusplus
}
#endif

#endif
