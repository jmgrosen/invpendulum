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

#pragma pack(push, 1)
typedef struct eqep_ {
  int32_t count; // 0h
  int32_t count_init; // 4h
  int32_t count_max; // 8h
  int32_t count_cmp; // Ch
  int32_t index_latch; // 10h
  int32_t strobe_latch; // 14h
  int32_t count_latch; // 18h
  int32_t unit_timer; // 1Ch
  int32_t unit_period; // 20h
  uint16_t watchdog_timer; // 24h
  uint16_t watchdog_period; // 26h
  uint16_t decoder_control; // 28h
  uint16_t control; // 2Ah
} eqep;
#pragma pack(pop)

int eqep_setup(void);
int eqep_teardown(void);

int eqep_init(const char *name, uint32_t addr, uint32_t period, volatile eqep **mapping);
int eqep_destroy(volatile eqep **mapping);

#ifdef __cplusplus
}
#endif

#endif
