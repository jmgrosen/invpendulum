#include <stdio.h>
#include <stddef.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>
#include <inttypes.h>

#include "common.h"
#include "eqep.h"

#define PAGE_SIZE 4096
#define EQEP_POS_OFFSET 0x0000
#define EQEP_QPOSMAX	0x08
#define EQEP_QEPCTL     0x2a

static int mem_fd = -1;

static int load_ocp_num(void) {
  return 3;
}

static int create_rw_map(int fd, uint32_t offset, volatile void **map) {
  void *mapping = mmap(0, PAGE_SIZE * 2, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset & (~(PAGE_SIZE-1)));
  if (mapping == MAP_FAILED) {
    perror("mmap");
    return -1;
  }
  *map = (volatile void *) (mapping + (offset & (PAGE_SIZE-1)));
  return 0;
}

int eqep_setup(void) {
  if ((mem_fd = open("/dev/mem", O_RDWR)) < 0) {
    mem_fd = -1;
    return 1;
  }

  return 0;
}

int eqep_teardown(void) {
  if (close(mem_fd) < 0) {
    return -1;
  }

  return 0;
}

static void set_helper(void *mapping, int offset, uint16_t value) {
  *(uint16_t*) (mapping + offset) = value;
}

int eqep_init(const char *eqep_name, uint32_t addr, uint32_t period, volatile eqep **mapping) {
  int ocp_num;
  char path[128];
  int enabled_fd;
  int mode_fd;
  int period_fd;

  if (load_device_tree(eqep_name) < 0) {
    return -1;
  }

  printf("loaded device tree\n");

  if ((ocp_num = load_ocp_num()) < 0) {
    return -1;
  }

  if (snprintf(path,
	       sizeof(path),
	       "/sys/devices/ocp.%d/%x.epwmss/%x.eqep/enabled",
	       ocp_num,
	       addr & 0xFFFFF000,
	       addr) >= sizeof(path)) {
    return -1;
  }

  if ((enabled_fd = open(path, O_WRONLY)) < 0) {
    perror("open(enabled)");
    return -1;
  }

  if (write(enabled_fd, "1", 1) < 0) {
    perror("write(enabled)");
    close(enabled_fd);
    return -1;
  }

  if (close(enabled_fd) < 0) {
    perror("close(enabled)");
    return -1;
  }

  if (snprintf(path,
	       sizeof(path),
	       "/sys/devices/ocp.%d/%x.epwmss/%x.eqep/mode",
	       ocp_num,
	       addr & 0xFFFFF000,
	       addr) >= sizeof(path)) {
    return -1;
  }

  if ((mode_fd = open(path, O_WRONLY)) < 0) {
    return -1;
  }

  if (write(mode_fd, "0", 1) < 0) {
    close(mode_fd);
    return -1;
  }

  if (close(mode_fd) < 0) {
    return -1;
  }

  /* if (snprintf(path, */
  /*              sizeof(path), */
  /*              "/sys/devices/ocp.%d/%x.epwmss/%x.eqep/period", */
  /*              ocp_num, */
  /*              addr & 0xFFFFF000, */
  /*              addr) >= sizeof(path)) { */
  /*   return -1; */
  /* } */

  /* if ((period_fd = open(path, O_WRONLY)) < 0) { */
  /*   return -1; */
  /* } */

  /* if (dprintf(period_fd, "%" PRIu32, period) < 0) { */
  /*   close(period_fd); */
  /*   return -1; */
  /* } */

  /* if (close(period_fd) < 0) { */
  /*   return -1; */
  /* } */

  int ret = create_rw_map(mem_fd, addr, (volatile void **) mapping);
  volatile eqep *device = *mapping;
  // why doesn't this work?
  // printf("precontrol is 0x%x\n", device->control);
  // device->control = (uint16_t) ((1 << 12) | (1 << 3));
  // printf("control is 0x%x\n", device->control);
  set_helper((void*) *mapping, 0x2a, 0x1008);
  return ret;
}

int eqep_destroy(volatile eqep **mapping) {
  if (munmap(mapping, 2 * PAGE_SIZE) < 0) {
    return -1;
  }

  return 0;
}
