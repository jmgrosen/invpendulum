#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>
#include <inttypes.h>

#define PAGE_SIZE 4096
#define EQEP_POS_OFFSET 0x0000

static int mem_fd = -1;

static int load_ocp_num(void) {
  return 3;
}

static int create_rw_map(int fd, uint32_t offset, volatile int32_t **map) {
  void *mapping = mmap(0, PAGE_SIZE, PROT_READ, MAP_SHARED, fd, offset & (~(PAGE_SIZE-1)));
  if (mapping == MAP_FAILED) {
    perror("mmap");
    return -1;
  }
  *map = (volatile int32_t *) (mapping + (offset & (PAGE_SIZE-1)));
  return 0;
}

int eqep_setup(void) {
  if ((mem_fd = open("/dev/mem", O_RDONLY)) < 0) {
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

int eqep_init(uint32_t addr, uint32_t period, volatile int32_t **mapping) {
  int ocp_num;
  char path[128];
  int enabled_fd;
  int mode_fd;
  int period_fd;

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

  if (snprintf(path,
	       sizeof(path),
	       "/sys/devices/ocp.%d/%x.epwmss/%x.eqep/period",
	       ocp_num,
	       addr & 0xFFFFF000,
	       addr) >= sizeof(path)) {
    return -1;
  }

  if ((period_fd = open(path, O_WRONLY)) < 0) {
    return -1;
  }

  if (dprintf(period_fd, "%" PRIu32, period) < 0) {
    close(period_fd);
    return -1;
  }

  if (close(period_fd) < 0) {
    return -1;
  }

  return create_rw_map(mem_fd, addr + EQEP_POS_OFFSET, mapping);
}

int eqep_destroy(volatile int32_t **mapping) {
  if (munmap(mapping, PAGE_SIZE) < 0) {
    return -1;
  }

  return 0;
}
