#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>

// Finds a directory, starting in `start`, that begins with `prefix`.
// Useful for Linux's annoyingly non-deterministic sysfs entries.
static int build_path(const char *start, const char *prefix, char *out, size_t out_size) {
  const char *dir_name;
  DIR *path;
  struct dirent *entry;
  int printf_result;

  path = opendir(start);
  if (path != NULL) {
    while ((entry = readdir(path))) {
      dir_name = entry->d_name;
      if (strstr(dir_name, prefix) == dir_name) {
	printf_result = snprintf(out, out_size, "%s/%s", start, dir_name);
        closedir(path);
        return printf_result >= out_size ? -2 : 0;
      }
    }
    closedir(path);
  }

  return -1;
}

int load_device_tree(const char *name) {
  char *ctrl_dir;
  char slots_path[128];
  int slots_fd;
  size_t name_len;

  /* printf("going to build path\n"); */

  /* if (!build_path("/sys/devices", "bone_capemgr", ctrl_dir, sizeof(ctrl_dir))) { */
  /*   return -1; */
  /* } */

  /* printf("built path\n"); */

  ctrl_dir = "/sys/devices/bone_capemgr.9";

  if (snprintf(slots_path, sizeof(slots_path), "%s/slots", ctrl_dir) >= sizeof(slots_path)) {
    return -2;
  }

  printf(slots_path);

  if ((slots_fd = open(slots_path, O_WRONLY)) < 0) {
    return -3;
  }

  printf("opened slots, name is %s\n", name);

  name_len = strlen(name); // hopefully name is nul-terminated...
  if (write(slots_fd, name, name_len) < 0) {
    return -3;
  }

  printf("wrote slots");

  if (close(slots_fd) < 0) {
    return -3;
  }

  printf("loaded device tree\n");

  return 0;
}
