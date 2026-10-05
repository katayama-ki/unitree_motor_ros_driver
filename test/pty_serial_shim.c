// Test-only preload: Linux PTYs lack the UART low-latency ioctls required by the
// SDK. Emulate those two ioctls exclusively on the test's named PTY; every other
// ioctl goes to the kernel. Never linked into the production driver.
#define _GNU_SOURCE
#include <linux/serial.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>

int ioctl(int fd, unsigned long request, ...) {
  va_list args;
  va_start(args, request);
  void* argument = va_arg(args, void*);
  va_end(args);
  if (request == TIOCGSERIAL || request == TIOCSSERIAL) {
    const char* test_pty = getenv("UNITREE_TEST_PTY");
    char name[256];
    if (test_pty && ttyname_r(fd, name, sizeof(name)) == 0 &&
        strncmp(name, "/dev/pts/", 9) == 0 && strcmp(name, test_pty) == 0) {
      if (request == TIOCGSERIAL) {
        struct serial_struct* serial = argument;
        memset(serial, 0, sizeof(*serial));
        serial->baud_base = 24000000;
      }
      return 0;
    }
  }
  return syscall(SYS_ioctl, fd, request, argument);
}
