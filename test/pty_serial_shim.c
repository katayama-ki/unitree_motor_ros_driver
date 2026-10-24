// SPDX-FileCopyrightText: Copyright katayama-ki
// SPDX-License-Identifier: MIT

// Test-only preload: Linux PTYs lack the UART low-latency ioctls required by the
// SDK. Emulate those two ioctls exclusively on the test's named PTY, forwarding
// them otherwise. Pass TIOCEXCL/TIOCNXCL through so the driver's port lock is
// exercised on the PTY. Reject all other requests without reading variadic arguments.
// Never linked into the production driver.
#define _GNU_SOURCE
#include <errno.h>
#include <linux/serial.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>

// Must match glibc's ioctl prototype exactly to interpose it.
int ioctl(int fd, unsigned long request, ...)  // NOLINT(runtime/int)
{
  if (request == TIOCEXCL || request == TIOCNXCL) {
    return syscall(SYS_ioctl, fd, request);
  }
  if (request != TIOCGSERIAL && request != TIOCSSERIAL) {
    errno = ENOTTY;
    return -1;
  }
  va_list args;
  va_start(args, request);
  struct serial_struct * argument = va_arg(args, struct serial_struct *);
  va_end(args);
  const char * test_pty = getenv("UNITREE_TEST_PTY");
  char name[256];
  if (
    test_pty && ttyname_r(fd, name, sizeof(name)) == 0 && strncmp(name, "/dev/pts/", 9) == 0 &&
    strcmp(name, test_pty) == 0)
  {
    if (request == TIOCGSERIAL) {
      memset(argument, 0, sizeof(*argument));
      argument->baud_base = 24000000;
    }
    return 0;
  }
  return syscall(SYS_ioctl, fd, request, argument);
}
