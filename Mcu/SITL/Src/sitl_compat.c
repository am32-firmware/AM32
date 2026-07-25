/*
  sitl_compat.c - small portability helpers for non Linux hosts
 */

#include "sitl.h"

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

/*
  a non blocking, close-on-exec UDP socket. macOS has no
  SOCK_NONBLOCK/SOCK_CLOEXEC socket() flags
 */
int sitl_udp_socket(void)
{
#ifdef __APPLE__
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd >= 0) {
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
        fcntl(fd, F_SETFD, FD_CLOEXEC);
    }
    return fd;
#else
    return socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
#endif
}
