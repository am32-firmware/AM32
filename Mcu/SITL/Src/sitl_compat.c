/*
  sitl_compat.c - small portability helpers for non Linux hosts
 */

#include "sitl.h"
#include "sitl_net.h"

#include <unistd.h>

#ifdef _WIN32
void sitl_net_startup(void)
{
    static bool started;
    if (!started) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
        started = true;
    }
}

// a non blocking UDP socket. Winsock has no SOCK_NONBLOCK/SOCK_CLOEXEC
// flags, so set non-blocking with ioctlsocket and clear handle
// inheritance so a re-exec on reset does not leak the socket
int sitl_udp_socket(void)
{
    sitl_net_startup();
    SOCKET fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd != INVALID_SOCKET) {
        u_long nb = 1;
        ioctlsocket(fd, FIONBIO, &nb);
        SetHandleInformation((HANDLE)fd, HANDLE_FLAG_INHERIT, 0);
    }
    // socket handles fit in an int in practice; INVALID_SOCKET maps to -1
    return (int)fd;
}
#else
#include <fcntl.h>

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
        // A connected UDP send can raise SIGPIPE on macOS after an
        // asynchronous socket error. The simulator handles send errors
        // itself, so never let one terminate the whole SITL process.
        const int one = 1;
        setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
    }
    return fd;
#else
    return socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
#endif
}
#endif
