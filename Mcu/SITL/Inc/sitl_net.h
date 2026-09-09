#pragma once
/*
  socket-layer portability for the SITL UDP/multicast sockets. POSIX
  hosts use the BSD sockets API directly; native Windows (MinGW) maps it
  onto Winsock2. winsock2.h must be included before windows.h, so this
  header is included ahead of any <windows.h>.
 */
#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600   // WSAPoll, inet_pton, struct pollfd
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

// POSIX spellings Winsock lacks. The SITL sockets are created
// non-blocking, so a zero recv flag matches MSG_DONTWAIT semantics.
#ifndef MSG_DONTWAIT
#define MSG_DONTWAIT 0
#endif
#define poll WSAPoll

// Winsock takes char* buffers and int lengths where POSIX takes void*
// and size_t. Wrap the transfer calls to cast at every call site; the
// preprocessor does not re-expand a macro inside its own replacement, so
// the inner name still resolves to the Winsock function.
#define sendto(s, b, n, f, a, l) \
    sendto((s), (const char*)(b), (int)(n), (f), (a), (int)(l))
#define recvfrom(s, b, n, f, a, l) \
    recvfrom((s), (char*)(b), (int)(n), (f), (a), (l))
#define send(s, b, n, f)  send((s), (const char*)(b), (int)(n), (f))
#define recv(s, b, n, f)  recv((s), (char*)(b), (int)(n), (f))

// initialise Winsock once (idempotent); a no-op on POSIX
void sitl_net_startup(void);
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#define closesocket close     // POSIX closes sockets with close()
static inline void sitl_net_startup(void) {}
#endif
