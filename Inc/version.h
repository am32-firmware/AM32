/*
  update this file for new releases
 */
#define VERSION_MAJOR 2
#define VERSION_MINOR 21

// Dedicated KISS Ultra artifacts (ULTRA_DEDICATED, see Inc/ultra.h) are
// versioned n00.x: stock major n times 100, stock minor x. Stock 2.21 is
// Ultra 200.21, continuing the 100.x line of the AM32 Ultra fork.
// The Makefile derives the artifact file name the same way.
#ifdef ULTRA_DEDICATED
#define REPORTED_VERSION_MAJOR (VERSION_MAJOR * 100)
#else
#define REPORTED_VERSION_MAJOR VERSION_MAJOR
#endif

#define EEPROM_VERSION 4
