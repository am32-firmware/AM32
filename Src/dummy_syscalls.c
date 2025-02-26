/*
  dummy system calls to avoid linker errors with gcc 14.x
 */

int _close_r(void) { return -1; }
int _lseek_r(void) { return -1; }
int _read_r(void) { return -1; }
int _write_r(void) { return -1; }
