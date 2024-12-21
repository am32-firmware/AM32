#pragma once

#include <inttypes.h>

void debug_initialize();
void debug_write_string(const char* string);
void debug_write_int(uint32_t i);

void debug_set_1();
void debug_set_2();
void debug_set_3();

void debug_reset_1();
void debug_reset_2();
void debug_reset_3();

void debug_toggle_1();
void debug_toggle_2();
void debug_toggle_3();
