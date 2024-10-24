#pragma once

#include <stdbool.h>

void vreg5V_initialize();
void vreg5V_enable();
void vreg5V_disable();
bool vreg5V_pgood();
void vreg5V_set_frequency();
