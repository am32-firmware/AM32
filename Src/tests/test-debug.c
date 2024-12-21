// This program tests the functions in
// "debug.h"

#include "debug.h"
#include "mcu.h"

int main()
{
    mcu_setup(250);
    debug_initialize();
    while (1)
    {
        debug_write_string("hello world\n");
        debug_write_int(12345678);
    }
}