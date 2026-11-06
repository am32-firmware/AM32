#pragma once
#define INTERVAL_TIMER_COUNT test_count()
#define SET_INTERVAL_TIMER_COUNT(v) (timer_count = (v))
#define SET_AND_ENABLE_COM_INT(v) do { timer_delay = (v); timer_enabled = true; } while (0)
#define DISABLE_COM_TIMER_INT() (timer_enabled = false)
#define COM_TIMER_CLEAR_PENDING() ((void)0)
#define SET_DUTY_CYCLE_ALL(v) ((void)(v))
