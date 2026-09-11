/*
  a 2 pole filter applied to RawCommand to provide a configurable
  acceleration filter
 */
#pragma once

float Filter2P_apply(const float sample, float cutoff_freq, float sample_rate);
