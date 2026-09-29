#pragma once

inline float custom_fminf(float a, float b) {
    return (a < b) ? a : b;
}

inline float custom_fmaxf(float a, float b) {
    return (a > b) ? a : b;
}

// inline int float_to_uint(float x, float x_min, float x_max, int bits) {
//     float span = x_max - x_min;
//     float offset = x_min;

//     if (x < x_min) x = x_min;
//     if (x > x_max) x = x_max;
//     return (int)((x - offset) * ((1 << bits) - 1) / span);
// }

// inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
// }