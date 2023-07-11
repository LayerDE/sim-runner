#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
    float rad2deg(float in);
    float deg2rad(float in);
    int sign(float in);
    int calc_average(const int *x, int cnt);
    bool isNear(float a,float b, float range);
#ifdef __cplusplus
}
#endif