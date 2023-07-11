#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "defines.h"
#include "config.h"

// bobbycar
int sign(float in){
  if(in > 0.0f)
    return 1;
   else if(in < 0.0f)
    return -1;
   else
    return 0;
}

float rad2deg(float in){
  return in * 45.0 / atan(1);
}
float deg2rad(float in){
  return in * atan(1) / 45.0;
}

int calc_average(const int *x, int cnt){
  int sum = 0;
  for(int y = 0; y < cnt; y++)
    sum+=x[y];
  return sum/cnt;
}

bool isNear(float a,float b, float range){
    if(ABS(a-b)<range)
        return true;
    else
        return false;
}
