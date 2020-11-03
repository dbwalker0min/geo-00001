//
// Created by david on 10/30/2020.
//

#ifndef GEO_00001_MAG_CAL_H
#define GEO_00001_MAG_CAL_H

#include <stdint-gcc.h>

void mag_cal_start();
void mag_cal_stop(int16_t offset[3]);
void mag_cal_apply(int16_t d[3]);
bool mag_cal_state();
void mag_cal_cancel();

#endif //GEO_00001_MAG_CAL_H
