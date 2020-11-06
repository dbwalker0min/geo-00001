//
// Created by david on 10/23/2020.
//

#ifndef GEO_00001_CALIBRATION_CONSTANTS_H
#define GEO_00001_CALIBRATION_CONSTANTS_H

#include <stdint-gcc.h>
void init_calibration_constants();

void save_pot_parameters(uint16_t pot_min, uint16_t pot_zero, uint16_t pot_1_turn);
void get_pot_params(uint16_t *pot_max, uint16_t *pot_zero, uint16_t *pot_1_turn);

void save_mag_bias(int16_t bias[3]);
int16_t* get_mag_bias();

void save_backlash(uint16_t backlash);
uint16_t get_backlash();

int angle_to_pot(int angle);

#endif //GEO_00001_CALIBRATION_CONSTANTS_H
