//
// Created by david on 10/23/2020.
//

#ifndef GEO_00001_CALIBRATION_CONSTANTS_H
#define GEO_00001_CALIBRATION_CONSTANTS_H

#include <stdint-gcc.h>
void init_calibration_constants();

void save_pot_parameters(uint16_t pot_max, uint16_t pot_zero, uint16_t pot_1_turn);
void get_pot_params(uint16_t *pot_max, uint16_t *pot_zero, uint16_t *pot_1_turn);
uint16_t limit_pot(uint16_t pot_in);

#endif //GEO_00001_CALIBRATION_CONSTANTS_H
