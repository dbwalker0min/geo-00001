//
// Created by david on 10/10/2020.
//

#ifndef GEO_00001_MOTOR_H
#define GEO_00001_MOTOR_H

#include <stdbool.h>

void init_motor();
void motor_isr();
void move_to_angle(int angle);

#endif //GEO_00001_MOTOR_H
