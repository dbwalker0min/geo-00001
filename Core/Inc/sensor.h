//
// Created by david on 10/10/2020.
//

#ifndef GEO_00001_SENSOR_H
#define GEO_00001_SENSOR_H

#define ADC_SENSOR ADC1

void sensor_isr();
void init_sensor();

unsigned duty_cycle(unsigned voltage_mv);

#endif //GEO_00001_SENSOR_H
