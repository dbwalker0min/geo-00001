//
// Created by david on 10/10/2020.
//

#include "motor.h"
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_tim.h"

#define ADC_MOTOR ADC2

void init_motor() {
  LL_ADC_DisableDeepPowerDown(ADC_MOTOR);

  // I need to enable the PWM outputs

}

static uint16_t imotor;

void motor_isr()
{
  // the ADC on the motor current is complete. Calibrate the ADC after the fact
  imotor = LL_ADC_REG_ReadConversionData12(ADC_MOTOR);

  // disable and calibrate the ADC after the conversion
  LL_ADC_Disable(ADC_MOTOR);
  LL_ADC_StartCalibration(ADC_MOTOR, LL_ADC_SINGLE_ENDED);

  // should do stuff here while the calibration is occurring
  while(LL_ADC_IsCalibrationOnGoing(ADC_MOTOR));
  LL_ADC_Enable(ADC_MOTOR);
}