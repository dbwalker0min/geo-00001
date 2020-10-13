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
  LL_TIM_CC_EnableChannel(
      TIM1,
      LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
      LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
      LL_TIM_CHANNEL_CH3);

  LL_ADC_EnableIT_EOS(ADC_MOTOR);

  // go ahead and run the timer. The intial values of the CC registers are zero so the motor will be off
  LL_TIM_EnableCounter(TIM1);
  LL_ADC_Enable(ADC_MOTOR);
  while(!LL_ADC_IsEnabled(ADC_MOTOR));

  LL_ADC_REG_StartConversion(ADC_MOTOR);
}

static uint16_t imotor;

void motor_isr()
{
  GPIOH->BSRR |= GPIO_BSRR_BS3;

  // the ADC on the motor current is complete. Calibrate the ADC after the fact
  imotor = LL_ADC_REG_ReadConversionData12(ADC_MOTOR);

  // disable and calibrate the ADC after the conversion
  LL_ADC_Disable(ADC_MOTOR);
  LL_ADC_StartCalibration(ADC_MOTOR, LL_ADC_SINGLE_ENDED);

  // should do stuff here while the calibration is occurring
  while(LL_ADC_IsCalibrationOnGoing(ADC_MOTOR));

  LL_ADC_Enable(ADC_MOTOR);
  while(!LL_ADC_IsEnabled(ADC_MOTOR));

  LL_ADC_REG_StartConversion(ADC_MOTOR);

  GPIOH->BSRR |= GPIO_BSRR_BR3;

}