//
// Created by david on 10/10/2020.
//

#include <cmsis_os2.h>
#include <stm32l4xx_ll_dma.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <stdbool.h>
#include <memory.h>
#include "motor.h"
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_tim.h"
#include "SEGGER_RTT.h"
#include "arm_math.h"

#define ADC_MOTOR ADC1
#define DMA_ADC_MOTOR DMA1
#define DMA_CHANNEL_ADC_MOTOR LL_DMA_CHANNEL_1
#define F_Q31(f) (q31_t)(f*2147483648)
static struct {
  uint16_t imot;
  uint16_t vin;
  uint16_t pot;
  uint16_t vref;
} adc_buffer;

static enum {
  MOTOR_OFF,
  MOTOR_CONS_V_FOR_TIME,
  MOTOR_SERVO,
} motor_mode;

static int voltage;
static unsigned time;

// this is half of a 100 element Nuttall filter
static const uint32_t coeffs[] = {
    43289, 52621, 81422, 132113, 208721, 316864, 463734, 658056, 910046, 1231332, 1634862, 2134777, 2746251, 3485295,
    4368522, 5412874, 6635303, 8052426, 9680136, 11533195, 13624799, 15966134, 18565934, 21430047, 24561018, 27957714,
    31614994, 35523433, 39669124, 44033558, 48593589, 53321506, 58185191, 63148396, 68171105, 73210009, 78219064,
    83150135, 87953710, 92579675, 96978124, 101100205, 104898957, 108330146, 111353065, 113931276, 116033296,
    117633183, 118711034, 119253364,
};

// minimum input voltage in mV
static float vin_min = 12.0f;

#define VREFINT (*VREFINT_CAL_ADDR)
#define CRLF "\r\n"

// hold the state of the filter
static q31_t pot_filter_state[4];

// note that the filter coefficients are scaled by two so it
// will need to be shifted by 1 on the output
// The filter was computed with scipy:
// b, a = signal.iirdesign(50, 1000, 1, 40, fs=10000)
// a=[ 1.         -1.96521911  0.96629417]
// b=[ 0.01130592 -0.02153679  0.01130592]
q31_t pot_filter_coeffs[] = {
    F_Q31(0.01130592/2),      // b[0]
    F_Q31(-0.02153679/2),     // b[1]
    F_Q31(0.01130592/2),      // b[2]
    F_Q31(1.96521911/2),     // a[1]
    F_Q31(-0.96629417/2),      // a[2]
};
static const arm_biquad_casd_df1_inst_q31 pot_filter = {
    .numStages = 1,
    .pState = pot_filter_state,
    .pCoeffs = pot_filter_coeffs,
    .postShift = 1,
};

static arm_pid_instance_q31 motor_pid;
static unsigned pot_setpoint;

// A command to turn on the motor at a constant voltage for a time
BaseType_t command_vmot(char *wbuf, size_t buf_len, const char*cmd) {
  BaseType_t len;
  int temp_v = atoi(FreeRTOS_CLIGetParameter(cmd, 1, &len));
  int temp_t = atoi(FreeRTOS_CLIGetParameter(cmd, 2, &len));

  // set them all atomically
  __disable_irq();
  time = temp_t;
  voltage = temp_v;
  motor_mode = MOTOR_CONS_V_FOR_TIME;
  __enable_irq();

  strncpy(wbuf, "Done", buf_len);
  return pdFALSE;
}

CLI_Command_Definition_t cmd_vmot_def = {
    .pxCommandInterpreter = command_vmot,
    .pcCommand = "vmot",
    .pcHelpString = "vmot <mV> <nticks>:\r\n Turn the motor at a fixed voltage\r\n\r\n",
    .cExpectedNumberOfParameters = 2,
};

BaseType_t command_servo(char *wbuf, size_t buf_len, const char*cmd) {
  BaseType_t len;
  unsigned pot = atoi(FreeRTOS_CLIGetParameter(cmd, 1, &len));
  if (pot > 4000 || pot < 100) {
    strncpy(wbuf, "Argument out of range" CRLF, buf_len);
  } else {
    pot_setpoint = pot;
    motor_mode = MOTOR_SERVO;
    strncpy(wbuf, "Done" CRLF, buf_len);
  }
  return pdFALSE;
}

CLI_Command_Definition_t cmd_servo_def = {
    .pxCommandInterpreter = command_servo,
    .pcCommand = "servo",
    .pcHelpString = "servo <pot>" CRLF " Servo the motor to the specified pot position" CRLF CRLF,
    .cExpectedNumberOfParameters = 1,
};

BaseType_t command_stop(char *wbuf, size_t buf_len, const char*cmd) {
  motor_mode = MOTOR_OFF;
  return pdFALSE;
}

CLI_Command_Definition_t cmd_motor_stop = {
    .pxCommandInterpreter = command_stop,
    .pcCommand = "servo",
    .pcHelpString = "stop" CRLF " Immediately stop the motor" CRLF CRLF,
    .cExpectedNumberOfParameters = 0,
};

// return true if input voltage greater than specified vin.
static bool check_vin() {
  static unsigned factor;
  if (factor == 0)
    factor = ((unsigned)(491.2/16.2*3.0/vin_min*4096)*VREFINT) >> 12u;
  return factor*adc_buffer.vin > (adc_buffer.vref << 12u);
}

static void set_motor_v(int voltage_mv) {
  static int factor;
  if (factor == 0)
    factor = (int)(((unsigned)(491.2*3000)*VREFINT)/((unsigned)(16.2*800)));
  int tmp = (voltage_mv*adc_buffer.vref)/((factor*adc_buffer.vin)/4096);
  if (tmp >= 0) {
    if (tmp > 800) tmp = 799;
    LL_TIM_OC_SetCompareCH2(TIM1, 0);
    LL_TIM_OC_SetCompareCH1(TIM1, tmp);
  } else {
    if (tmp < -800) tmp = -799;
    LL_TIM_OC_SetCompareCH1(TIM1, 0);
    LL_TIM_OC_SetCompareCH2(TIM1, -tmp);
  }
}

static uint8_t rtt_buffer[2048];

void init_motor() {

  LL_ADC_DisableDeepPowerDown(ADC_MOTOR);

  SEGGER_RTT_ConfigUpBuffer(1, "JScope_u2", rtt_buffer, 2048, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  // wait 20us for ADC power on
  for (int i=0; i<16*20; i++)
    __NOP();

  LL_DMA_SetPeriphAddress(DMA_ADC_MOTOR, DMA_CHANNEL_ADC_MOTOR, LL_ADC_DMA_GetRegAddr(ADC_MOTOR, LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_SetMemoryAddress(DMA_ADC_MOTOR, DMA_CHANNEL_ADC_MOTOR, (uint32_t)&adc_buffer);
  LL_DMA_SetDataLength(DMA_ADC_MOTOR, DMA_CHANNEL_ADC_MOTOR, 4);
  LL_DMA_EnableChannel(DMA_ADC_MOTOR, DMA_CHANNEL_ADC_MOTOR);

  // I need to enable the PWM outputs
  LL_TIM_CC_EnableChannel(
      TIM1,
      LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
      LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
      LL_TIM_CHANNEL_CH3);

  LL_ADC_EnableIT_EOS(ADC_MOTOR);

  // go ahead and run the timer. The intial values of the CC registers are zero so the motor will be off
  LL_TIM_EnableCounter(TIM1);

  // perform a calibration
  LL_ADC_StartCalibration(ADC_MOTOR, LL_ADC_SINGLE_ENDED);
  while(LL_ADC_IsCalibrationOnGoing(ADC_MOTOR));

  // enable the ADC and start conversion
  LL_ADC_Enable(ADC_MOTOR);
  while(!LL_ADC_IsEnabled(ADC_MOTOR));

  LL_ADC_REG_StartConversion(ADC_MOTOR);

  // set the motor PID gains
  motor_pid.Kp = 0;   // gain is in units of mV/pot LSB in Q32.16
  motor_pid.Kd = 0;
  motor_pid.Ki = 0;

  FreeRTOS_CLIRegisterCommand(&cmd_vmot_def);
  FreeRTOS_CLIRegisterCommand(&cmd_servo_def);
  FreeRTOS_CLIRegisterCommand(&cmd_motor_stop);

}

q31_t filtered_pot;
int16_t filtered_pot_word;
q31_t filtered_pot2;
int speed;

void motor_isr()
{
  static int count = 0;
  GPIOH->BSRR |= GPIO_BSRR_BS3;

  // disable and calibrate the ADC after the conversion
  LL_ADC_Disable(ADC_MOTOR);

  q31_t src = ((int)adc_buffer.pot)*65536;
  arm_biquad_cascade_df1_fast_q31(&pot_filter, &src, &filtered_pot, 1);
  filtered_pot_word = filtered_pot/65536;

  if (++count == 500) {
    speed = filtered_pot2 - filtered_pot;
    filtered_pot2 = filtered_pot;
    count = 0;
  }

  if (!check_vin()) motor_mode = MOTOR_OFF;

  // SEGGER_RTT_Write(1, &adc_buffer.pot, 2);

  switch (motor_mode) {
  case MOTOR_OFF:
    LL_TIM_OC_SetCompareCH1(TIM1, 0);
    LL_TIM_OC_SetCompareCH2(TIM1, 0);
    break;
  case MOTOR_CONS_V_FOR_TIME:
    set_motor_v(voltage);
    if (--time == 0)
      motor_mode = MOTOR_OFF;
    break;
  case MOTOR_SERVO:
  {
    unsigned error = pot_setpoint - filtered_pot_word;
    q31_t control = arm_pid_q31(&motor_pid, error);
    if (control > 12000)
      control = 12000;
    else if (control < -12000)
      control = -12000;
    set_motor_v(control);
  }
  default:
    break;
  }

  while(LL_ADC_IsDisableOngoing(ADC_MOTOR));

  LL_ADC_REG_StopConversion(ADC_MOTOR);
  // SEGGER_RTT_Write(1, &adc_buffer.pot, 2);

  while(LL_ADC_REG_IsStopConversionOngoing(ADC_MOTOR));

  LL_ADC_StartCalibration(ADC_MOTOR, LL_ADC_SINGLE_ENDED);

  // should do stuff here while the calibration is occurring
  while(LL_ADC_IsCalibrationOnGoing(ADC_MOTOR));

  LL_ADC_Enable(ADC_MOTOR);
  while(!LL_ADC_IsEnabled(ADC_MOTOR));

  LL_ADC_REG_StartConversion(ADC_MOTOR);

  GPIOH->BSRR |= GPIO_BSRR_BR3;

}
