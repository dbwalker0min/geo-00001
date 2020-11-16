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
#include "usb_io.h"
#include "calibration_constants.h"
#include "printf.h"


#define ADC_MOTOR ADC1
#define DMA_ADC_MOTOR DMA1
#define DMA_CHANNEL_ADC_MOTOR LL_DMA_CHANNEL_1
#define F_Q31(f) (q31_t)(f*2147483648)
static struct {
  uint16_t vref;
  uint16_t pot;
  uint16_t vin;
} adc_buffer;

static uint16_t pot_min;
static uint16_t pot_1_turn;

static arm_pid_instance_q31 motor_pid;
static unsigned pot_setpoint;
static int deadband_pot = 2;

static q31_t filtered_pot;
static int16_t filtered_pot_word;
static q31_t filtered_pot2;
static int speed;
// this means the CCW backlash has been taken up
static bool cw_backlash_taken;

static enum {
  MOTOR_OFF,
  MOTOR_CONS_V_FOR_TIME,
  MOTOR_SERVO,
} motor_mode;

static int voltage;
static unsigned time;

static inline int iabs(int x)
{
  return x < 0 ? -x : x;
}

static const uint16_t mech_pot_min = 4096*5/10;
static const uint16_t mech_pot_max = 4096*7/10;

// minimum input voltage in mV
static float vin_min = 12.0f;
static bool unlocked = false;

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

bool nudge_by_pot(unsigned int pot);
void move_to_pot(unsigned int pos);


#pragma GCC push_options
#pragma GCC optimize ("O0")
// find the pot that corresponds to the angle in [-deg_overlap, 360]
int angle_to_pot(int angle) {
  uint16_t p_min = get_pot_min();
  uint16_t p_1_turn = get_pot_1_turn();

  unsigned pot = p_1_turn - ((p_1_turn - p_min) * angle) / 360;
  return (int)pot;
}

void move_to_angle(int angle) {
  // angle is in [-180, 180)
  if (angle < 0)
    angle += 360;
  // angle is now in [0, 360)

  // sometimes, there are two positions that are valid because the setpoint is [-deg_overlap, 360]
  if (angle >= 360 - deg_overlap) {
    // find the angle that minimizes the movement. Here, angle is in [360-deg_overlap, 360)
    int current_angle = angle_now();    // current_angle in [-deg_overlap, 360]
    int move1 = iabs(angle - current_angle);  // move1: [-deg_overlap, 360 + deg_overlap]
    int move2 = iabs(angle - 360 - current_angle);  // move2: [-deg_overlap - 360, deg_overlap]

    SEGGER_RTT_printf(0, "move1 %d, move2 %d" CRLF, move1, move2);
    if (move2 < move1)
      angle -= 360;
  }

  int pot = angle_to_pot(angle);
  SEGGER_RTT_printf(0, "Angle %d, pot %d" CRLF, angle, pot);
  move_to_pot(pot);
}
#pragma GCC pop_options

static const char nudge_prompt[] =
    "Mechanism setup mode" CRLF
    "a = CCW  1 deg; s = CW  1 deg" CRLF
    "A = CCW 25 deg; S = CW 25 deg" CRLF
    "< = CCW 90 deg; > = CW 90 deg" CRLF
    "u - unlock rotation limits (dangerous!)" CRLF
    "L - set CW rotation limit (index position)" CRLF
    "l - go to CCW rotation limit" CRLF
    "T - set 360 deg CCW from limit" CRLF
    "t - go to 360 deg from limit" CRLF
    "? - print settings (pot units)" CRLF
    "b - set mechanism backlash" CRLF
    "h Print this help" CRLF
    "q quit" CRLF;

BaseType_t command_motor(char *wbuf, size_t buf_len, const char *cmd) {
  char ch;
  bool done = false;
  bool dirty = false;

  get_pot_params(&pot_min, &pot_1_turn);

  // it is where it is
  pot_setpoint = filtered_pot_word;
  static char printf_buf[80];

  write_usb(nudge_prompt, sizeof(nudge_prompt));

  while (!done) {
    ch = get_usb_char();
    switch (ch) {
    case 'a' :
      nudge_by_pot(pot_setpoint + 2);
      break;
    case 's' :
      nudge_by_pot(pot_setpoint - 2);
      break;
    case 'A':
      nudge_by_pot(pot_setpoint + (int)POT_FROM_ANGLE(25));
      break;
    case 'S':
      nudge_by_pot(pot_setpoint - (int)POT_FROM_ANGLE(25));
      break;
    case ',':
    case '<':
      nudge_by_pot(pot_setpoint + (int)POT_FROM_ANGLE(90));
      break;
    case '.':
    case '>':
      nudge_by_pot(pot_setpoint - (int)POT_FROM_ANGLE(90));
      break;
    case 'h' :
      print_usb_string(nudge_prompt);
      break;
    case 'u':
      unlocked = true;
      print_usb_string("Mechanism unlocked!" CRLF);
      dirty = true;
      break;
    case 'b':
      {
        unsigned backlash = get_backlash();
        snprintf(wbuf, buf_len, "Enter backlash (0-9) pot units (current value = %d)" CRLF, backlash);
        print_usb_string(wbuf);

        char bch = get_usb_char();
        if (bch >= '0' && bch <= '9')
          backlash = bch - '0';
        else if (bch >= 'A' && bch <= 'F')
          backlash = bch - 'A' + 10;
        else if (bch >= 'a' && bch <= 'f')
          backlash = bch - 'a' + 10;

        if (backlash != get_backlash()) {
          save_backlash(backlash);
          snprintf(wbuf, buf_len, CRLF "new value is %d" CRLF, backlash);
          print_usb_string(wbuf);
        } else {
          print_usb_string(CRLF "backlash unchanged" CRLF);
        }
      }
      break;
    case 'L':
      // if I change the setpoint, I should change the 1 Turn value as well
      // consider the backlash in setting the limit
      {
        unsigned limit = pot_setpoint;
        if (cw_backlash_taken)
          limit += get_backlash();

        if (limit != pot_min) {
          pot_min = limit;
          dirty = true;
          print_usb_string("Limit set" CRLF);
        }
        unlocked = false;
      }
      break;
    case 'l':
      // this is a special case because the backlash isn't ever a factor
      move_to_pot(pot_min);
      break;
    case 'T': {
      unsigned limit = pot_setpoint;

      // take care of backlash
      if (cw_backlash_taken)
        limit += get_backlash();

      if (pot_1_turn != limit) {
        pot_1_turn = limit;
        dirty = true;
        print_usb_string("1 Turn value set" CRLF);
      }
      unlocked = false;
    }
    break;
    case 't':
      move_to_pot(pot_1_turn);
      break;
    case '?':
      snprintf(wbuf, buf_len,
               "pot limit:   %d Ohms" CRLF
               "pot 1 turn:  %d Ohms" CRLF
               "current pot: %d Ohms" CRLF
               "backlash:    %d" CRLF,
               10000*pot_min/4096, 10000*pot_1_turn/4096, 10000*pot_setpoint/4096, get_backlash());
      print_usb_string(wbuf);
      break;
    case 'q':
      done = true;
      unlocked = false;
      if (dirty) {
        print_usb_string("Changes made. Save changes?");
        char chr;
        do {
          chr = get_usb_char();
        } while (chr != 'Y' && chr != 'y' && chr != 'N' && chr != 'n');
        if (chr == 'Y' || chr == 'y') {
            print_usb_string(CRLF "Saving movement parameters" CRLF);
          save_pot_parameters(pot_min, pot_1_turn);
        }
      }
    default:
      motor_mode = MOTOR_OFF;
      pot_setpoint = filtered_pot_word;
      break;
    }
  }
  strncpy(wbuf, CRLF "Nudge mode complete" CRLF, buf_len);
  return pdFALSE;
}

CLI_Command_Definition_t cmd_motor = {
    .pxCommandInterpreter = command_motor,
    .pcCommand = "motor",
    .pcHelpString = "motor" CRLF " Setup the motion parameters" CRLF CRLF,
    .cExpectedNumberOfParameters = 0,
};

// from the current pot setpoint, where is the mechanism now in a range of [-deg_overlap, 360]
int angle_now() {
  // remove the backlash
  unsigned pos = filtered_pot_word + (cw_backlash_taken ? get_backlash() : 0);

  return ((int)(get_pot_1_turn() - pos)*360)/(get_pot_1_turn() - get_pot_min());
}

// Go to the absolute pot position considering backlash. The position is relative to
// the pot position if moving in the CCW direction (all CCW backlash taken up)
void move_to_pot(unsigned int pos) {
  unsigned backlash = get_backlash();
  unsigned pot = pos;

  // this is the current position of the mechanism considering backlash
  unsigned p0 = cw_backlash_taken ? pot_setpoint - backlash : pot_setpoint;
  if (pos == p0) return;

  if (pos > p0) {
    // this is a CW motion so the pot must be offset by the backlash
    pot += get_backlash();      // move extra for the backlash
    cw_backlash_taken = true;   // the move will take up the cw backlash
  } else {
    // turn CCW
    cw_backlash_taken = false;  // it is the ccw backlash that will be taken up
  }
  pot_setpoint = pot;
  motor_mode = MOTOR_SERVO;
}

// this makes a relative move of the mechanism
bool nudge_by_pot(unsigned int pot) {
  unsigned int tmp;
  bool limited = false;

  // in all of the nudges, I want to take out the backlash if the
  // motor changes direction
  if (pot > pot_setpoint && cw_backlash_taken)
    pot += get_backlash();
  else if (pot < pot_setpoint && !cw_backlash_taken)
    pot -= get_backlash();

  if (!unlocked) {
    if (pot > mech_pot_max)
      tmp = mech_pot_max;
    else if (pot < mech_pot_min)
      tmp = mech_pot_min;
    else
      tmp = pot;
    limited = tmp != pot;
  } else
    tmp = pot;

  if (tmp != pot_setpoint) {
    cw_backlash_taken = tmp < pot_setpoint;
    pot_setpoint = tmp;
    motor_mode = MOTOR_SERVO;
  }
  if (limited)
    print_usb_string("Motion limited. To move beyond the limits, unlock." CRLF);

  return limited;
}

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

void init_motor() {
  LL_ADC_DisableDeepPowerDown(ADC_MOTOR);

  // wait 20us for ADC power on
  for (int i=0; i<16*20; i++)
    __NOP();

  LL_DMA_SetPeriphAddress(DMA_ADC_MOTOR, DMA_CHANNEL_ADC_MOTOR, LL_ADC_DMA_GetRegAddr(ADC_MOTOR, LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_SetMemoryAddress(DMA_ADC_MOTOR, DMA_CHANNEL_ADC_MOTOR, (uint32_t)&adc_buffer);
  LL_DMA_SetDataLength(DMA_ADC_MOTOR, DMA_CHANNEL_ADC_MOTOR, 3);
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
  motor_pid.Kp = 400u << 21u;   // gain is in units of mV/pot LSB in Q32.16
  motor_pid.Kd = 0;
  motor_pid.Ki = 0;

  arm_pid_init_q31(&motor_pid, 1);

  FreeRTOS_CLIRegisterCommand(&cmd_motor);

  get_pot_params(&pot_min, &pot_1_turn);
}


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
    int error = pot_setpoint - filtered_pot_word;
    if (iabs(error) < deadband_pot)
      motor_mode = MOTOR_OFF;
    else {
      voltage = arm_pid_q31(&motor_pid, error*1024);
      voltage += error > 0 ? 1200 : -1200;        // use a minimum value for the motor voltage
      if (voltage > 12000)
        voltage = 12000;
      else if (voltage < -12000)
        voltage = -12000;
      set_motor_v(voltage);
    }
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
