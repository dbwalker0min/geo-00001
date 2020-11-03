//
// Created by david on 10/10/2020.
//

#include <cmsis_os2.h>
#include <SEGGER_RTT.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <memory.h>
#include <printf.h>
#include <arm_math.h>
#include <stdbool.h>
#include <motor.h>
#include <calibration_constants.h>
#include "compass.h"
#include "i2c.h"
#include "stm32l4xx_ll_gpio.h"
#include "mag_cal.h"

#define LIS3_I2C_ADDR 0x38
#define CRLF "\r\n"

static __NO_RETURN void task_compass(void* arg);

const osThreadAttr_t compass_task_attr = {
    .name = "compass",
};

/* Coefficients for a two-stage biquad with the following design:
 *    filter = sig.iirfilter(4, 0.25, fs=10, ftype='bessel', btype='lowpass', output='sos')
 * This is a 4th order Bessel filter with bandwidth of 1Hz
 */
#define num_stages 2
static const float coeffs[] = {
    3.0098335934859017e-05f,
    6.0196671869718034e-05f,
    3.0098335934859017e-05f,
    1.7326299977616086f,
    -0.7518816491241208f,
    1.0f,
    2.0f,
    1.0f,
    1.7886595314713387f,
    -0.8136741842868104f,
};

static float32_t state_mag_x[num_stages*4];
static float32_t state_mag_y[num_stages*4];

static float filtered_mag_x;
static float filtered_mag_y;
static bool pause_loop;
float hdg;

// there are two instances of the biquad
static const arm_biquad_casd_df1_inst_f32 mag_x_filterdef = {
    .numStages = num_stages,
    .pState = state_mag_x,
    .pCoeffs = coeffs,
};
static const arm_biquad_casd_df1_inst_f32 mag_y_filterdef = {
    .numStages = num_stages,
    .pState = state_mag_y,
    .pCoeffs = coeffs,
};

static BaseType_t command_magcal(char *wbuf, size_t buf_len, const char *cmd);

CLI_Command_Definition_t cmd_mcal = {
    .pxCommandInterpreter = command_magcal,
    .pcCommand = "mcal",
    .pcHelpString = "mcal 0|1|on|off|show|cancel" CRLF " Start, stop, cancel, or show a magnetic calibration" CRLF CRLF,
    .cExpectedNumberOfParameters = 1,
};

void on_host_port_opened() {
  pause_loop = true;
}

void on_host_port_closed() {
  pause_loop = false;
}

void init_compass()
{
  osThreadNew(&task_compass, 0, &compass_task_attr);
  FreeRTOS_CLIRegisterCommand(&cmd_mcal);
}

static int compass_reset()
{
  return i2c_w(LIS3_I2C_ADDR, "\x21\x04", 2);
}

static int compass_config()
{
  compass_reset();
  osDelay(2);

  i2c_w(LIS3_I2C_ADDR, "\x21\x00", 2);    // CTRL_REG2 : FS 4 Gauss
  i2c_w(LIS3_I2C_ADDR, "\x20\x70", 2);    // CTRL_REG1 : ODR 10Hz,XY UHP
  i2c_w(LIS3_I2C_ADDR, "\x23\x0C", 2);    // CTRL_REG4 : UHP mode Z axis
  i2c_w(LIS3_I2C_ADDR, "\x24\x40", 2);    // CLRL_REG5 : BDU on
  i2c_w(LIS3_I2C_ADDR, "\x22\x00", 2);    // CTRL_REG3 : Continuous conversions
}

static int16_t compass_data[3];

__NO_RETURN void task_compass(void* arg)
{
  unsigned startup_delay = 100;

  // access the WHOAMI register (0x0F)
  uint8_t buf;

  i2c_wr(LIS3_I2C_ADDR, "\x0F", 1, &buf, 1);
  // SEGGER_RTT_printf(0, "Who am I = %d\n", buf);

  // reconfigure compass for proper operation
  compass_config();

  // wait 1 second before running

  // perform calibration
  while (1) {
    osDelay(1);
    if (LL_GPIO_ReadInputPort(GPIOA) & (1u<<6u)) {
      i2c_wr(LIS3_I2C_ADDR, "\xA8", 1, &compass_data, 6);
      mag_cal_apply(compass_data);
      float x = (float)compass_data[0];
      float y = (float)compass_data[1];
      arm_biquad_cascade_df1_f32(&mag_x_filterdef, &x, &filtered_mag_x, 1);
      arm_biquad_cascade_df1_f32(&mag_y_filterdef, &y, &filtered_mag_y, 1);

      // compute the heading in degrees
      hdg = atan2f(filtered_mag_y, -filtered_mag_x)*57.29577951f - 10.f;
      if (!pause_loop && startup_delay == 0)
        move_to_angle(-(int)hdg);
    } else {
      osDelay(1);
    }
    if (startup_delay) startup_delay--;
  }
}

BaseType_t command_magcal(char *wbuf, size_t buf_len, const char *cmd) {
  BaseType_t len;
  const char* param = FreeRTOS_CLIGetParameter(cmd, 1, &len);
  if (param == NULL) {
    param = mag_cal_state() ? "0" : "1";
  }

  if (strcmp(param, "1") == 0 || strcmp(param, "on") == 0) {
    strncpy(wbuf, "Calibration started" CRLF, buf_len);
    mag_cal_start();
  } else if (strcmp(param, "0") == 0 || strcmp(param, "off") == 0) {
    int16_t offsets[3];

    mag_cal_stop(offsets);
    snprintf(wbuf, buf_len, "Calibration complete" CRLF "offset = [%d, %d, %d]" CRLF,
             offsets[0], offsets[1], offsets[2]);
  } else if (strcmp(param, "show") == 0) {
    int16_t *cal = get_mag_bias();
    snprintf(wbuf, buf_len, "bias = [%d, %d, %d]" CRLF, cal[0], cal[1], cal[2]);
  } else if (strcmp(param, "cancel") == 0) {
    mag_cal_cancel();
    strncpy(wbuf, "Calibration canceled" CRLF, buf_len);
  } else {
    strncpy(wbuf, "Unrecoginzed argument must be 0|1|on|off" CRLF, buf_len);
  }
  return pdFALSE;
}

void compass_isr()
{
}