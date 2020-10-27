//
// Created by david on 10/10/2020.
//

#include <cmsis_os2.h>
#include <SEGGER_RTT.h>
#include "compass.h"
#include "i2c.h"
#include "stm32l4xx_ll_gpio.h"

#define LIS3_I2C_ADDR 0x38

static __NO_RETURN void task_compass(void* arg);

const osThreadAttr_t compass_task_attr = {
    .name = "compass",
};

void init_compass()
{
  osThreadNew(&task_compass, 0, &compass_task_attr);
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
  i2c_w(LIS3_I2C_ADDR, "\x20\x68", 2);    // CTRL_REG1 : ODR 2.5Hz,XY UHP
  i2c_w(LIS3_I2C_ADDR, "\x23\x0C", 2);    // CTRL_REG4 : UHP mode Z axis
  i2c_w(LIS3_I2C_ADDR, "\x24\x40", 2);    // CLRL_REG5 : BDU on
  i2c_w(LIS3_I2C_ADDR, "\x22\x00", 2);    // CTRL_REG3 : Continuous conversions
}

static int16_t compass_data[3];

__NO_RETURN void task_compass(void* arg)
{
  // access the WHOAMI register (0x0F)
  uint8_t buf;

  i2c_wr(LIS3_I2C_ADDR, "\x0F", 1, &buf, 1);
  SEGGER_RTT_printf(0, "Who am I = %d\n", buf);

  // reconfigure compass for proper operation
  compass_config();

  // perform calibration
  while (1) {
    osDelay(1);
    if (LL_GPIO_ReadInputPort(GPIOA) & (1u<<6u)) {
      i2c_wr(LIS3_I2C_ADDR, "\xA8", 1, &compass_data, 6);
      SEGGER_RTT_printf(0, "m:%d,%d,%d\n", compass_data[0], compass_data[1], compass_data[2]);
    } else {
      osDelay(1);
    }
  }
}

void compass_isr()
{
}