//
// Created by david on 10/10/2020.
//

#include <cmsis_os2.h>
#include <SEGGER_RTT.h>
#include "compass.h"
#include "i2c.h"

#define LIS_3_I2C_ADDR 0x38

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
  return i2c_w(LIS_3_I2C_ADDR, "\x21\x04", 2);
}

int16_t buf[3];

static int compass_self_test()
{
  compass_reset();
  i2c_w(LIS_3_I2C_ADDR, "\x21\x40", 2); // full scale is 12 Gauss for self test
  i2c_w(LIS_3_I2C_ADDR, "\x20\x19", 2); // turn on self test
  i2c_w(LIS_3_I2C_ADDR, "\x22\x03", 2); // Single conversion mode
  i2c_wr(LIS_3_I2C_ADDR, "\xA8", 1, buf, 6);

  osDelay(10);    // wait 100ms for conversion complete
  i2c_wr(LIS_3_I2C_ADDR, "\xA8", 1, buf, 6);
}

__NO_RETURN void task_compass(void* arg)
{
  // access the WHOAMI register (0x0F)
  uint8_t buf;

  i2c_wr(LIS_3_I2C_ADDR, "\x0F", 1, &buf, 1);
  SEGGER_RTT_printf(0, "Who am I = %d\n", buf);

  compass_self_test();

  uint8_t config_buf[8];
  i2c_wr(LIS_3_I2C_ADDR, "\xA0", 1, config_buf, 8);

  // perform calibration
  while (1) {
    osDelay(1);
  }
}