//
// Created by david on 10/16/2020.
//

#include <stdlib.h>
#include <memory.h>
#include "led.h"
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "stm32l4xx_ll_gpio.h"

// A command to turn on the motor at a constant voltage for a time
BaseType_t command_led(char *wbuf, size_t buf_len, const char*cmd) {
  BaseType_t len;
  const char *param = FreeRTOS_CLIGetParameter(cmd, 1, &len);
  enum {
    LED_OFF,
    LED_RED,
    LED_GRN,
    LED_BOTH,
  } led = LED_OFF;

  if (len)
    switch (*param) {
  case 'b':
  case 'B':
    led = LED_BOTH;
    break;
  case 'g':
  case 'G':
    led = LED_GRN;
    break;
  case 'r':
  case 'R':
    led = LED_RED;
  default:
    break;
  }

  unsigned bsrr = 0;
  if (led & LED_RED)
    bsrr |= GPIO_BSRR_BS4;
  else
    bsrr |= GPIO_BSRR_BR4;

  if (led & LED_GRN)
    bsrr |= GPIO_BSRR_BS1;
  else
    bsrr |= GPIO_BSRR_BR1;

  GPIOB->BSRR = bsrr;

  strncpy(wbuf, "Done", buf_len);
  return pdFALSE;
}

static const CLI_Command_Definition_t led_cmd_def = {
    .pcCommand = "led",
    .pxCommandInterpreter = &command_led,
    .pcHelpString = "led Red|Green|Both\r\n Turn the LED on or off\r\n\r\n",
    .cExpectedNumberOfParameters = 1,
};

void init_led() {
  FreeRTOS_CLIRegisterCommand(&led_cmd_def);
}