//
// Created by david on 10/10/2020.
//
#include <FreeRTOS.h>
#include <stm32l4xx_ll_adc.h>
#include <stm32l4xx_ll_dma.h>
#include <stdbool.h>
#include "sensor.h"
#include "stdint.h"
#include "cmsis_os2.h"

#define ADC_SENSOR ADC1
#define DMA_SENSOR DMA1
#define DMA_CHAN_SENSOR LL_DMA_CHANNEL_1

#define VREFINT (*(uint16_t*)0x1FFF75AA)

static __NO_RETURN void task_sensor(void* arg);

static unsigned vref_comp_const1;
static unsigned vref_comp_const2;
static const unsigned duty_cycle_factor_numerator = (unsigned)(16.2*4096*800/491.2/3) << 6u;
static unsigned duty_cycle_factor;

static const osThreadAttr_t sensor_task_attr = {
    .name = "sensor",
};

static const osEventFlagsAttr_t event_attr = {
    .name = "sensor",
};

// buffer to hold the ADC results
static struct {
  uint16_t pot;
  uint16_t vin;
  uint16_t vref;
} adc_data_buffer;

static osEventFlagsId_t conv_complete;

void sensor_isr()
{
  osEventFlagsSet(conv_complete, 1);
}

void init_sensor()
{
  static uint8_t cb[sizeof(StaticEventGroup_t)];

  // create the task for reading the sensor
  osThreadNew(&task_sensor, 0, &sensor_task_attr);

  // create the event flag for the event
  conv_complete = osEventFlagsNew(&event_attr);

}

// these access functions are meant to execute fast

// this function returns the duty cycle for the specified
// voltage in deci volts

// this function returns the motor current given the measurement value
unsigned current_over(unsigned adc_value, unsigned limit_ma)
{

}

bool vin_over(unsigned voltage_dv)
{
  return voltage_dv * adc_data_buffer.vref > vref_comp_const2;
}

unsigned duty_cycle(unsigned int voltage_mv) {
  return ((voltage_mv*adc_data_buffer.vref*duty_cycle_factor)/adc_data_buffer.vin) >> 6u;
}

static __NO_RETURN void task_sensor(void* arg)
{
  // setup the DMA buffer and enable the channel
  LL_DMA_SetMemoryAddress(DMA_SENSOR, DMA_CHAN_SENSOR, (uint32_t)&adc_data_buffer);
  LL_DMA_SetDataLength(DMA_SENSOR, DMA_CHAN_SENSOR, 3);
  LL_DMA_SetPeriphAddress(DMA_SENSOR, DMA_CHAN_SENSOR, LL_ADC_DMA_GetRegAddr(ADC_SENSOR, LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_EnableChannel(DMA_SENSOR, DMA_CHAN_SENSOR);

  // precalculate a constant used to compute voltages
  unsigned numerator = (3u*VREFINT) << 16u;
  vref_comp_const1 = (unsigned)(491.2 * 3 * 1000 / 16.2) * VREFINT >> 12u;

  while (1) {
    osDelay(1);

    // calibrate
    LL_ADC_StartCalibration(ADC_SENSOR, LL_ADC_SINGLE_ENDED);

    do {
      __NOP();
    } while(LL_ADC_IsCalibrationOnGoing(ADC_SENSOR));

    // enable and wait for ADC ready
    LL_ADC_Enable(ADC_SENSOR);

    do {
      __NOP();
    } while(!LL_ADC_IsActiveFlag_ADRDY(ADC_SENSOR));

    osEventFlagsClear(conv_complete, 1);

    // trigger the ADC
    LL_ADC_REG_StartConversion(ADC_SENSOR);

    osEventFlagsWait(conv_complete, 1, osFlagsWaitAny, 2);

    LL_ADC_Disable(ADC_SENSOR);

    // helper constant for comparing vin
    vref_comp_const2 = adc_data_buffer.vref * vref_comp_const1;
    duty_cycle_factor = duty_cycle_factor_numerator / adc_data_buffer.vin;
  }

}

