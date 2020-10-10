//
// Created by david on 10/10/2020.
//
#include <FreeRTOS.h>
#include <stm32l4xx_ll_adc.h>
#include <stm32l4xx_ll_dma.h>
#include "sensor.h"
#include "stdint.h"
#include "cmsis_os2.h"

#define ADC_SENSOR ADC1
#define DMA_SENSOR DMA1
#define DMA_CHAN_SENSOR LL_DMA_CHANNEL_1

static __NO_RETURN void task_sensor(void* arg);

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

static __NO_RETURN void task_sensor(void* arg)
{
  // setup the DMA buffer and enable the channel
  LL_DMA_SetMemoryAddress(DMA_SENSOR, DMA_CHAN_SENSOR, (uint32_t)&adc_data_buffer);
  LL_DMA_SetDataLength(DMA_SENSOR, DMA_CHAN_SENSOR, sizeof(adc_data_buffer));
  LL_DMA_SetPeriphAddress(DMA_SENSOR, DMA_CHAN_SENSOR, LL_ADC_DMA_GetRegAddr(ADC_SENSOR, LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_EnableChannel(DMA_SENSOR, DMA_CHAN_SENSOR);

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

  }

}