//
// Created by david on 10/11/2020.
//

#include <cmsis_os2.h>
#include "i2c.h"
#include "stm32l4xx_ll_i2c.h"
#include "SEGGER_RTT.h"

#define I2Cx I2C1

// TODO: Only one I2C supported
static struct {
  uint8_t *wbuf1;
  uint8_t *wbuf2;
  uint8_t *rbuf;
  uint8_t nw1;
  uint8_t nw2;
  uint8_t nr;
  enum {
    I2C_IDLE,
    I2C_WRITE,
    I2C_READ,
    I2C_ERROR_NACK,
    I2C_ERROR_TIMEOUT,
  } i2c_state;
  uint8_t addr;
  uint8_t num_wr;
  uint8_t num_rd;
} i2c_transaction;

const osEventFlagsAttr_t i2c_event_attr = {
    .name = "compass",
};

osEventFlagsId_t i2c_ready;

int i2c_wwr(
    uint8_t addr,
    void *wbuf1, uint8_t wlen1,
    void *wbuf2, uint8_t wlen2,
    void *rbuf,  uint8_t rlen)
{
  i2c_transaction.wbuf1 = wbuf1;
  i2c_transaction.wbuf2 = wbuf2;
  i2c_transaction.rbuf = rbuf;
  i2c_transaction.nw1 = wlen1;
  i2c_transaction.nw2 = wlen1;
  i2c_transaction.nr = rlen;
  i2c_transaction.num_wr = 0;
  i2c_transaction.num_rd = 0;

  LL_I2C_SetSlaveAddr(I2Cx, addr);
  if (wlen1 + wlen2) {
    LL_I2C_SetTransferRequest(I2Cx, LL_I2C_REQUEST_WRITE);
    LL_I2C_SetTransferSize(I2Cx, wlen1 + wlen2);
    i2c_transaction.i2c_state = I2C_WRITE;
  } else {
    LL_I2C_SetTransferRequest(I2Cx, LL_I2C_REQUEST_READ);
    LL_I2C_SetTransferSize(I2Cx, rlen);
    i2c_transaction.i2c_state = I2C_READ;
  }

  osEventFlagsClear(i2c_ready, 1);
  LL_I2C_GenerateStartCondition(I2Cx);

  // TODO: hardcoded timeout
  unsigned ret = osEventFlagsWait(i2c_ready, 1, osFlagsWaitAny, 10);
  if (ret == osFlagsErrorTimeout)
    return I2C_ERROR_TIMEOUT;
  else
    return i2c_transaction.i2c_state;
}

void i2c_init()
{
  i2c_ready = osEventFlagsNew(&i2c_event_attr);
  I2Cx->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE;
  LL_I2C_DisableAutoEndMode(I2Cx);
}


void i2c_evt_irq_handler(I2C_TypeDef *i2c)
{
  unsigned status = i2c->ISR;

  if (status & I2C_ISR_NACKF) {
    i2c_transaction.i2c_state = I2C_ERROR_NACK;
    osEventFlagsSet(i2c_ready, 1);
  } else if (status & I2C_ISR_TXIS) {
    LL_I2C_TransmitData8(i2c, *i2c_transaction.wbuf1++);
    i2c_transaction.num_wr++;
    if (i2c_transaction.num_wr == i2c_transaction.nw1)
      i2c_transaction.wbuf1 = i2c_transaction.wbuf2;
  } else if (status & I2C_ISR_RXNE) {
    *i2c_transaction.rbuf++ = LL_I2C_ReceiveData8(i2c);
    i2c_transaction.num_rd++;
  } else if (status & I2C_ISR_TC) {
    if (i2c_transaction.i2c_state == I2C_WRITE && i2c_transaction.nr) {
      i2c_transaction.i2c_state = I2C_READ;
      LL_I2C_SetTransferRequest(i2c, LL_I2C_REQUEST_READ);
      LL_I2C_GenerateStartCondition(i2c);
      LL_I2C_SetTransferSize(i2c, i2c_transaction.nr);
    } else {
      LL_I2C_GenerateStopCondition(i2c);
      osEventFlagsSet(i2c_ready, 1);
      i2c_transaction.i2c_state = I2C_IDLE;
    }
  }
}

void i2c_err_irq_handler(I2C_TypeDef *i2c)
{

}

