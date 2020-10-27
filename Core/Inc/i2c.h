//
// Created by david on 10/11/2020.
//

#ifndef GEO_00001_I2C_H
#define GEO_00001_I2C_H

#include "stm32l4xx_ll_i2c.h"

void i2c_evt_irq_handler(I2C_TypeDef *i2c);
void i2c_err_irq_handler(I2C_TypeDef *i2c);
void i2c_init();

int i2c_wwr(
    uint8_t addr,
    const void *wbuf1, uint8_t wlen1,
    const void *wbuf2, uint8_t wlen2,
    void *rbuf,  uint8_t rlen);

static inline int i2c_wr(
    uint8_t addr,
    const void *wbuf, uint8_t wlen,
    void *rbuf, uint8_t rlen)
{
  return i2c_wwr(addr, wbuf, wlen, 0, 0, rbuf, rlen);
}

static inline int i2c_ww(
    uint8_t addr,
    const void *wbuf1, uint8_t wlen1,
    const void *wbuf2, uint8_t wlen2) {
  return i2c_wwr(addr, wbuf1, wlen1, wbuf2, wlen2, 0, 0);
}

static inline int i2c_w(
    uint8_t addr,
    const void *wbuf1, uint8_t wlen1) {
  return i2c_wwr(addr, wbuf1, wlen1, 0, 0, 0, 0);
}
#endif //GEO_00001_I2C_H
