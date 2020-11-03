//
// Created by david on 10/28/2020.
//

#include <stdint-gcc.h>
#include <memory.h>
#include <stdbool.h>
#include "mag_cal.h"
#include "calibration_constants.h"

static struct {
  uint32_t count;
  int32_t sum[3];
  int64_t BB[6];
  int64_t z_B[3];
} cal_stats;

static bool mag_cal = false;

static void mag_cal_clear();
static void mag_cal_calculate();

void mag_cal_clear() {
  memset(&cal_stats, 0, sizeof(cal_stats));
}

bool mag_cal_state() {
  return mag_cal;
}

void mag_cal_cancel() {
  mag_cal = false;
}

void mag_cal_calculate(int16_t adjustment[3]) {
  // compute ~B x ~B^T. The idea is to do all of the math with integers, then change to floating point to
  // preserve precision
  float a[6];
  float b[3];

  unsigned idx = 0;
  int64_t z_bar = cal_stats.BB[0] + cal_stats.BB[3] + cal_stats.BB[5];
  for (int i = 0; i < 3; i++) {
    int64_t p1 = cal_stats.count * cal_stats.z_B[i];
    int64_t p2 = z_bar * cal_stats.sum[i];
    b[i] = (float) (p1 - p2);
    for (int j = i; j < 3; j++) {
      a[idx] = 2.0f * (float) (cal_stats.count * cal_stats.BB[idx] -
                               (int64_t) cal_stats.sum[i] * cal_stats.sum[j]);
      idx++;
    }
  }

  // perform the Cholesky (LDL) decomposition of the fisher matrix. Note that this can be done in-place
  // (0) D1 = A11
  // (1) L21 = A21/D1
  // (2) L31 = A31/D1
  // (3) D2 = A22 - L21^2*D1
  // (4) L32 = (A32 - L31*L21*D1)/D2
  // (5) D3 = A33 - L31^2*D1 - L32^2*D2
  a[0] = a[0];
  a[1] = a[1] / a[0];
  a[2] = a[2] / a[0];
  a[3] = a[3] - a[1] * a[1] * a[0];
  a[4] = (a[4] - a[2] * a[1] * a[0]) / a[3];
  a[5] = a[5] - a[2] * a[2] * a[0] - a[4] * a[4] * a[3];

  // now that I've factored the array, use forward substitution
  b[0] = b[0];
  b[1] = b[1] - a[1] * b[0];
  b[2] = b[2] - a[2] * b[0] - a[4] * b[1];

  // invert the diagonal matrix
  b[0] = b[0] / a[0];
  b[1] = b[1] / a[3];
  b[2] = b[2] / a[5];

  // now do back substitution
  b[2] = b[2];
  b[1] = b[1] - a[4] * b[2];
  b[0] = b[0] - a[2] * b[2] - a[1] * b[1];

  // b is now an adjustment to the current magnetic calibration
  int16_t *bias = get_mag_bias();
  for (int i = 0; i < 3; i++) {
    int16_t adj = (int16_t) b[i];
    if (adjustment) adjustment[i] = adj;
    bias[i] += adj;
  }
  save_mag_bias(bias);
}

void mag_cal_apply(int16_t *d) {
  uint32_t mag_b = 0;
  int16_t * bias = get_mag_bias();

  for (int i=0; i<3; i++)
    d[i] = d[i] - bias[i];

  if (mag_cal) {
    for (int i = 0; i < 3; i++) {
      cal_stats.sum[i] += d[i];
      mag_b += d[i] * d[i];
    }

    unsigned idx = 0;
    for (int i = 0; i < 3; i++) {
      cal_stats.z_B[i] += (int64_t) mag_b * d[i];
      for (int j = i; j < 3; j++)
        cal_stats.BB[idx++] += (int64_t) d[i] * d[j];
    }
    cal_stats.count++;
  }
}

void mag_cal_stop(int16_t offset[3]) {
  mag_cal = false;
  mag_cal_calculate(offset);
}

void mag_cal_start() {
  mag_cal_clear();
  mag_cal = true;
}
