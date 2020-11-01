//
// Created by david on 10/23/2020.
//
#include "calibration_constants.h"
#include <memory.h>
#include <mag_cal.h>
#include "stm32l4xx.h"
#include "stm32l4xx_hal_flash.h"
#include "stm32l4xx_hal_flash_ex.h"

// This uses 4k of flash located at the end of flash to store calibration constants. Each constant
// is stored using a 8-byte block.

struct __packed cal_block {
  enum {
    CAL_POT_PARAMS = 1,     // pot value (less than the max pot value) pointed to the reference
    CAL_MAGNETIC,           // magnetic calibration. Actually three values representing the x,y and z offsets
    CAL_HDG_OFFSET,         // offset in degress of the heading. Can be used to reorient the sensor
    CAL_BACKLASH,           // Backlash of mechanism in pot LSBs
    CAL_UNPROGRAMMED = 0xFFFF, // The unprogrammed variant of the type
  } type;
  union {
    int16_t v16[3];         // three 16-bit integers
    int16_t i16;            // a single 16-bit integer
    uint16_t u16;
    struct {
      uint16_t pot_max;
      uint16_t pot_zero;
      uint16_t pot_1_turn;
    };
  };
};

struct calibration_data {
  int16_t pot_max;
  int16_t pot_zero;
  int16_t pot_1_turn;
  int16_t hdg_offset;
  uint16_t backlash;
  int16_t mag_bias[3];
};

// this is the midpoint of the pot minus 5 degrees
const int16_t pot_max_default = (int)(0.5*4096) - (int)(5.0/360*1.5/10*4096);
const int16_t pot_max_minus_360_default = pot_max_default - (int)(1.5/10*4096);
const int16_t pot_zero_default = (pot_max_default + pot_max_minus_360_default)/2;

const struct calibration_data cal_defaults = {
    .pot_max = pot_max_default,
    .pot_zero = pot_zero_default,
    .pot_1_turn = pot_max_minus_360_default,
    .mag_bias = {0, 0, 0},
    .hdg_offset = 0,
    .backlash = 0,
};

// this is the overlap in degrees to prevent chattering
static const int deg_overlap = 10;
static struct calibration_data cal_data;

// "derived" constants
static int16_t min_angle;
static int16_t pot_min;     // This is pot_max - 1 Turn - 10 degrees

unsigned n_cal_blocks = 4096/8;

static struct cal_block __attribute__((section (".cal"))) calblocks[1];

// prototypes
static struct cal_block *cal_data_blk_ptr;
static void write_cal_block(const struct cal_block *cb);
void erase_cal();
void compute_derived_constants();
int pot_to_angle(unsigned pot);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "misc-no-recursion"
// these functions look recursive, but the recursion was eliminated
void save_mag_bias(int16_t bias[3]) {
  struct cal_block cb;

  cb.type = CAL_MAGNETIC;
  memcpy(cb.v16, bias, sizeof(cb.v16));
  memcpy(cal_data.mag_bias, bias, sizeof(cb.v16));
  write_cal_block(&cb);
}

void save_backlash(uint16_t backlash) {
  struct cal_block cb;

  cal_data.backlash = backlash;

  cb.type = CAL_BACKLASH;
  cb.u16 = backlash;
  write_cal_block(&cb);
}

uint16_t get_backlash() {
  return cal_data.backlash;
}

int16_t *get_mag_bias() {
  return cal_data.mag_bias;
}

void save_pot_parameters(uint16_t pot_max, uint16_t pot_zero, uint16_t pot_1_turn) {
  struct cal_block cb;

  // make the calibration data match what is saved
  cal_data.pot_max = pot_max;
  cal_data.pot_1_turn = pot_1_turn;
  cal_data.pot_zero = pot_zero;

  cb.type = CAL_POT_PARAMS;
  cb.pot_max = pot_max;
  cb.pot_zero = pot_zero;
  cb.pot_1_turn = pot_1_turn;
  write_cal_block(&cb);
  compute_derived_constants();
}

void write_cal_block(const struct cal_block *cb) {
  if ((size_t) cal_data_blk_ptr == FLASH_END + 1) {
    erase_cal();
    // this stops a recursive call
    if (cb->type != CAL_POT_PARAMS)
      save_pot_parameters(cal_data.pot_max, cal_data.pot_zero, cal_data.pot_1_turn);
    if (cb->type != CAL_MAGNETIC)
      save_mag_bias(cal_data.mag_bias);
  }

  HAL_FLASH_Unlock();
  HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, (size_t) cal_data_blk_ptr++, *(uint64_t *) cb);
  HAL_FLASH_Lock();
}
#pragma clang diagnostic pop

void erase_cal() {
  FLASH_EraseInitTypeDef page_erase = {
      .Page = ((size_t)calblocks - FLASH_BASE)/FLASH_PAGE_SIZE,
      .NbPages = 2,
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .Banks = FLASH_BANK_1,
  };
  HAL_FLASH_Unlock();
  uint32_t error;
  HAL_FLASHEx_Erase(&page_erase, &error);
  HAL_FLASH_Lock();
}

void compute_derived_constants() {
  min_angle = pot_to_angle(cal_data.pot_max);
  pot_min = cal_data.pot_1_turn + (cal_data.pot_1_turn - cal_data.pot_max) * (deg_overlap + 1);
}

int pot_to_angle(unsigned int pot) {
  return -(360*((int)pot - cal_data.pot_zero))/(cal_data.pot_1_turn - cal_data.pot_max);
}

int angle_to_pot(int angle) {
  unsigned pot = cal_data.pot_zero + ((cal_data.pot_1_turn - cal_data.pot_max)*angle)/360;
  if (pot > cal_data.pot_max)
    return -1;
  if (pot < cal_data.pot_1_turn - deg_overlap*2)
    return 1;
  return pot;
}


uint16_t limit_pot(uint16_t pot) {
  if (pot > cal_data.pot_max)
    return cal_data.pot_max;
  
  if (pot < pot_min)
    return cal_data.pot_max;
  return pot;
}

static inline int iabs(int i) {
  return i >= 0 ? i : -i;
}

int angle_to_pot_from_angle(int angle, int from_angle);

void get_pot_params(uint16_t *pot_max, uint16_t *pot_zero, uint16_t *pot_1_turn) {
  *pot_max = cal_data.pot_max;
  *pot_zero = cal_data.pot_zero;
  *pot_1_turn = cal_data.pot_1_turn;
}

void init_calibration_constants()
{
  cal_data_blk_ptr = calblocks;
  memcpy(&cal_data, &cal_defaults, sizeof(cal_data));

  // go through all of the calibration blocks
  unsigned done = 0;
  for (int i=0; i < n_cal_blocks && !done; i++ ) {
    switch (cal_data_blk_ptr->type) {
    case CAL_POT_PARAMS:
      cal_data.pot_max = cal_data_blk_ptr->pot_max;
      cal_data.pot_zero = cal_data_blk_ptr->pot_zero;
      cal_data.pot_1_turn = cal_data_blk_ptr->pot_1_turn;
      cal_data_blk_ptr++;
      break;
    case CAL_MAGNETIC:
      save_mag_bias(cal_data_blk_ptr->v16);
      cal_data_blk_ptr++;
      break;
    case CAL_BACKLASH:
      cal_data.backlash = cal_data_blk_ptr->u16;
      cal_data_blk_ptr++;
      break;
    case CAL_UNPROGRAMMED:
      done = memcmp(cal_data_blk_ptr->v16, "\xFF\xFF\xFF\xFF\xFF\xFF", 6) == 0;
      break;
    default:
      break;
    }
  }
  // at this point, cal_data_blk_ptr is pointing to the next free entry and
  // the calibration variables are set
  compute_derived_constants();
}

