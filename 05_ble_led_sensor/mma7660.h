#ifndef __MMA7660_H__
#define __MMA7660_H__

#include "nrf_drv_twi_dma.h"
#include <stdint.h>
#include <stdbool.h>

#define MMA7660_DEFAULT_ADDRESS   0x4c

#define MMA7660_X     0x00
#define MMA7660_Y     0x01
#define MMA7660_Z     0x02
#define MMA7660_TILT  0x03
#define MMA7660_SRST  0x04
#define MMA7660_SPCNT 0x05
#define MMA7660_INTSU 0x06
#define MMA7660_MODE  0x07
#define MMA7660_SR    0x08
#define MMA7660_PDET  0x09
#define MMA7660_PD    0x0A

typedef enum
{
    SAMPLES_PER_SEC_120 = 0,
    SAMPLES_PER_SEC_64,
    SAMPLES_PER_SEC_32,
    SAMPLES_PER_SEC_16,
    SAMPLES_PER_SEC_8,
    SAMPLES_PER_SEC_4,
    SAMPLES_PER_SEC_2,
    SAMPLES_PER_SEC_1,
} mma7660_mode_t;

typedef struct __attribute__((__packed__))
{
        int8_t orientation_data;
}  mma7660_orientation_output_t;

typedef struct __attribute__((__packed__))
{
    mma7660_orientation_output_t x;
    mma7660_orientation_output_t y;
    mma7660_orientation_output_t z;
} mma7660_accelerometer_data_t;

uint32_t mma7660_register_write(nrf_drv_twi_t const * const p_twi_instance, uint8_t reg, uint8_t val);

uint32_t mma7660_register_read(nrf_drv_twi_t const * const p_twi_instance, uint8_t reg, uint8_t *buf, uint8_t num_of_bytes);

uint32_t mma7660_init(nrf_drv_twi_t const * const p_twi_instance, mma7660_mode_t samples_per_second);

uint32_t mma7660_read_xyz(nrf_drv_twi_t const * const p_twi_instance, mma7660_accelerometer_data_t * p_data);

#endif
