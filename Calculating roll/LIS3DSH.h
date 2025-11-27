/*
 * LIS3DSH.h
 *
 *  Created on: Nov 21, 2025
 *      Author: rudradeep-palit
 */
#ifndef LIS3DSH_H_
#define LIS3DSH_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

// Register addresses
#define LIS3DSH_WHO_AM_I_ADDR      0x0F
#define LIS3DSH_CTRL_REG4_ADDR     0x20
#define LIS3DSH_CTRL_REG5_ADDR     0x24
#define LIS3DSH_CTRL_REG6_ADDR     0x25

#define LIS3DSH_OUT_X_L_ADDR       0x28
#define LIS3DSH_OUT_X_H_ADDR       0x29
#define LIS3DSH_OUT_Y_L_ADDR       0x2A
#define LIS3DSH_OUT_Y_H_ADDR       0x2B
#define LIS3DSH_OUT_Z_L_ADDR       0x2C
#define LIS3DSH_OUT_Z_H_ADDR       0x2D

typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
} LIS3DSH_RawData_t;

void     LIS3DSH_Init(void);
uint8_t  LIS3DSH_ReadWHOAMI(void);
void     LIS3DSH_ReadRaw(LIS3DSH_RawData_t *raw);
void     LIS3DSH_ConvertToG(const LIS3DSH_RawData_t *raw,
                            float *ax, float *ay, float *az);

#endif /* LIS3DSH_H_ */

