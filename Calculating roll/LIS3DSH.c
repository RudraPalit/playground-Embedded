/*
 * LIS3DSH.c
 *
 *  Created on: Nov 21, 2025
 *      Author: rudradeep-palit
 */

#include "LIS3DSH.h"

// SPI handle from main.c
extern SPI_HandleTypeDef hspi1;

// Your CS pin: PE3 (already configured in MX_GPIO_Init)
#define LIS3DSH_CS_PORT     GPIOE
#define LIS3DSH_CS_PIN      GPIO_PIN_3

static void CS_Select(void)
{
    HAL_GPIO_WritePin(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN, GPIO_PIN_RESET);
}

static void CS_Deselect(void)
{
    HAL_GPIO_WritePin(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN, GPIO_PIN_SET);
}

static void LIS3DSH_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F; // write: bit7 = 0
    tx[1] = data;

    CS_Select();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    CS_Deselect();
}

static void LIS3DSH_ReadMulti(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    // read + auto increment: bit7=1 (read), bit6=1 (multibyte)
    uint8_t addr = reg | 0x80;   // read bit only

    CS_Select();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, len, HAL_MAX_DELAY);
    CS_Deselect();
}

static uint8_t LIS3DSH_ReadReg(uint8_t reg)
{
    uint8_t val;
    LIS3DSH_ReadMulti(reg, &val, 1);
    return val;
}

uint8_t LIS3DSH_ReadWHOAMI(void)
{
    return LIS3DSH_ReadReg(LIS3DSH_WHO_AM_I_ADDR);   // should be 0x3F
}

void LIS3DSH_Init(void)
{
    // Make sure CS is high (deselected)
    CS_Deselect();
    HAL_Delay(5);

    uint8_t who = LIS3DSH_ReadWHOAMI();
    // Optional: check in debugger that who == 0x3F
    (void)who;

    // CTRL_REG4: ODR = 100 Hz, enable X/Y/Z (0x67)
    LIS3DSH_WriteReg(LIS3DSH_CTRL_REG4_ADDR, 0x67);

    // CTRL_REG5: ±2g, BW default (0x00)
    LIS3DSH_WriteReg(LIS3DSH_CTRL_REG5_ADDR, 0x00);

    // CTRL_REG6: enable address auto increment (bit4 = 1 -> 0x10)
    LIS3DSH_WriteReg(LIS3DSH_CTRL_REG6_ADDR, 0x10);

    // CTRL_REG3: DR_EN = 1 (enable data-ready)
    LIS3DSH_WriteReg(0x23, 0x80);

    HAL_Delay(10);
}


void LIS3DSH_ReadRaw(LIS3DSH_RawData_t *raw)
{
    uint8_t buf[6];

    LIS3DSH_ReadMulti(LIS3DSH_OUT_X_L_ADDR, buf, 6);

    raw->X = (int16_t)((buf[1] << 8) | buf[0]);
    raw->Y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->Z = (int16_t)((buf[5] << 8) | buf[4]);
}

void LIS3DSH_ConvertToG(const LIS3DSH_RawData_t *raw,
                        float *ax, float *ay, float *az)
{
    // For ±2g, sensitivity ≈ 0.06 mg/LSB = 0.00006 g/LSB
    const float SENS_G_PER_LSB = 0.00006f;

    if (ax) *ax = raw->X * SENS_G_PER_LSB;
    if (ay) *ay = raw->Y * SENS_G_PER_LSB;
    if (az) *az = raw->Z * SENS_G_PER_LSB;
}



