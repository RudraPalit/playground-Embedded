# STM32F4 Discovery – LIS3DSH Accelerometer Driver + Tilt Angle Demo

This project demonstrates SPI communication with the onboard **LIS3DSH accelerometer** on the STM32F4-Discovery board.  
It includes a custom driver, raw data acquisition, g-unit conversion, and real-time **roll/pitch angle** computation.  
Results are streamed over **SWO (ITM)** using `printf()`.

---

## Overview

This project shows how to:
- Configure SPI1 in Mode 3
- Read accelerometer data from LIS3DSH
- Convert raw measurements into physical units (g)
- Adjust for the sensor’s inverted mounting
- Compute **roll** and **pitch** using trigonometric functions
- Print results live using SWO + ITM with a custom `_write()` function

The code is deliberately simple and educational, suitable for learning embedded systems and IMU basics.

---

## Features

- Custom LIS3DSH driver (`LIS3DSH.c/.h`)
- Manual chip-select via GPIO PE3
- WHO_AM_I device verification
- Multi-byte SPI reads using auto-increment
- Sensitivity conversion for ±2 g mode
- Z-axis inversion correction
- Roll & pitch angle calculation
- SWO printf output via ITM port 0
- Hardware FPU enabled for fast float math

---

## Hardware Setup

The LIS3DSH accelerometer is already connected on the STM32F4-Discovery board:

| Signal | STM32F407 Pin |
|--------|----------------|
| SCK    | PA5 |
| MISO   | PA6 |
| MOSI   | PA7 |
| CS     | PE3 |

No external wiring required.

---

## Driver Architecture (`LIS3DSH.c`)

### 1. Chip Select Control

```c
static void CS_Select(void) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

static void CS_Deselect(void) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}
```

### 2. SPI Register Access

**Write:**

```c
tx[0] = reg & 0x7F; // write: bit7 = 0
tx[1] = data;
```

**Read Multi-byte:**

```c
uint8_t addr = reg | 0x80; // read: bit7 = 1
CS_Select();
HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
HAL_SPI_Receive(&hspi1, buffer, len, HAL_MAX_DELAY);
CS_Deselect();
```

### 3. Sensor Initialization

```c
LIS3DSH_WriteReg(CTRL4, 0x67); // 100Hz, X/Y/Z enabled
LIS3DSH_WriteReg(CTRL5, 0x00); // ±2g range
LIS3DSH_WriteReg(CTRL6, 0x10); // ADD_INC = 1
LIS3DSH_WriteReg(CTRL3, 0x80); // DR_EN
```

### 4. RAW → g Conversion

```c
const float SENS = 0.00006f; // g/LSB

*ax = raw->X * SENS;
*ay = raw->Y * SENS;
*az = raw->Z * SENS;
```

---

## Main Program Flow (`main.c`)

### 1. Enable FPU

```c
SCB->CPACR |= (0xF << 20);
```

### 2. Initialize HAL, clock, GPIO, SPI, LIS3DSH

Standard CubeMX initialization.

### 3. SWO Printf Redirection

```c
int _write(int file, char *ptr, int len) {
    for(int i = 0; i < len; i++)
        ITM_SendChar(ptr[i]);
    return len;
}
```

### 4. Read Sensor, Convert, Compute Angles

```c
LIS3DSH_ReadRaw(&raw);
LIS3DSH_ConvertToG(&raw, &ax, &ay, &az);

// LIS3DSH is mounted upside-down
az = -az;

float roll  = atan2f(ay, az) * 180.0f / M_PI;
float pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;

printf("RAW  X=%d Y=%d Z=%d\n", raw.X, raw.Y, raw.Z);
printf("G    ax=%.4f ay=%.4f az=%.4f\n", ax, ay, az);
printf("ANG  roll=%.2f  pitch=%.2f\n\n", roll, pitch);
```

---

## Example Output

```
RAW  X=320 Y=330 Z=17040
G    ax=0.0190 ay=0.0200 az=1.0200
ANG  roll=0.92 deg  pitch=-1.05 deg
```

---

## Possible Extensions

- Low-pass filtering (EMA)  
- LED tilt indicators  
- Gyro + accelerometer fusion (Complementary/Kalman filter)  
- UART data streaming + Python plotting  
- Full IMU abstraction layer  

---

## Author

Rudradeep Palit  
Embedded Systems & Firmware Developer

