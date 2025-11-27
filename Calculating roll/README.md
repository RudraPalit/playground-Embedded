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

# LIS3DSH Accelerometer Driver (SPI) for STM32F407

This section  describes the standalone **LIS3DSH accelerometer driver** implemented in `LIS3DSH.c` / `LIS3DSH.h`.  
The driver provides a clean and minimal API for configuring the LIS3DSH and reading 3-axis acceleration data over **SPI1** on the STM32F407.

---

## 📌 Source and References

This driver is based on and adapted from:  
**Ruturajn’s Accelerometer-SPI STM32F407 project**  
🔗 https://github.com/Ruturajn/Accelerometer-SPI-Interface-with-STM32F407VG/tree/main

The register definitions and configuration values follow the official STMicroelectronics documentation:

- **LIS3DSH Datasheet**  
- **LIS3DSH Application Notes**  
- **STM32F407 Reference Manual (RM0090)**  
- **SPI peripheral documentation (RM0090, Ch. 28)**

---

## 🧩 Driver Overview

The driver handles:

- Manual chip-select control (GPIO-based NSS)
- SPI register read/write transactions
- Multi-byte reads using the LIS3DSH’s **ADD_INC** addressing mode
- Register initialization (CTRL3/CTRL4/CTRL5/CTRL6)
- WHO_AM_I verification
- Raw acceleration extraction (X/Y/Z 16-bit signed)
- Conversion from raw LSB → **g-units**

The driver is intentionally lightweight and easy to integrate into any STM32 HAL project.

---

# 1. Chip Select (CS) Handling

LIS3DSH is connected to **PE3**, so the driver manually controls CS:

```c
#define LIS3DSH_CS_PORT GPIOE
#define LIS3DSH_CS_PIN  GPIO_PIN_3

static void CS_Select(void) {
    HAL_GPIO_WritePin(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN, GPIO_PIN_RESET);
}

static void CS_Deselect(void) {
    HAL_GPIO_WritePin(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN, GPIO_PIN_SET);
}
```

**Why manual CS?**  
- The LIS3DSH requires precise timing around multi-byte reads  
- Manual control avoids HAL’s automatic NSS toggling  
- This method matches the LIS3DSH datasheet timing diagrams (SPI mode 3)

---

# 2. SPI Register Access

## 2.1 Writing a Register

```c
tx[0] = reg & 0x7F;  // bit7 = 0 → write
tx[1] = data;
```

The write sequence follows the *“Register Write Mode”* timing specified in the datasheet.

---

## 2.2 Reading One or More Registers

The LIS3DSH uses bit7 = 1 for read access:

```c
uint8_t addr = reg | 0x80;  // bit7 = 1 (read)
```

Unlike other ST sensors (LIS3DH/LIS2DH),  
**LIS3DSH does NOT use bit6 to enable multi-byte auto-increment**.  

Instead, the auto-increment feature is controlled by a **bit in CTRL6**:

```c
LIS3DSH_WriteReg(LIS3DSH_CTRL_REG6_ADDR, 0x10); // ADD_INC = 1
```

### Multi-byte read transaction:

```c
CS_Select();
HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
HAL_SPI_Receive(&hspi1, buffer, len, HAL_MAX_DELAY);
CS_Deselect();
```

This behavior follows **Section 6.3 “SPI Serial Interface”** in the LIS3DSH datasheet.

---

# 3. Device Initialization

The driver configures the LIS3DSH using standard control registers:

```c
LIS3DSH_WriteReg(CTRL4, 0x67); // ODR = 100 Hz, X/Y/Z enable
LIS3DSH_WriteReg(CTRL5, 0x00); // ±2g sensitivity
LIS3DSH_WriteReg(CTRL6, 0x10); // ADD_INC enable
LIS3DSH_WriteReg(CTRL3, 0x80); // DR_EN: Data-ready interrupt enable
```

### Meaning of these registers (from datasheet):

- **CTRL4 (0x20)**  
  - ODR bits = 0110 (100 Hz)  
  - X, Y, Z axis enable bits = 1  
- **CTRL5 (0x24)**  
  - FSCALE = 000 → ±2 g  
- **CTRL6 (0x25)**  
  - ADD_INC bit = 1 → auto-increment  
- **CTRL3 (0x23)**  
  - DR_EN = 1 → enables new-data-ready latching

---

# 4. Reading Raw Acceleration

The LIS3DSH outputs acceleration on **six consecutive registers**:  
`OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H`

The driver reads all 6 at once:

```c
uint8_t buf[6];
LIS3DSH_ReadMulti(LIS3DSH_OUT_X_L_ADDR, buf, 6);
```

Signed 16-bit reconstruction:

```c
raw->X = (int16_t)((buf[1] << 8) | buf[0]);
raw->Y = (int16_t)((buf[3] << 8) | buf[2]);
raw->Z = (int16_t)((buf[5] << 8) | buf[4]);
```

This matches the **LIS3DSH output register map** described in the datasheet.

---

# 5. Converting RAW → g

The LIS3DSH sensitivity depends on the full-scale setting.  
For **±2 g**, the sensitivity is:

```
0.06 mg/LSB = 0.00006 g/LSB
```

So the driver multiplies raw values by this constant:

```c
const float SENS = 0.00006f;

*ax = raw->X * SENS;
*ay = raw->Y * SENS;
*az = raw->Z * SENS;
```

These numbers match the **“Sensitivity Characteristics”** table in the datasheet.

---

# 📄 API Summary (Public Functions)

| Function | Description |
|----------|-------------|
| `LIS3DSH_Init()` | Configures LIS3DSH via SPI |
| `LIS3DSH_ReadWHOAMI()` | Returns the 0x3F ID |
| `LIS3DSH_ReadRaw(raw*)` | Reads 16-bit raw X/Y/Z |
| `LIS3DSH_ConvertToG(raw, ax, ay, az)` | Converts to g-units |

---

# ✔ Notes

- The LIS3DSH is mounted **upside-down** on the Discovery board.  
  Flipping Z in application code (`az = -az`) compensates for this.
- The SPI mode used is **CPOL=1, CPHA=1 (Mode 3)**.
- The driver does not use interrupts; it uses **polling**.

---

# 📘 Credits

Driver adapted and extended from:  
https://github.com/Ruturajn/Accelerometer-SPI-Interface-with-STM32F407VG

Extended, cleaned, and documented for improved readability and correctness.

---
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
- Gyro + accelerometer fusion (Complementary/Kalman filter)  
- UART data streaming + Python plotting  
- Full IMU abstraction layer  

---

## Author

Rudradeep Palit  
Embedded Systems & Firmware Developer

