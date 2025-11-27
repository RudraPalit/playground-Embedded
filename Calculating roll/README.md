📘 STM32F4 Discovery – LIS3DSH Accelerometer Driver + Tilt Angle Demo

A fun and technically solid project showing SPI sensor interfacing, real-time math, and SWO debugging on STM32.

🧭 Overview

This project demonstrates how to interface the LIS3DSH accelerometer on the STM32F4-Discovery board using a custom SPI driver, convert sensor readings into physical units (g), and compute roll and pitch angles in real-time. Output is streamed over SWO / ITM using printf(), meaning no UART wiring is required.

The codebase is intentionally simple, readable, and educational—perfect for anyone learning embedded systems, IMUs, or bare-metal debugging techniques on ARM Cortex-M microcontrollers.

📂 Features

Custom LIS3DSH driver using SPI1 (mode 3)

Manual chip select handling using PE3

WHO_AM_I check for hardware validation

Multi-byte register reads with auto-increment

RAW → g conversion using datasheet sensitivity

Correct Z-axis sign handling (sensor is inverted on Discovery)

Real-time roll & pitch from accelerometer math

SWO printf via _write() redirection

FPU enabled for fast floating-point performance

🧱 Hardware Setup

You only need the STM32F4-Discovery board.
All connections are pre-wired on the board.

Sensor → MCU Mapping (Internal)
LIS3DSH	STM32F407 Pin
SCK	PA5
MISO	PA6
MOSI	PA7
CS	PE3
Power	On-board
🧩 Driver Architecture (LIS3DSH.c)

The LIS3DSH driver is intentionally minimalistic but robust.
It contains three core components:

### 1️⃣ Chip Select Control
static void CS_Select(void)   { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); }
static void CS_Deselect(void) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   }


We manually toggle PE3 to start/end SPI transactions.

This ensures precise control and avoids HAL’s automatic NSS behavior.

2️⃣ SPI Register Access
Write:
tx[0] = reg & 0x7F;   // write = bit7 = 0
tx[1] = data;

Read Multi-byte:
uint8_t addr = reg | 0x80;   // bit7 = 1 (read)
CS_Select();
HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
HAL_SPI_Receive(&hspi1, buffer, len, HAL_MAX_DELAY);
CS_Deselect();


Notes:

LIS3DSH uses the address auto-increment bit inside CTRL6, not in the address byte.

Multi-byte reads fetch X_L, X_H, Y_L, Y_H, Z_L, Z_H in one shot.

3️⃣ Sensor Initialization
LIS3DSH_WriteReg(CTRL4, 0x67);   // 100Hz, enable X/Y/Z
LIS3DSH_WriteReg(CTRL5, 0x00);   // ±2g range
LIS3DSH_WriteReg(CTRL6, 0x10);   // ADD_INC = 1
LIS3DSH_WriteReg(CTRL3, 0x80);   // data-ready enable


Sensor outputs new data at 100 Hz.

±2g sensitivity gives best resolution.

Auto-increment allows fast 6-byte reads.

Data-ready ensures stable sample timing.

4️⃣ RAW Reading + Conversion
raw->X = (buf[1] << 8) | buf[0];
ax = raw->X * 0.00006f;   // sensitivity for ±2g


Sensitivity = 0.06 mg/LSB = 0.00006 g/LSB

Converting to g-units makes tilt calculations meaningful.

🧠 Main Program Flow (main.c)

The main program orchestrates everything:

1️⃣ Enable FPU
SCB->CPACR |= (0xF << 20);


Enables CP10/CP11 for fast float math (atan2f, sqrtf, multiplications).

2️⃣ Initialize HAL + Peripherals

System clock → HSI @ 16 MHz

SPI1 → Mode 3

GPIOE → PE3 as output

SWO ITM enabled for debugging

3️⃣ Printf Redirection to SWO
int _write(...) {
    ITM_SendChar(ptr[i]);
}


Lets you use printf()

Sends characters out via SWO ITM Stimulus Port 0

View inside CubeIDE → SWV → ITM Console

4️⃣ Sensor Loop (Read → Convert → Angles → Print)
LIS3DSH_ReadRaw(&raw);
LIS3DSH_ConvertToG(&raw, &ax, &ay, &az);
az = -az; // board-mounted compensation

Angle math:
roll  = atan2f(ay, az);
pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

Output:
printf("RAW X=%d Y=%d Z=%d\n", raw.X, raw.Y, raw.Z);
printf("G ax=%.4f ay=%.4f az=%.4f\n", ax, ay, az);
printf("roll=%.2f pitch=%.2f\n");

📟 Example Output
RAW X=320 Y=330 Z=17040
G    ax=0.019 ay=0.020 az=1.020
ANG  roll=0.92 deg  pitch=-1.05 deg


Board flat → angles near 0°.
Tilt → numbers change smoothly.

🎯 Why This Project Matters

Shows real low-level SPI communication

Demonstrates sensor fusion fundamentals using only an accelerometer

Uses SWO debugging, a massively underrated STM32 feature

Clean, small driver code that is easy to extend

Great template for more complex IMU projects

🔮 Extensions / Future Work

Add EMA low-pass filtering to smooth angles

Fuse with a gyroscope (L3GD20) for full 6-DoF

Add UART logging + Python plotting

Display angles on the board’s LEDs

Use the sensor for fall detection / tilt alarms
