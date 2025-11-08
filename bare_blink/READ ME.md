# STM32F411 Bare-Metal Boot Process  
### Understanding how the code is initially loaded and executed

This project demonstrates a true bare-metal startup on an STM32F411, before HAL and Cube MX
It explains how the linker script and startup file work together to bring the microcontroller from reset to a running `main()`.

---

## Boot Sequence

When the STM32 powers on or resets, the Cortex-M core starts execution from address `0x0800 0000`, which maps to on-chip Flash.

| Address | Contents | Purpose |
|----------|-----------|----------|
| `0x0800 0000` | Initial Stack Pointer value | Top of SRAM (e.g. `0x2002 0000`) |
| `0x0800 0004` | Address of `Reset_Handler` | Entry point after reset |

These two 32-bit words form the vector table.  
The CPU:
1. Loads the stack pointer from `0x0800 0000`
2. Jumps to the reset handler at `0x0800 0004`

---

## Memory Regions

The STM32F411 has two key regions the linker must describe:

```ld
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}
```

## Sections Defined by the Linker

C programs are divided into sections, each with its own purpose and lifetime.

| Section | Lives in | Contains | Lifetime |
|----------|-----------|-----------|-----------|
| .text | Flash | Program code (functions) | Permanent |
| .rodata | Flash | Constant data (`const`) | Permanent |
| .data | RAM (copied from Flash) | Globals with initial values | Runtime |
| .bss | RAM | Uninitialized globals (set to 0) | Runtime |
| Stack | RAM (top) | Local variables, call frames | Runtime |

The linker script places these sections and defines symbols such as  
`_sidata`, `_sdata`, `_edata`, `_sbss`, `_ebss` — address markers used by the startup code.

Think of the linker as the architect: it designs the memory layout but does not move anything yet.

---

## Startup File — The "Mover and Cleaner"

The startup assembly file (`startup_stm32f411.s`) runs immediately after reset.  
It uses the linker-defined symbols to prepare RAM so that C code can execute safely.

### ### Step 1 — Copy .data

Variables with initial values (for example, `int counter = 5;`) are stored in Flash right after the program code.  
At startup, they are copied into RAM:

```
ldr r0, =_sdata     ; destination (RAM)
ldr r1, =_edata
ldr r2, =_sidata    ; source (Flash)
copy_data:
    cmp r0, r1
    ldrlt r3, [r2], #4
    strlt r3, [r0], #4
    blt copy_data
```
### Step 2 — Clear .bss

Uninitialized globals must start as zero.
The startup file explicitly clears this region:
```
ldr r0, =_sbss
ldr r1, =_ebss
movs r2, #0
clear_bss:
    cmp r0, r1
    strlt r2, [r0], #4
    blt clear_bss
```
### Step 3 — Call main()

Once RAM is ready, the CPU jumps into the C program:

  bl main

If main() ever returns, the CPU loops forever.


### Why Flash and RAM Are Both Used

Flash keeps code and initial values even when power is off.  
RAM is fast and writable, needed for active variables.

Therefore, .data exists twice:

  - A template in Flash (initial values)
  - A working copy in RAM (used at runtime)

The startup file copies from Flash → RAM before main() runs.


### What .bss Is and Why It Is Cleared

.bss means Block Started by Symbol.  
It holds uninitialized global or static variables.  
C requires these to start at zero, but after reset, RAM may contain random data.  
The startup file zeroes this entire region so the program starts predictably.

---

### Stack Layout

RAM grows upward from 0x20000000;  
the stack pointer starts at the top (0x20020000) and grows downward.

  | Higher addresses  ← Stack (grows down)|
  |------------------|
  |                  |
  |   unused space   | 
  |                  |  
  | .bss (zeroed)    |
  | .data (copied)   |
  | Lower addresses  |

This layout keeps the stack and data regions separate and prevents overlap.

