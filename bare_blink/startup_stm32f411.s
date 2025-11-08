.syntax unified
.cpu cortex-m4
.thumb

/* -------------------------------------------------------------------------- */
/* Vector table: placed at 0x08000000                                         */
/* -------------------------------------------------------------------------- */
.section .isr_vector,"a",%progbits
.word 0x20020000       /* Initial Stack Pointer (end of 128 KB SRAM) */
.word Reset_Handler     /* Reset Handler address */
.word NMI_Handler
.word HardFault_Handler
.space (48 * 4)         /* reserve space for rest of vectors */

/* -------------------------------------------------------------------------- */
/* Default handlers (infinite loops)                                          */
/* -------------------------------------------------------------------------- */
.text
.global Reset_Handler
.type Reset_Handler, %function

NMI_Handler:
HardFault_Handler:
Default_Handler:
    b Default_Handler

/* -------------------------------------------------------------------------- */
/* External symbols provided by linker.ld                                     */
/* -------------------------------------------------------------------------- */
.extern _sidata  /* Start of .data initialization values in Flash */
.extern _sdata   /* Start of .data section in RAM */
.extern _edata   /* End of .data section in RAM */
.extern _sbss    /* Start of .bss section in RAM */
.extern _ebss    /* End of .bss section in RAM */
.extern main

/* -------------------------------------------------------------------------- */
/* Reset Handler                                                              */
/* -------------------------------------------------------------------------- */
Reset_Handler:
    /* 1. Copy .data from Flash (_sidata) to RAM (_sdata.._edata) */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
copy_data:
    cmp r0, r1
    ittt lt
    ldrlt r3, [r2], #4
    strlt r3, [r0], #4
    blt copy_data

    /* 2. Zero initialize .bss section (_sbss.._ebss) */
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
clear_bss:
    cmp r0, r1
    it lt
    strlt r2, [r0], #4
    blt clear_bss

    /* 3. Call main() */
    bl main

hang:
    b hang

