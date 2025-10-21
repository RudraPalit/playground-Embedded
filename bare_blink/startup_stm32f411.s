.syntax unified
.cpu cortex-m4
.thumb

/* Vector table */
.section .isr_vector,"a",%progbits
.word 0x20020000          /* Initial stack pointer (top of 128 KB SRAM) */
.word Reset_Handler       /* Reset handler address */

/* Reset handler */
.text
.global Reset_Handler
Reset_Handler:
    bl main               /* Branch to C main() */
loop:
    b loop                /* If main ever returns, loop forever */

