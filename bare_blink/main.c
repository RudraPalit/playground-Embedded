 #include <stdint.h>

#define PERIPH_BASE     0x40000000UL
#define AHB1_OFFSET     0x00020000UL
#define RCC_OFFSET      0x00003800UL
#define GPIOA_OFFSET    0x00000000UL

#define RCC_BASE        (PERIPH_BASE + AHB1_OFFSET + RCC_OFFSET)
#define GPIOA_BASE      (PERIPH_BASE + AHB1_OFFSET + GPIOA_OFFSET)

#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_ODR       (*(volatile uint32_t *)(GPIOA_BASE + 0x14))

#define RCC_AHB1ENR_GPIOAEN (1U << 0)
#define GPIO_MODER5_OUT     (1U << (5 * 2))
#define GPIO_PIN5           (1U << 5)

void delay(volatile uint32_t t) { while (t--) __asm__("nop"); }

int main(void) {
    /* Enable GPIOA clock */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* Set PA5 as output */
    GPIOA_MODER &= ~(3U << (5 * 2)); // clear mode bits
    GPIOA_MODER |=  (1U << (5 * 2)); // set to output

    /* Blink */
    while (1) {
        GPIOA_ODR ^= GPIO_PIN5;  // toggle LED
        delay(1000000);
    }
}

