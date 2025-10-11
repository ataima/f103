/**
 ******************************************************************************
 * @file           : stm32f103_regs.h
 * @brief          : Definizioni registri STM32F103C8T6
 * @author         : STM32F103 Project
 ******************************************************************************
 * @attention
 *
 * Questo file contiene TUTTE le definizioni dei registri del microcontrollore
 * STM32F103C8T6 (memory-mapped peripherals).
 *
 * EMULAZIONE SOFTWARE:
 * Per emulare il sistema in software, è possibile ridefinire PERIPH_BASE
 * prima di includere questo header, e tutti i registri saranno riposizionati.
 *
 * Esempio:
 *   #define PERIPH_BASE  0x80000000UL  // Area emulata
 *   #include "stm32f103_regs.h"
 *
 ******************************************************************************
 */

#ifndef STM32F103_REGS_H_
#define STM32F103_REGS_H_

#include <stdint.h>

/* =============================================================================
 * BASE ADDRESSES - Possono essere ridefinite per emulazione
 * ============================================================================= */

#ifndef PERIPH_BASE
#define PERIPH_BASE           0x40000000UL  /* Base periferici */
#endif

#ifndef FLASH_BASE
#define FLASH_BASE            0x08000000UL  /* Base FLASH memory */
#endif

#ifndef SRAM_BASE
#define SRAM_BASE             0x20000000UL  /* Base SRAM */
#endif

/* APB1 Bus (36 MHz max) */
#define APB1PERIPH_BASE       (PERIPH_BASE + 0x00000000UL)

/* APB2 Bus (72 MHz max) */
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)

/* AHB Bus (72 MHz) */
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)

/* =============================================================================
 * PERIFERICI APB1 (36 MHz MAX)
 * ============================================================================= */

/* Timer 2-7 */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400UL)

/* RTC */
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800UL)

/* Watchdog */
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000UL)

/* SPI/I2S */
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)

/* USART */
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000UL)

/* I2C */
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)

/* USB */
#define USB_BASE              (APB1PERIPH_BASE + 0x5C00UL)

/* CAN */
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400UL)

/* Backup registers */
#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00UL)

/* Power control */
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)

/* DAC */
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400UL)

/* =============================================================================
 * PERIFERICI APB2 (72 MHz MAX)
 * ============================================================================= */

/* AFIO (Alternate Function I/O) */
#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000UL)

/* EXTI (External Interrupt) */
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400UL)

/* GPIO */
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800UL)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00UL)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000UL)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400UL)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800UL)

/* ADC */
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400UL)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800UL)

/* TIM1 (Advanced timer) */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00UL)

/* SPI1 */
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)

/* USART1 */
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800UL)

/* =============================================================================
 * PERIFERICI AHB (72 MHz)
 * ============================================================================= */

/* DMA */
#define DMA1_BASE             (AHBPERIPH_BASE + 0x0000UL)
#define DMA2_BASE             (AHBPERIPH_BASE + 0x0400UL)

/* RCC (Reset and Clock Control) */
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000UL)

/* Flash Interface */
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x2000UL)

/* CRC */
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000UL)

/* =============================================================================
 * RCC - RESET AND CLOCK CONTROL
 * ============================================================================= */

/* Registri RCC */
#define RCC_CR                (*(volatile u32 *)(RCC_BASE + 0x00))  /* Control Register */
#define RCC_CFGR              (*(volatile u32 *)(RCC_BASE + 0x04))  /* Configuration Register */
#define RCC_CIR               (*(volatile u32 *)(RCC_BASE + 0x08))  /* Clock Interrupt Register */
#define RCC_APB2RSTR          (*(volatile u32 *)(RCC_BASE + 0x0C))  /* APB2 Peripheral Reset Register */
#define RCC_APB1RSTR          (*(volatile u32 *)(RCC_BASE + 0x10))  /* APB1 Peripheral Reset Register */
#define RCC_AHBENR            (*(volatile u32 *)(RCC_BASE + 0x14))  /* AHB Peripheral Clock Enable Register */
#define RCC_APB2ENR           (*(volatile u32 *)(RCC_BASE + 0x18))  /* APB2 Peripheral Clock Enable Register */
#define RCC_APB1ENR           (*(volatile u32 *)(RCC_BASE + 0x1C))  /* APB1 Peripheral Clock Enable Register */
#define RCC_BDCR              (*(volatile u32 *)(RCC_BASE + 0x20))  /* Backup Domain Control Register */
#define RCC_CSR               (*(volatile u32 *)(RCC_BASE + 0x24))  /* Control/Status Register */

/* RCC_CR bits */
#define RCC_CR_HSION          BIT(0)    /* Internal High Speed clock enable */
#define RCC_CR_HSIRDY         BIT(1)    /* Internal High Speed clock ready */
#define RCC_CR_HSITRIM_POS    (3)       /* Internal High Speed clock trimming */
#define RCC_CR_HSICAL_POS     (8)       /* Internal High Speed clock calibration */
#define RCC_CR_HSEON          BIT(16)   /* External High Speed clock enable */
#define RCC_CR_HSERDY         BIT(17)   /* External High Speed clock ready */
#define RCC_CR_HSEBYP         BIT(18)   /* External High Speed clock bypass */
#define RCC_CR_CSSON          BIT(19)   /* Clock Security System enable */
#define RCC_CR_PLLON          BIT(24)   /* PLL enable */
#define RCC_CR_PLLRDY         BIT(25)   /* PLL clock ready */

/* RCC_CFGR bits */
#define RCC_CFGR_SW_HSI       (0UL << 0)    /* HSI selected as system clock */
#define RCC_CFGR_SW_HSE       (1UL << 0)    /* HSE selected as system clock */
#define RCC_CFGR_SW_PLL       (2UL << 0)    /* PLL selected as system clock */
#define RCC_CFGR_SW_MASK      (3UL << 0)

#define RCC_CFGR_SWS_HSI      (0UL << 2)    /* HSI used as system clock */
#define RCC_CFGR_SWS_HSE      (1UL << 2)    /* HSE used as system clock */
#define RCC_CFGR_SWS_PLL      (2UL << 2)    /* PLL used as system clock */
#define RCC_CFGR_SWS_MASK     (3UL << 2)

#define RCC_CFGR_HPRE_DIV1    (0UL << 4)    /* HCLK = SYSCLK */
#define RCC_CFGR_HPRE_DIV2    (8UL << 4)    /* HCLK = SYSCLK / 2 */
#define RCC_CFGR_HPRE_DIV4    (9UL << 4)    /* HCLK = SYSCLK / 4 */
#define RCC_CFGR_HPRE_DIV8    (10UL << 4)   /* HCLK = SYSCLK / 8 */
#define RCC_CFGR_HPRE_DIV16   (11UL << 4)   /* HCLK = SYSCLK / 16 */
#define RCC_CFGR_HPRE_MASK    (0xFUL << 4)

#define RCC_CFGR_PPRE1_DIV1   (0UL << 8)    /* PCLK1 = HCLK */
#define RCC_CFGR_PPRE1_DIV2   (4UL << 8)    /* PCLK1 = HCLK / 2 */
#define RCC_CFGR_PPRE1_DIV4   (5UL << 8)    /* PCLK1 = HCLK / 4 */
#define RCC_CFGR_PPRE1_DIV8   (6UL << 8)    /* PCLK1 = HCLK / 8 */
#define RCC_CFGR_PPRE1_DIV16  (7UL << 8)    /* PCLK1 = HCLK / 16 */
#define RCC_CFGR_PPRE1_MASK   (7UL << 8)

#define RCC_CFGR_PPRE2_DIV1   (0UL << 11)   /* PCLK2 = HCLK */
#define RCC_CFGR_PPRE2_DIV2   (4UL << 11)   /* PCLK2 = HCLK / 2 */
#define RCC_CFGR_PPRE2_DIV4   (5UL << 11)   /* PCLK2 = HCLK / 4 */
#define RCC_CFGR_PPRE2_DIV8   (6UL << 11)   /* PCLK2 = HCLK / 8 */
#define RCC_CFGR_PPRE2_DIV16  (7UL << 11)   /* PCLK2 = HCLK / 16 */
#define RCC_CFGR_PPRE2_MASK   (7UL << 11)

#define RCC_CFGR_ADCPRE_DIV2  (0UL << 14)   /* ADCCLK = PCLK2 / 2 */
#define RCC_CFGR_ADCPRE_DIV4  (1UL << 14)   /* ADCCLK = PCLK2 / 4 */
#define RCC_CFGR_ADCPRE_DIV6  (2UL << 14)   /* ADCCLK = PCLK2 / 6 */
#define RCC_CFGR_ADCPRE_DIV8  (3UL << 14)   /* ADCCLK = PCLK2 / 8 */
#define RCC_CFGR_ADCPRE_MASK  (3UL << 14)

#define RCC_CFGR_PLLSRC_HSI_DIV2  (0UL << 16)  /* PLL input = HSI / 2 */
#define RCC_CFGR_PLLSRC_HSE       (1UL << 16)  /* PLL input = HSE */

#define RCC_CFGR_PLLXTPRE_HSE     (0UL << 17)  /* HSE not divided for PLL */
#define RCC_CFGR_PLLXTPRE_HSE_DIV2 (1UL << 17) /* HSE / 2 for PLL */

#define RCC_CFGR_PLLMUL2      (0UL << 18)   /* PLL x 2 */
#define RCC_CFGR_PLLMUL3      (1UL << 18)   /* PLL x 3 */
#define RCC_CFGR_PLLMUL4      (2UL << 18)   /* PLL x 4 */
#define RCC_CFGR_PLLMUL5      (3UL << 18)   /* PLL x 5 */
#define RCC_CFGR_PLLMUL6      (4UL << 18)   /* PLL x 6 */
#define RCC_CFGR_PLLMUL7      (5UL << 18)   /* PLL x 7 */
#define RCC_CFGR_PLLMUL8      (6UL << 18)   /* PLL x 8 */
#define RCC_CFGR_PLLMUL9      (7UL << 18)   /* PLL x 9 */
#define RCC_CFGR_PLLMUL10     (8UL << 18)   /* PLL x 10 */
#define RCC_CFGR_PLLMUL11     (9UL << 18)   /* PLL x 11 */
#define RCC_CFGR_PLLMUL12     (10UL << 18)  /* PLL x 12 */
#define RCC_CFGR_PLLMUL13     (11UL << 18)  /* PLL x 13 */
#define RCC_CFGR_PLLMUL14     (12UL << 18)  /* PLL x 14 */
#define RCC_CFGR_PLLMUL15     (13UL << 18)  /* PLL x 15 */
#define RCC_CFGR_PLLMUL16     (14UL << 18)  /* PLL x 16 */
#define RCC_CFGR_PLLMUL_MASK  (0xFUL << 18)

#define RCC_CFGR_MCO_NOCLK    (0UL << 24)   /* No clock output */
#define RCC_CFGR_MCO_SYSCLK   (4UL << 24)   /* System clock output */
#define RCC_CFGR_MCO_HSI      (5UL << 24)   /* HSI clock output */
#define RCC_CFGR_MCO_HSE      (6UL << 24)   /* HSE clock output */
#define RCC_CFGR_MCO_PLL_DIV2 (7UL << 24)   /* PLL / 2 clock output */

/* RCC_APB2ENR bits */
#define RCC_APB2ENR_AFIOEN    BIT(0)    /* Alternate Function IO clock enable */
#define RCC_APB2ENR_IOPAEN    BIT(2)    /* IO port A clock enable */
#define RCC_APB2ENR_IOPBEN    BIT(3)    /* IO port B clock enable */
#define RCC_APB2ENR_IOPCEN    BIT(4)    /* IO port C clock enable */
#define RCC_APB2ENR_IOPDEN    BIT(5)    /* IO port D clock enable */
#define RCC_APB2ENR_IOPEEN    BIT(6)    /* IO port E clock enable */
#define RCC_APB2ENR_ADC1EN    BIT(9)    /* ADC1 clock enable */
#define RCC_APB2ENR_ADC2EN    BIT(10)   /* ADC2 clock enable */
#define RCC_APB2ENR_TIM1EN    BIT(11)   /* TIM1 clock enable */
#define RCC_APB2ENR_SPI1EN    BIT(12)   /* SPI1 clock enable */
#define RCC_APB2ENR_USART1EN  BIT(14)   /* USART1 clock enable */

/* RCC_APB1ENR bits */
#define RCC_APB1ENR_TIM2EN    BIT(0)    /* TIM2 clock enable */
#define RCC_APB1ENR_TIM3EN    BIT(1)    /* TIM3 clock enable */
#define RCC_APB1ENR_TIM4EN    BIT(2)    /* TIM4 clock enable */
#define RCC_APB1ENR_TIM5EN    BIT(3)    /* TIM5 clock enable */
#define RCC_APB1ENR_TIM6EN    BIT(4)    /* TIM6 clock enable */
#define RCC_APB1ENR_TIM7EN    BIT(5)    /* TIM7 clock enable */
#define RCC_APB1ENR_WWDGEN    BIT(11)   /* Window Watchdog clock enable */
#define RCC_APB1ENR_SPI2EN    BIT(14)   /* SPI2 clock enable */
#define RCC_APB1ENR_SPI3EN    BIT(15)   /* SPI3 clock enable */
#define RCC_APB1ENR_USART2EN  BIT(17)   /* USART2 clock enable */
#define RCC_APB1ENR_USART3EN  BIT(18)   /* USART3 clock enable */
#define RCC_APB1ENR_UART4EN   BIT(19)   /* UART4 clock enable */
#define RCC_APB1ENR_UART5EN   BIT(20)   /* UART5 clock enable */
#define RCC_APB1ENR_I2C1EN    BIT(21)   /* I2C1 clock enable */
#define RCC_APB1ENR_I2C2EN    BIT(22)   /* I2C2 clock enable */
#define RCC_APB1ENR_USBEN     BIT(23)   /* USB clock enable */
#define RCC_APB1ENR_CAN1EN    BIT(25)   /* CAN1 clock enable */
#define RCC_APB1ENR_BKPEN     BIT(27)   /* Backup interface clock enable */
#define RCC_APB1ENR_PWREN     BIT(28)   /* Power interface clock enable */
#define RCC_APB1ENR_DACEN     BIT(29)   /* DAC clock enable */

/* =============================================================================
 * FLASH INTERFACE
 * ============================================================================= */

#define FLASH_ACR             (*(volatile u32 *)(FLASH_R_BASE + 0x00))  /* Access Control Register */
#define FLASH_KEYR            (*(volatile u32 *)(FLASH_R_BASE + 0x04))  /* Key Register */
#define FLASH_OPTKEYR         (*(volatile u32 *)(FLASH_R_BASE + 0x08))  /* Option Key Register */
#define FLASH_SR              (*(volatile u32 *)(FLASH_R_BASE + 0x0C))  /* Status Register */
#define FLASH_CR              (*(volatile u32 *)(FLASH_R_BASE + 0x10))  /* Control Register */
#define FLASH_AR              (*(volatile u32 *)(FLASH_R_BASE + 0x14))  /* Address Register */
#define FLASH_OBR             (*(volatile u32 *)(FLASH_R_BASE + 0x1C))  /* Option Bytes Register */
#define FLASH_WRPR            (*(volatile u32 *)(FLASH_R_BASE + 0x20))  /* Write Protection Register */

/* FLASH_ACR bits */
#define FLASH_ACR_LATENCY_0   (0UL << 0)    /* 0 wait states (SYSCLK <= 24 MHz) */
#define FLASH_ACR_LATENCY_1   (1UL << 0)    /* 1 wait state  (24 MHz < SYSCLK <= 48 MHz) */
#define FLASH_ACR_LATENCY_2   (2UL << 0)    /* 2 wait states (48 MHz < SYSCLK <= 72 MHz) */
#define FLASH_ACR_LATENCY_MASK (7UL << 0)

#define FLASH_ACR_HLFCYA      BIT(3)        /* Flash half cycle access enable */
#define FLASH_ACR_PRFTBE      BIT(4)        /* Prefetch buffer enable */
#define FLASH_ACR_PRFTBS      BIT(5)        /* Prefetch buffer status (read-only) */

/* =============================================================================
 * GPIO
 * ============================================================================= */

/* Registri GPIO (esempio per GPIOA, valido per tutti) */
#define GPIO_CRL(base)        (*(volatile u32 *)((base) + 0x00))  /* Config Register Low (pin 0-7) */
#define GPIO_CRH(base)        (*(volatile u32 *)((base) + 0x04))  /* Config Register High (pin 8-15) */
#define GPIO_IDR(base)        (*(volatile u32 *)((base) + 0x08))  /* Input Data Register */
#define GPIO_ODR(base)        (*(volatile u32 *)((base) + 0x0C))  /* Output Data Register */
#define GPIO_BSRR(base)       (*(volatile u32 *)((base) + 0x10))  /* Bit Set/Reset Register */
#define GPIO_BRR(base)        (*(volatile u32 *)((base) + 0x14))  /* Bit Reset Register */
#define GPIO_LCKR(base)       (*(volatile u32 *)((base) + 0x18))  /* Lock Register */

/* GPIO Specific registers */
#define GPIOA_CRL             GPIO_CRL(GPIOA_BASE)
#define GPIOA_CRH             GPIO_CRH(GPIOA_BASE)
#define GPIOA_IDR             GPIO_IDR(GPIOA_BASE)
#define GPIOA_ODR             GPIO_ODR(GPIOA_BASE)
#define GPIOA_BSRR            GPIO_BSRR(GPIOA_BASE)
#define GPIOA_BRR             GPIO_BRR(GPIOA_BASE)

#define GPIOB_CRL             GPIO_CRL(GPIOB_BASE)
#define GPIOB_CRH             GPIO_CRH(GPIOB_BASE)
#define GPIOB_IDR             GPIO_IDR(GPIOB_BASE)
#define GPIOB_ODR             GPIO_ODR(GPIOB_BASE)
#define GPIOB_BSRR            GPIO_BSRR(GPIOB_BASE)
#define GPIOB_BRR             GPIO_BRR(GPIOB_BASE)

#define GPIOC_CRL             GPIO_CRL(GPIOC_BASE)
#define GPIOC_CRH             GPIO_CRH(GPIOC_BASE)
#define GPIOC_IDR             GPIO_IDR(GPIOC_BASE)
#define GPIOC_ODR             GPIO_ODR(GPIOC_BASE)
#define GPIOC_BSRR            GPIO_BSRR(GPIOC_BASE)
#define GPIOC_BRR             GPIO_BRR(GPIOC_BASE)

/* GPIO Mode definitions (per CRL/CRH) */
#define GPIO_MODE_INPUT       0x0   /* Input mode */
#define GPIO_MODE_OUTPUT_10   0x1   /* Output 10 MHz */
#define GPIO_MODE_OUTPUT_2    0x2   /* Output 2 MHz */
#define GPIO_MODE_OUTPUT_50   0x3   /* Output 50 MHz */

#define GPIO_CNF_INPUT_ANALOG     (0x0 << 2)  /* Analog input */
#define GPIO_CNF_INPUT_FLOATING   (0x1 << 2)  /* Floating input */
#define GPIO_CNF_INPUT_PULLUPDOWN (0x2 << 2)  /* Input with pull-up/down */
#define GPIO_CNF_OUTPUT_PUSHPULL  (0x0 << 2)  /* Output push-pull */
#define GPIO_CNF_OUTPUT_OPENDRAIN (0x1 << 2)  /* Output open-drain */
#define GPIO_CNF_OUTPUT_AF_PP     (0x2 << 2)  /* Alternate function push-pull */
#define GPIO_CNF_OUTPUT_AF_OD     (0x3 << 2)  /* Alternate function open-drain */

/* =============================================================================
 * USART/UART
 * ============================================================================= */

/* Registri USART (validi per USART1/2/3, UART4/5) */
#define USART_SR(base)        (*(volatile u32 *)((base) + 0x00))  /* Status Register */
#define USART_DR(base)        (*(volatile u32 *)((base) + 0x04))  /* Data Register */
#define USART_BRR(base)       (*(volatile u32 *)((base) + 0x08))  /* Baud Rate Register */
#define USART_CR1(base)       (*(volatile u32 *)((base) + 0x0C))  /* Control Register 1 */
#define USART_CR2(base)       (*(volatile u32 *)((base) + 0x10))  /* Control Register 2 */
#define USART_CR3(base)       (*(volatile u32 *)((base) + 0x14))  /* Control Register 3 */
#define USART_GTPR(base)      (*(volatile u32 *)((base) + 0x18))  /* Guard time and prescaler */

/* USART_SR bits */
#define USART_SR_PE           BIT(0)    /* Parity error */
#define USART_SR_FE           BIT(1)    /* Framing error */
#define USART_SR_NE           BIT(2)    /* Noise error */
#define USART_SR_ORE          BIT(3)    /* Overrun error */
#define USART_SR_IDLE         BIT(4)    /* IDLE line detected */
#define USART_SR_RXNE         BIT(5)    /* Read data register not empty */
#define USART_SR_TC           BIT(6)    /* Transmission complete */
#define USART_SR_TXE          BIT(7)    /* Transmit data register empty */
#define USART_SR_LBD          BIT(8)    /* LIN break detection */
#define USART_SR_CTS          BIT(9)    /* CTS flag */

/* USART_CR1 bits */
#define USART_CR1_SBK         BIT(0)    /* Send break */
#define USART_CR1_RWU         BIT(1)    /* Receiver wakeup */
#define USART_CR1_RE          BIT(2)    /* Receiver enable */
#define USART_CR1_TE          BIT(3)    /* Transmitter enable */
#define USART_CR1_IDLEIE      BIT(4)    /* IDLE interrupt enable */
#define USART_CR1_RXNEIE      BIT(5)    /* RXNE interrupt enable */
#define USART_CR1_TCIE        BIT(6)    /* Transmission complete interrupt enable */
#define USART_CR1_TXEIE       BIT(7)    /* TXE interrupt enable */
#define USART_CR1_PEIE        BIT(8)    /* PE interrupt enable */
#define USART_CR1_PS          BIT(9)    /* Parity selection */
#define USART_CR1_PCE         BIT(10)   /* Parity control enable */
#define USART_CR1_WAKE        BIT(11)   /* Wakeup method */
#define USART_CR1_M           BIT(12)   /* Word length (0=8bit, 1=9bit) */
#define USART_CR1_UE          BIT(13)   /* USART enable */

/* =============================================================================
 * SYSTICK TIMER (parte del core ARM Cortex-M3)
 * ============================================================================= */

#define SCS_BASE              (0xE000E000UL)
#define SYSTICK_BASE          (SCS_BASE + 0x0010UL)

#define SYSTICK_CTRL          (*(volatile u32 *)(SYSTICK_BASE + 0x00))  /* Control and Status */
#define SYSTICK_LOAD          (*(volatile u32 *)(SYSTICK_BASE + 0x04))  /* Reload Value */
#define SYSTICK_VAL           (*(volatile u32 *)(SYSTICK_BASE + 0x08))  /* Current Value */
#define SYSTICK_CALIB         (*(volatile u32 *)(SYSTICK_BASE + 0x0C))  /* Calibration */

#define SYSTICK_CTRL_ENABLE   BIT(0)    /* Counter enable */
#define SYSTICK_CTRL_TICKINT  BIT(1)    /* Counting down to zero triggers SysTick exception */
#define SYSTICK_CTRL_CLKSOURCE BIT(2)   /* 0=external clock, 1=processor clock */
#define SYSTICK_CTRL_COUNTFLAG BIT(16)  /* Returns 1 if timer counted to 0 since last read */

/* =============================================================================
 * NVIC - NESTED VECTORED INTERRUPT CONTROLLER
 * ============================================================================= */

#define NVIC_BASE             (SCS_BASE + 0x0100UL)

/* NVIC Interrupt Set Enable Registers (ISER) */
#define NVIC_ISER0            (*(volatile u32 *)(NVIC_BASE + 0x000))
#define NVIC_ISER1            (*(volatile u32 *)(NVIC_BASE + 0x004))
#define NVIC_ISER2            (*(volatile u32 *)(NVIC_BASE + 0x008))

/* NVIC Interrupt Clear Enable Registers (ICER) */
#define NVIC_ICER0            (*(volatile u32 *)(NVIC_BASE + 0x080))
#define NVIC_ICER1            (*(volatile u32 *)(NVIC_BASE + 0x084))
#define NVIC_ICER2            (*(volatile u32 *)(NVIC_BASE + 0x088))

/* NVIC Interrupt Priority Registers (IPR) */
#define NVIC_IPR(n)           (*(volatile u32 *)(NVIC_BASE + 0x300 + (n) * 4))

/* =============================================================================
 * SCB - SYSTEM CONTROL BLOCK
 * ============================================================================= */

#define SCB_BASE              (SCS_BASE + 0x0D00UL)

#define SCB_CPUID             (*(volatile u32 *)(SCB_BASE + 0x00))  /* CPUID Base Register */
#define SCB_ICSR              (*(volatile u32 *)(SCB_BASE + 0x04))  /* Interrupt Control State */
#define SCB_VTOR              (*(volatile u32 *)(SCB_BASE + 0x08))  /* Vector Table Offset */
#define SCB_AIRCR             (*(volatile u32 *)(SCB_BASE + 0x0C))  /* Application Interrupt/Reset Control */
#define SCB_SCR               (*(volatile u32 *)(SCB_BASE + 0x10))  /* System Control Register */
#define SCB_CCR               (*(volatile u32 *)(SCB_BASE + 0x14))  /* Configuration Control */
#define SCB_SHPR1             (*(volatile u32 *)(SCB_BASE + 0x18))  /* System Handler Priority 1 */
#define SCB_SHPR2             (*(volatile u32 *)(SCB_BASE + 0x1C))  /* System Handler Priority 2 */
#define SCB_SHPR3             (*(volatile u32 *)(SCB_BASE + 0x20))  /* System Handler Priority 3 */
#define SCB_SHCSR             (*(volatile u32 *)(SCB_BASE + 0x24))  /* System Handler Control/State */

#endif /* STM32F103_REGS_H_ */
