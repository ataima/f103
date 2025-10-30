/**
 * @file gpio.h
 * @brief Configurazione e gestione GPIO per controller CNC 3 assi
 * @details
 * Questo modulo gestisce tutti i pin GPIO del controller CNC basato su STM32F103C8T6.
 * Include la mappatura completa dei pin per:
 * - Motori passo-passo (STEP, DIR, ENABLE)
 * - Encoder rotativi (canali A/B per 3 assi)
 * - Finecorsa (MIN/MAX per 3 assi)
 * - Display LCD 4x20 (modalità 4-bit)
 * - Comunicazione UART3 (ESP32)
 * - LED di stato
 *
 * FASE 1: Tutti i pin configurati come GPIO standard
 * FASE 2: STEP e ENCODER passeranno in modalità Alternate Function con timer HW
 */

#ifndef GPIO_H
#define GPIO_H

#include "common.h"

/* ============================================================================
 * DEFINIZIONI PIN - MOTORI PASSO-PASSO
 * ============================================================================
 * I motori stepper richiedono 3 segnali per asse:
 * - STEP: impulsi di passo (FASE 1: GPIO, FASE 2: PWM da timer)
 * - DIR: direzione di rotazione (sempre GPIO)
 * - ENABLE: abilitazione driver (sempre GPIO, attivo basso)
 */

/* Pin STEP (impulsi di passo) - Per ora GPIO, poi PWM via timer */
#define STEP_X_PORT         GPIOA
#define STEP_X_PIN          0        // PA0 - TIM2_CH1 (20kHz PWM in futuro)

#define STEP_Y_PORT         GPIOA
#define STEP_Y_PIN          1        // PA1 - TIM2_CH2 (20kHz PWM in futuro)

#define STEP_Z_PORT         GPIOB
#define STEP_Z_PIN          0        // PB0 - TIM3_CH3 (20kHz PWM in futuro)

/* Pin DIR (direzione) - Sempre GPIO output */
#define DIR_X_PORT          GPIOB
#define DIR_X_PIN           12       // PB12 - 5V tolerant

#define DIR_Y_PORT          GPIOB
#define DIR_Y_PIN           13       // PB13 - 5V tolerant

#define DIR_Z_PORT          GPIOB
#define DIR_Z_PIN           14       // PB14 - 5V tolerant

/* Pin ENABLE (abilitazione driver) - Sempre GPIO output, attivo LOW */
#define ENABLE_X_PORT       GPIOB
#define ENABLE_X_PIN        3        // PB3

#define ENABLE_Y_PORT       GPIOB
#define ENABLE_Y_PIN        4        // PB4

#define ENABLE_Z_PORT       GPIOB
#define ENABLE_Z_PIN        5        // PB5


/* ============================================================================
 * DEFINIZIONI PIN - ENCODER ROTATIVI
 * ============================================================================
 * Ogni encoder ha 2 canali (A e B) sfasati di 90° per:
 * - Rilevare direzione di rotazione
 * - Contare impulsi con risoluzione x4 (quadratura)
 * FASE 1: GPIO input per test
 * FASE 2: Modalità encoder hardware dei timer
 */

/* Encoder Asse X - TIM3 CH1/CH2 */
#define ENC_X_A_PORT        GPIOA
#define ENC_X_A_PIN         6        // PA6 - TIM3_CH1

#define ENC_X_B_PORT        GPIOA
#define ENC_X_B_PIN         7        // PA7 - TIM3_CH2

/* Encoder Asse Y - TIM4 CH1/CH2 */
#define ENC_Y_A_PORT        GPIOB
#define ENC_Y_A_PIN         6        // PB6 - TIM4_CH1, 5V tolerant

#define ENC_Y_B_PORT        GPIOB
#define ENC_Y_B_PIN         7        // PB7 - TIM4_CH2, 5V tolerant

/* Encoder Asse Z - TIM1 CH1/CH2 */
#define ENC_Z_A_PORT        GPIOA
#define ENC_Z_A_PIN         8        // PA8 - TIM1_CH1, 5V tolerant

#define ENC_Z_B_PORT        GPIOA
#define ENC_Z_B_PIN         9        // PA9 - TIM1_CH2, 5V tolerant (anche UART1 TX)


/* ============================================================================
 * DEFINIZIONI PIN - FINECORSA (LIMIT SWITCHES)
 * ============================================================================
 * Ogni asse ha 2 finecorsa (MIN e MAX) per delimitare l'area di lavoro.
 * Configurati come input con pull-up interno.
 * FASE 1: GPIO input con polling
 * FASE 2: Interrupt EXTI per risposta immediata
 */

/* Finecorsa Asse X */
#define X_MIN_PORT          GPIOB
#define X_MIN_PIN           8        // PB8 - EXTI8, 5V tolerant

#define X_MAX_PORT          GPIOB
#define X_MAX_PIN           9        // PB9 - EXTI9, 5V tolerant

/* Finecorsa Asse Y */
#define Y_MIN_PORT          GPIOA
#define Y_MIN_PIN           2        // PA2 - EXTI2

#define Y_MAX_PORT          GPIOA
#define Y_MAX_PIN           3        // PA3 - EXTI3

/* Finecorsa Asse Z */
#define Z_MIN_PORT          GPIOA
#define Z_MIN_PIN           4        // PA4 - EXTI4

#define Z_MAX_PORT          GPIOA
#define Z_MAX_PIN           5        // PA5 - EXTI5


/* ============================================================================
 * DEFINIZIONI PIN - I/O GENERICI (IO_1 a IO_6)
 * ============================================================================
 * 6 pin GPIO configurabili per scopi generici (espansione futura).
 * Possono essere utilizzati per LCD, relay, LED aggiuntivi, ecc.
 * Tutti configurati come output push-pull.
 */

#define IO_1_PORT           GPIOC
#define IO_1_PIN            14       // PC14 - I/O generico 1

#define IO_2_PORT           GPIOC
#define IO_2_PIN            15       // PC15 - I/O generico 2

#define IO_3_PORT           GPIOB
#define IO_3_PIN            1        // PB1 - I/O generico 3

#define IO_4_PORT           GPIOA
#define IO_4_PIN            10       // PA10 - I/O generico 4, 5V tolerant

#define IO_5_PORT           GPIOA
#define IO_5_PIN            15       // PA15 - I/O generico 5, 5V tolerant

#define IO_6_PORT           GPIOB
#define IO_6_PIN            15       // PB15 - I/O generico 6, 5V tolerant


/* ============================================================================
 * DEFINIZIONI PIN - COMUNICAZIONE UART3
 * ============================================================================
 * Comunicazione seriale con modulo ESP32 per WiFi/Bluetooth.
 * UART3 con pin 5V tolerant per compatibilità con ESP32 a 3.3V.
 */

#define UART3_TX_PORT       GPIOB
#define UART3_TX_PIN        10       // PB10 - USART3 TX, 5V tolerant

#define UART3_RX_PORT       GPIOB
#define UART3_RX_PIN        11       // PB11 - USART3 RX, 5V tolerant


/* ============================================================================
 * DEFINIZIONI PIN - LED DI STATO
 * ============================================================================
 * LED integrato sulla Blue Pill, collegato a PC13.
 * Attivo LOW (scrive 0 per accendere).
 */

#define LED_STATUS_PORT     GPIOC
#define LED_STATUS_PIN      13       // PC13 - LED onboard, attivo LOW


/* ============================================================================
 * CODICI DI RITORNO
 * ============================================================================
 */
#define GPIO_OK             0
#define GPIO_ERROR          -1


/* ============================================================================
 * FUNZIONI PUBBLICHE - INIZIALIZZAZIONE PER GRUPPI
 * ============================================================================
 */

/**
 * @brief Inizializza i pin GPIO per i motori passo-passo
 * @details Configura:
 *          - STEP_X/Y/Z come output push-pull a 50MHz (per ora GPIO, poi PWM)
 *          - DIR_X/Y/Z come output push-pull a 50MHz
 *          - ENABLE_X/Y/Z come output push-pull, inizialmente HIGH (motori disabilitati)
 * @return GPIO_OK se successo, GPIO_ERROR altrimenti
 */
int gpio_init_motors(void);

/**
 * @brief Inizializza i pin GPIO per gli encoder rotativi
 * @details Configura tutti i canali A/B degli encoder come input con pull-up.
 *          FASE 1: GPIO input normale
 *          FASE 2: Verranno riconfigurati come Alternate Function per timer encoder
 * @return GPIO_OK se successo, GPIO_ERROR altrimenti
 */
int gpio_init_encoders(void);

/**
 * @brief Inizializza i pin GPIO per i finecorsa (limit switches)
 * @details Configura tutti i pin MIN/MAX come input con pull-up interno.
 *          Assumendo finecorsa normalmente aperti (NO) che vanno a GND quando premuti.
 *          FASE 1: Lettura via polling
 *          FASE 2: Verranno aggiunti interrupt EXTI
 * @return GPIO_OK se successo, GPIO_ERROR altrimenti
 */
int gpio_init_limit_switches(void);

/**
 * @brief Inizializza i pin GPIO generici (IO_1 a IO_6)
 * @details Configura tutti i pin IO_x come output push-pull a 2MHz.
 *          Questi pin possono essere utilizzati per LCD, relay, LED, ecc.
 *          Tutti inizializzati a LOW.
 * @return GPIO_OK se successo, GPIO_ERROR altrimenti
 */
int gpio_init_io(void);

/**
 * @brief Inizializza il pin del LED di stato
 * @details Configura PC13 come output push-pull a 2MHz, inizialmente spento (HIGH).
 * @return GPIO_OK se successo, GPIO_ERROR altrimenti
 */
int gpio_init_led(void);

/**
 * @brief Inizializza TUTTI i GPIO del sistema CNC
 * @details Chiama in sequenza tutte le funzioni di inizializzazione per gruppi:
 *          1. Clock GPIOA/B/C
 *          2. Motori
 *          3. Encoder
 *          4. Finecorsa
 *          5. I/O generici
 *          6. LED
 * @note I pin UART3 sono gestiti direttamente da uart_init()
 * @return GPIO_OK se tutte le inizializzazioni hanno successo, GPIO_ERROR altrimenti
 */
int gpio_init_all(void);


/* ============================================================================
 * FUNZIONI HELPER - CONTROLLO PIN
 * ============================================================================
 */

/**
 * @brief Imposta un pin GPIO a livello HIGH
 * @param port Puntatore al registro GPIO (es. GPIOA)
 * @param pin Numero del pin (0-15)
 */
static inline void gpio_set(volatile uint32_t *port, uint8_t pin) {
    // Usa BSRR (Bit Set Reset Register) per operazione atomica
    port[4] = (1U << pin);  // BSRR offset = 0x10 / 4 = 4
}

/**
 * @brief Imposta un pin GPIO a livello LOW
 * @param port Puntatore al registro GPIO (es. GPIOA)
 * @param pin Numero del pin (0-15)
 */
static inline void gpio_reset(volatile uint32_t *port, uint8_t pin) {
    // Usa BSRR (Bit Set Reset Register) per operazione atomica
    port[4] = (1U << (pin + 16));  // BSRR bit 16-31 per reset
}

/**
 * @brief Legge lo stato di un pin GPIO
 * @param port Puntatore al registro GPIO (es. GPIOA)
 * @param pin Numero del pin (0-15)
 * @return 1 se HIGH, 0 se LOW
 */
static inline uint8_t gpio_read(volatile uint32_t *port, uint8_t pin) {
    // Usa IDR (Input Data Register)
    return (port[2] & (1U << pin)) ? 1 : 0;  // IDR offset = 0x08 / 4 = 2
}

/**
 * @brief Toggle (inverte) lo stato di un pin GPIO
 * @param port Puntatore al registro GPIO (es. GPIOA)
 * @param pin Numero del pin (0-15)
 */
static inline void gpio_toggle(volatile uint32_t *port, uint8_t pin) {
    // Legge ODR e inverte il bit
    port[3] ^= (1U << pin);  // ODR offset = 0x0C / 4 = 3
}

#endif /* GPIO_H */
