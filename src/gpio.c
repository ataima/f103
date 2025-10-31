/**
 * @file gpio.c
 * @brief Implementazione gestione GPIO per controller CNC 3 assi
 * @details
 * Configurazione completa di tutti i GPIO utilizzati nel controller CNC:
 * - 9 pin per motori stepper (STEP, DIR, ENABLE)
 * - 6 pin per encoder rotativi (A/B per 3 assi)
 * - 6 pin per finecorsa (MIN/MAX per 3 assi)
 * - 6 pin I/O generici (espansione futura)
 * - 2 pin UART3 (comunicazione ESP32)
 * - 1 pin LED di stato
 *
 * NOTA: Le configurazioni seguono il manuale di riferimento STM32F103C8T6.
 *       Ogni registro GPIO viene scritto direttamente senza HAL/SPL.
 */
#include "common.h"
#include "gpio.h"
#include "log.h"

/* ============================================================================
 * FUNZIONI PRIVATE - CONFIGURAZIONE BASSO LIVELLO
 * ============================================================================
 */

/**
 * @brief Configura un pin GPIO come output push-pull
 * @param port Puntatore al registro GPIO (es. GPIOA)
 * @param pin Numero del pin (0-15)
 * @param speed Velocità: 0=input, 1=10MHz, 2=2MHz, 3=50MHz
 * @details Utilizza i registri CRL (pin 0-7) o CRH (pin 8-15).
 *          Ogni pin usa 4 bit: [CNF1 CNF0 MODE1 MODE0]
 *          - MODE = 01 (10MHz), 10 (2MHz), 11 (50MHz)
 *          - CNF = 00 (push-pull output)
 */
static void gpio_config_output(volatile uint32_t *port, uint8_t pin, uint8_t speed)
{
    // Determina quale registro usare: CRL (0-7) o CRH (8-15)
    volatile uint32_t *cr = (pin < 8) ? &port[0] : &port[1];  // CRL=offset 0, CRH=offset 1
    uint8_t pos = (pin < 8) ? pin : (pin - 8);                // Posizione nel registro (0-7)
    uint32_t shift = pos * 4;                                  // Ogni pin usa 4 bit

    // Leggi valore corrente
    uint32_t tmp = *cr;

    // Azzera i 4 bit del pin
    tmp &= ~(0xFU << shift);

    // Imposta MODE (bit 0-1) e CNF=00 (bit 2-3) per output push-pull
    // speed: 1=10MHz (01), 2=2MHz (10), 3=50MHz (11)
    tmp |= (speed & 0x3) << shift;

    // Scrivi il registro
    *cr = tmp;
}

/**
 * @brief Configura un pin GPIO come input con pull-up/pull-down
 * @param port Puntatore al registro GPIO (es. GPIOA)
 * @param pin Numero del pin (0-15)
 * @param pull 0=floating, 1=pull-up, 2=pull-down
 * @details Per input:
 *          - MODE = 00 (input)
 *          - CNF = 01 (floating), 10 (pull-up/pull-down)
 *          Il tipo di pull si imposta tramite ODR: 1=pull-up, 0=pull-down
 */
static void gpio_config_input(volatile uint32_t *port, uint8_t pin, uint8_t pull)
{
    // Determina quale registro usare: CRL (0-7) o CRH (8-15)
    volatile uint32_t *cr = (pin < 8) ? &port[0] : &port[1];
    uint8_t pos = (pin < 8) ? pin : (pin - 8);
    uint32_t shift = pos * 4;

    // Leggi valore corrente
    uint32_t tmp = *cr;

    // Azzera i 4 bit del pin
    tmp &= ~(0xFU << shift);

    if (pull == 0) {
        // Input floating: MODE=00, CNF=01
        tmp |= (0x4U << shift);
    } else {
        // Input pull-up/down: MODE=00, CNF=10
        tmp |= (0x8U << shift);

        // Imposta pull-up (1) o pull-down (0) tramite ODR
        if (pull == 1) {
            port[3] |= (1U << pin);   // ODR offset = 0x0C / 4 = 3
        } else {
            port[3] &= ~(1U << pin);
        }
    }

    // Scrivi il registro
    *cr = tmp;
}

/**
 * @brief Configura un pin GPIO come alternate function push-pull
 * @param port Puntatore al registro GPIO (es. GPIOA)
 * @param pin Numero del pin (0-15)
 * @param speed Velocità: 1=10MHz, 2=2MHz, 3=50MHz
 * @details Per alternate function output:
 *          - MODE = 01/10/11 (velocità)
 *          - CNF = 10 (AF push-pull)
 * @note Attualmente non usata ma mantenuta per configurazioni future (timer PWM)
 */
__attribute__((unused)) static void gpio_config_af(volatile uint32_t *port, uint8_t pin, uint8_t speed)
{
    volatile uint32_t *cr = (pin < 8) ? &port[0] : &port[1];
    uint8_t pos = (pin < 8) ? pin : (pin - 8);
    uint32_t shift = pos * 4;

    uint32_t tmp = *cr;
    tmp &= ~(0xFU << shift);

    // MODE = speed, CNF = 10 (AF push-pull)
    tmp |= ((speed & 0x3) | 0x8) << shift;

    *cr = tmp;
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - INIZIALIZZAZIONE PER GRUPPI
 * ============================================================================
 */

/**
 * @brief Inizializza i pin GPIO per i motori passo-passo
 * @details Configura:
 *          - STEP_X/Y/Z: output push-pull a 50MHz (per PWM veloce)
 *          - DIR_X/Y/Z: output push-pull a 50MHz
 *          - ENABLE_X/Y/Z: output push-pull a 2MHz, inizialmente HIGH (disabilitati)
 * @return GPIO_OK
 */
int gpio_init_motors(void)
{
    log_debug("Inizializzazione GPIO motori stepper");

    // STEP pins - Output 50MHz (necessario per PWM 20kHz futuri)
    gpio_config_output(STEP_X_PORT, STEP_X_PIN, 3);  // PA0
    gpio_config_output(STEP_Y_PORT, STEP_Y_PIN, 3);  // PA1
    gpio_config_output(STEP_Z_PORT, STEP_Z_PIN, 3);  // PB0

    // Inizializza STEP a LOW
    gpio_reset(STEP_X_PORT, STEP_X_PIN);
    gpio_reset(STEP_Y_PORT, STEP_Y_PIN);
    gpio_reset(STEP_Z_PORT, STEP_Z_PIN);

    // DIR pins - Output 50MHz
    gpio_config_output(DIR_X_PORT, DIR_X_PIN, 3);    // PB12
    gpio_config_output(DIR_Y_PORT, DIR_Y_PIN, 3);    // PB13
    gpio_config_output(DIR_Z_PORT, DIR_Z_PIN, 3);    // PB14

    // Inizializza DIR a LOW (direzione positiva)
    gpio_reset(DIR_X_PORT, DIR_X_PIN);
    gpio_reset(DIR_Y_PORT, DIR_Y_PIN);
    gpio_reset(DIR_Z_PORT, DIR_Z_PIN);

    // ENABLE pins - Output 2MHz, attivo LOW
    gpio_config_output(ENABLE_X_PORT, ENABLE_X_PIN, 2);  // PB3
    gpio_config_output(ENABLE_Y_PORT, ENABLE_Y_PIN, 2);  // PB4
    gpio_config_output(ENABLE_Z_PORT, ENABLE_Z_PIN, 2);  // PB5

    // Inizializza ENABLE a HIGH (motori disabilitati)
    gpio_set(ENABLE_X_PORT, ENABLE_X_PIN);
    gpio_set(ENABLE_Y_PORT, ENABLE_Y_PIN);
    gpio_set(ENABLE_Z_PORT, ENABLE_Z_PIN);

    log_info("GPIO motori configurati: STEP/DIR/ENABLE per 3 assi");

    return GPIO_OK;
}

/**
 * @brief Inizializza i pin GPIO per gli encoder rotativi
 * @details Configura tutti i canali A/B degli encoder come input con pull-up.
 *          FASE 1: GPIO input normale per test
 *          FASE 2: Verranno riconfigurati come AF per timer in modalità encoder
 * @return GPIO_OK
 */
int gpio_init_encoders(void)
{
    log_debug("Inizializzazione GPIO encoder");

    // Encoder X - PA6/PA7 (TIM3 CH1/CH2)
    gpio_config_input(ENC_X_A_PORT, ENC_X_A_PIN, 1);  // Pull-up
    gpio_config_input(ENC_X_B_PORT, ENC_X_B_PIN, 1);

    // Encoder Y - PB6/PB7 (TIM4 CH1/CH2)
    gpio_config_input(ENC_Y_A_PORT, ENC_Y_A_PIN, 1);
    gpio_config_input(ENC_Y_B_PORT, ENC_Y_B_PIN, 1);

    // Encoder Z - PA8/PA9 (TIM1 CH1/CH2)
    gpio_config_input(ENC_Z_A_PORT, ENC_Z_A_PIN, 1);
    gpio_config_input(ENC_Z_B_PORT, ENC_Z_B_PIN, 1);

    log_info("GPIO encoder configurati: 6 canali (A/B per 3 assi) in input pull-up");

    return GPIO_OK;
}

/**
 * @brief Inizializza i pin GPIO per i finecorsa (limit switches)
 * @details Configura tutti i pin MIN/MAX come input con pull-up interno.
 *          Assumendo finecorsa normalmente aperti (NO) che vanno a GND quando attivati.
 *          FASE 1: Lettura via polling
 *          FASE 2: Verranno aggiunti interrupt EXTI
 * @return GPIO_OK
 */
int gpio_init_limit_switches(void)
{
    log_debug("Inizializzazione GPIO finecorsa");

    // Finecorsa X - PB8/PB9
    gpio_config_input(X_MIN_PORT, X_MIN_PIN, 1);  // Pull-up
    gpio_config_input(X_MAX_PORT, X_MAX_PIN, 1);

    // Finecorsa Y - PA2/PA3
    gpio_config_input(Y_MIN_PORT, Y_MIN_PIN, 1);
    gpio_config_input(Y_MAX_PORT, Y_MAX_PIN, 1);

    // Finecorsa Z - PA4/PA5
    gpio_config_input(Z_MIN_PORT, Z_MIN_PIN, 1);
    gpio_config_input(Z_MAX_PORT, Z_MAX_PIN, 1);

    log_info("GPIO finecorsa configurati: 6 pin (MIN/MAX per 3 assi) in input pull-up");

    return GPIO_OK;
}

/**
 * @brief Inizializza i pin GPIO generici (IO_1 a IO_6)
 * @details Configura tutti i pin IO_x come output push-pull a 2MHz.
 *          Possono essere utilizzati per LCD, relay, LED aggiuntivi, ecc.
 *          Tutti inizializzati a LOW.
 * @return GPIO_OK
 */
int gpio_init_io(void)
{
    log_debug("Inizializzazione GPIO I/O generici");

    // Configura tutti gli I/O come output 2MHz
    gpio_config_output(IO_1_PORT, IO_1_PIN, 2);  // PC14
    gpio_config_output(IO_2_PORT, IO_2_PIN, 2);  // PC15
    gpio_config_output(IO_3_PORT, IO_3_PIN, 2);  // PB1
    gpio_config_output(IO_4_PORT, IO_4_PIN, 2);  // PA10
    gpio_config_output(IO_5_PORT, IO_5_PIN, 2);  // PA15
    gpio_config_output(IO_6_PORT, IO_6_PIN, 2);  // PB15

    // Inizializza tutti a LOW
    gpio_reset(IO_1_PORT, IO_1_PIN);
    gpio_reset(IO_2_PORT, IO_2_PIN);
    gpio_reset(IO_3_PORT, IO_3_PIN);
    gpio_reset(IO_4_PORT, IO_4_PIN);
    gpio_reset(IO_5_PORT, IO_5_PIN);
    gpio_reset(IO_6_PORT, IO_6_PIN);

    log_info("GPIO I/O generici configurati: 6 pin output a 2MHz");

    return GPIO_OK;
}

/**
 * @brief Inizializza il pin del LED di stato
 * @details Configura PC13 come output push-pull a 2MHz, inizialmente spento (HIGH).
 *          Il LED sulla Blue Pill è attivo LOW (0=acceso, 1=spento).
 * @return GPIO_OK
 */
int gpio_init_led(void)
{
    log_debug("Inizializzazione GPIO LED di stato");

    // Configura come output 2MHz
    gpio_config_output(LED_STATUS_PORT, LED_STATUS_PIN, 2);  // PC13

    // Inizializza a HIGH (LED spento, attivo LOW)
    gpio_set(LED_STATUS_PORT, LED_STATUS_PIN);

    log_info("GPIO LED configurato: PC13 output (attivo LOW)");

    return GPIO_OK;
}

/**
 * @brief Inizializza TUTTI i GPIO del sistema CNC
 * @details Abilita i clock per GPIOA/B/C e chiama tutte le funzioni di inizializzazione.
 *          Ordine di inizializzazione:
 *          1. Abilitazione clock RCC per GPIOA, GPIOB, GPIOC
 *          2. Motori stepper
 *          3. Encoder
 *          4. Finecorsa
 *          5. I/O generici
 *          6. UART3
 *          7. LED di stato
 * @return GPIO_OK se successo, GPIO_ERROR in caso di errore
 */
int gpio_init_all(void)
{
    log_info("=== Inizializzazione completa GPIO sistema CNC ===");

    /* -----------------------------------------------------------------------
     * STEP 1: Abilitazione clock GPIO
     * -----------------------------------------------------------------------
     * I clock devono essere abilitati PRIMA di accedere ai registri GPIO.
     * RCC_APB2ENR controlla i clock per le periferiche APB2, inclusi i GPIO.
     * Bit 2 = IOPAEN (GPIOA)
     * Bit 3 = IOPBEN (GPIOB)
     * Bit 4 = IOPCEN (GPIOC)
     */
    log_debug("Abilitazione clock GPIOA/B/C");

    RCC_APB2ENR |= (1U << 2) |  // IOPAEN
                   (1U << 3) |  // IOPBEN
                   (1U << 4);   // IOPCEN

    log_info("Clock GPIO abilitati: GPIOA, GPIOB, GPIOC");

    /* -----------------------------------------------------------------------
     * STEP 2: Inizializzazione gruppi GPIO
     * ----------------------------------------------------------------------- */

    // Motori
    if (gpio_init_motors() != GPIO_OK) {
        log_error("Errore inizializzazione GPIO motori");
        return GPIO_ERROR;
    }

    // Encoder
    if (gpio_init_encoders() != GPIO_OK) {
        log_error("Errore inizializzazione GPIO encoder");
        return GPIO_ERROR;
    }

    // Finecorsa
    if (gpio_init_limit_switches() != GPIO_OK) {
        log_error("Errore inizializzazione GPIO finecorsa");
        return GPIO_ERROR;
    }

    // I/O generici
    if (gpio_init_io() != GPIO_OK) {
        log_error("Errore inizializzazione GPIO I/O generici");
        return GPIO_ERROR;
    }

    // LED
    if (gpio_init_led() != GPIO_OK) {
        log_error("Errore inizializzazione GPIO LED");
        return GPIO_ERROR;
    }

    // NOTA: I pin UART3 (PB10/PB11) sono configurati direttamente da uart_init()

    log_info("=== Inizializzazione GPIO completata con successo ===");

    return GPIO_OK;
}
