/**
 * @file encoder.c
 * @brief Implementazione driver encoder rotativi su TIM1/TIM3/TIM4
 * @details
 * Configura i timer in Encoder Interface Mode per lettura automatica
 * di encoder in quadratura (segnali A e B con sfasamento 90°).
 *
 * ENCODER INTERFACE MODE:
 * Il timer conta automaticamente i fronti dei segnali A e B, gestendo:
 * - Direzione di rotazione (UP/DOWN)
 * - Risoluzione x4 (conta su tutti i fronti di A e B)
 * - Filtraggio input per debouncing
 *
 * CONFIGURAZIONE HARDWARE:
 * I pin GPIO per encoder sono configurati come:
 * - Alternate Function Push-Pull per TIM_CHx
 * - Input Floating (il timer gestisce internamente il pull-up)
 * - Remap se necessario (es. TIM1_CH1/CH2 su PA8/PA9)
 */
#include "common.h"
#include "encoder.h"
#include "gpio.h"
#include "log.h"

/* ============================================================================
 * FUNZIONI PUBBLICHE - INIZIALIZZAZIONE
 * ============================================================================
 */

/**
 * @brief Inizializza TIM3 come encoder per asse X (PA6/PA7)
 */
static int encoder_init_tim3(void)
{
    /* -----------------------------------------------------------------------
     * STEP 1: ABILITA CLOCK TIM3 (APB1)
     * ----------------------------------------------------------------------- */
    RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* -----------------------------------------------------------------------
     * STEP 2: DISABILITA TIMER DURANTE CONFIGURAZIONE
     * ----------------------------------------------------------------------- */
    TIM3_CR1 = 0;

    /* -----------------------------------------------------------------------
     * STEP 3: CONFIGURA ENCODER MODE
     * -----------------------------------------------------------------------
     * SMCR (Slave Mode Control Register):
     * - SMS[2:0] = 011 (Encoder mode 3: conta su entrambi fronti TI1 e TI2)
     */
    TIM3_SMCR = TIM_SMCR_SMS_ENCODER3;

    /* -----------------------------------------------------------------------
     * STEP 4: CONFIGURA INPUT CAPTURE
     * -----------------------------------------------------------------------
     * CCMR1 (Capture/Compare Mode Register 1):
     * - CC1S[1:0] = 01: CC1 channel è input, mappato su TI1
     * - CC2S[1:0] = 01: CC2 channel è input, mappato su TI2
     * - IC1F[3:0] = 0000: no filtro (opzionale, aggiungere se necessario)
     */
    TIM3_CCMR1 = TIM_CCMR1_CC1S_TI1 | TIM_CCMR1_CC2S_TI2;

    /* -----------------------------------------------------------------------
     * STEP 5: CONFIGURA POLARITÀ INPUT
     * -----------------------------------------------------------------------
     * CCER (Capture/Compare Enable Register):
     * - CC1E = 1: abilita Capture/Compare 1
     * - CC1P = 0: polarità normale (non invertita)
     * - CC2E = 1: abilita Capture/Compare 2
     * - CC2P = 0: polarità normale
     */
    TIM3_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

    /* -----------------------------------------------------------------------
     * STEP 6: CONFIGURA AUTO-RELOAD (COUNTER MAX VALUE)
     * -----------------------------------------------------------------------
     * ARR a 0xFFFF permette al counter di andare da 0 a 65535 (16-bit)
     */
    TIM3_ARR = 0xFFFF;

    /* -----------------------------------------------------------------------
     * STEP 7: CONFIGURA PRESCALER
     * -----------------------------------------------------------------------
     * PSC = 0: nessun prescaler, massima risoluzione
     */
    TIM3_PSC = 0;

    /* -----------------------------------------------------------------------
     * STEP 8: AZZERA COUNTER
     * ----------------------------------------------------------------------- */
    TIM3_CNT = 0;

    /* -----------------------------------------------------------------------
     * STEP 9: ABILITA TIMER
     * -----------------------------------------------------------------------
     * CR1: CEN = 1 (Counter Enable)
     */
    TIM3_CR1 = TIM_CR1_CEN;

    log_debug("TIM3 Encoder inizializzato (asse X, PA6/PA7)");
    return ENCODER_OK;
}

/**
 * @brief Inizializza TIM4 come encoder per asse Y (PB6/PB7)
 */
static int encoder_init_tim4(void)
{
    /* Abilita clock TIM4 */
    RCC_APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* Disabilita timer */
    TIM4_CR1 = 0;

    /* Encoder mode 3 (conta su tutti fronti) */
    TIM4_SMCR = TIM_SMCR_SMS_ENCODER3;

    /* Input capture: CC1=TI1, CC2=TI2 */
    TIM4_CCMR1 = TIM_CCMR1_CC1S_TI1 | TIM_CCMR1_CC2S_TI2;

    /* Abilita capture, polarità normale */
    TIM4_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

    /* Auto-reload 16-bit */
    TIM4_ARR = 0xFFFF;

    /* No prescaler */
    TIM4_PSC = 0;

    /* Azzera counter */
    TIM4_CNT = 0;

    /* Abilita timer */
    TIM4_CR1 = TIM_CR1_CEN;

    log_debug("TIM4 Encoder inizializzato (asse Y, PB6/PB7)");
    return ENCODER_OK;
}

/**
 * @brief Inizializza TIM1 come encoder per asse Z (PA8/PA9)
 */
static int encoder_init_tim1(void)
{
    /* Abilita clock TIM1 (APB2) */
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;

    /* Disabilita timer */
    TIM1_CR1 = 0;

    /* Encoder mode 3 */
    TIM1_SMCR = TIM_SMCR_SMS_ENCODER3;

    /* Input capture */
    TIM1_CCMR1 = TIM_CCMR1_CC1S_TI1 | TIM_CCMR1_CC2S_TI2;

    /* Abilita capture */
    TIM1_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

    /* Auto-reload */
    TIM1_ARR = 0xFFFF;

    /* No prescaler */
    TIM1_PSC = 0;

    /* Azzera counter */
    TIM1_CNT = 0;

    /* Abilita timer */
    TIM1_CR1 = TIM_CR1_CEN;

    log_debug("TIM1 Encoder inizializzato (asse Z, PA8/PA9)");
    return ENCODER_OK;
}

/**
 * @brief Inizializza tutti gli encoder
 */
int encoder_init(void)
{
    log_info("Inizializzazione encoder...");

    /* Inizializza i 3 timer encoder */
    if (encoder_init_tim3() != ENCODER_OK) {
        log_error("Errore init encoder X (TIM3)");
        return ENCODER_ERROR;
    }

    if (encoder_init_tim4() != ENCODER_OK) {
        log_error("Errore init encoder Y (TIM4)");
        return ENCODER_ERROR;
    }

    if (encoder_init_tim1() != ENCODER_OK) {
        log_error("Errore init encoder Z (TIM1)");
        return ENCODER_ERROR;
    }

    log_info("Encoder inizializzati: TIM1(Z), TIM3(X), TIM4(Y)");
    return ENCODER_OK;
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - LETTURA ENCODER (con reset)
 * ============================================================================
 */

i16 encoder_read_x(void)
{
    /* Leggi counter come signed 16-bit */
    i16 delta = (i16)(TIM3_CNT & 0xFFFF);

    /* Azzera counter per prossima lettura */
    TIM3_CNT = 0;

    return delta;
}

i16 encoder_read_y(void)
{
    i16 delta = (i16)(TIM4_CNT & 0xFFFF);
    TIM4_CNT = 0;
    return delta;
}

i16 encoder_read_z(void)
{
    i16 delta = (i16)(TIM1_CNT & 0xFFFF);
    TIM1_CNT = 0;
    return delta;
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - RESET
 * ============================================================================
 */

void encoder_reset_x(void)
{
    TIM3_CNT = 0;
}

void encoder_reset_y(void)
{
    TIM4_CNT = 0;
}

void encoder_reset_z(void)
{
    TIM1_CNT = 0;
}

void encoder_reset_all(void)
{
    TIM3_CNT = 0;
    TIM4_CNT = 0;
    TIM1_CNT = 0;
    log_debug("Tutti i counter encoder azzerati");
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - LETTURA DIRETTA (senza reset)
 * ============================================================================
 */

i16 encoder_get_count_x(void)
{
    return (i16)(TIM3_CNT & 0xFFFF);
}

i16 encoder_get_count_y(void)
{
    return (i16)(TIM4_CNT & 0xFFFF);
}

i16 encoder_get_count_z(void)
{
    return (i16)(TIM1_CNT & 0xFFFF);
}
