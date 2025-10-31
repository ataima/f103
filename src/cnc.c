/**
 * @file cnc.c
 * @brief Implementazione sistema gestione stato macchina CNC ed emergenze
 * @details
 * Gestisce macchina a stati, interrupt finecorsa con retrazione automatica,
 * e recupero sequenziale assi in emergenza.
 *
 * ARCHITETTURA INTERRUPT:
 * - EXTI (linee 2,3,4,5,8,9): Finecorsa con priorità alta
 * - TIM2: Generazione STEP emergenza con priorità normale
 * - Gli encoder (TIM1/TIM3/TIM4) restano attivi durante emergenza
 */
#include "common.h"
#include "cnc.h"
#include "gpio.h"
#include "log.h"
#include "systick.h"
#include "encoder.h"


/* ============================================================================
 * FORWARD DECLARATIONS - CALLBACK PRIVATO
 * ============================================================================
 */
static void cnc_encoder_isr_callback(void);

/* ============================================================================
 * VARIABILI GLOBALI
 * ============================================================================
 */

/**
 * @brief Stato corrente della macchina CNC
 */
volatile cnc_state_t current_state = ST_IDLE;

/**
 * @brief Coda emergenze da processare
 */
volatile emergency_queue_t emergency_queue = {
    .count = 0,
    .current_index = 0
};

/**
 * @brief Posizioni assolute encoder
 */
volatile encoder_positions_t encoder_positions = {
    .x = 0,
    .y = 0,
    .z = 0
};


/* ============================================================================
 * FORWARD DECLARATIONS - FUNZIONI PRIVATE
 * ============================================================================
 */
static const char* cnc_get_state_name_internal(cnc_state_t state);
static void cnc_handle_limit_interrupt(u8 axis, bool min_limit);
static void cnc_start_axis_recovery(u8 axis, bool min_limit);


/* ============================================================================
 * VARIABILI PRIVATE - GESTIONE RECUPERO ASSI
 * ============================================================================
 */

/**
 * @brief Asse correntemente in recupero (0=X, 1=Y, 2=Z, 0xFF=nessuno)
 */
static volatile u8 recovery_axis = 0xFF;

/**
 * @brief Stato STEP durante generazione impulsi (0=LOW, 1=HIGH)
 */
static volatile bool step_state = false;

/**
 * @brief Contatore STEP generati durante recupero
 */
static volatile u32 step_count = 0;


/* ============================================================================
 * VARIABILI PRIVATE - DEBOUNCING EXTI
 * ============================================================================
 */

/**
 * @brief Timestamp ultimo interrupt EXTI per asse X (ms)
 */
static volatile u32 exti_last_time_x = 0;

/**
 * @brief Timestamp ultimo interrupt EXTI per asse Y (ms)
 */
static volatile u32 exti_last_time_y = 0;

/**
 * @brief Timestamp ultimo interrupt EXTI per asse Z (ms)
 */
static volatile u32 exti_last_time_z = 0;


/* ============================================================================
 * FUNZIONI PRIVATE - UTILITÀ
 * ============================================================================
 */

/**
 * @brief Disabilita un asse specifico (ENABLE = HIGH)
 * @param axis Asse da disabilitare (0=X, 1=Y, 2=Z)
 */
static void disable_axis(u8 axis)
{
    switch(axis) {
        case 0: gpio_set(ENABLE_X_PORT, ENABLE_X_PIN); break;
        case 1: gpio_set(ENABLE_Y_PORT, ENABLE_Y_PIN); break;
        case 2: gpio_set(ENABLE_Z_PORT, ENABLE_Z_PIN); break;
    }
}

/**
 * @brief Abilita un asse specifico (ENABLE = LOW)
 * @param axis Asse da abilitare (0=X, 1=Y, 2=Z)
 */
static void enable_axis(u8 axis)
{
    switch(axis) {
        case 0: gpio_reset(ENABLE_X_PORT, ENABLE_X_PIN); break;
        case 1: gpio_reset(ENABLE_Y_PORT, ENABLE_Y_PIN); break;
        case 2: gpio_reset(ENABLE_Z_PORT, ENABLE_Z_PIN); break;
    }
}

/**
 * @brief Legge lo stato del pin DIR di un asse
 * @param axis Asse da leggere (0=X, 1=Y, 2=Z)
 * @return Stato corrente del pin DIR (0 o 1)
 */
static bool read_dir(u8 axis)
{
    switch(axis) {
        case 0: return gpio_read(DIR_X_PORT, DIR_X_PIN);
        case 1: return gpio_read(DIR_Y_PORT, DIR_Y_PIN);
        case 2: return gpio_read(DIR_Z_PORT, DIR_Z_PIN);
        default: return 0;
    }
}

/**
 * @brief Imposta il pin DIR di un asse
 * @param axis Asse (0=X, 1=Y, 2=Z)
 * @param dir Direzione (true=HIGH, false=LOW)
 */
static void write_dir(u8 axis, bool dir)
{
    switch(axis) {
        case 0:
            if(dir) gpio_set(DIR_X_PORT, DIR_X_PIN);
            else gpio_reset(DIR_X_PORT, DIR_X_PIN);
            break;
        case 1:
            if(dir) gpio_set(DIR_Y_PORT, DIR_Y_PIN);
            else gpio_reset(DIR_Y_PORT, DIR_Y_PIN);
            break;
        case 2:
            if(dir) gpio_set(DIR_Z_PORT, DIR_Z_PIN);
            else gpio_reset(DIR_Z_PORT, DIR_Z_PIN);
            break;
    }
}

/**
 * @brief Legge lo stato di un finecorsa
 * @param axis Asse (0=X, 1=Y, 2=Z)
 * @param min_limit true=MIN, false=MAX
 * @return Stato finecorsa (0=premuto, 1=rilasciato con pull-up)
 */
static bool read_limit_switch(u8 axis, bool min_limit)
{
    if (min_limit) {
        switch(axis) {
            case 0: return gpio_read(X_MIN_PORT, X_MIN_PIN);
            case 1: return gpio_read(Y_MIN_PORT, Y_MIN_PIN);
            case 2: return gpio_read(Z_MIN_PORT, Z_MIN_PIN);
            default: return 1;
        }
    } else {
        switch(axis) {
            case 0: return gpio_read(X_MAX_PORT, X_MAX_PIN);
            case 1: return gpio_read(Y_MAX_PORT, Y_MAX_PIN);
            case 2: return gpio_read(Z_MAX_PORT, Z_MAX_PIN);
            default: return 1;
        }
    }
}

/**
 * @brief Genera impulso STEP su asse specifico
 * @param axis Asse (0=X, 1=Y, 2=Z)
 * @param state Stato STEP (true=HIGH, false=LOW)
 */
static void write_step(u8 axis, bool state)
{
    switch(axis) {
        case 0:
            if(state) gpio_set(STEP_X_PORT, STEP_X_PIN);
            else gpio_reset(STEP_X_PORT, STEP_X_PIN);
            break;
        case 1:
            if(state) gpio_set(STEP_Y_PORT, STEP_Y_PIN);
            else gpio_reset(STEP_Y_PORT, STEP_Y_PIN);
            break;
        case 2:
            if(state) gpio_set(STEP_Z_PORT, STEP_Z_PIN);
            else gpio_reset(STEP_Z_PORT, STEP_Z_PIN);
            break;
    }
}

/**
 * @brief Disabilita interrupt EXTI per un asse specifico
 * @param axis Asse (0=X, 1=Y, 2=Z)
 * @details Disabilita le linee EXTI corrispondenti ai finecorsa MIN/MAX
 *          dell'asse. Usato durante homing per evitare interrupt spurii.
 */
static void exti_disable_axis(u8 axis)
{
    switch(axis) {
        case 0:  /* X: EXTI8 (X_MIN), EXTI9 (X_MAX) */
            EXTI_IMR &= ~(BIT(8) | BIT(9));
            break;
        case 1:  /* Y: EXTI2 (Y_MIN), EXTI3 (Y_MAX) */
            EXTI_IMR &= ~(BIT(2) | BIT(3));
            break;
        case 2:  /* Z: EXTI4 (Z_MIN), EXTI5 (Z_MAX) */
            EXTI_IMR &= ~(BIT(4) | BIT(5));
            break;
    }
}

/**
 * @brief Riabilita interrupt EXTI per un asse specifico
 * @param axis Asse (0=X, 1=Y, 2=Z)
 * @details Riabilita le linee EXTI dopo homing o recovery.
 *          Resetta anche i pending flag per evitare falsi trigger.
 */
static void exti_enable_axis(u8 axis)
{
    switch(axis) {
        case 0:  /* X: EXTI8 (X_MIN), EXTI9 (X_MAX) */
            EXTI_PR = (BIT(8) | BIT(9));      /* Clear pending flags */
            EXTI_IMR |= (BIT(8) | BIT(9));    /* Enable interrupts */
            break;
        case 1:  /* Y: EXTI2 (Y_MIN), EXTI3 (Y_MAX) */
            EXTI_PR = (BIT(2) | BIT(3));
            EXTI_IMR |= (BIT(2) | BIT(3));
            break;
        case 2:  /* Z: EXTI4 (Z_MIN), EXTI5 (Z_MAX) */
            EXTI_PR = (BIT(4) | BIT(5));
            EXTI_IMR |= (BIT(4) | BIT(5));
            break;
    }
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - HOMING
 * ============================================================================
 */

/**
 * @brief Posizione 0,0,0
 */

/**
 * @brief Esegue homing dell'asse Z verso finecorsa MIN
 * @details Procedura con EXTI disabilitati e polling diretto:
 *          1. Disabilita EXTI asse Z
 *          2. Imposta direzione verso MIN
 *          3. Genera STEP a 1kHz con polling GPIO (no interrupt)
 *          4. Quando Z_MIN premuto: debounce 50ms
 *          5. Inverti direzione e retract fino a rilascio
 *          6. Debounce 50ms + attesa stabilizzazione 100ms
 *          7. Riabilita EXTI asse Z
 *
 * @return 0 se successo, -1 se errore/timeout
 *
 * @note Bloccante: attende fino a completamento homing (max 40 secondi)
 * @note USA POLLING GPIO, NON interrupt EXTI (per evitare rimbalzi)
 */
int cnc_home_z(void)
{
    log_info("Homing asse Z...");

    /* Imposta stato homing */
    cnc_state_t old_state = current_state;
    current_state = ST_HOMING;

    /* === FASE 1: DISABILITA EXTI ASSE Z === */
    exti_disable_axis(2);
    log_debug("EXTI Z disabilitati per homing");

    /* Verifica che finecorsa MIN non sia già premuto */
    if (gpio_read(Z_MIN_PORT, Z_MIN_PIN) == 0) {
        log_warning("Z_MIN già premuto, eseguo retraction");
        /* Inverti direzione e retract */
        gpio_set(DIR_Z_PORT, DIR_Z_PIN);
        enable_axis(2);
        TIM2_ARR = (1000000 / 500) - 1;  /* 500Hz lento */
        TIM2_CNT = 0;
        TIM2_SR = 0;
        step_state = false;
        recovery_axis = 2;
        TIM2_CR1 |= TIM_CR1_CEN;

        /* Attendi rilascio */
        u32 start_time = systick_get_tick();
        while (gpio_read(Z_MIN_PORT, Z_MIN_PIN) == 0) {
            if (systick_timeout(start_time, 10000)) {
                TIM2_CR1 &= ~TIM_CR1_CEN;
                disable_axis(2);
                recovery_axis = 0xFF;
                exti_enable_axis(2);
                current_state = old_state;
                log_error("Homing Z: timeout retraction iniziale");
                return -1;
            }
            systick_delay_ms(10);
        }
        TIM2_CR1 &= ~TIM_CR1_CEN;
        disable_axis(2);
        recovery_axis = 0xFF;
        systick_delay_ms(HOMING_DEBOUNCE_MS);
    }

    /* === FASE 2: MOVIMENTO VERSO MIN (POLLING) === */
    gpio_reset(DIR_Z_PORT, DIR_Z_PIN);  /* Direzione verso MIN */
    enable_axis(2);

    TIM2_ARR = (1000000 / 1000) - 1;  /* 1kHz */
    TIM2_CNT = 0;
    TIM2_SR = 0;

    step_state = false;
    recovery_axis = 2;
    TIM2_CR1 |= TIM_CR1_CEN;

    /* Polling GPIO: attendi finecorsa MIN */
    u32 start_time = systick_get_tick();
    while (gpio_read(Z_MIN_PORT, Z_MIN_PIN) != 0) {
        if (systick_timeout(start_time, 30000)) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
            disable_axis(2);
            recovery_axis = 0xFF;
            exti_enable_axis(2);
            current_state = old_state;
            log_error("Homing Z: timeout 30s");
            return -1;
        }
        systick_delay_ms(10);
    }

    log_info("Z_MIN toccato, debounce...");
    systick_delay_ms(HOMING_DEBOUNCE_MS);

    /* === FASE 3: RETRACTION (INVERTI DIREZIONE) === */
    TIM2_CR1 &= ~TIM_CR1_CEN;
    write_step(2, false);  /* STEP LOW */

    gpio_set(DIR_Z_PORT, DIR_Z_PIN);  /* Inverti direzione */
    systick_delay_ms(5);  /* Tempo setup DIR */

    TIM2_ARR = (1000000 / 500) - 1;  /* 500Hz lento per retraction */
    TIM2_CNT = 0;
    TIM2_SR = 0;
    step_state = false;
    TIM2_CR1 |= TIM_CR1_CEN;

    /* Polling GPIO: attendi rilascio finecorsa */
    start_time = systick_get_tick();
    while (gpio_read(Z_MIN_PORT, Z_MIN_PIN) == 0) {
        if (systick_timeout(start_time, 10000)) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
            disable_axis(2);
            recovery_axis = 0xFF;
            exti_enable_axis(2);
            current_state = old_state;
            log_error("Homing Z: timeout retraction");
            return -1;
        }
        systick_delay_ms(10);
    }

    log_info("Z_MIN rilasciato, debounce finale...");

    /* === FASE 4: CLEANUP E STABILIZZAZIONE === */
    TIM2_CR1 &= ~TIM_CR1_CEN;
    write_step(2, false);
    disable_axis(2);
    recovery_axis = 0xFF;

    systick_delay_ms(HOMING_DEBOUNCE_MS);  /* Debounce rilascio */
    systick_delay_ms(100);  /* Stabilizzazione finale */

    /* === FASE 5: RIABILITA EXTI === */
    exti_enable_axis(2);
    log_debug("EXTI Z riabilitati");

    current_state = old_state;
    log_info("Homing Z completato");
    return 0;
}


/**
 * @brief Esegue homing dell'asse Y verso finecorsa MIN
 * @return 0 se successo, -1 se errore/timeout
 */
int cnc_home_y(void)
{
    log_info("Homing asse Y...");

    cnc_state_t old_state = current_state;
    current_state = ST_HOMING;

    /* DISABILITA EXTI ASSE Y */
    exti_disable_axis(1);
    log_debug("EXTI Y disabilitati per homing");

    /* Verifica finecorsa già premuto */
    if (gpio_read(Y_MIN_PORT, Y_MIN_PIN) == 0) {
        log_warning("Y_MIN già premuto, eseguo retraction");
        gpio_set(DIR_Y_PORT, DIR_Y_PIN);
        enable_axis(1);
        TIM2_ARR = (1000000 / 500) - 1;
        TIM2_CNT = 0;
        TIM2_SR = 0;
        step_state = false;
        recovery_axis = 1;
        TIM2_CR1 |= TIM_CR1_CEN;

        u32 start_time = systick_get_tick();
        while (gpio_read(Y_MIN_PORT, Y_MIN_PIN) == 0) {
            if (systick_timeout(start_time, 10000)) {
                TIM2_CR1 &= ~TIM_CR1_CEN;
                disable_axis(1);
                recovery_axis = 0xFF;
                exti_enable_axis(1);
                current_state = old_state;
                log_error("Homing Y: timeout retraction iniziale");
                return -1;
            }
            systick_delay_ms(10);
        }
        TIM2_CR1 &= ~TIM_CR1_CEN;
        disable_axis(1);
        recovery_axis = 0xFF;
        systick_delay_ms(HOMING_DEBOUNCE_MS);
    }

    /* MOVIMENTO VERSO MIN */
    gpio_reset(DIR_Y_PORT, DIR_Y_PIN);
    enable_axis(1);

    TIM2_ARR = (1000000 / 1000) - 1;
    TIM2_CNT = 0;
    TIM2_SR = 0;

    step_state = false;
    recovery_axis = 1;
    TIM2_CR1 |= TIM_CR1_CEN;

    u32 start_time = systick_get_tick();
    while (gpio_read(Y_MIN_PORT, Y_MIN_PIN) != 0) {
        if (systick_timeout(start_time, 30000)) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
            disable_axis(1);
            recovery_axis = 0xFF;
            exti_enable_axis(1);
            current_state = old_state;
            log_error("Homing Y: timeout 30s");
            return -1;
        }
        systick_delay_ms(10);
    }

    log_info("Y_MIN toccato, debounce...");
    systick_delay_ms(HOMING_DEBOUNCE_MS);

    /* RETRACTION */
    TIM2_CR1 &= ~TIM_CR1_CEN;
    write_step(1, false);

    gpio_set(DIR_Y_PORT, DIR_Y_PIN);
    systick_delay_ms(5);

    TIM2_ARR = (1000000 / 500) - 1;
    TIM2_CNT = 0;
    TIM2_SR = 0;
    step_state = false;
    TIM2_CR1 |= TIM_CR1_CEN;

    start_time = systick_get_tick();
    while (gpio_read(Y_MIN_PORT, Y_MIN_PIN) == 0) {
        if (systick_timeout(start_time, 10000)) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
            disable_axis(1);
            recovery_axis = 0xFF;
            exti_enable_axis(1);
            current_state = old_state;
            log_error("Homing Y: timeout retraction");
            return -1;
        }
        systick_delay_ms(10);
    }

    log_info("Y_MIN rilasciato, debounce finale...");

    /* CLEANUP */
    TIM2_CR1 &= ~TIM_CR1_CEN;
    write_step(1, false);
    disable_axis(1);
    recovery_axis = 0xFF;

    systick_delay_ms(HOMING_DEBOUNCE_MS);
    systick_delay_ms(100);

    /* RIABILITA EXTI */
    exti_enable_axis(1);
    log_debug("EXTI Y riabilitati");

    current_state = old_state;
    log_info("Homing Y completato");
    return 0;
}


/**
 * @brief Esegue homing dell'asse X verso finecorsa MIN
 * @return 0 se successo, -1 se errore/timeout
 */
int cnc_home_x(void)
{
    log_info("Homing asse X...");

    cnc_state_t old_state = current_state;
    current_state = ST_HOMING;

    /* DISABILITA EXTI ASSE X */
    exti_disable_axis(0);
    log_debug("EXTI X disabilitati per homing");

    /* Verifica finecorsa già premuto */
    if (gpio_read(X_MIN_PORT, X_MIN_PIN) == 0) {
        log_warning("X_MIN già premuto, eseguo retraction");
        gpio_set(DIR_X_PORT, DIR_X_PIN);
        enable_axis(0);
        TIM2_ARR = (1000000 / 500) - 1;
        TIM2_CNT = 0;
        TIM2_SR = 0;
        step_state = false;
        recovery_axis = 0;
        TIM2_CR1 |= TIM_CR1_CEN;

        u32 start_time = systick_get_tick();
        while (gpio_read(X_MIN_PORT, X_MIN_PIN) == 0) {
            if (systick_timeout(start_time, 10000)) {
                TIM2_CR1 &= ~TIM_CR1_CEN;
                disable_axis(0);
                recovery_axis = 0xFF;
                exti_enable_axis(0);
                current_state = old_state;
                log_error("Homing X: timeout retraction iniziale");
                return -1;
            }
            systick_delay_ms(10);
        }
        TIM2_CR1 &= ~TIM_CR1_CEN;
        disable_axis(0);
        recovery_axis = 0xFF;
        systick_delay_ms(HOMING_DEBOUNCE_MS);
    }

    /* MOVIMENTO VERSO MIN */
    gpio_reset(DIR_X_PORT, DIR_X_PIN);
    enable_axis(0);

    TIM2_ARR = (1000000 / 1000) - 1;
    TIM2_CNT = 0;
    TIM2_SR = 0;

    step_state = false;
    recovery_axis = 0;
    TIM2_CR1 |= TIM_CR1_CEN;

    u32 start_time = systick_get_tick();
    while (gpio_read(X_MIN_PORT, X_MIN_PIN) != 0) {
        if (systick_timeout(start_time, 30000)) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
            disable_axis(0);
            recovery_axis = 0xFF;
            exti_enable_axis(0);
            current_state = old_state;
            log_error("Homing X: timeout 30s");
            return -1;
        }
        systick_delay_ms(10);
    }

    log_info("X_MIN toccato, debounce...");
    systick_delay_ms(HOMING_DEBOUNCE_MS);

    /* RETRACTION */
    TIM2_CR1 &= ~TIM_CR1_CEN;
    write_step(0, false);

    gpio_set(DIR_X_PORT, DIR_X_PIN);
    systick_delay_ms(5);

    TIM2_ARR = (1000000 / 500) - 1;
    TIM2_CNT = 0;
    TIM2_SR = 0;
    step_state = false;
    TIM2_CR1 |= TIM_CR1_CEN;

    start_time = systick_get_tick();
    while (gpio_read(X_MIN_PORT, X_MIN_PIN) == 0) {
        if (systick_timeout(start_time, 10000)) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
            disable_axis(0);
            recovery_axis = 0xFF;
            exti_enable_axis(0);
            current_state = old_state;
            log_error("Homing X: timeout retraction");
            return -1;
        }
        systick_delay_ms(10);
    }

    log_info("X_MIN rilasciato, debounce finale...");

    /* CLEANUP */
    TIM2_CR1 &= ~TIM_CR1_CEN;
    write_step(0, false);
    disable_axis(0);
    recovery_axis = 0xFF;

    systick_delay_ms(HOMING_DEBOUNCE_MS);
    systick_delay_ms(100);

    /* RIABILITA EXTI */
    exti_enable_axis(0);
    log_debug("EXTI X riabilitati");

    current_state = old_state;
    log_info("Homing X completato");
    return 0;
}


/**
 * @brief Esegue homing completo di tutti gli assi (Z → Y → X)
 * @details Sequenza sicura: prima Z (alza utensile), poi Y, poi X
 * @return 0 se successo, -1 se errore
 */
int cnc_home(void)
{
    log_info("=== Inizio homing completo ===");

    /* Homing Z (alza utensile per sicurezza) */
    int res = cnc_home_z();
    if (res != 0) {
        log_error("Homing fallito sull'asse Z");
        return res;
    }

    /* Homing Y */
    res = cnc_home_y();
    if (res != 0) {
        log_error("Homing fallito sull'asse Y");
        return res;
    }

    /* Homing X */
    res = cnc_home_x();
    if (res != 0) {
        log_error("Homing fallito sull'asse X");
        return res;
    }

    log_info("=== Homing completo: tutti gli assi a posizione 0,0,0 ===");
    return 0;
}

/* ============================================================================
 * FUNZIONI PUBBLICHE - INIZIALIZZAZIONE
 * ============================================================================
 */

/**
 * @brief Inizializza sistema CNC con interrupt finecorsa e TIM2
 */
int cnc_init(void)
{
	int res;
    log_info("Inizializzazione sistema CNC...");

    /* -----------------------------------------------------------------------
     * INIZIALIZZAZIONE STATO
     * ----------------------------------------------------------------------- */
    current_state = ST_IDLE;
    emergency_queue.count = 0;
    emergency_queue.current_index = 0;
    recovery_axis = 0xFF;

    /* Disabilita tutti i motori per sicurezza */
    disable_axis(0);
    disable_axis(1);
    disable_axis(2);

    log_debug("Stato iniziale: ST_IDLE, motori disabilitati");

     /* -----------------------------------------------------------------------
     * CONFIGURAZIONE EXTI PER FINECORSA
     * -----------------------------------------------------------------------
     * Configura interrupt su falling edge (1→0) per tutti i 6 finecorsa.
     *
     * Mappatura EXTI:
     * - X_MIN (PB8) → EXTI8
     * - X_MAX (PB9) → EXTI9
     * - Y_MIN (PA2) → EXTI2
     * - Y_MAX (PA3) → EXTI3
     * - Z_MIN (PA4) → EXTI4
     * - Z_MAX (PA5) → EXTI5
     */

    /* Abilita clock AFIO (necessario per EXTI) */
    RCC_APB2ENR |= RCC_APB2ENR_AFIOEN;

    /* Configura AFIO_EXTICR per mappare pin GPIO a linee EXTI */
    /* EXTI2 → PA2 (Y_MIN) */
    AFIO_EXTICR1 &= ~(0xF << 8);   // Clear EXTI2 bits
    AFIO_EXTICR1 |= (0x0 << 8);    // PA2 (0000 = GPIOA)

    /* EXTI3 → PA3 (Y_MAX) */
    AFIO_EXTICR1 &= ~(0xF << 12);  // Clear EXTI3 bits
    AFIO_EXTICR1 |= (0x0 << 12);   // PA3 (0000 = GPIOA)

    /* EXTI4 → PA4 (Z_MIN) */
    AFIO_EXTICR2 &= ~(0xF << 0);   // Clear EXTI4 bits
    AFIO_EXTICR2 |= (0x0 << 0);    // PA4 (0000 = GPIOA)

    /* EXTI5 → PA5 (Z_MAX) */
    AFIO_EXTICR2 &= ~(0xF << 4);   // Clear EXTI5 bits
    AFIO_EXTICR2 |= (0x0 << 4);    // PA5 (0000 = GPIOA)

    /* EXTI8 → PB8 (X_MIN) */
    AFIO_EXTICR3 &= ~(0xF << 0);   // Clear EXTI8 bits
    AFIO_EXTICR3 |= (0x1 << 0);    // PB8 (0001 = GPIOB)

    /* EXTI9 → PB9 (X_MAX) */
    AFIO_EXTICR3 &= ~(0xF << 4);   // Clear EXTI9 bits
    AFIO_EXTICR3 |= (0x1 << 4);    // PB9 (0001 = GPIOB)

    /* Configura EXTI: Interrupt Mask Register (abilita interrupt) */
    EXTI_IMR |= (BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(8) | BIT(9));

    /* Configura EXTI: Falling Trigger Selection (1→0) */
    EXTI_FTSR |= (BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(8) | BIT(9));

    /* Disabilita Rising Trigger (non vogliamo interrupt su 0→1) */
    EXTI_RTSR &= ~(BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(8) | BIT(9));

    log_debug("EXTI configurato: linee 2,3,4,5,8,9 su falling edge");

    /* -----------------------------------------------------------------------
     * CONFIGURAZIONE NVIC (PRIORITÀ INTERRUPT)
     * -----------------------------------------------------------------------
     * Priorità alta per EXTI (finecorsa hanno massima priorità).
     *
     * STM32F103 usa 4 bit di priorità (16 livelli), divisi in:
     * - Preemption priority (2 bit): determina quale ISR può interrompere quale
     * - Sub-priority (2 bit): risolve conflitti tra interrupt stesso livello
     *
     * Priorità più bassa = numero più alto (0 = massima priorità)
     */

    /* Configura NVIC Priority Group: 2 bit preemption, 2 bit sub */
    NVIC_SetPriorityGrouping(5);  // PRIGROUP = 5 (0b101) → 2:2 split

    /* Abilita interrupt EXTI con priorità alta (preemption=0, sub=0) */
    NVIC_SetPriority(EXTI2_IRQn, NVIC_EncodePriority(5, 0, 0));
    NVIC_EnableIRQ(EXTI2_IRQn);

    NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(5, 0, 0));
    NVIC_EnableIRQ(EXTI3_IRQn);

    NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(5, 0, 0));
    NVIC_EnableIRQ(EXTI4_IRQn);

    NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(5, 0, 0));  // EXTI5,6,7,8,9 condividono IRQ
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    log_debug("NVIC configurato: EXTI priorità 0 (massima)");

    /* -----------------------------------------------------------------------
     * CONFIGURAZIONE TIM2 PER GENERAZIONE STEP EMERGENZA
     * -----------------------------------------------------------------------
     * TIM2 genera interrupt a frequenza EMER_STEP_FREQ_X (default 1kHz).
     * Ad ogni interrupt, togliamo lo stato STEP per generare impulso.
     *
     * TIM2 configurazione:
     * - Clock: APB1 (36 MHz) × 2 = 72 MHz (timer clock doubler attivo)
     * - Prescaler: 72-1 → 1 MHz tick
     * - Auto-reload: 1000-1 → interrupt a 1 kHz
     *
     * Il timer parte disabilitato, viene avviato quando serve.
     */

    /* Abilita clock TIM2 (su APB1) */
    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Configura prescaler: 72 MHz / 72 = 1 MHz tick rate */
    TIM2_PSC = 72 - 1;

    /* Configura auto-reload per 1 kHz (default, verrà cambiato per asse) */
    TIM2_ARR = 1000 - 1;

    /* Abilita interrupt Update (UIE) */
    TIM2_DIER = TIM_DIER_UIE;

    /* Clear pending flags */
    TIM2_SR = 0;

    /* NON avviare timer ancora (CEN=0) */
    TIM2_CR1 = 0;

    /* Configura NVIC per TIM2 con priorità normale (preemption=2, sub=0) */
    NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(5, 2, 0));
    NVIC_EnableIRQ(TIM2_IRQn);

    log_debug("TIM2 configurato: 1kHz base, interrupt abilitato");

    log_info("Sistema CNC inizializzato con successo");

    /* -----------------------------------------------------------------------
     * HOMING AUTOMATICO
     * -----------------------------------------------------------------------
     * Esegue homing di tutti gli assi (Z → Y → X) per trovare posizione 0.
     * L'homing è una procedura di CALIBRAZIONE, non un movimento di lavoro.
     * Gli encoder non contano durante questa fase.
     */
    res = cnc_home();
    if (res != 0) {
        log_error("Homing fallito! Sistema in stato sicuro (motori disabilitati)");
        return res;
    }

    /* -----------------------------------------------------------------------
     * INIZIALIZZAZIONE ENCODER
     * -----------------------------------------------------------------------
     * Inizializza i timer TIM1/TIM3/TIM4 in modalità Encoder Interface.
     * IMPORTANTE: Gli encoder vengono inizializzati DOPO l'homing perché:
     *   1. L'homing trova la posizione fisica 0 (finecorsa MIN)
     *   2. Gli encoder partono da 0 quando la macchina è già calibrata
     *   3. Gli encoder tracciano solo i movimenti di LAVORO, non calibrazione
     */
    if (encoder_init() != ENCODER_OK) {
    	log_error("Errore inizializzazione encoder!");
    	return -1;
    }

    /* Azzera posizioni assolute (partenza da posizione 0 dopo homing) */
    encoder_positions.x = 0;
    encoder_positions.y = 0;
    encoder_positions.z = 0;
    log_info("Encoder inizializzati a posizione 0,0,0");

    /* -----------------------------------------------------------------------
     * REGISTRAZIONE CALLBACK ENCODER IN SYSTICK (50Hz)
     * -----------------------------------------------------------------------
     * Registra la funzione cnc_encoder_isr_callback() per essere chiamata
     * automaticamente ogni 20ms (50Hz) dal SysTick ISR.
     * Da questo momento gli encoder tracciano ogni movimento.
     */
    systick_register_encoder_callback(cnc_encoder_isr_callback);
    log_info("Tracking encoder attivo (50Hz)");

    return 0;  /* Homing completato e encoder pronti */
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - GESTIONE STATO
 * ============================================================================
 */

cnc_state_t cnc_get_state(void)
{
    return current_state;
}

void cnc_set_state(cnc_state_t new_state)
{
    EVAL_LOG(cnc_state_t old_state = current_state;)
    current_state = new_state;
    log_info("Cambio stato: %s → %s", cnc_get_state_name_internal(old_state), cnc_get_state_name());
}

/**
 * @brief Ottiene nome stato (versione interna con parametro)
 */
static const char* cnc_get_state_name_internal(cnc_state_t state)
{
    switch(state) {
        case ST_IDLE:           return "IDLE";
        case ST_RUNNING:        return "RUNNING";
        case ST_HOMING:         return "HOMING";
        case ST_TEST:           return "TEST";
        case ST_EMERGENCY_STOP: return "EMERGENCY_STOP";
        case ST_ALARM_X:        return "ALARM_X";
        case ST_ALARM_Y:        return "ALARM_Y";
        case ST_ALARM_Z:        return "ALARM_Z";
        default:                return "UNKNOWN";
    }
}

const char* cnc_get_state_name(void)
{
    return cnc_get_state_name_internal(current_state);
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - GESTIONE EMERGENZA
 * ============================================================================
 */

bool cnc_has_emergency(void)
{
    return (emergency_queue.count > 0);
}

void cnc_process_emergency(void)
{
    /* Se non ci sono emergenze, non fare nulla */
    if (emergency_queue.count == 0) {
        return;
    }

    /* Se c'è già un recupero in corso, aspetta che finisca */
    if (recovery_axis != 0xFF) {
        return;  // TIM2 ISR sta già gestendo il recupero
    }

    /* Prendi il prossimo asse dalla coda */
    if (emergency_queue.current_index < emergency_queue.count) {
        emergency_axis_t *ea = (emergency_axis_t *)&emergency_queue.queue[emergency_queue.current_index];
        cnc_start_axis_recovery(ea->axis, ea->min_limit);
        emergency_queue.current_index++;
    } else {
        /* Tutti gli assi recuperati */
        emergency_queue.count = 0;
        emergency_queue.current_index = 0;
        current_state = ST_IDLE;
        log_info("Tutti gli assi recuperati, ritorno a ST_IDLE");
    }
}

void cnc_clear_emergency(void)
{
    /* Disabilita TIM2 se attivo */
    TIM2_CR1 &= ~TIM_CR1_CEN;

    /* Svuota coda */
    emergency_queue.count = 0;
    emergency_queue.current_index = 0;
    recovery_axis = 0xFF;

    /* Disabilita tutti i motori */
    disable_axis(0);
    disable_axis(1);
    disable_axis(2);

    current_state = ST_IDLE;
    log_warning("Emergenza resettata manualmente");
}


/* ============================================================================
 * FUNZIONI PRIVATE - CALLBACK ENCODER DA ISR
 * ============================================================================
 */

/**
 * @brief Callback chiamato da SysTick ISR ogni 20ms (50Hz)
 * @details Legge i delta encoder e aggiorna le posizioni assolute.
 *          Questa funzione è chiamata in contesto interrupt!
 * @note VELOCE: solo letture/scritture registri, NO logging
 */
static void cnc_encoder_isr_callback(void)
{
    /* Leggi i delta dai timer encoder (e azzerano i timer) */
    i16 delta_x = encoder_read_x();
    i16 delta_y = encoder_read_y();
    i16 delta_z = encoder_read_z();

    /* Aggiorna le posizioni assolute (operazione veloce, ~3 cicli CPU) */
    encoder_positions.x += (i32)delta_x;
    encoder_positions.y += (i32)delta_y;
    encoder_positions.z += (i32)delta_z;

    /* NO LOGGING QUI - siamo in ISR! */
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - GESTIONE POSIZIONI ENCODER
 * ============================================================================
 */

/**
 * @brief Aggiorna le posizioni encoder leggendo i delta dai timer
 * @details DEPRECATA: Ora la lettura avviene automaticamente in ISR a 50Hz.
 *          Questa funzione è mantenuta per compatibilità ma non fa nulla.
 * @note Non è più necessario chiamarla nel main loop!
 */
void cnc_update_encoder_positions(void)
{
    /* La lettura encoder ora avviene automaticamente in SysTick ISR a 50Hz
     * tramite il callback cnc_encoder_isr_callback().
     * Questa funzione è vuota e mantenuta solo per compatibilità API. */
}

i32 cnc_get_position_x(void)
{
    return encoder_positions.x;
}

i32 cnc_get_position_y(void)
{
    return encoder_positions.y;
}

i32 cnc_get_position_z(void)
{
    return encoder_positions.z;
}

void cnc_reset_position_x(void)
{
    encoder_positions.x = 0;
    encoder_reset_x();
    log_info("Posizione X azzerata");
}

void cnc_reset_position_y(void)
{
    encoder_positions.y = 0;
    encoder_reset_y();
    log_info("Posizione Y azzerata");
}

void cnc_reset_position_z(void)
{
    encoder_positions.z = 0;
    encoder_reset_z();
    log_info("Posizione Z azzerata");
}

void cnc_reset_all_positions(void)
{
    encoder_positions.x = 0;
    encoder_positions.y = 0;
    encoder_positions.z = 0;
    encoder_reset_all();
    log_info("Tutte le posizioni encoder azzerate");
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - MOVIMENTO REAL-TIME (NUOVA API ✨)
 * ============================================================================
 */

/**
 * @brief Variabile globale stato movimento RT (usata da ISR)
 */
static volatile move_rt_state_t g_move_rt_state;

/**
 * @brief Helper: max tra 3 valori
 */
static inline u32 max3_u32(u32 a, u32 b, u32 c) {
    u32 max_ab = (a > b) ? a : b;
    return (max_ab > c) ? max_ab : c;
}

/**
 * @brief Calcola profilo velocità trapezoidale per traiettoria
 * @details Profilo standard: 25% accelerazione, 50% velocità costante, 25% decelerazione
 */
static void cnc_calculate_trajectory_profile(u32 total_steps, trajectory_profile_t *profile)
{
    profile->total_steps = total_steps;
    profile->accel_steps = total_steps / 4;  // 25% accelerazione
    profile->decel_steps = total_steps / 4;  // 25% decelerazione
    profile->const_steps = total_steps - profile->accel_steps - profile->decel_steps;  // 50% costante
}

/**
 * @brief Inizializza movimento lineare 3D real-time
 */
int cnc_move_init_rt(move_rt_state_t *state, point3d_t start, point3d_t end,
                     u32 min_hz, u32 max_hz)
{
    if (state == NULL || min_hz == 0 || max_hz == 0 || min_hz > max_hz) {
        return -1;
    }

    /* Azzera struttura */
    state->active = false;
    state->completed = false;

    /* Punti */
    state->start = start;
    state->end = end;
    state->current = start;

    /* Calcola delta e segni */
    i32 dx = end.x - start.x;
    i32 dy = end.y - start.y;
    i32 dz = end.z - start.z;

    state->sx = (dx > 0) ? 1 : (dx < 0) ? -1 : 0;
    state->sy = (dy > 0) ? 1 : (dy < 0) ? -1 : 0;
    state->sz = (dz > 0) ? 1 : (dz < 0) ? -1 : 0;

    state->dx = (dx < 0) ? -dx : dx;
    state->dy = (dy < 0) ? -dy : dy;
    state->dz = (dz < 0) ? -dz : dz;

    /* Determina asse dominante e step totali */
    state->total_steps = max3_u32(state->dx, state->dy, state->dz);

    if (state->total_steps == 0) {
        /* Movimento nullo */
        state->completed = true;
        return 0;
    }

    if (state->dx >= state->dy && state->dx >= state->dz) {
        state->dominant_axis = 0;  /* X */
    } else if (state->dy >= state->dx && state->dy >= state->dz) {
        state->dominant_axis = 1;  /* Y */
    } else {
        state->dominant_axis = 2;  /* Z */
    }

    /* Inizializza errori Bresenham */
    state->err_xy = state->dx >> 1;
    state->err_xz = state->dx >> 1;

    /* Contatori */
    state->current_step = 0;

    /* Profilo velocità */
    state->min_freq_hz = min_hz;
    state->max_freq_hz = max_hz;

    /* Calcola profilo trapezoidale (usa funzione esistente) */
    cnc_calculate_trajectory_profile(state->total_steps, &state->profile);

    return 0;
}

/**
 * @brief Calcola step successivo Bresenham 3D (INCREMENTALE)
 * @return Maschera assi: bit0=X, bit1=Y, bit2=Z, 0=completato
 */
u8 cnc_move_step_rt(move_rt_state_t *state)
{
    if (state == NULL || state->completed) {
        return 0;
    }

    /* Verifica se raggiunti tutti gli step */
    if (state->current_step >= state->total_steps) {
        state->completed = true;
        return 0;
    }

    u8 axis_mask = 0;

    /* Algoritmo Bresenham 3D basato su asse dominante */
    switch (state->dominant_axis) {
        case 0:  /* X dominante */
            /* Muovi sempre X */
            axis_mask |= 0x01;
            state->current.x += state->sx;

            /* Verifica Y */
            state->err_xy -= state->dy;
            if (state->err_xy < 0) {
                axis_mask |= 0x02;
                state->current.y += state->sy;
                state->err_xy += state->dx;
            }

            /* Verifica Z */
            state->err_xz -= state->dz;
            if (state->err_xz < 0) {
                axis_mask |= 0x04;
                state->current.z += state->sz;
                state->err_xz += state->dx;
            }
            break;

        case 1:  /* Y dominante */
            /* Muovi sempre Y */
            axis_mask |= 0x02;
            state->current.y += state->sy;

            /* Verifica X */
            state->err_xy -= state->dx;
            if (state->err_xy < 0) {
                axis_mask |= 0x01;
                state->current.x += state->sx;
                state->err_xy += state->dy;
            }

            /* Verifica Z */
            state->err_xz -= state->dz;
            if (state->err_xz < 0) {
                axis_mask |= 0x04;
                state->current.z += state->sz;
                state->err_xz += state->dy;
            }
            break;

        case 2:  /* Z dominante */
            /* Muovi sempre Z */
            axis_mask |= 0x04;
            state->current.z += state->sz;

            /* Verifica X */
            state->err_xy -= state->dx;
            if (state->err_xy < 0) {
                axis_mask |= 0x01;
                state->current.x += state->sx;
                state->err_xy += state->dz;
            }

            /* Verifica Y */
            state->err_xz -= state->dy;
            if (state->err_xz < 0) {
                axis_mask |= 0x02;
                state->current.y += state->sy;
                state->err_xz += state->dz;
            }
            break;
    }

    /* Incrementa contatore step */
    state->current_step++;

    return axis_mask;
}

/**
 * @brief Calcola velocità istantanea (profilo trapezoidale)
 */
u32 cnc_move_get_speed_rt(const volatile move_rt_state_t *state)
{
    if (state == NULL || state->total_steps == 0) {
        return state->min_freq_hz;
    }

    u32 step = state->current_step;
    u32 min_hz = state->min_freq_hz;
    u32 max_hz = state->max_freq_hz;

    /* Fase accelerazione */
    if (step < state->profile.accel_steps) {
        /* Interpolazione lineare: min_hz → max_hz */
        u32 delta = max_hz - min_hz;
        return min_hz + (delta * step) / state->profile.accel_steps;
    }

    /* Fase velocità costante */
    if (step < state->profile.accel_steps + state->profile.const_steps) {
        return max_hz;
    }

    /* Fase decelerazione */
    u32 decel_start = state->profile.accel_steps + state->profile.const_steps;
    u32 decel_step = step - decel_start;
    u32 delta = max_hz - min_hz;
    return max_hz - (delta * decel_step) / state->profile.decel_steps;
}

/**
 * @brief Ottiene percentuale completamento
 */
u8 cnc_move_get_percentage_rt(const volatile move_rt_state_t *state)
{
    if (state == NULL || state->total_steps == 0) {
        return 100;
    }
    return (u8)((state->current_step * 100) / state->total_steps);
}

/**
 * @brief Esegue movimento real-time (bloccante)
 */
int cnc_move_execute_rt(move_rt_state_t *state)
{
    if (state == NULL || state->completed) {
        return -1;
    }

    log_info("Movimento RT: (%d,%d,%d) -> (%d,%d,%d), %u step",
             (int)state->start.x, (int)state->start.y, (int)state->start.z,
             (int)state->end.x, (int)state->end.y, (int)state->end.z,
             state->total_steps);

    /* Copia stato in variabile globale per ISR */
    g_move_rt_state = *state;
    g_move_rt_state.active = true;

    /* Configura TIM2 per prima velocità */
    u32 initial_freq = cnc_move_get_speed_rt(&g_move_rt_state);
    TIM2_ARR = (1000000 / initial_freq) - 1;
    TIM2_CNT = 0;
    TIM2_SR = 0;

    /* Avvia TIM2 */
    TIM2_CR1 |= TIM_CR1_CEN;

    /* Attendi completamento */
    u32 start_time = systick_get_tick();
    u32 last_log_step = 0;

    while (!g_move_rt_state.completed) {
        /* Timeout 60s */
        if (systick_timeout(start_time, 60000)) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
            g_move_rt_state.active = false;
            log_error("Movimento RT: timeout 60s");
            return -1;
        }

        /* Check emergenza */
        if (cnc_has_emergency()) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
            g_move_rt_state.active = false;
            log_warning("Movimento RT: abort emergenza");
            return -1;
        }

        /* Log ogni 100 step */
        if (g_move_rt_state.current_step >= last_log_step + 100) {
            last_log_step = g_move_rt_state.current_step;
            u8 perc = cnc_move_get_percentage_rt(&g_move_rt_state);
            u32 freq = cnc_move_get_speed_rt(&g_move_rt_state);
            log_info("Step:%u/%u (%u%%) Vel:%u Hz Pos:(%d,%d,%d)",
                     g_move_rt_state.current_step,
                     g_move_rt_state.total_steps,
                     perc, freq,
                     (int)g_move_rt_state.current.x,
                     (int)g_move_rt_state.current.y,
                     (int)g_move_rt_state.current.z);
        }

        __WFI();  /* Sleep fino a prossimo interrupt */
    }

    /* Ferma TIM2 */
    TIM2_CR1 &= ~TIM_CR1_CEN;
    g_move_rt_state.active = false;

    /* Copia stato finale */
    *state = g_move_rt_state;

    log_info("Movimento RT completato");
    return 0;
}

/* ============================================================================
 * FUNZIONI INTERNE - GESTIONE RECUPERO ASSI
 * ============================================================================
 */

static void cnc_handle_limit_interrupt(u8 axis, bool min_limit)
{
    /* -----------------------------------------------------------------------
     * FASE 1: DISABILITAZIONE IMMEDIATA TUTTI GLI ASSI COINVOLTI
     * ----------------------------------------------------------------------- */

    /* Verifica quali finecorsa sono attivi */
    bool x_min = (gpio_read(X_MIN_PORT, X_MIN_PIN) == 0);
    bool x_max = (gpio_read(X_MAX_PORT, X_MAX_PIN) == 0);
    bool y_min = (gpio_read(Y_MIN_PORT, Y_MIN_PIN) == 0);
    bool y_max = (gpio_read(Y_MAX_PORT, Y_MAX_PIN) == 0);
    bool z_min = (gpio_read(Z_MIN_PORT, Z_MIN_PIN) == 0);
    bool z_max = (gpio_read(Z_MAX_PORT, Z_MAX_PIN) == 0);

    /* Disabilita immediatamente TUTTI gli assi con finecorsa attivi */
    if (x_min || x_max) disable_axis(0);
    if (y_min || y_max) disable_axis(1);
    if (z_min || z_max) disable_axis(2);

    /* -----------------------------------------------------------------------
     * FASE 2: ACCODAMENTO ASSI DA RECUPERARE
     * ----------------------------------------------------------------------- */

    /* Reset coda (prima emergenza sovrascrive eventuali precedenti) */
    emergency_queue.count = 0;
    emergency_queue.current_index = 0;

    /* Aggiungi assi alla coda (ordine: X, Y, Z) */
    if (x_min) {
        emergency_queue.queue[emergency_queue.count].axis = 0;
        emergency_queue.queue[emergency_queue.count].min_limit = true;
        emergency_queue.count++;
    }
    if (x_max) {
        emergency_queue.queue[emergency_queue.count].axis = 0;
        emergency_queue.queue[emergency_queue.count].min_limit = false;
        emergency_queue.count++;
    }
    if (y_min) {
        emergency_queue.queue[emergency_queue.count].axis = 1;
        emergency_queue.queue[emergency_queue.count].min_limit = true;
        emergency_queue.count++;
    }
    if (y_max) {
        emergency_queue.queue[emergency_queue.count].axis = 1;
        emergency_queue.queue[emergency_queue.count].min_limit = false;
        emergency_queue.count++;
    }
    if (z_min) {
        emergency_queue.queue[emergency_queue.count].axis = 2;
        emergency_queue.queue[emergency_queue.count].min_limit = true;
        emergency_queue.count++;
    }
    if (z_max) {
        emergency_queue.queue[emergency_queue.count].axis = 2;
        emergency_queue.queue[emergency_queue.count].min_limit = false;
        emergency_queue.count++;
    }

    /* Imposta stato emergenza generale */
    current_state = ST_EMERGENCY_STOP;

    /* Log emergenza (NON usare log_error per evitare overhead in ISR) */
    /* Il main loop loggerà i dettagli */
}

static void cnc_start_axis_recovery(u8 axis, bool min_limit)
{
    /* Imposta stato allarme specifico */
    switch(axis) {
        case 0: current_state = ST_ALARM_X; break;
        case 1: current_state = ST_ALARM_Y; break;
        case 2: current_state = ST_ALARM_Z; break;
    }

    /* Salva asse in recupero */
    recovery_axis = axis;
    step_count = 0;
    step_state = false;

    /* Abilita solo questo asse */
    enable_axis(axis);

    /* Leggi direzione corrente e invertila */
    bool current_dir = read_dir(axis);
    bool new_dir = !current_dir;
    write_dir(axis, new_dir);

    /* Imposta frequenza TIM2 in base all'asse */
    u32 freq;
    switch(axis) {
        case 0: freq = EMER_STEP_FREQ_X; break;
        case 1: freq = EMER_STEP_FREQ_Y; break;
        case 2: freq = EMER_STEP_FREQ_Z; break;
        default: freq = 1000;
    }

    /* Calcola ARR per frequenza desiderata: ARR = (1MHz / freq) - 1 */
    TIM2_ARR = (1000000 / freq) - 1;

    /* Reset e avvia timer */
    TIM2_CNT = 0;
    TIM2_SR = 0;  // Clear flags
    TIM2_CR1 |= TIM_CR1_CEN;  // Avvia timer

    /* Log dettagliato (sarà bufferizzato) */
    EVAL_LOG(const char *axis_name = (axis == 0) ? "X" : (axis == 1) ? "Y" : "Z";)
    EVAL_LOG(const char *limit_name = min_limit ? "MIN" : "MAX";)
    log_warning("Recupero asse %s: finecorsa %s attivo, retrazione a %u Hz", axis_name, limit_name, freq);
}


/* ============================================================================
 * INTERRUPT SERVICE ROUTINES
 * ============================================================================
 */

/**
 * @brief ISR TIM2 - Generazione impulsi STEP
 * @details Gestisce TRE modalità:
 *          1. Movimento REAL-TIME (g_move_rt_state.active) - NUOVA! ✨
 *          2. Movimento ARRAY legacy (execute_moves_buffer != NULL) - DEPRECATA
 *          3. Recupero emergenza / homing (recovery_axis != 0xFF)
 */
void TIM2_IRQHandler(void)
{
    /* Clear interrupt flag */
    if (TIM2_SR & TIM_SR_UIF) {
        TIM2_SR &= ~TIM_SR_UIF;
    }

    /* ========================================================================
     * MODALITÀ 1: MOVIMENTO REAL-TIME (NUOVA API - step simultanei!)
     * ========================================================================
     */
    if (g_move_rt_state.active && !g_move_rt_state.completed) {
        static bool step_state_rt = false;
        step_state_rt = !step_state_rt;

        if (step_state_rt) {
            /* Rising edge: genera step multi-asse */
            u8 mask = cnc_move_step_rt((move_rt_state_t*)&g_move_rt_state);

            if (mask == 0) {
                /* Movimento completato */
                g_move_rt_state.completed = true;
                TIM2_CR1 &= ~TIM_CR1_CEN;
                return;
            }

            /* Set impulsi SIMULTANEI (step multi-asse!) */
            if (mask & 0x01) gpio_set(STEP_X_PORT, STEP_X_PIN);
            if (mask & 0x02) gpio_set(STEP_Y_PORT, STEP_Y_PIN);
            if (mask & 0x04) gpio_set(STEP_Z_PORT, STEP_Z_PIN);

            /* Calcola e aggiorna velocità on-the-fly */
            u32 new_freq = cnc_move_get_speed_rt(&g_move_rt_state);
            TIM2_ARR = (1000000 / new_freq) - 1;

        } else {
            /* Falling edge: reset tutti gli impulsi */
            gpio_reset(STEP_X_PORT, STEP_X_PIN);
            gpio_reset(STEP_Y_PORT, STEP_Y_PIN);
            gpio_reset(STEP_Z_PORT, STEP_Z_PIN);
        }

        return;
    }

    /* ========================================================================
     * MODALITÀ 2: RECUPERO EMERGENZA / HOMING
     * ========================================================================
     * Genera STEP per recovery emergenza O homing.
     * Durante homing: emergency_queue.count == 0 (solo genera STEP, polling esterno)
     * Durante recovery: emergency_queue.count > 0 (genera STEP + controlla finecorsa)
     */

    /* Se nessun asse in recupero, ferma timer */
    if (recovery_axis == 0xFF) {
        TIM2_CR1 &= ~TIM_CR1_CEN;
        return;
    }

    /* Toggle STEP pin */
    step_state = !step_state;
    write_step(recovery_axis, step_state);

    /* Conta solo fronte di salita (un impulso completo = HIGH + LOW) */
    if (step_state) {
        step_count++;
    }

    /* ========================================================================
     * VERIFICA RILASCIO FINECORSA - SOLO IN MODALITÀ RECOVERY (NON HOMING)
     * ========================================================================
     * Durante homing, il controllo finecorsa è fatto nel main loop con polling.
     * Durante recovery emergenza, controlliamo qui per auto-stop.
     */
    if (emergency_queue.count > 0 && emergency_queue.current_index > 0) {
        /* Ottieni info asse dalla coda */
        u8 current_idx = emergency_queue.current_index - 1;  // -1 perché già incrementato
        emergency_axis_t *ea = (emergency_axis_t *)&emergency_queue.queue[current_idx];

        bool limit_released = read_limit_switch(ea->axis, ea->min_limit);

        if (limit_released) {
            /* Finecorsa rilasciato! */

            /* Salva asse recuperato prima di resettarlo */
            u8 recovered_axis = recovery_axis;

            /* Ferma timer */
            TIM2_CR1 &= ~TIM_CR1_CEN;

            /* Assicurati che STEP sia LOW */
            write_step(recovery_axis, false);

            /* Disabilita asse */
            disable_axis(recovery_axis);

            /* Log completamento */
            EVAL_LOG( const char *axis_name = (recovery_axis == 0) ? "X" : (recovery_axis == 1) ? "Y" : "Z";)
            log_info("Asse %s recuperato: %u step eseguiti", axis_name, step_count);

            /* Marca asse come completato */
            recovery_axis = 0xFF;

            /* RIABILITA EXTI dopo recovery completato
             * Nota: riabilitiamo qui invece che in cnc_process_emergency per evitare
             * che EXTI scatti prima che il recovery sia completamente terminato */
            exti_enable_axis(recovered_axis);

            /* Il main loop processerà il prossimo asse in cnc_process_emergency() */
        }
    }
    /* Se emergency_queue.count == 0, siamo in HOMING:
     * - TIM2 genera solo STEP
     * - Il controllo finecorsa è fatto nel main loop con polling GPIO
     * - Nessun auto-stop, il main loop fermerà TIM2 quando necessario */
}

/**
 * @brief ISR EXTI2 - Finecorsa Y_MIN (PA2)
 * @details Con filtro debouncing temporale e auto-disable
 */
void EXTI2_IRQHandler(void)
{
    if (EXTI_PR & BIT(2)) {
        EXTI_PR = BIT(2);  // Clear pending flag

        /* Filtro debouncing temporale: ignora se < 50ms dall'ultimo */
        u32 now = systick_get_tick();
        if ((now - exti_last_time_y) < EXTI_DEBOUNCE_MS) {
            return;  /* Rimbalzo meccanico, ignora */
        }

        /* Interrupt valido: auto-disable EXTI asse Y */
        exti_disable_axis(1);

        /* Aggiorna timestamp */
        exti_last_time_y = now;

        /* Processa interrupt */
        cnc_handle_limit_interrupt(1, true);  // Asse Y, MIN
    }
}

/**
 * @brief ISR EXTI3 - Finecorsa Y_MAX (PA3)
 * @details Con filtro debouncing temporale e auto-disable
 */
void EXTI3_IRQHandler(void)
{
    if (EXTI_PR & BIT(3)) {
        EXTI_PR = BIT(3);

        /* Filtro debouncing temporale */
        u32 now = systick_get_tick();
        if ((now - exti_last_time_y) < EXTI_DEBOUNCE_MS) {
            return;
        }

        /* Auto-disable EXTI asse Y */
        exti_disable_axis(1);

        /* Aggiorna timestamp */
        exti_last_time_y = now;

        /* Processa interrupt */
        cnc_handle_limit_interrupt(1, false);  // Asse Y, MAX
    }
}

/**
 * @brief ISR EXTI4 - Finecorsa Z_MIN (PA4)
 * @details Con filtro debouncing temporale e auto-disable
 */
void EXTI4_IRQHandler(void)
{
    if (EXTI_PR & BIT(4)) {
        EXTI_PR = BIT(4);

        /* Filtro debouncing temporale */
        u32 now = systick_get_tick();
        if ((now - exti_last_time_z) < EXTI_DEBOUNCE_MS) {
            return;
        }

        /* Auto-disable EXTI asse Z */
        exti_disable_axis(2);

        /* Aggiorna timestamp */
        exti_last_time_z = now;

        /* Processa interrupt */
        cnc_handle_limit_interrupt(2, true);  // Asse Z, MIN
    }
}

/**
 * @brief ISR EXTI9_5 - Finecorsa Z_MAX (PA5), X_MIN (PB8), X_MAX (PB9)
 * @details Handler condiviso con filtro debouncing e auto-disable
 */
void EXTI9_5_IRQHandler(void)
{
    u32 now = systick_get_tick();

    /* PA5 - Z_MAX */
    if (EXTI_PR & BIT(5)) {
        EXTI_PR = BIT(5);

        /* Filtro debouncing */
        if ((now - exti_last_time_z) >= EXTI_DEBOUNCE_MS) {
            /* Auto-disable EXTI asse Z */
            exti_disable_axis(2);

            /* Aggiorna timestamp */
            exti_last_time_z = now;

            /* Processa interrupt */
            cnc_handle_limit_interrupt(2, false);  // Asse Z, MAX
        }
    }

    /* PB8 - X_MIN */
    if (EXTI_PR & BIT(8)) {
        EXTI_PR = BIT(8);

        /* Filtro debouncing */
        if ((now - exti_last_time_x) >= EXTI_DEBOUNCE_MS) {
            /* Auto-disable EXTI asse X */
            exti_disable_axis(0);

            /* Aggiorna timestamp */
            exti_last_time_x = now;

            /* Processa interrupt */
            cnc_handle_limit_interrupt(0, true);  // Asse X, MIN
        }
    }

    /* PB9 - X_MAX */
    if (EXTI_PR & BIT(9)) {
        EXTI_PR = BIT(9);

        /* Filtro debouncing */
        if ((now - exti_last_time_x) >= EXTI_DEBOUNCE_MS) {
            /* Auto-disable EXTI asse X */
            exti_disable_axis(0);

            /* Aggiorna timestamp */
            exti_last_time_x = now;

            /* Processa interrupt */
            cnc_handle_limit_interrupt(0, false);  // Asse X, MAX
        }
    }
}
