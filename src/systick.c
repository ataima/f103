/**
 * @file systick.c
 * @brief Implementazione timer di sistema basato su SysTick
 * @details
 * Il SysTick è un timer hardware a 24-bit integrato nel core ARM Cortex-M3.
 * Viene utilizzato per generare interrupt periodici ogni 1ms, fornendo
 * un tick counter affidabile per timing e scheduling.
 *
 * REGISTRI SYSTICK (Cortex-M3):
 * ================================
 * Base address: 0xE000E010
 *
 * SYST_CSR (Control and Status Register) - Offset 0x00:
 *   Bit 16: COUNTFLAG - Set a 1 quando counter raggiunge 0 (read-clear)
 *   Bit 2:  CLKSOURCE - 0=external clock, 1=processor clock (HCLK)
 *   Bit 1:  TICKINT   - 0=no interrupt, 1=interrupt quando counter=0
 *   Bit 0:  ENABLE    - 0=counter disabled, 1=counter enabled
 *
 * SYST_RVR (Reload Value Register) - Offset 0x04:
 *   Bits 23-0: RELOAD - Valore da ricaricare nel counter (max 0xFFFFFF)
 *
 * SYST_CVR (Current Value Register) - Offset 0x08:
 *   Bits 23-0: CURRENT - Valore corrente del counter (scrittura azzera)
 *
 * SYST_CALIB (Calibration Value Register) - Offset 0x0C:
 *   Non usato in questa implementazione
 */

#include "systick.h"
#include "log.h"

/* ============================================================================
 * ALIAS REGISTRI SYSTICK
 * ============================================================================
 * I registri SysTick sono già definiti in stm32f103_regs.h.
 * Creiamo degli alias per compatibilità con i nomi usati nei commenti.
 */

/* Alias per i registri (usano le definizioni da stm32f103_regs.h) */
#define SYST_CSR            SYSTICK_CTRL   /* Control and Status Register */
#define SYST_RVR            SYSTICK_LOAD   /* Reload Value Register */
#define SYST_CVR            SYSTICK_VAL    /* Current Value Register */

/* Alias per i bit (usano le definizioni da stm32f103_regs.h) */
#define SYST_CSR_ENABLE     SYSTICK_CTRL_ENABLE
#define SYST_CSR_TICKINT    SYSTICK_CTRL_TICKINT
#define SYST_CSR_CLKSOURCE  SYSTICK_CTRL_CLKSOURCE
#define SYST_CSR_COUNTFLAG  SYSTICK_CTRL_COUNTFLAG

/* Costanti */
#define SYSTICK_MAX_RELOAD  0x00FFFFFFUL  /* Valore massimo per RVR (24-bit) */


/* ============================================================================
 * VARIABILI GLOBALI PRIVATE
 * ============================================================================
 */

/**
 * @brief Tick counter incrementato dall'interrupt ogni 1ms
 * @note volatile perché modificato dall'ISR
 * @note u32 permette ~49.7 giorni prima del wraparound
 */
static volatile u32 systick_counter = 0;

/**
 * @brief Contatore interrupt per statistiche/debug
 */
static volatile u32 systick_interrupt_count = 0;

/**
 * @brief Flag di inizializzazione
 */
static volatile bool systick_initialized = false;


/* ============================================================================
 * INTERRUPT SERVICE ROUTINE
 * ============================================================================
 */

/**
 * @brief Interrupt handler del SysTick
 * @details Questa funzione viene chiamata automaticamente ogni 1ms dall'hardware.
 *          Il nome "SysTick_Handler" è standard ARM e definito nel vettore
 *          degli interrupt (startup_stm32f103c8tx.s).
 *
 * @note Questa ISR deve essere VELOCE: incrementa solo i counter
 * @note NON chiamare funzioni che usano il logging (possibile deadlock)
 * @note L'interrupt viene automaticamente acknowledged dalla lettura di SYST_CSR
 */
void SysTick_Handler(void)
{
    /* Incrementa il tick counter (ogni 1ms) */
    systick_counter++;

    /* Incrementa contatore statistiche */
    systick_interrupt_count++;

    /* NOTA: Non è necessario clear del flag interrupt - fatto automaticamente
     *       dalla lettura del registro SYST_CSR da parte dell'hardware */
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - INIZIALIZZAZIONE
 * ============================================================================
 */

/**
 * @brief Inizializza il SysTick timer per interrupt a 1ms
 */
int systick_init(u32 sysclk_hz)
{
    u32 reload_value;

    /* -----------------------------------------------------------------------
     * VALIDAZIONE PARAMETRI
     * ----------------------------------------------------------------------- */

    /* Verifica che SYSCLK sia ragionevole (almeno 1MHz) */
    if (sysclk_hz < 1000000) {
        log_error("SysTick init: SYSCLK troppo bassa (%u Hz)", sysclk_hz);
        return SYSTICK_ERROR;
    }

    /* Calcola reload value per 1ms tick: reload = (SYSCLK / 1000) - 1 */
    reload_value = (sysclk_hz / 1000) - 1;

    /* Verifica che il reload value stia nel range 24-bit */
    if (reload_value > SYSTICK_MAX_RELOAD) {
        log_error("SysTick init: reload value troppo grande (%u)", reload_value);
        return SYSTICK_ERROR;
    }

    log_debug("SysTick init: SYSCLK=%u Hz, reload=%u", sysclk_hz, reload_value);

    /* -----------------------------------------------------------------------
     * CONFIGURAZIONE REGISTRI SYSTICK
     * ----------------------------------------------------------------------- */

    /* STEP 1: Disabilita SysTick durante configurazione */
    SYST_CSR = 0;

    /* STEP 2: Azzera il tick counter software */
    systick_counter = 0;
    systick_interrupt_count = 0;

    /* STEP 3: Imposta il reload value (determina la frequenza degli interrupt) */
    SYST_RVR = reload_value;

    /* STEP 4: Azzera il current value (il counter parte da SYST_RVR) */
    SYST_CVR = 0;  /* Qualsiasi scrittura azzera il counter */

    /* STEP 5: Abilita SysTick con interrupt usando HCLK come clock source
     *
     * SYST_CSR bits:
     * - ENABLE (bit 0) = 1: abilita counter
     * - TICKINT (bit 1) = 1: abilita interrupt quando counter raggiunge 0
     * - CLKSOURCE (bit 2) = 1: usa processor clock (HCLK = SYSCLK)
     */
    SYST_CSR = SYST_CSR_ENABLE | SYST_CSR_TICKINT | SYST_CSR_CLKSOURCE;

    systick_initialized = true;

    log_info("SysTick inizializzato: 1ms tick @ %u Hz", sysclk_hz);

    return SYSTICK_OK;
}

/**
 * @brief Disabilita il SysTick timer
 */
void systick_disable(void)
{
    /* Disabilita counter e interrupt */
    SYST_CSR = 0;

    systick_initialized = false;

    log_info("SysTick disabilitato");
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - TICK COUNTER
 * ============================================================================
 */

/**
 * @brief Ottiene il valore corrente del tick counter
 */
u32 systick_get_tick(void)
{
    /* La lettura di u32 è atomica su Cortex-M3, non serve disabilitare interrupt */
    return systick_counter;
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - DELAY
 * ============================================================================
 */

/**
 * @brief Delay bloccante in millisecondi
 */
void systick_delay_ms(u32 ms)
{
    u32 start_tick;

    /* Se delay è 0, return immediato */
    if (ms == 0) {
        return;
    }

    /* Salva il tick corrente */
    start_tick = systick_get_tick();

    /* Attendi fino a quando non sono passati 'ms' millisecondi
     *
     * NOTA: La sottrazione (systick_get_tick() - start_tick) gestisce
     *       correttamente il wraparound grazie all'aritmetica modulo 2^32.
     *
     * Esempio con wraparound:
     * start_tick = 0xFFFFFFFF (ultimo tick prima del wrap)
     * ms = 10
     * Dopo 10ms: systick_get_tick() = 0x00000009
     * Differenza: 0x00000009 - 0xFFFFFFFF = 0x0000000A = 10 (corretto!)
     */
    while ((systick_get_tick() - start_tick) < ms) {
        /* Mette la CPU in sleep fino al prossimo interrupt
         * Questo riduce il consumo energetico rispetto a busy-wait puro
         * La CPU si risveglia ad ogni interrupt (incluso SysTick) */
        __WFI();
    }
}

/**
 * @brief Verifica se è trascorso un certo tempo da un tick di riferimento
 */
bool systick_timeout(u32 start, u32 ms)
{
    /* La sottrazione gestisce il wraparound (vedi commenti in systick_delay_ms) */
    return (systick_get_tick() - start) >= ms;
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - STATISTICHE
 * ============================================================================
 */

/**
 * @brief Ottiene il numero totale di interrupt SysTick processati
 */
u32 systick_get_interrupt_count(void)
{
    return systick_interrupt_count;
}

/**
 * @brief Ottiene il valore corrente del SysTick hardware counter
 */
u32 systick_get_hw_counter(void)
{
    /* Legge il current value register (24-bit down counter)
     * Il valore conta all'INDIETRO da SYST_RVR verso 0 */
    return SYST_CVR & 0x00FFFFFF;
}
