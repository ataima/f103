/**
 * @file systick.h
 * @brief Timer di sistema basato su SysTick per scheduling a 1ms
 * @details
 * Questo modulo implementa un timer di sistema utilizzando il SysTick timer
 * integrato nel core ARM Cortex-M3. Il SysTick genera interrupt ogni 1ms,
 * fornendo un tick counter affidabile per:
 * - Funzioni di delay non bloccanti
 * - Misurazione del tempo
 * - Scheduling e timeout
 * - Debouncing e timing vari
 *
 * CARATTERISTICHE SYSTICK:
 * - Timer hardware integrato nel core ARM Cortex-M3
 * - 24-bit down counter
 * - Clock source: HCLK/8 o HCLK (configurabile)
 * - Interrupt dedicato con priorità configurabile
 * - Non richiede periferiche esterne
 *
 * CONFIGURAZIONE:
 * - Frequenza tick: 1000 Hz (1ms per tick)
 * - Clock source: HCLK (72 MHz)
 * - Reload value: 72000 - 1 = 71999
 * - Range tick counter: 0 a 2^32-1 (wraparound automatico dopo ~49 giorni)
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include "common.h"

/* ============================================================================
 * CODICI DI RITORNO
 * ============================================================================
 */
#define SYSTICK_OK              0
#define SYSTICK_ERROR           -1


/* ============================================================================
 * FUNZIONI PUBBLICHE - INIZIALIZZAZIONE
 * ============================================================================
 */

/**
 * @brief Inizializza il SysTick timer per interrupt a 1ms
 * @details Configura il SysTick per generare interrupt ogni 1ms usando HCLK come sorgente.
 *          Con SYSCLK a 72 MHz:
 *          - Reload value = (72000000 / 1000) - 1 = 71999
 *          - Ogni tick = 1ms
 *
 *          Registri configurati:
 *          - SYST_RVR: Reload Value Register (valore di ricarica)
 *          - SYST_CVR: Current Value Register (azzerato per partire subito)
 *          - SYST_CSR: Control and Status Register (enable timer + interrupt)
 *
 * @param sysclk_hz Frequenza SYSCLK in Hz (deve essere 72000000 per questo progetto)
 * @return SYSTICK_OK se successo, SYSTICK_ERROR se parametri non validi
 *
 * @note Questa funzione deve essere chiamata DOPO SystemClock_Config()
 * @note Il SysTick usa l'interrupt handler SysTick_Handler() definito in systick.c
 */
int systick_init(u32 sysclk_hz);

/**
 * @brief Disabilita il SysTick timer
 * @details Ferma il timer e disabilita gli interrupt.
 *          Il tick counter viene preservato ma non incrementato.
 */
void systick_disable(void);


/* ============================================================================
 * FUNZIONI PUBBLICHE - TICK COUNTER
 * ============================================================================
 */

/**
 * @brief Ottiene il valore corrente del tick counter
 * @details Il tick counter viene incrementato ogni 1ms dall'interrupt SysTick.
 *          Dopo 2^32-1 tick (~49.7 giorni) il counter fa wraparound a 0.
 *
 * @return Numero di tick trascorsi dall'inizializzazione (in millisecondi)
 *
 * @note Thread-safe: la lettura di u32 è atomica su Cortex-M3
 * @note Attenzione al wraparound: per misurare differenze di tempo usa
 *       la sottrazione (tick_end - tick_start) che gestisce correttamente
 *       l'overflow grazie all'aritmetica modulo 2^32
 */
u32 systick_get_tick(void);

/**
 * @brief Ottiene il numero di tick trascorsi dall'ultimo avvio
 * @details Alias per systick_get_tick(), per compatibilità con altre API
 * @return Tick counter corrente in millisecondi
 */
static inline u32 systick_millis(void) {
    return systick_get_tick();
}


/* ============================================================================
 * FUNZIONI PUBBLICHE - DELAY
 * ============================================================================
 */

/**
 * @brief Delay bloccante in millisecondi
 * @details Attende per il numero specificato di millisecondi usando il SysTick.
 *          Questa funzione è bloccante: la CPU resta in loop fino allo scadere del tempo.
 *
 * @param ms Numero di millisecondi da attendere (0 = return immediato)
 *
 * @note Gestisce correttamente il wraparound del tick counter
 * @note Precisione: ±1ms (dipende dal momento in cui viene chiamata rispetto all'interrupt)
 * @note Non usa busy-wait della CPU: chiama __WFI() per mettere CPU in sleep
 *
 * @example
 * systick_delay_ms(500);  // Attende 500ms
 */
void systick_delay_ms(u32 ms);

/**
 * @brief Verifica se è trascorso un certo tempo da un tick di riferimento
 * @details Controlla se sono passati almeno 'ms' millisecondi dal tick 'start'.
 *          Gestisce correttamente il wraparound del tick counter.
 *
 * @param start Tick di riferimento iniziale (ottenuto con systick_get_tick())
 * @param ms Numero di millisecondi da verificare
 * @return true se sono trascorsi >= ms millisecondi, false altrimenti
 *
 * @note Questa funzione è non-bloccante: ritorna immediatamente
 * @note Utile per implementare timeout e delay non bloccanti
 *
 * @example
 * u32 start = systick_get_tick();
 * while (!systick_timeout(start, 1000)) {
 *     // Fai qualcosa per max 1 secondo
 *     if (condition_met) break;
 * }
 */
bool systick_timeout(u32 start, u32 ms);


/* ============================================================================
 * FUNZIONI PUBBLICHE - STATISTICHE
 * ============================================================================
 */

/**
 * @brief Ottiene il numero totale di interrupt SysTick processati
 * @details Utile per debug e verifica che il SysTick funzioni correttamente.
 * @return Numero di interrupt SysTick dall'inizializzazione
 */
u32 systick_get_interrupt_count(void);

/**
 * @brief Ottiene il valore corrente del SysTick hardware counter (24-bit)
 * @details Il SysTick hardware è un down-counter che parte da SYST_RVR e
 *          conta verso 0. Quando raggiunge 0, viene ricaricato e genera interrupt.
 *          Questo valore può essere usato per timing ad alta risoluzione.
 *
 * @return Valore corrente del counter (0 a SYST_RVR)
 *
 * @note Risoluzione: ~13.9ns @ 72MHz (1 tick HW = 1 ciclo CPU)
 * @note Il valore conta all'INDIETRO (da RVR verso 0)
 * @note Utile per misure di tempo sub-millisecondo
 */
u32 systick_get_hw_counter(void);


/* ============================================================================
 * FUNZIONI PUBBLICHE - CALLBACK ENCODER (per integrazione CNC)
 * ============================================================================
 */

/**
 * @brief Registra callback per lettura encoder a 50Hz
 * @details Il callback viene invocato dal SysTick ISR ogni 20ms (50Hz).
 *          Utile per leggere encoder senza polling nel main loop.
 *
 * @param callback Puntatore a funzione void(void) da chiamare ogni 20ms
 *                 Impostare a NULL per disabilitare callback
 *
 * @note Il callback viene eseguito in contesto interrupt (ISR)!
 * @note Il callback deve essere VELOCE (pochi µs) per non degradare timing
 * @note NON usare logging o operazioni pesanti nel callback
 *
 * @example
 * void encoder_update_callback(void) {
 *     // Lettura encoder veloce
 *     cnc_update_encoder_positions();
 * }
 * systick_register_encoder_callback(encoder_update_callback);
 */
void systick_register_encoder_callback(void (*callback)(void));


/* ============================================================================
 * FUNZIONI INLINE - CONVERSIONI TEMPO
 * ============================================================================
 */

/**
 * @brief Converte millisecondi in tick
 * @param ms Millisecondi
 * @return Numero di tick (1 tick = 1ms)
 */
static inline u32 systick_ms_to_ticks(u32 ms) {
    return ms;  // 1 tick = 1ms
}

/**
 * @brief Converte tick in millisecondi
 * @param ticks Numero di tick
 * @return Millisecondi
 */
static inline u32 systick_ticks_to_ms(u32 ticks) {
    return ticks;  // 1 tick = 1ms
}

/**
 * @brief Converte secondi in tick
 * @param seconds Secondi
 * @return Numero di tick
 */
static inline u32 systick_seconds_to_ticks(u32 seconds) {
    return seconds * 1000;
}

#endif /* SYSTICK_H */
