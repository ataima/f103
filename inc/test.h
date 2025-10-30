/**
 * @file test.h
 * @brief Funzioni di test per validazione hardware CNC
 * @details
 * Questo modulo contiene funzioni di test per verificare il corretto
 * funzionamento dell'hardware del controller CNC, inclusi:
 * - Test finecorsa (limit switches)
 * - Test encoder (future)
 * - Test motori (future)
 * - Test I/O generici (future)
 *
 * I test possono essere abilitati/disabilitati tramite macro per ottimizzare
 * le dimensioni del firmware in produzione.
 */

#ifndef TEST_H
#define TEST_H

#include "common.h"

/* ============================================================================
 * CONFIGURAZIONE TEST - ABILITA/DISABILITA
 * ============================================================================
 * Imposta a 1 per abilitare il test, 0 per disabilitare.
 * Quando disabilitato, il codice del test viene completamente rimosso
 * dal binario grazie alle macro EVAL_TEST_*.
 */

/**
 * @brief Abilita test finecorsa
 * @details Quando abilitato, il test monitora continuamente tutti i finecorsa
 *          e blocca il LED di stato quando uno viene attivato.
 */
#define ENABLE_TEST_LIMIT   1


/* ============================================================================
 * MACRO DI VALUTAZIONE CONDIZIONALE
 * ============================================================================
 * Queste macro permettono di includere/escludere codice in base alla
 * configurazione dei test, rendendo il codice più leggibile rispetto a
 * blocchi #if/#endif sparsi nel codice.
 *
 * Utilizzo:
 *   EVAL_TEST_LIMIT(test_limit());  // Esegue test_limit() solo se abilitato
 */

#if ENABLE_TEST_LIMIT
#define EVAL_TEST_LIMIT(MSG)            MSG
#else
#define EVAL_TEST_LIMIT(MSG)            /* Niente - codice rimosso dal preprocessore */
#endif


/* ============================================================================
 * FUNZIONI PUBBLICHE - TEST FINECORSA
 * ============================================================================
 */

#if ENABLE_TEST_LIMIT

/**
 * @brief Test continuo dei finecorsa
 * @details Verifica lo stato di tutti i 6 finecorsa (X/Y/Z MIN/MAX) e:
 *          - Logga un messaggio quando un finecorsa viene attivato (transizione 1→0)
 *          - Blocca il LED di stato quando almeno un finecorsa è attivo
 *          - Riprende il lampeggio LED quando tutti i finecorsa sono rilasciati
 *
 * @return true se almeno un finecorsa è attivo (LED deve fermarsi)
 *         false se tutti i finecorsa sono rilasciati (LED può lampeggiare)
 *
 * @note Questa funzione deve essere chiamata nel main loop
 * @note Utilizza il sistema di logging per tracciare gli eventi
 * @note Mantiene uno stato interno per rilevare transizioni (evita spam di log)
 *
 * @example
 * while(1) {
 *     bool limit_active = EVAL_TEST_LIMIT(test_limit());
 *     if (!limit_active) {
 *         // Esegui pattern LED normale
 *     }
 * }
 */
bool test_limit(void);

#endif /* ENABLE_TEST_LIMIT */


/* ============================================================================
 * FUNZIONI PUBBLICHE - TEST FUTURI (PLACEHOLDER)
 * ============================================================================
 */

// Qui verranno aggiunti altri test in futuro:
// - test_encoders() - Verifica encoder rotativi
// - test_motors() - Test step/dir/enable dei motori
// - test_io() - Test I/O generici

#endif /* TEST_H */
