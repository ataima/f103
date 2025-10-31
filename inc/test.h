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
#define ENABLE_TEST_LIMIT       0  // Disabilitato - test completato

/**
 * @brief Abilita test STEP I/O (loopback STEP → LIMIT_MIN)
 * @details Testa i pin STEP_X/Y/Z con loopback su X/Y/Z_MIN.
 *          Collegamento esterno richiesto:
 *          - PA0 (STEP_X) → PB8 (X_MIN)
 *          - PA1 (STEP_Y) → PA2 (Y_MIN)
 *          - PB0 (STEP_Z) → PA4 (Z_MIN)
 */
#define ENABLE_TEST_STEP_IO     0

/**
 * @brief Abilita test DIR I/O (loopback DIR → LIMIT_MAX)
 * @details Testa i pin DIR_X/Y/Z con loopback su X/Y/Z_MAX.
 *          Collegamento esterno richiesto:
 *          - PB12 (DIR_X) → PB9 (X_MAX)
 *          - PB13 (DIR_Y) → PA3 (Y_MAX)
 *          - PB14 (DIR_Z) → PA5 (Z_MAX)
 */
#define ENABLE_TEST_DIR_IO      0

/**
 * @brief Abilita test ENABLE I/O (loopback ENABLE → ENCODER_A)
 * @details Testa i pin ENABLE_X/Y/Z con loopback su ENC_X/Y/Z_A.
 *          Collegamento esterno richiesto:
 *          - PB3 (ENABLE_X) → PA6 (ENC_X_A)
 *          - PB4 (ENABLE_Y) → PB6 (ENC_Y_A)
 *          - PB5 (ENABLE_Z) → PA8 (ENC_Z_A)
 */
#define ENABLE_TEST_ENABLE_IO   0


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

#if ENABLE_TEST_STEP_IO
#define EVAL_TEST_STEP_IO(MSG)          MSG
#else
#define EVAL_TEST_STEP_IO(MSG)          /* Niente - codice rimosso dal preprocessore */
#endif

#if ENABLE_TEST_DIR_IO
#define EVAL_TEST_DIR_IO(MSG)           MSG
#else
#define EVAL_TEST_DIR_IO(MSG)           /* Niente - codice rimosso dal preprocessore */
#endif

#if ENABLE_TEST_ENABLE_IO
#define EVAL_TEST_ENABLE_IO(MSG)        MSG
#else
#define EVAL_TEST_ENABLE_IO(MSG)        /* Niente - codice rimosso dal preprocessore */
#endif





/* ============================================================================
 * CONFIGURAZIONE TEST
 * ============================================================================
 */

/**
 * @brief Abilita test movimento Bresenham 3D
 * @details Se impostato a 1, esegue un movimento di test prima del main loop.
 *          Movimento: da (0,0,0) a (500,300,200) con profilo trapezoidale.
 *          Log posizione ogni 100 step.
 */
#define ENABLE_BRESENHAM_TEST   1

#if ENABLE_BRESENHAM_TEST
#define EVAL_TEST_BRESENHAM(MSG)        MSG
#else
#define EVAL_TEST_BRESENHAM(MSG)        /* Niente - codice rimosso dal preprocessore */
#endif



/**
 * @brief Test movimento 3D con algoritmo Bresenham
 * @details Esegue un movimento lineare 3D da origine a (500,300,200).
 *          Applica profilo velocità trapezoidale 500Hz-5000Hz.
 *          Logga posizione ogni 100 step (gestito da cnc_execute_moves).
 */

#if ENABLE_BRESENHAM_TEST
void test_bresenham_movement(void);
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
 * FUNZIONI PUBBLICHE - TEST STEP I/O
 * ============================================================================
 */

#if ENABLE_TEST_STEP_IO

/**
 * @brief Test loopback STEP pins → LIMIT_MIN pins
 * @details Verifica i pin STEP_X/Y/Z facendo toggle e leggendo X/Y/Z_MIN.
 *          Richiede collegamenti esterni:
 *          - PA0 (STEP_X) → PB8 (X_MIN)
 *          - PA1 (STEP_Y) → PA2 (Y_MIN)
 *          - PB0 (STEP_Z) → PA4 (Z_MIN)
 *
 * @return true se almeno un mismatch rilevato (LED deve fermarsi)
 *         false se tutti gli I/O funzionano correttamente
 *
 * @note Toggle ogni ~500ms per permettere debug visivo
 */
bool test_step_io(void);

#endif /* ENABLE_TEST_STEP_IO */


/* ============================================================================
 * FUNZIONI PUBBLICHE - TEST DIR I/O
 * ============================================================================
 */

#if ENABLE_TEST_DIR_IO

/**
 * @brief Test loopback DIR pins → LIMIT_MAX pins
 * @details Verifica i pin DIR_X/Y/Z facendo toggle e leggendo X/Y/Z_MAX.
 *          Richiede collegamenti esterni:
 *          - PB12 (DIR_X) → PB9 (X_MAX)
 *          - PB13 (DIR_Y) → PA3 (Y_MAX)
 *          - PB14 (DIR_Z) → PA5 (Z_MAX)
 *
 * @return true se almeno un mismatch rilevato (LED deve fermarsi)
 *         false se tutti gli I/O funzionano correttamente
 *
 * @note Toggle ogni ~500ms per permettere debug visivo
 */
bool test_dir_io(void);

#endif /* ENABLE_TEST_DIR_IO */


/* ============================================================================
 * FUNZIONI PUBBLICHE - TEST ENABLE I/O
 * ============================================================================
 */

#if ENABLE_TEST_ENABLE_IO

/**
 * @brief Test loopback ENABLE pins → ENCODER_A pins
 * @details Verifica i pin ENABLE_X/Y/Z facendo toggle e leggendo ENC_X/Y/Z_A.
 *          Richiede collegamenti esterni:
 *          - PB3 (ENABLE_X) → PA6 (ENC_X_A)
 *          - PB4 (ENABLE_Y) → PB6 (ENC_Y_A)
 *          - PB5 (ENABLE_Z) → PA8 (ENC_Z_A)
 *
 * @return true se almeno un mismatch rilevato (LED deve fermarsi)
 *         false se tutti gli I/O funzionano correttamente
 *
 * @note Toggle ogni ~500ms per permettere debug visivo
 */
bool test_enable_io(void);

#endif /* ENABLE_TEST_ENABLE_IO */

#endif /* TEST_H */
