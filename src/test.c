/**
 * @file test.c
 * @brief Implementazione funzioni di test hardware CNC
 * @details
 * Questo modulo implementa funzioni di test per verificare il corretto
 * funzionamento dei componenti hardware del controller CNC.
 */

#include "test.h"
#include "gpio.h"
#include "log.h"

/* ============================================================================
 * TEST FINECORSA
 * ============================================================================
 */

#if ENABLE_TEST_LIMIT

/**
 * @brief Struttura per memorizzare lo stato precedente dei finecorsa
 * @details Usata per rilevare transizioni (edge detection) ed evitare
 *          spam di log quando un finecorsa rimane premuto.
 */
typedef struct {
    bool x_min;
    bool x_max;
    bool y_min;
    bool y_max;
    bool z_min;
    bool z_max;
} limit_switch_state_t;

/**
 * @brief Stato precedente dei finecorsa
 * @note static per mantenere lo stato tra chiamate successive
 */
static limit_switch_state_t prev_state = {
    .x_min = true,  // true = non premuto (pull-up)
    .x_max = true,
    .y_min = true,
    .y_max = true,
    .z_min = true,
    .z_max = true
};

/**
 * @brief Flag di prima esecuzione per inizializzazione
 */
static bool first_run = true;


/**
 * @brief Test continuo dei finecorsa
 */
bool test_limit(void)
{
    bool any_limit_active = false;

    /* -----------------------------------------------------------------------
     * INIZIALIZZAZIONE PRIMA ESECUZIONE
     * ----------------------------------------------------------------------- */
    if (first_run) {
        log_info("=== Test Finecorsa Attivo ===");
        log_info("Premi i finecorsa per testare. LED si ferma quando attivo.");
        first_run = false;
    }

    /* -----------------------------------------------------------------------
     * LETTURA STATO CORRENTE FINECORSA
     * -----------------------------------------------------------------------
     * Nota: gpio_read() ritorna 1 (HIGH) quando NON premuto (pull-up)
     *                     ritorna 0 (LOW) quando premuto (collegato a GND)
     */

    bool x_min_current = gpio_read(X_MIN_PORT, X_MIN_PIN);
    bool x_max_current = gpio_read(X_MAX_PORT, X_MAX_PIN);
    bool y_min_current = gpio_read(Y_MIN_PORT, Y_MIN_PIN);
    bool y_max_current = gpio_read(Y_MAX_PORT, Y_MAX_PIN);
    bool z_min_current = gpio_read(Z_MIN_PORT, Z_MIN_PIN);
    bool z_max_current = gpio_read(Z_MAX_PORT, Z_MAX_PIN);

    /* -----------------------------------------------------------------------
     * RILEVAMENTO TRANSIZIONI E LOGGING
     * -----------------------------------------------------------------------
     * Logga solo quando avviene una transizione (edge):
     * - 1→0 (HIGH→LOW): finecorsa premuto
     * - 0→1 (LOW→HIGH): finecorsa rilasciato
     */

    /* X MIN */
    if (x_min_current != prev_state.x_min) {
        if (x_min_current == 0) {
            log_warning("X_MIN attivato! (PB8)");
        } else {
            log_info("X_MIN rilasciato");
        }
        prev_state.x_min = x_min_current;
    }

    /* X MAX */
    if (x_max_current != prev_state.x_max) {
        if (x_max_current == 0) {
            log_warning("X_MAX attivato! (PB9)");
        } else {
            log_info("X_MAX rilasciato");
        }
        prev_state.x_max = x_max_current;
    }

    /* Y MIN */
    if (y_min_current != prev_state.y_min) {
        if (y_min_current == 0) {
            log_warning("Y_MIN attivato! (PA2)");
        } else {
            log_info("Y_MIN rilasciato");
        }
        prev_state.y_min = y_min_current;
    }

    /* Y MAX */
    if (y_max_current != prev_state.y_max) {
        if (y_max_current == 0) {
            log_warning("Y_MAX attivato! (PA3)");
        } else {
            log_info("Y_MAX rilasciato");
        }
        prev_state.y_max = y_max_current;
    }

    /* Z MIN */
    if (z_min_current != prev_state.z_min) {
        if (z_min_current == 0) {
            log_warning("Z_MIN attivato! (PA4)");
        } else {
            log_info("Z_MIN rilasciato");
        }
        prev_state.z_min = z_min_current;
    }

    /* Z MAX */
    if (z_max_current != prev_state.z_max) {
        if (z_max_current == 0) {
            log_warning("Z_MAX attivato! (PA5)");
        } else {
            log_info("Z_MAX rilasciato");
        }
        prev_state.z_max = z_max_current;
    }

    /* -----------------------------------------------------------------------
     * VERIFICA SE ALMENO UN FINECORSA È ATTIVO
     * -----------------------------------------------------------------------
     * Ritorna true se almeno un finecorsa è premuto (livello LOW = 0)
     * Questo fermerà il LED nel main loop
     */
    if (x_min_current == 0 || x_max_current == 0 ||
        y_min_current == 0 || y_max_current == 0 ||
        z_min_current == 0 || z_max_current == 0) {
        any_limit_active = true;
    }

    return any_limit_active;
}

#endif /* ENABLE_TEST_LIMIT */
