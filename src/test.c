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
#include "systick.h"

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


/* ============================================================================
 * TEST STEP I/O (LOOPBACK)
 * ============================================================================
 */

#if ENABLE_TEST_STEP_IO

/**
 * @brief Test loopback STEP pins → LIMIT_MIN pins
 */
bool test_step_io(void)
{
    static bool first_run = true;
    static u32 last_toggle_time = 0;
    static bool output_state = false;  // Stato corrente degli output
    bool mismatch_detected = false;

    /* Inizializzazione prima esecuzione */
    if (first_run) {
        log_info("=== Test STEP I/O Attivo ===");
        log_info("Collega: PA0->PB8, PA1->PA2, PB0->PA4");
        last_toggle_time = systick_get_tick();
        first_run = false;
    }

    /* Toggle output ogni 500ms */
    if (systick_timeout(last_toggle_time, 500)) {
        output_state = !output_state;
        last_toggle_time = systick_get_tick();

        /* Imposta output STEP */
        if (output_state) {
            gpio_set(STEP_X_PORT, STEP_X_PIN);
            gpio_set(STEP_Y_PORT, STEP_Y_PIN);
            gpio_set(STEP_Z_PORT, STEP_Z_PIN);
        } else {
            gpio_reset(STEP_X_PORT, STEP_X_PIN);
            gpio_reset(STEP_Y_PORT, STEP_Y_PIN);
            gpio_reset(STEP_Z_PORT, STEP_Z_PIN);
        }

        log_debug("STEP outputs -> %d", output_state);
    }

    /* Piccolo delay per stabilizzazione segnale */
    systick_delay_ms(10);

    /* Leggi input corrispondenti */
    bool x_min_read = gpio_read(X_MIN_PORT, X_MIN_PIN);
    bool y_min_read = gpio_read(Y_MIN_PORT, Y_MIN_PIN);
    bool z_min_read = gpio_read(Z_MIN_PORT, Z_MIN_PIN);

    /* Verifica corrispondenza (nota: i pin hanno pull-up, quindi invertiti) */
    bool expected_input = !output_state;  // Inversione per pull-up

    if (x_min_read != expected_input) {
        log_error("STEP_X mismatch! Output=%d, X_MIN=%d", output_state, x_min_read);
        mismatch_detected = true;
    }
    if (y_min_read != expected_input) {
        log_error("STEP_Y mismatch! Output=%d, Y_MIN=%d", output_state, y_min_read);
        mismatch_detected = true;
    }
    if (z_min_read != expected_input) {
        log_error("STEP_Z mismatch! Output=%d, Z_MIN=%d", output_state, z_min_read);
        mismatch_detected = true;
    }

    return mismatch_detected;
}

#endif /* ENABLE_TEST_STEP_IO */


/* ============================================================================
 * TEST DIR I/O (LOOPBACK)
 * ============================================================================
 */

#if ENABLE_TEST_DIR_IO

/**
 * @brief Test loopback DIR pins → LIMIT_MAX pins
 */
bool test_dir_io(void)
{
    static bool first_run = true;
    static u32 last_toggle_time = 0;
    static bool output_state = false;
    bool mismatch_detected = false;

    /* Inizializzazione prima esecuzione */
    if (first_run) {
        log_info("=== Test DIR I/O Attivo ===");
        log_info("Collega: PB12->PB9, PB13->PA3, PB14->PA5");
        last_toggle_time = systick_get_tick();
        first_run = false;
    }

    /* Toggle output ogni 500ms */
    if (systick_timeout(last_toggle_time, 500)) {
        output_state = !output_state;
        last_toggle_time = systick_get_tick();

        /* Imposta output DIR */
        if (output_state) {
            gpio_set(DIR_X_PORT, DIR_X_PIN);
            gpio_set(DIR_Y_PORT, DIR_Y_PIN);
            gpio_set(DIR_Z_PORT, DIR_Z_PIN);
        } else {
            gpio_reset(DIR_X_PORT, DIR_X_PIN);
            gpio_reset(DIR_Y_PORT, DIR_Y_PIN);
            gpio_reset(DIR_Z_PORT, DIR_Z_PIN);
        }

        log_debug("DIR outputs -> %d", output_state);
    }

    /* Piccolo delay per stabilizzazione segnale */
    systick_delay_ms(10);

    /* Leggi input corrispondenti */
    bool x_max_read = gpio_read(X_MAX_PORT, X_MAX_PIN);
    bool y_max_read = gpio_read(Y_MAX_PORT, Y_MAX_PIN);
    bool z_max_read = gpio_read(Z_MAX_PORT, Z_MAX_PIN);

    /* Verifica corrispondenza (nota: i pin hanno pull-up, quindi invertiti) */
    bool expected_input = !output_state;  // Inversione per pull-up

    if (x_max_read != expected_input) {
        log_error("DIR_X mismatch! Output=%d, X_MAX=%d", output_state, x_max_read);
        mismatch_detected = true;
    }
    if (y_max_read != expected_input) {
        log_error("DIR_Y mismatch! Output=%d, Y_MAX=%d", output_state, y_max_read);
        mismatch_detected = true;
    }
    if (z_max_read != expected_input) {
        log_error("DIR_Z mismatch! Output=%d, Z_MAX=%d", output_state, z_max_read);
        mismatch_detected = true;
    }

    return mismatch_detected;
}

#endif /* ENABLE_TEST_DIR_IO */


/* ============================================================================
 * TEST ENABLE I/O (LOOPBACK)
 * ============================================================================
 */

#if ENABLE_TEST_ENABLE_IO

/**
 * @brief Test loopback ENABLE pins → ENCODER_A pins
 */
bool test_enable_io(void)
{
    static bool first_run = true;
    static u32 last_toggle_time = 0;
    static bool output_state = false;
    bool mismatch_detected = false;

    /* Inizializzazione prima esecuzione */
    if (first_run) {
        log_info("=== Test ENABLE I/O Attivo ===");
        log_info("Collega: PB3->PA6, PB4->PB6, PB5->PA8");
        last_toggle_time = systick_get_tick();
        first_run = false;
    }

    /* Toggle output ogni 500ms */
    if (systick_timeout(last_toggle_time, 500)) {
        output_state = !output_state;
        last_toggle_time = systick_get_tick();

        /* Imposta output ENABLE */
        if (output_state) {
            gpio_set(ENABLE_X_PORT, ENABLE_X_PIN);
            gpio_set(ENABLE_Y_PORT, ENABLE_Y_PIN);
            gpio_set(ENABLE_Z_PORT, ENABLE_Z_PIN);
        } else {
            gpio_reset(ENABLE_X_PORT, ENABLE_X_PIN);
            gpio_reset(ENABLE_Y_PORT, ENABLE_Y_PIN);
            gpio_reset(ENABLE_Z_PORT, ENABLE_Z_PIN);
        }

        log_debug("ENABLE outputs -> %d", output_state);
    }

    /* Piccolo delay per stabilizzazione segnale */
    systick_delay_ms(10);

    /* Leggi input corrispondenti */
    bool enc_x_a_read = gpio_read(ENC_X_A_PORT, ENC_X_A_PIN);
    bool enc_y_a_read = gpio_read(ENC_Y_A_PORT, ENC_Y_A_PIN);
    bool enc_z_a_read = gpio_read(ENC_Z_A_PORT, ENC_Z_A_PIN);

    /* Verifica corrispondenza (nota: i pin hanno pull-up, quindi invertiti) */
    bool expected_input = !output_state;  // Inversione per pull-up

    if (enc_x_a_read != expected_input) {
        log_error("ENABLE_X mismatch! Output=%d, ENC_X_A=%d", output_state, enc_x_a_read);
        mismatch_detected = true;
    }
    if (enc_y_a_read != expected_input) {
        log_error("ENABLE_Y mismatch! Output=%d, ENC_Y_A=%d", output_state, enc_y_a_read);
        mismatch_detected = true;
    }
    if (enc_z_a_read != expected_input) {
        log_error("ENABLE_Z mismatch! Output=%d, ENC_Z_A=%d", output_state, enc_z_a_read);
        mismatch_detected = true;
    }

    return mismatch_detected;
}

#endif /* ENABLE_TEST_ENABLE_IO */
