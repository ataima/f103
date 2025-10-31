/**
 * @file encoder.h
 * @brief Driver per encoder rotativi su TIM1/TIM3/TIM4 in modalità Encoder
 * @details
 * Questo modulo configura i timer TIM1/TIM3/TIM4 in modalità Encoder Interface
 * per leggere encoder rotativi in quadratura sui 3 assi X/Y/Z del CNC.
 *
 * MODALITÀ ENCODER:
 * I timer in modalità encoder leggono automaticamente i segnali A/B in quadratura
 * e aggiornano il counter (CNT) in base alla direzione di rotazione:
 * - Rotazione oraria → CNT incrementa (UP)
 * - Rotazione antioraria → CNT decrementa (DOWN)
 *
 * MAPPATURA PIN:
 * - Asse X: TIM3_CH1 (PA6), TIM3_CH2 (PA7)
 * - Asse Y: TIM4_CH1 (PB6), TIM4_CH2 (PB7)
 * - Asse Z: TIM1_CH1 (PA8), TIM1_CH2 (PA9)
 *
 * CONFIGURAZIONE:
 * - Counter a 16-bit signed (-32768 a +32767)
 * - Encoder mode TI1/TI2 (conta su entrambi i fronti di A e B → x4 risoluzione)
 * - No prescaler (massima risoluzione)
 * - No interrupt (polling mode)
 *
 * UTILIZZO:
 * 1. Chiamare encoder_init() dopo gpio_init_all()
 * 2. Periodicamente (es. 50Hz) chiamare encoder_read_x/y/z() per ottenere delta
 * 3. Dopo la lettura, il counter viene azzerato automaticamente per la prossima lettura
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "common.h"

/* ============================================================================
 * COSTANTI E CONFIGURAZIONE
 * ============================================================================
 */

/**
 * @brief Valore massimo counter encoder (16-bit)
 * @note In realtà il counter è signed, quindi range è -32768 a +32767
 */
#define ENCODER_MAX_COUNT       32767
#define ENCODER_MIN_COUNT       (-32768)

/**
 * @brief Codici di ritorno
 */
#define ENCODER_OK              0
#define ENCODER_ERROR           (-1)


/* ============================================================================
 * FUNZIONI PUBBLICHE - INIZIALIZZAZIONE
 * ============================================================================
 */

/**
 * @brief Inizializza tutti i timer encoder (TIM1, TIM3, TIM4)
 * @details Configura i 3 timer in modalità Encoder Interface:
 *          - TIM3: Encoder asse X (PA6/PA7)
 *          - TIM4: Encoder asse Y (PB6/PB7)
 *          - TIM1: Encoder asse Z (PA8/PA9)
 *
 *          Per ogni timer:
 *          - Abilita clock del timer
 *          - Configura modalità encoder (TI1/TI2, counting su entrambi fronti)
 *          - Imposta ARR (Auto-Reload) a 0xFFFF (16-bit)
 *          - Azzera counter (CNT = 0)
 *          - Configura GPIO come Alternate Function (floating input)
 *          - Avvia timer (CEN = 1)
 *
 * @return ENCODER_OK se successo, ENCODER_ERROR se errore
 *
 * @note Deve essere chiamato DOPO gpio_init_all() per i clock GPIO
 * @note I pin encoder sono già configurati come input in gpio_init_encoders()
 *       ma qui vengono riconfigurati come Alternate Function
 */
int encoder_init(void);


/* ============================================================================
 * FUNZIONI PUBBLICHE - LETTURA ENCODER
 * ============================================================================
 */

/**
 * @brief Legge il delta encoder asse X e azzera il counter
 * @details Legge TIM3_CNT come signed 16-bit, poi resetta CNT a 0.
 *          Il valore ritornato rappresenta gli step dall'ultima lettura.
 *
 * @return Delta encoder X (positivo = movimento in una direzione,
 *         negativo = movimento opposto)
 *
 * @note Questa funzione deve essere chiamata periodicamente (es. 50Hz)
 * @note Il counter viene azzerato dopo ogni lettura
 */
i16 encoder_read_x(void);

/**
 * @brief Legge il delta encoder asse Y e azzera il counter
 * @return Delta encoder Y
 */
i16 encoder_read_y(void);

/**
 * @brief Legge il delta encoder asse Z e azzera il counter
 * @return Delta encoder Z
 */
i16 encoder_read_z(void);


/* ============================================================================
 * FUNZIONI PUBBLICHE - RESET
 * ============================================================================
 */

/**
 * @brief Azzera il counter dell'encoder X
 * @details Imposta TIM3_CNT = 0
 */
void encoder_reset_x(void);

/**
 * @brief Azzera il counter dell'encoder Y
 * @details Imposta TIM4_CNT = 0
 */
void encoder_reset_y(void);

/**
 * @brief Azzera il counter dell'encoder Z
 * @details Imposta TIM1_CNT = 0
 */
void encoder_reset_z(void);

/**
 * @brief Azzera tutti i counter encoder
 */
void encoder_reset_all(void);


/* ============================================================================
 * FUNZIONI PUBBLICHE - LETTURA DIRETTA (senza reset)
 * ============================================================================
 */

/**
 * @brief Legge il valore corrente del counter encoder X (senza azzerare)
 * @return Valore corrente TIM3_CNT (signed 16-bit)
 */
i16 encoder_get_count_x(void);

/**
 * @brief Legge il valore corrente del counter encoder Y (senza azzerare)
 * @return Valore corrente TIM4_CNT (signed 16-bit)
 */
i16 encoder_get_count_y(void);

/**
 * @brief Legge il valore corrente del counter encoder Z (senza azzerare)
 * @return Valore corrente TIM1_CNT (signed 16-bit)
 */
i16 encoder_get_count_z(void);


#endif /* ENCODER_H */
