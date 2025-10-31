/**
 * @file cnc.h
 * @brief Sistema di gestione stato macchina CNC e emergenze finecorsa
 * @details
 * Questo modulo implementa la macchina a stati del controller CNC e
 * la gestione delle emergenze da finecorsa con retrazione automatica.
 *
 * FUNZIONALITÀ PRINCIPALI:
 * - Macchina a stati per controllo operativo
 * - Gestione interrupt finecorsa (falling edge 1→0)
 * - Retrazione automatica degli assi in emergenza
 * - Disabilitazione immediata motori su allarme
 * - Recupero sequenziale posizione asse per asse
 *
 * STRATEGIA EMERGENZA FINECORSA:
 * 1. Interrupt EXTI su falling edge di qualsiasi finecorsa
 * 2. Disabilitazione IMMEDIATA di tutti gli assi coinvolti (ENABLE=HIGH)
 * 3. Accodamento assi da recuperare
 * 4. Recupero sequenziale (un asse alla volta):
 *    - Abilita solo l'asse da recuperare
 *    - Inverte direzione movimento
 *    - Genera STEP a frequenza definita (1kHz default)
 *    - Attende rilascio finecorsa
 *    - Disabilita asse
 * 5. Ritorno a stato sicuro (ST_IDLE o ST_EMERGENCY_STOP)
 */

#ifndef CNC_H
#define CNC_H

#include "common.h"

/* ============================================================================
 * CONFIGURAZIONE FREQUENZE EMERGENZA E MOVIMENTO
 * ============================================================================
 * Frequenze di retrazione per ogni asse quando si attiva un finecorsa.
 * Valori in Hz (step/secondo).
 */

/**
 * @brief Frequenza step asse X durante retrazione emergenza (Hz)
 */
#define EMER_STEP_FREQ_X    1000

/**
 * @brief Frequenza step asse Y durante retrazione emergenza (Hz)
 */
#define EMER_STEP_FREQ_Y    1000

/**
 * @brief Frequenza step asse Z durante retrazione emergenza (Hz)
 */
#define EMER_STEP_FREQ_Z    1000

/**
 * @brief Frequenza minima step (Hz) - Usata per partenza/arrivo in profilo trapezoidale
 */
#define MIN_STEP_FREQ       500

/**
 * @brief Frequenza massima step (Hz) - Velocità di crociera
 */
#define MAX_STEP_FREQ       5000

/**
 * @brief Dimensione massima buffer comandi movimento
 */
#define MAX_MOVE_BUFFER     1000

/**
 * @brief Tempo minimo debouncing tra interrupt EXTI (ms)
 * @details Filtra impulsi spurii da rimbalzi meccanici finecorsa
 */
#define EXTI_DEBOUNCE_MS    50

/**
 * @brief Tempo debouncing durante homing su transizioni finecorsa (ms)
 * @details Attesa stabilizzazione segnale dopo press/release finecorsa
 */
#define HOMING_DEBOUNCE_MS  50


/* ============================================================================
 * ENUM STATI MACCHINA
 * ============================================================================
 */

/**
 * @brief Stati della macchina CNC
 * @details La macchina a stati controlla il flusso operativo del controller.
 *
 * TRANSIZIONI TIPICHE:
 *   ST_IDLE → ST_HOMING → ST_IDLE
 *   ST_IDLE → ST_RUNNING → ST_IDLE
 *   ST_IDLE → ST_TEST → ST_IDLE
 *   qualsiasi → ST_ALARM_X/Y/Z → ST_EMERGENCY_STOP → ST_IDLE
 */
typedef enum {
    /**
     * @brief Inattivo - sistema pronto per ricevere comandi
     * @details Motori disabilitati, nessuna operazione in corso.
     *          Stato sicuro dopo inizializzazione o emergenza risolta.
     */
    ST_IDLE = 0,

    /**
     * @brief Operazione normale - esecuzione G-code o movimento manuale
     * @details Motori abilitati, sistema esegue movimenti coordinati.
     */
    ST_RUNNING,

    /**
     * @brief Homing in corso - ricerca origine assi
     * @details Movimento controllato verso finecorsa per azzeramento coordinate.
     */
    ST_HOMING,

    /**
     * @brief Modalità test hardware - validazione I/O
     * @details Esecuzione test loopback o funzionali (vedi test.h/test.c).
     */
    ST_TEST,

    /**
     * @brief Emergenza generale - arresto di sicurezza
     * @details Tutti gli assi disabilitati, attesa intervento operatore.
     *          Può derivare da pulsante emergenza o errore critico.
     */
    ST_EMERGENCY_STOP,

    /**
     * @brief Allarme asse X - finecorsa attivato, retrazione in corso
     * @details Sistema in recupero automatico asse X.
     */
    ST_ALARM_X,

    /**
     * @brief Allarme asse Y - finecorsa attivato, retrazione in corso
     * @details Sistema in recupero automatico asse Y.
     */
    ST_ALARM_Y,

    /**
     * @brief Allarme asse Z - finecorsa attivato, retrazione in corso
     * @details Sistema in recupero automatico asse Z.
     */
    ST_ALARM_Z

} cnc_state_t;


/* ============================================================================
 * STRUTTURA GESTIONE EMERGENZA
 * ============================================================================
 */

/**
 * @brief Informazioni su asse in emergenza
 * @details Usata per gestione coda recupero assi.
 */
typedef struct {
    u8 axis;           /**< Asse interessato: 0=X, 1=Y, 2=Z */
    bool min_limit;    /**< true=finecorsa MIN attivo, false=finecorsa MAX attivo */
} emergency_axis_t;

/**
 * @brief Coda emergenze da processare
 * @details Mantiene lista degli assi da recuperare sequenzialmente.
 */
typedef struct {
    emergency_axis_t queue[3];  /**< Coda assi (max 3 contemporanei) */
    u8 count;                   /**< Numero di assi in coda */
    u8 current_index;           /**< Indice asse correntemente in recupero */
} emergency_queue_t;


/* ============================================================================
 * STRUTTURA POSIZIONI ENCODER
 * ============================================================================
 */

/**
 * @brief Posizioni assolute degli encoder (32-bit signed)
 * @details Mantiene la posizione assoluta di ogni asse in step/impulsi.
 *          Aggiornate periodicamente (50Hz) leggendo i delta dai timer encoder.
 */
typedef struct {
    i32 x;  /**< Posizione assoluta asse X (step) */
    i32 y;  /**< Posizione assoluta asse Y (step) */
    i32 z;  /**< Posizione assoluta asse Z (step) */
} encoder_positions_t;


/* ============================================================================
 * STRUTTURE DATI BRESENHAM 3D
 * ============================================================================
 */

/**
 * @brief Punto 3D in coordinate intere (step)
 */
typedef struct {
    i32 x;  /**< Coordinata X in step */
    i32 y;  /**< Coordinata Y in step */
    i32 z;  /**< Coordinata Z in step */
} point3d_t;

/**
 * @brief Comando movimento singolo asse (batch)
 * @details Rappresenta un movimento batch su un singolo asse.
 *          Ottimizzazione: raggruppa step consecutivi sullo stesso asse.
 */
typedef struct {
    u8 axis;       /**< Asse: 0=X, 1=Y, 2=Z */
    i32 steps;     /**< Numero step (signed per direzione: >0 positivo, <0 negativo) */
    u32 freq_hz;   /**< Frequenza STEP in Hz per questo comando */
} axis_move_t;

/**
 * @brief Profilo traiettoria trapezoidale
 * @details Definisce accelerazione/velocità costante/decelerazione
 */
typedef struct {
    u32 total_steps;   /**< Step totali nella traiettoria */
    u32 accel_steps;   /**< Step in fase accelerazione */
    u32 const_steps;   /**< Step a velocità costante */
    u32 decel_steps;   /**< Step in fase decelerazione */
} trajectory_profile_t;


/* ============================================================================
 * VARIABILI GLOBALI
 * ============================================================================
 */

/**
 * @brief Stato corrente della macchina CNC
 * @note Volatile perché modificato da ISR
 */
extern volatile cnc_state_t current_state;

/**
 * @brief Coda emergenze da processare
 * @note Volatile perché modificato da ISR
 */
extern volatile emergency_queue_t emergency_queue;

/**
 * @brief Posizioni assolute encoder
 * @note Volatile perché lette/scritte da più contesti
 */
extern volatile encoder_positions_t encoder_positions;


/* ============================================================================
 * FUNZIONI PUBBLICHE - INIZIALIZZAZIONE
 * ============================================================================
 */

/**
 * @brief Inizializza il sistema CNC
 * @details Configura:
 *          - Stato iniziale (ST_IDLE)
 *          - Interrupt EXTI per finecorsa (falling edge)
 *          - Timer TIM2 per generazione step emergenza
 *          - Priorità interrupt (EXTI alta, TIM2 normale)
 *
 * @return 0 se OK, -1 se errore
 *
 * @note Deve essere chiamato DOPO gpio_init_all() e systick_init()
 * @note Configura NVIC per interrupt ad alta priorità
 */
int cnc_init(void);


/* ============================================================================
 * FUNZIONI PUBBLICHE - GESTIONE STATO
 * ============================================================================
 */

/**
 * @brief Ottiene lo stato corrente della macchina
 * @return Stato corrente (cnc_state_t)
 */
cnc_state_t cnc_get_state(void);

/**
 * @brief Imposta lo stato della macchina
 * @param new_state Nuovo stato da impostare
 * @note Alcuni cambi di stato possono essere rifiutati per sicurezza
 */
void cnc_set_state(cnc_state_t new_state);

/**
 * @brief Ottiene il nome testuale dello stato corrente
 * @return Stringa descrittiva dello stato (es: "IDLE", "RUNNING", etc.)
 * @note Utile per logging e debug
 */
const char* cnc_get_state_name(void);


/* ============================================================================
 * FUNZIONI PUBBLICHE - GESTIONE EMERGENZA
 * ============================================================================
 */

/**
 * @brief Processa la coda emergenze (da chiamare nel main loop)
 * @details Gestisce il recupero sequenziale degli assi in emergenza.
 *          Deve essere chiamata continuamente nel main loop quando
 *          ci sono emergenze in coda.
 *
 * @note Non bloccante - ritorna immediatamente
 * @note Il recupero effettivo è gestito da TIM2 ISR
 */
void cnc_process_emergency(void);

/**
 * @brief Verifica se ci sono emergenze in coda
 * @return true se ci sono assi da recuperare, false altrimenti
 */
bool cnc_has_emergency(void);

/**
 * @brief Reset manuale sistema emergenza
 * @details Svuota la coda emergenze e ritorna a ST_IDLE.
 *          Usare solo dopo aver verificato che tutti i finecorsa
 *          siano rilasciati.
 *
 * @warning NON chiamare se finecorsa ancora attivi!
 */
void cnc_clear_emergency(void);


/* ============================================================================
 * FUNZIONI PUBBLICHE - GESTIONE POSIZIONI ENCODER
 * ============================================================================
 */

/**
 * @brief Aggiorna le posizioni encoder leggendo i delta dai timer
 * @details Questa funzione deve essere chiamata periodicamente (50Hz)
 *          nel main loop. Per ogni asse:
 *          1. Legge il delta dal timer encoder (TIM1/3/4)
 *          2. Azzera il timer
 *          3. Aggiorna la posizione assoluta (enc_abs_x/y/z += delta)
 *
 * @note Non bloccante, esecuzione veloce (~pochi µs)
 * @note Chiamare con timeout 20ms (50Hz) nel main loop
 */
void cnc_update_encoder_positions(void);

/**
 * @brief Ottiene la posizione corrente asse X
 * @return Posizione assoluta asse X in step
 */
i32 cnc_get_position_x(void);

/**
 * @brief Ottiene la posizione corrente asse Y
 * @return Posizione assoluta asse Y in step
 */
i32 cnc_get_position_y(void);

/**
 * @brief Ottiene la posizione corrente asse Z
 * @return Posizione assoluta asse Z in step
 */
i32 cnc_get_position_z(void);

/**
 * @brief Azzera la posizione asse X
 * @details Imposta enc_abs_x = 0 e resetta il timer encoder TIM3
 */
void cnc_reset_position_x(void);

/**
 * @brief Azzera la posizione asse Y
 * @details Imposta enc_abs_y = 0 e resetta il timer encoder TIM4
 */
void cnc_reset_position_y(void);

/**
 * @brief Azzera la posizione asse Z
 * @details Imposta enc_abs_z = 0 e resetta il timer encoder TIM1
 */
void cnc_reset_position_z(void);

/**
 * @brief Azzera tutte le posizioni encoder
 * @details Imposta tutte le posizioni a 0 e resetta tutti i timer encoder
 * @note Utile per homing o reset generale
 */
void cnc_reset_all_positions(void);

/**
 * @brief Esegue homing completo di tutti gli assi (Z → Y → X)
 * @details Sequenza sicura: prima Z (alza utensile), poi Y, poi X.
 *          Ogni asse viene portato al finecorsa MIN e la posizione viene azzerata.
 * @return 0 se successo, -1 se errore/timeout
 * @note Bloccante: può durare fino a 2 minuti (40s per asse × 3)
 * @note Utile per inizializzazione macchina o reset generale
 */
int cnc_home(void);


/* ============================================================================
 * FUNZIONI PUBBLICHE - BRESENHAM 3D E GENERAZIONE TRAIETTORIE
 * ============================================================================
 */

/**
 * @brief Genera traiettoria lineare 3D ottimizzata con algoritmo di Bresenham
 * @details Calcola una sequenza di comandi batch axis_move_t che approssimano
 *          una linea retta da start a end nello spazio 3D.
 *
 *          Algoritmo di Bresenham 3D ottimizzato:
 *          - Raggruppa step consecutivi sullo stesso asse in batch
 *          - Riduce numero comandi del 80-90% rispetto a step-by-step
 *          - Deterministico e senza floating point
 *
 * @param start Punto di partenza (step)
 * @param end Punto di arrivo (step)
 * @param moves Buffer per comandi generati (min MAX_MOVE_BUFFER elementi)
 * @return Numero di comandi generati, 0 se movimento nullo, -1 se errore
 *
 * @note Il buffer moves deve essere allocato dal chiamante (min 1000 elementi)
 * @note I comandi generati hanno freq_hz = 0 (da impostare successivamente)
 */
int cnc_bresenham_line(point3d_t start, point3d_t end, axis_move_t *moves);

/**
 * @brief Calcola profilo velocità trapezoidale per una traiettoria
 * @details Calcola parametri accelerazione/crociera/decelerazione per
 *          un movimento smooth con profilo trapezoidale.
 *
 * @param total_steps Numero totale step nella traiettoria
 * @param profile Output: profilo calcolato
 *
 * @note Profilo standard: 25% accel + 50% const + 25% decel
 */
void cnc_calculate_trajectory_profile(u32 total_steps, trajectory_profile_t *profile);

/**
 * @brief Applica profilo velocità trapezoidale ai comandi movimento
 * @details Assegna frequenza STEP ad ogni comando in base alla posizione
 *          nel profilo (accelerazione → velocità costante → decelerazione)
 *
 * @param moves Array comandi da modificare
 * @param move_count Numero comandi nell'array
 * @param min_hz Frequenza minima (partenza/arrivo)
 * @param max_hz Frequenza massima (crociera)
 *
 * @note Modifica il campo freq_hz di ogni axis_move_t
 */
void cnc_apply_speed_profile(axis_move_t *moves, u32 move_count, u32 min_hz, u32 max_hz);

/**
 * @brief Esegue sequenza di comandi movimento con controllo TIM2
 * @details Esegue fisicamente i comandi axis_move_t generando STEP
 *          sui pin GPIO tramite TIM2 interrupt.
 *
 * @param moves Array comandi da eseguire
 * @param move_count Numero comandi
 * @return 0 se completato con successo, -1 se interrotto (emergenza/timeout)
 *
 * @note Bloccante: attende completamento di tutti i comandi
 * @note Può essere interrotto da emergenza limit switch
 * @note Usa TIM2 per generazione STEP a frequenza programmabile
 */
int cnc_execute_moves(axis_move_t *moves, u32 move_count);

/**
 * @brief Valida che una sequenza di comandi arrivi correttamente a destinazione
 * @details Simula l'esecuzione dei comandi e verifica che la posizione finale
 *          corrisponda al punto end previsto.
 *
 * @param start Punto di partenza
 * @param end Punto di arrivo atteso
 * @param moves Array comandi da validare
 * @param move_count Numero comandi
 * @return true se valido, false se posizione finale non corrisponde
 *
 * @note Usata per debug/testing, non necessaria in produzione
 */
bool cnc_validate_trajectory(point3d_t start, point3d_t end, axis_move_t *moves, u32 move_count);

/* ============================================================================
 * NOTA: Le seguenti funzioni sono INTERNE e non devono essere chiamate
 *       direttamente. Sono usate dagli ISR (EXTI, TIM2) per gestire emergenze.
 *
 * - cnc_handle_limit_interrupt() - Chiamata da EXTI ISR
 * - cnc_start_axis_recovery() - Chiamata da cnc_process_emergency()
 *
 * Queste funzioni sono dichiarate come static in cnc.c
 * ============================================================================
 */


#endif /* CNC_H */
