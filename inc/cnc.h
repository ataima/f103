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
 * @brief Profilo traiettoria trapezoidale
 * @details Definisce accelerazione/velocità costante/decelerazione
 */
typedef struct {
    u32 total_steps;   /**< Step totali nella traiettoria */
    u32 accel_steps;   /**< Step in fase accelerazione */
    u32 const_steps;   /**< Step a velocità costante */
    u32 decel_steps;   /**< Step in fase decelerazione */
} trajectory_profile_t;

/**
 * @brief Stato movimento real-time con Bresenham 3D incrementale
 * @details Sostituisce array pre-calcolato axis_move_t[MAX_MOVE_BUFFER].
 *          Calcola step e velocità on-the-fly durante esecuzione.
 *
 * VANTAGGI:
 * - RAM: ~100 bytes vs 12 KB array statico
 * - Step simultanei: risolve segmentazione traiettoria
 * - Velocità dinamica: profilo trapezoidale calcolato real-time
 * - Movimenti illimitati: nessun limite 1000 step
 *
 * ALGORITMO:
 * 1. Bresenham 3D incrementale (decide quali assi muovere ogni step)
 * 2. Calcolo percentuale: current_step / total_steps
 * 3. Velocità trapezoidale: f(percentuale) → Hz
 */
typedef struct {
    /* Punti traiettoria */
    point3d_t start;        /**< Punto partenza (12 bytes) */
    point3d_t end;          /**< Punto arrivo (12 bytes) */
    point3d_t current;      /**< Posizione corrente (12 bytes) */

    /* Delta e direzioni */
    i32 dx, dy, dz;         /**< Delta assoluti |end - start| (12 bytes) */
    i32 sx, sy, sz;         /**< Segni direzione: +1, 0, -1 (12 bytes) */

    /* Bresenham 3D stato */
    i32 err_xy, err_xz;     /**< Errori accumulati Bresenham (8 bytes) */
    u8 dominant_axis;       /**< Asse dominante: 0=X, 1=Y, 2=Z (1 byte) */
    u8 _pad1[3];            /**< Padding allineamento (3 bytes) */

    /* Contatori step */
    u32 total_steps;        /**< Step totali (max delta) (4 bytes) */
    u32 current_step;       /**< Step corrente [0..total_steps] (4 bytes) */

    /* Profilo velocità trapezoidale */
    u32 min_freq_hz;        /**< Velocità minima (accel/decel) (4 bytes) */
    u32 max_freq_hz;        /**< Velocità massima (crociera) (4 bytes) */
    trajectory_profile_t profile; /**< Profilo precalcolato (16 bytes) */

    /* Flags stato */
    volatile bool active;   /**< Movimento in esecuzione (1 byte) */
    volatile bool completed; /**< Movimento completato (1 byte) */
    u8 _pad2[2];            /**< Padding allineamento (2 bytes) */
} move_rt_state_t;          /**< TOTALE: ~104 bytes */


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
 * FUNZIONI PUBBLICHE - MOVIMENTO REAL-TIME
 * ============================================================================
 */

/**
 * @brief Inizializza movimento lineare 3D real-time
 * @details Configura stato movimento con algoritmo Bresenham 3D incrementale.
 *          Calcola profilo velocità trapezoidale basato su distanza totale.
 *
 * @param state Puntatore a stato movimento da inizializzare
 * @param start Punto di partenza
 * @param end Punto di arrivo
 * @param min_hz Velocità minima (accelerazione/decelerazione) in Hz
 * @param max_hz Velocità massima (crociera) in Hz
 *
 * @return 0 se OK, -1 se errore (parametri invalidi)
 *
 * @note NON avvia esecuzione, solo setup. Usare cnc_move_execute_rt()
 * @note Richiede solo ~104 bytes RAM (vs 12 KB array statico)
 */
int cnc_move_init_rt(move_rt_state_t *state, point3d_t start, point3d_t end,
                     u32 min_hz, u32 max_hz);

/**
 * @brief Calcola step successivo Bresenham 3D
 * @details Algoritmo incrementale, decide quali assi muovere questo step.
 *          Supporta step SIMULTANEI multi-asse (risolve segmentazione).
 *
 * @param state Puntatore a stato movimento
 *
 * @return Maschera assi: bit0=X, bit1=Y, bit2=Z (0x01=solo X, 0x07=tutti)
 *         0 se movimento completato
 *
 * @note Chiamata da TIM2_IRQHandler per ogni step
 * @note Aggiorna current_step e current position
 *
 * @example
 *   u8 mask = cnc_move_step_rt(&state);
 *   if (mask & 0x01) muovi_asse_X();
 *   if (mask & 0x02) muovi_asse_Y();
 *   if (mask & 0x04) muovi_asse_Z();
 */
u8 cnc_move_step_rt(move_rt_state_t *state);

/**
 * @brief Calcola velocità istantanea basata su percentuale completamento
 * @details Profilo trapezoidale dinamico:
 *          - Accelerazione: min_hz → max_hz (fase 1)
 *          - Crociera: max_hz costante (fase 2)
 *          - Decelerazione: max_hz → min_hz (fase 3)
 *
 * @param state Puntatore a stato movimento (const volatile)
 *
 * @return Frequenza step in Hz per posizione corrente
 *
 * @note Chiamata da TIM2_IRQHandler per aggiornare TIM2_ARR
 * @note Calcolo real-time, nessun array pre-calcolato
 */
u32 cnc_move_get_speed_rt(const volatile move_rt_state_t *state);

/**
 * @brief Ottiene percentuale completamento movimento
 * @param state Puntatore a stato movimento (const volatile)
 * @return Percentuale 0-100
 */
u8 cnc_move_get_percentage_rt(const volatile move_rt_state_t *state);

/**
 * @brief Esegue movimento real-time (bloccante)
 * @details Avvia TIM2 ISR per generazione step automatica.
 *          Attende completamento o emergenza.
 *
 * @param state Puntatore a stato movimento (deve essere inizializzato)
 *
 * @return 0 se OK, -1 se abort/timeout/emergenza
 *
 * @note Funzione bloccante (usa WFI per risparmio energetico)
 * @note Logga posizione ogni 100 step
 * @note Interrompibile da emergenza finecorsa
 */
int cnc_move_execute_rt(move_rt_state_t *state);


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
