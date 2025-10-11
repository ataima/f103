/**
 ******************************************************************************
 * @file           : log.h
 * @brief          : Sistema di logging con buffer circolare in RAM dedicata
 * @author         : Generato per STM32F103C8T6
 ******************************************************************************
 * @attention
 *
 * Questo modulo implementa un sistema di logging thread-safe con buffer
 * circolare allocato in una area dedicata di RAM (2KB).
 *
 * Caratteristiche:
 * - Buffer circolare di 2KB in RAM dedicata (0x20004800 - 0x20004FFF)
 * - Lock statico per accesso thread-safe
 * - Funzioni push/pop: log_write() e log_read()
 * - Macro di convenienza con prefissi: log_error, log_warning, log_info, log_debug
 * - Resistente a overflow (messaggi più vecchi vengono sovrascritti)
 * - Leggibile via debugger o comando seriale
 *
 ******************************************************************************
 */

#ifndef LOG_H_
#define LOG_H_

#include "common.h"

/* =============================================================================
 * CODICI DI RITORNO
 * ============================================================================= */

#define LOG_OK                   0      /* Operazione riuscita */
#define LOG_ERROR_OVERFLOW      -1      /* Buffer pieno (messaggio vecchio sovrascritto) */
#define LOG_ERROR_EMPTY         -2      /* Buffer vuoto (nessun messaggio da leggere) */
#define LOG_ERROR_LOCKED        -3      /* Buffer bloccato da altro accesso */
#define LOG_ERROR_INVALID       -4      /* Parametro invalido */
#define LOG_ERROR_TRUNCATED     -5      /* Messaggio troncato per lunghezza eccessiva */


#if ENABLE_LOG

/* =============================================================================
 * CONFIGURAZIONE DEL BUFFER CIRCOLARE
 * ============================================================================= */

/**
 * Dimensione totale del buffer di log (2KB = 2048 bytes)
 * Questa deve corrispondere alla dimensione allocata nel linker script
 */
#define LOG_BUFFER_SIZE         2048U

/**
 * Dimensione massima di un singolo messaggio di log (incluso terminatore null)
 * Messaggi più lunghi verranno troncati
 */
#define LOG_MAX_MESSAGE_SIZE    128U

/**
 * Prefissi per i diversi livelli di log
 * Questi vengono automaticamente aggiunti dalle macro
 */
#define LOG_PREFIX_ERROR        "[ERR]:"
#define LOG_PREFIX_WARNING      "[WRN]:"
#define LOG_PREFIX_INFO         "[INF]:"
#define LOG_PREFIX_DEBUG        "[DBG]:"


/* =============================================================================
 * FUNZIONI PUBBLICHE
 * ============================================================================= */

/**
 * @brief  Inizializza il sistema di logging
 *
 * @details Questa funzione deve essere chiamata una sola volta all'avvio
 *          del sistema, dopo la configurazione del clock.
 *          Azzera il buffer circolare e inizializza i puntatori.
 *
 * @param  None
 * @retval int - LOG_OK se successo, codice errore altrimenti
 *
 * @note   Thread-safe: No (chiamare solo da main all'avvio)
 */
int log_init(void);

/**
 * @brief  Scrive un messaggio nel buffer circolare (PUSH)
 *
 * @details Aggiunge un messaggio al buffer circolare. Se il buffer è pieno,
 *          il messaggio più vecchio viene sovrascritto (comportamento FIFO).
 *          La funzione è thread-safe grazie al lock statico.
 *
 * @param  message - Puntatore alla stringa da loggare (null-terminated)
 *
 * @retval int - LOG_OK se successo
 *             - LOG_ERROR_INVALID se message è NULL
 *             - LOG_ERROR_LOCKED se il buffer è bloccato
 *             - LOG_ERROR_OVERFLOW se il buffer è pieno (messaggio comunque scritto)
 *             - LOG_ERROR_TRUNCATED se il messaggio è stato troncato
 *
 * @note   Thread-safe: Sì (usa lock statico)
 * @note   Interrupt-safe: Sì (disabilita interrupt durante l'accesso)
 * @note   Messaggi più lunghi di LOG_MAX_MESSAGE_SIZE vengono troncati
 *
 * @example
 *   log_write("Sistema avviato correttamente");
 *   log_write("Temperatura: 25.5°C");
 */
int log_write(const char *message);

/**
 * @brief  Legge un messaggio dal buffer circolare (POP)
 *
 * @details Estrae il messaggio più vecchio dal buffer circolare e lo copia
 *          nel buffer di destinazione. Il messaggio viene rimosso dal buffer.
 *          La funzione è thread-safe grazie al lock statico.
 *
 * @param  dest - Buffer di destinazione (deve essere almeno LOG_MAX_MESSAGE_SIZE)
 *
 * @retval int - LOG_OK se successo
 *             - LOG_ERROR_INVALID se dest è NULL
 *             - LOG_ERROR_EMPTY se non ci sono messaggi da leggere
 *             - LOG_ERROR_LOCKED se il buffer è bloccato
 *
 * @note   Thread-safe: Sì (usa lock statico)
 * @note   Interrupt-safe: Sì (disabilita interrupt durante l'accesso)
 * @note   Il buffer dest deve essere almeno LOG_MAX_MESSAGE_SIZE bytes
 *
 * @example
 *   char buffer[LOG_MAX_MESSAGE_SIZE];
 *   while (log_read(buffer) == LOG_OK) {
 *       printf("%s\n", buffer);
 *   }
 */
int log_read(char *dest);

/**
 * @brief  Ottiene il numero di messaggi presenti nel buffer
 *
 * @param  None
 * @retval u16 - Numero di messaggi nel buffer
 *
 * @note   Thread-safe: Sì (lettura atomica su Cortex-M3)
 */
u16 log_get_count(void);

/**
 * @brief  Ottiene il numero di byte utilizzati nel buffer
 *
 * @param  None
 * @retval u16 - Numero di byte occupati (max 2048)
 *
 * @note   Thread-safe: Sì (lettura atomica su Cortex-M3)
 */
u16 log_get_used_bytes(void);

/**
 * @brief  Ottiene il numero di byte liberi nel buffer
 *
 * @param  None
 * @retval u16 - Numero di byte disponibili (max 2048)
 *
 * @note   Thread-safe: Sì (lettura atomica su Cortex-M3)
 */
u16 log_get_free_bytes(void);

/**
 * @brief  Svuota completamente il buffer di log
 *
 * @details Rimuove tutti i messaggi dal buffer e resetta i puntatori.
 *          Utile per cancellare log vecchi prima di iniziare una nuova
 *          sessione di test.
 *
 * @param  None
 * @retval int - LOG_OK se successo, LOG_ERROR_LOCKED se bloccato
 *
 * @note   Thread-safe: Sì (usa lock statico)
 */
int log_clear(void);

/**
 * @brief  Ottiene l'indirizzo base del buffer per ispezione via debugger
 *
 * @details Restituisce l'indirizzo fisico del buffer di log, utile per
 *          ispezionare direttamente la memoria RAM durante il debug o
 *          per dump via comando seriale.
 *
 * @param  None
 * @retval void* - Indirizzo base del buffer (0x20004800)
 *
 * @note   Solo per debugging e dump memoria
 */
void* log_get_buffer_address(void);

/* =============================================================================
 * FUNZIONI DI LOGGING CON PREFISSI E FORMATTAZIONE
 * =============================================================================
 *
 * Queste funzioni aggiungono automaticamente il prefisso del livello di log
 * al messaggio e supportano formattazione variabile come printf.
 *
 * Formato supportato:
 *   - %d  : signed decimal (i32)
 *   - %u  : unsigned decimal (u32)
 *   - %x  : hexadecimal lowercase (u32)
 *   - %X  : hexadecimal uppercase (u32)
 *   - %b  : binary (u32)
 *   - %c  : character
 *   - %s  : string
 *   - %%  : literal '%'
 *
 * Esempio di output:
 *   [ERR]: Sensore non risponde
 *   [WRN]: Temperatura elevata: 85 C
 *   [INF]: Sistema avviato
 *   [DBG]: Valore registro: 0x1234
 */

/**
 * @brief  Log di errore con prefisso [ERR]: e formattazione printf-like
 * @param  fmt - Stringa di formato (come printf)
 * @param  ... - Argomenti variabili per la formattazione
 *
 * @retval int - LOG_OK se successo, codice errore altrimenti
 *
 * @example log_error("Sensore %d non risponde!", sensor_id);
 */
int log_error(const char *fmt, ...);

/**
 * @brief  Log di warning con prefisso [WRN]: e formattazione printf-like
 * @param  fmt - Stringa di formato (come printf)
 * @param  ... - Argomenti variabili per la formattazione
 *
 * @retval int - LOG_OK se successo, codice errore altrimenti
 *
 * @example log_warning("Batteria scarica: %u%%", battery_level);
 */
int log_warning(const char *fmt, ...);

/**
 * @brief  Log informativo con prefisso [INF]: e formattazione printf-like
 * @param  fmt - Stringa di formato (come printf)
 * @param  ... - Argomenti variabili per la formattazione
 *
 * @retval int - LOG_OK se successo, codice errore altrimenti
 *
 * @example log_info("Sistema avviato");
 */
int log_info(const char *fmt, ...);

/**
 * @brief  Log di debug con prefisso [DBG]: e formattazione printf-like
 * @param  fmt - Stringa di formato (come printf)
 * @param  ... - Argomenti variabili per la formattazione
 *
 * @retval int - LOG_OK se successo, codice errore altrimenti
 *
 * @example log_debug("Valore counter: %d", counter);
 */
int log_debug(const char *fmt, ...);

/* =============================================================================
 * NOTE DI UTILIZZO
 * =============================================================================
 *
 * 1. INIZIALIZZAZIONE:
 *    Chiamare log_init() una volta all'avvio, dopo SystemClock_Config():
 *
 *    int main(void) {
 *        SystemClock_Config();
 *        log_init();
 *        log_info("Sistema avviato");
 *        ...
 *    }
 *
 * 2. SCRITTURA LOG:
 *    Usare le funzioni di log con formattazione printf-like:
 *
 *    log_error("Sensore I2C non risponde");
 *    log_warning("Temperatura alta: %d C", temp);
 *    log_info("Configurazione completata");
 *    log_debug("GPIO toggle on pin %d", pin_num);
 *
 *    Oppure log_write() per messaggi già formattati:
 *
 *    char buffer[LOG_MAX_MESSAGE_SIZE];
 *    mini_snprintf(buffer, sizeof(buffer), "[INFO] Temp: %d.%d C", temp/10, temp%10);
 *    log_write(buffer);
 *
 * 3. LETTURA LOG (comando seriale):
 *    Leggere tutti i messaggi in sequenza:
 *
 *    char buffer[LOG_MAX_MESSAGE_SIZE];
 *    while (log_read(buffer) == LOG_OK) {
 *        uart_send(buffer);
 *        uart_send("\r\n");
 *    }
 *
 * 4. DUMP MEMORIA (debugging):
 *    Via debugger o comando seriale, leggere direttamente la memoria:
 *
 *    void* log_addr = log_get_buffer_address();
 *    // Dump 2KB da log_addr (0x20004800)
 *
 * 5. COMPORTAMENTO OVERFLOW:
 *    Quando il buffer è pieno, i messaggi più vecchi vengono sovrascritti
 *    automaticamente. Non si perde mai la capacità di loggare.
 *
 * 6. THREAD SAFETY:
 *    Tutte le funzioni sono thread-safe e interrupt-safe. Possono essere
 *    chiamate da main loop, ISR, task RTOS, ecc.
 *
 * 7. PERSISTENZA:
 *    Il buffer è in RAM (non Flash), quindi i log si perdono a power-off.
 *    Tuttavia, resistono a soft-reset se la RAM rimane alimentata.
 *
 * ============================================================================= */
#else
#define log_error(FMT,...)
#define log_warning(FMT,...)
#define log_info(FMT,...)
#define log_debug(FMT,...)
#define log_init()   0

#endif

#endif /* LOG_H_ */
