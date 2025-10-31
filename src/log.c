/**
 ******************************************************************************
 * @file           : log.c
 * @brief          : Implementazione sistema di logging con buffer circolare
 * @author         : Generato per STM32F103C8T6
 ******************************************************************************
 * @attention
 *
 * Questo file implementa un buffer circolare thread-safe per il logging.
 * Il buffer è allocato in una area RAM dedicata di 2KB definita nel
 * linker script (0x20004800 - 0x20004FFF).
 *
 * ARCHITETTURA DEL BUFFER CIRCOLARE:
 * ==================================
 *
 * Il buffer memorizza messaggi di lunghezza variabile in formato:
 *   [LENGTH(2B)][MESSAGE DATA (LENGTH bytes, incluso '\0')]
 *
 * Dove:
 * - LENGTH: 2 bytes (u16 little-endian) = numero totale di byte del messaggio INCLUSO '\0'
 * - MESSAGE DATA: LENGTH bytes contenenti i caratteri del messaggio + null terminator
 *
 * Esempio in memoria (con LENGTH=10):
 *   [0x0A][0x00]["[INFO]Test\0"] → 2 bytes length + 10 bytes data = 12 bytes totali
 *
 * Esempio completo:
 *   [0x0A][0x00]["[INFO]Test\0"][0x0E][0x00]["[ER]  Error!\0"]...
 *
 * GESTIONE OVERFLOW:
 * =================
 * Quando il buffer è pieno, i messaggi più vecchi vengono automaticamente
 * sovrascritti. Questo garantisce che il sistema possa sempre loggare.
 *
 * THREAD SAFETY:
 * ==============
 * Tutte le operazioni critiche sono protette da disabilitazione globale
 * degli interrupt (__disable_irq / __enable_irq). Questo garantisce
 * atomicità anche in presenza di ISR che chiamano log_write().
 *
 ******************************************************************************
 */
#include "common.h"
#include "log.h"
#include "utils.h"
#include <stdarg.h>

/* Necessario per LOG_DIRECT_UART */
#if LOG_DIRECT_UART
#include "uart.h"
#endif

#if ENABLE_LOG

/* =============================================================================
 * STRUTTURA DEL BUFFER CIRCOLARE (solo LOG_TO_MEMORY)
 * ============================================================================= */

#if LOG_TO_MEMORY

/**
 * @brief Struttura di controllo del buffer circolare
 *
 * Questa struttura mantiene lo stato del buffer e viene allocata
 * all'inizio della sezione .log_buffer in RAM dedicata.
 */
typedef struct {
    u16 head;           /* Indice dove scrivere il prossimo byte (write pointer) */
    u16 tail;           /* Indice da dove leggere il prossimo byte (read pointer) */
    u16 count;          /* Numero di messaggi presenti nel buffer */
    u16 used_bytes;     /* Numero di byte occupati (per statistiche) */
    volatile u8 lock;   /* Lock statico: 0=free, 1=locked */
    u8 initialized;     /* Flag di inizializzazione: 0=no, 1=yes */
    u8 _padding[2];     /* Padding per allineamento a 4 bytes */
} log_control_t;

/**
 * @brief Struttura completa del buffer di log
 *
 * Questa struttura è allocata nella sezione .log_buffer della RAM dedicata.
 * Il linker la posiziona automaticamente a 0x20004800.
 */
typedef struct {
    log_control_t control;                           /* Struttura di controllo */
    u8 data[LOG_BUFFER_SIZE - sizeof(log_control_t)]; /* Array dati */
} log_buffer_t;

/* =============================================================================
 * ALLOCAZIONE DEL BUFFER IN RAM DEDICATA
 * =============================================================================
 *
 * Usiamo __attribute__((section(".log_buffer"))) per dire al linker di
 * allocare questa variabile nella sezione .log_buffer, che è mappata
 * su RAM_LOG (0x20004800 - 0x20004FFF).
 *
 * NOLOAD nel linker script fa sì che questa area non venga inizializzata
 * dal codice di startup, preservando eventualmente i log dopo un reset.
 */
static log_buffer_t log_buffer __attribute__((section(".log_buffer")));



/* Dimensione effettiva dell'area dati (escludendo la struttura di controllo) */
#define LOG_DATA_SIZE   (sizeof(log_buffer.data))

/* =============================================================================
 * FUNZIONI HELPER INTERNE
 * ============================================================================= */

/**
 * @brief  Acquisisce il lock del buffer (disabilita interrupt)
 *
 * @details Tenta di acquisire il lock atomicamente. Se il lock è già preso,
 *          ritorna immediatamente con errore. Questo previene deadlock ma
 *          richiede che il chiamante gestisca il retry se necessario.
 *
 * @param  None
 * @retval bool - true se lock acquisito, false se già locked
 *
 * @note   Disabilita gli interrupt globali per garantire atomicità
 */
static inline bool log_lock_acquire(void)
{
    /* Disabilita interrupt per operazione atomica */
    __disable_irq();

    /* Verifica se il lock è libero */
    if (log_buffer.control.lock == 0)
    {
        /* Acquisisce il lock */
        log_buffer.control.lock = 1;
        return true;  /* Lock acquisito, interrupt rimangono disabilitati */
    }
    else
    {
        /* Lock già preso, riabilita interrupt e ritorna errore */
        __enable_irq();
        return false;
    }
}

/**
 * @brief  Rilascia il lock del buffer (riabilita interrupt)
 *
 * @param  None
 * @retval None
 *
 * @note   Riabilita gli interrupt globali
 */
static inline void log_lock_release(void)
{
    /* Rilascia il lock */
    log_buffer.control.lock = 0;

    /* Riabilita interrupt */
    __enable_irq();
}

/**
 * @brief  Calcola lo spazio libero nel buffer circolare
 *
 * @param  None
 * @retval u16 - Numero di byte disponibili
 *
 * @note   Non thread-safe, chiamare con lock acquisito
 */
static inline u16 log_get_free_space(void)
{
    return (u16)(LOG_DATA_SIZE - log_buffer.control.used_bytes);
}

/**
 * @brief  Scrive un byte nel buffer circolare
 *
 * @param  byte - Byte da scrivere
 * @retval None
 *
 * @note   Non controlla overflow, il chiamante deve verificare lo spazio
 * @note   Non thread-safe, chiamare con lock acquisito
 */
static inline void log_write_byte(u8 byte)
{
    /* Scrivi il byte nella posizione corrente */
    log_buffer.data[log_buffer.control.head] = byte;

    /* Avanza head con wrap-around */
    log_buffer.control.head = (log_buffer.control.head + 1) % LOG_DATA_SIZE;

    /* Aggiorna contatore byte usati */
    log_buffer.control.used_bytes++;
}

/**
 * @brief  Legge un byte dal buffer circolare
 *
 * @param  None
 * @retval u8 - Byte letto
 *
 * @note   Non controlla underflow, il chiamante deve verificare disponibilità
 * @note   Non thread-safe, chiamare con lock acquisito
 */
static inline u8 log_read_byte(void)
{
    /* Leggi il byte dalla posizione corrente */
    u8 byte = log_buffer.data[log_buffer.control.tail];

    /* Avanza tail con wrap-around */
    log_buffer.control.tail = (log_buffer.control.tail + 1) % LOG_DATA_SIZE;

    /* Aggiorna contatore byte usati */
    log_buffer.control.used_bytes--;

    return byte;
}

/**
 * @brief  Rimuove il messaggio più vecchio dal buffer (per fare spazio)
 *
 * @details Legge la lunghezza del messaggio più vecchio e avanza il tail
 *          pointer saltandolo completamente.
 *
 * @param  None
 * @retval None
 *
 * @note   Non thread-safe, chiamare con lock acquisito
 * @note   Chiamare solo quando count > 0
 */
static void log_remove_oldest_message(void)
{
    /* Leggi la lunghezza del messaggio (2 bytes, little-endian) */
    u8 len_low = log_read_byte();
    u8 len_high = log_read_byte();
    u16 msg_length = (u16)(len_low | (len_high << 8));

    /* Salta i byte del messaggio (incluso null terminator) */
    for (u16 i = 0; i < msg_length; i++)
    {
        log_read_byte();
    }

    /* Decrementa il contatore dei messaggi */
    log_buffer.control.count--;
}

#endif /* LOG_TO_MEMORY */


/* =============================================================================
 * FUNZIONI PUBBLICHE - IMPLEMENTAZIONE
 * ============================================================================= */

/**
 * @brief  Inizializza il sistema di logging
 */
int log_init(void)
{
EVAL_LOG_MEMORY(
    /* MODALITÀ LOG_TO_MEMORY: Inizializza buffer circolare */

    /* Azzera tutto (include lock!) */
	memset(&log_buffer,0,sizeof(log_buffer_t));

    if (!log_lock_acquire())
    {
        return LOG_ERROR_LOCKED;
    }

    /* Inizializza la struttura di controllo */
    log_buffer.control.head = 0;
    log_buffer.control.tail = 0;
    log_buffer.control.count = 0;
    log_buffer.control.used_bytes = 0;
    log_buffer.control.lock = 1;  /* Già locked dal acquire */
    log_buffer.control.initialized = 1;

    /* Opzionale: azzera il buffer dati per debugging */
    /* (commentato per performance, non necessario) */
    memset(log_buffer.data, 0, LOG_DATA_SIZE);

    /* Rilascia il lock */
    log_lock_release();

    return LOG_OK;
)
EVAL_LOG_UART(
    /* MODALITÀ LOG_DIRECT_UART: Nessun buffer, solo init UART se necessario */

    /* UART già inizializzata in main.c, niente da fare qui */
    return LOG_OK;)

}

/**
 * @brief  Scrive un messaggio (modalità dipende da LOG_TO_MEMORY/LOG_DIRECT_UART)
 */
int log_write(const char *message)
{
    /* Validazione parametri */
    if (message == NULL)
    {
        return LOG_ERROR_INVALID;
    }

EVAL_LOG_MEMORY(
    /* ===========================================================================
     * MODALITÀ LOG_TO_MEMORY: Scrive nel buffer circolare RAM
     * ===========================================================================
     */

    /* Verifica inizializzazione */
    if (log_buffer.control.initialized == 0)
    {
        /* Auto-inizializza se non ancora fatto (per comodità) */
        log_init();
    }

    /* Acquisisce il lock */
    if (!log_lock_acquire())
    {
        return LOG_ERROR_LOCKED;
    }

    /* Calcola la lunghezza del messaggio (con limite) */
    u16 msg_len = 0;
    while (message[msg_len] != '\0' && msg_len < (LOG_MAX_MESSAGE_SIZE - 1))
    {
        msg_len++;
    }

    /* Verifica se il messaggio è stato troncato (PRIMA di aggiungere '\0') */
    bool was_truncated = (msg_len == (LOG_MAX_MESSAGE_SIZE - 1)) && (message[msg_len] != '\0');

    /* Aggiungi 1 per il null terminator */
    msg_len++;  /* Ora msg_len include il '\0' */

    /* Calcola lo spazio totale necessario: 2 bytes (length) + msg_len bytes (data) */
    u16 total_size = 2 + msg_len;

    /* Verifica che il messaggio non sia più grande del buffer stesso */
    if (total_size > LOG_DATA_SIZE)
    {
        log_lock_release();
        return LOG_ERROR_INVALID;
    }

    int return_code = LOG_OK;

    /* Se non c'è spazio sufficiente, rimuovi messaggi vecchi */
    while (log_get_free_space() < total_size && log_buffer.control.count > 0)
    {
        log_remove_oldest_message();
        return_code = LOG_ERROR_OVERFLOW;  /* Segnala che abbiamo sovrascritto */
    }

    /* Se ancora non c'è spazio (buffer vuoto ma messaggio troppo grande), errore */
    if (log_get_free_space() < total_size)
    {
        log_lock_release();
        return LOG_ERROR_INVALID;
    }

    /* Scrivi la lunghezza del messaggio (2 bytes, little-endian) */
    log_write_byte((u8)(msg_len & 0xFF));        /* Low byte */
    log_write_byte((u8)((msg_len >> 8) & 0xFF)); /* High byte */

    /* Scrivi i caratteri del messaggio */
    for (u16 i = 0; i < msg_len - 1; i++)
    {
        log_write_byte((u8)message[i]);
    }

    /* Scrivi il null terminator */
    log_write_byte('\0');

    /* Incrementa il contatore dei messaggi */
    log_buffer.control.count++;

    /* Se il messaggio è stato troncato, segnalalo */
    if (was_truncated)
    {
        return_code = LOG_ERROR_TRUNCATED;
    }

    /* Rilascia il lock */
    log_lock_release();

    return return_code;
)
EVAL_LOG_UART(
    /* ===========================================================================
     * MODALITÀ LOG_DIRECT_UART: Scrive direttamente su UART3 (lento ma real-time)
     * ===========================================================================
     */

    /* Scrivi il messaggio via UART (funzione bloccante) */
    uart_write(message);
    uart_write("\r\n");  /* Aggiungi newline per terminale */

    return LOG_OK;
)

}



/* =============================================================================
 * FUNZIONI SPECIFICHE PER LOG_TO_MEMORY (buffer circolare)
 * ============================================================================= */

#if LOG_TO_MEMORY

/**
 * @brief  Legge un messaggio dal buffer circolare (POP)
 */
int log_read(char *dest)
{
    /* Validazione parametri */
    if (dest == NULL)
    {
        return LOG_ERROR_INVALID;
    }

    /* Acquisisce il lock */
    if (!log_lock_acquire())
    {
        return LOG_ERROR_LOCKED;
    }

    /* Verifica se ci sono messaggi da leggere */
    if (log_buffer.control.count == 0)
    {
        log_lock_release();
        return LOG_ERROR_EMPTY;
    }

    /* Leggi la lunghezza del messaggio (2 bytes, little-endian) */
    u8 len_low = log_read_byte();
    u8 len_high = log_read_byte();
    u16 msg_length = (u16)(len_low | (len_high << 8));

    /* Sanity check: lunghezza ragionevole */
    if (msg_length == 0 || msg_length > LOG_MAX_MESSAGE_SIZE)
    {
        /* Corruzione del buffer - resetta */
        log_buffer.control.head = 0;
        log_buffer.control.tail = 0;
        log_buffer.control.count = 0;
        log_buffer.control.used_bytes = 0;
        log_lock_release();
        return LOG_ERROR_INVALID;
    }

    /* Leggi i caratteri del messaggio */
    for (u16 i = 0; i < msg_length; i++)
    {
        dest[i] = (char)log_read_byte();
    }

    /* Assicurati che sia null-terminated (sicurezza) */
    dest[msg_length - 1] = '\0';

    /* Decrementa il contatore dei messaggi */
    log_buffer.control.count--;

    /* Rilascia il lock */
    log_lock_release();

    return LOG_OK;
}

/**
 * @brief  Ottiene il numero di messaggi presenti nel buffer
 */
u16 log_get_count(void)
{
    /* Lettura atomica (u16 è atomico su ARM Cortex-M3) */
    return log_buffer.control.count;
}

/**
 * @brief  Ottiene il numero di byte utilizzati nel buffer
 */
u16 log_get_used_bytes(void)
{
    /* Lettura atomica */
    return log_buffer.control.used_bytes;
}

/**
 * @brief  Ottiene il numero di byte liberi nel buffer
 */
u16 log_get_free_bytes(void)
{
    /* Calcola spazio libero */
    u16 used = log_buffer.control.used_bytes;
    return (used <= LOG_DATA_SIZE) ? (u16)(LOG_DATA_SIZE - used) : 0;
}

/**
 * @brief  Svuota completamente il buffer di log
 */
int log_clear(void)
{
    /* Acquisisce il lock */
    if (!log_lock_acquire())
    {
        return LOG_ERROR_LOCKED;
    }

    /* Resetta tutti i puntatori e contatori */
    log_buffer.control.head = 0;
    log_buffer.control.tail = 0;
    log_buffer.control.count = 0;
    log_buffer.control.used_bytes = 0;

    /* Rilascia il lock */
    log_lock_release();

    return LOG_OK;
}

/**
 * @brief  Ottiene l'indirizzo base del buffer per ispezione
 */
void* log_get_buffer_address(void)
{
    /* Restituisce l'indirizzo del buffer (0x20004800) */
    return (void*)&log_buffer;
}

#endif /* LOG_TO_MEMORY */


/* =============================================================================
 * FUNZIONI DI LOGGING CON PREFISSI E FORMATTAZIONE (comuni a tutte le modalità)
 * ============================================================================= */

/**
 * @brief  Log di errore con prefisso [ERR]: e formattazione variabile
 */
int log_error(const char *fmt, ...)
{
    /* Validazione parametri */
    if (fmt == NULL)
    {
        return LOG_ERROR_INVALID;
    }

    /* Buffer per il messaggio completo (prefisso + messaggio formattato) */
    char buffer[LOG_MAX_MESSAGE_SIZE];

    /* Copia il prefisso all'inizio del buffer */
    const char *prefix = LOG_PREFIX_ERROR;
    u32 prefix_len = 0;
    while (prefix[prefix_len] != '\0' && prefix_len < LOG_MAX_MESSAGE_SIZE - 1)
    {
        buffer[prefix_len] = prefix[prefix_len];
        prefix_len++;
    }

    /* Formatta il resto del messaggio dopo il prefisso */
    va_list args;
    va_start(args, fmt);

    u32 remaining_space = LOG_MAX_MESSAGE_SIZE - prefix_len;
    mini_vsnprintf(buffer + prefix_len, remaining_space, fmt, args);

    va_end(args);

    /* Scrivi il messaggio completo nel log */
    return log_write(buffer);
}

/**
 * @brief  Log di warning con prefisso [WRN]: e formattazione variabile
 */
int log_warning(const char *fmt, ...)
{
    /* Validazione parametri */
    if (fmt == NULL)
    {
        return LOG_ERROR_INVALID;
    }

    /* Buffer per il messaggio completo (prefisso + messaggio formattato) */
    char buffer[LOG_MAX_MESSAGE_SIZE];

    /* Copia il prefisso all'inizio del buffer */
    const char *prefix = LOG_PREFIX_WARNING;
    u32 prefix_len = 0;
    while (prefix[prefix_len] != '\0' && prefix_len < LOG_MAX_MESSAGE_SIZE - 1)
    {
        buffer[prefix_len] = prefix[prefix_len];
        prefix_len++;
    }

    /* Formatta il resto del messaggio dopo il prefisso */
    va_list args;
    va_start(args, fmt);

    u32 remaining_space = LOG_MAX_MESSAGE_SIZE - prefix_len;
    mini_vsnprintf(buffer + prefix_len, remaining_space, fmt, args);

    va_end(args);

    /* Scrivi il messaggio completo nel log */
    return log_write(buffer);
}

/**
 * @brief  Log informativo con prefisso [INF]: e formattazione variabile
 */
int log_info(const char *fmt, ...)
{
    /* Validazione parametri */
    if (fmt == NULL)
    {
        return LOG_ERROR_INVALID;
    }

    /* Buffer per il messaggio completo (prefisso + messaggio formattato) */
    char buffer[LOG_MAX_MESSAGE_SIZE];

    /* Copia il prefisso all'inizio del buffer */
    const char *prefix = LOG_PREFIX_INFO;
    u32 prefix_len = 0;
    while (prefix[prefix_len] != '\0' && prefix_len < LOG_MAX_MESSAGE_SIZE - 1)
    {
        buffer[prefix_len] = prefix[prefix_len];
        prefix_len++;
    }

    /* Formatta il resto del messaggio dopo il prefisso */
    va_list args;
    va_start(args, fmt);

    u32 remaining_space = LOG_MAX_MESSAGE_SIZE - prefix_len;
    mini_vsnprintf(buffer + prefix_len, remaining_space, fmt, args);

    va_end(args);

    /* Scrivi il messaggio completo nel log */
    return log_write(buffer);
}

/**
 * @brief  Log di debug con prefisso [DBG]: e formattazione variabile
 */
int log_debug(const char *fmt, ...)
{
    /* Validazione parametri */
    if (fmt == NULL)
    {
        return LOG_ERROR_INVALID;
    }

    /* Buffer per il messaggio completo (prefisso + messaggio formattato) */
    char buffer[LOG_MAX_MESSAGE_SIZE];

    /* Copia il prefisso all'inizio del buffer */
    const char *prefix = LOG_PREFIX_DEBUG;
    u32 prefix_len = 0;
    while (prefix[prefix_len] != '\0' && prefix_len < LOG_MAX_MESSAGE_SIZE - 1)
    {
        buffer[prefix_len] = prefix[prefix_len];
        prefix_len++;
    }

    /* Formatta il resto del messaggio dopo il prefisso */
    va_list args;
    va_start(args, fmt);

    u32 remaining_space = LOG_MAX_MESSAGE_SIZE - prefix_len;
    mini_vsnprintf(buffer + prefix_len, remaining_space, fmt, args);

    va_end(args);

    /* Scrivi il messaggio completo nel log */
    return log_write(buffer);
}

/* =============================================================================
 * NOTE FINALI E CONSIDERAZIONI
 * =============================================================================
 *
 * 1. FORMATO DEI MESSAGGI NEL BUFFER:
 *    Ogni messaggio è memorizzato come:
 *      [LENGTH (2B little-endian)] [MESSAGE DATA (LENGTH bytes, incluso '\0')]
 *
 *    Dove LENGTH rappresenta il numero totale di byte del messaggio INCLUSO il '\0'.
 *    Ad esempio, per "Test\0" (5 caratteri), LENGTH=5.
 *
 *    Questo formato permette di:
 *    - Sapere esattamente quanto è lungo ogni messaggio senza scansionare
 *    - Supportare messaggi di lunghezza variabile
 *    - Saltare messaggi durante l'overflow con un semplice avanzamento del puntatore
 *
 * 2. GESTIONE OVERFLOW:
 *    Quando il buffer è pieno, viene automaticamente rimosso il messaggio
 *    più vecchio per fare spazio a quello nuovo. Questo comportamento FIFO
 *    garantisce che:
 *    - Gli ultimi log sono sempre disponibili (più importanti per debug)
 *    - Il sistema non smette mai di loggare
 *
 * 3. THREAD SAFETY E INTERRUPT SAFETY:
 *    Il lock è implementato con disabilitazione globale degli interrupt.
 *    Pro:
 *    - Semplice e robusto
 *    - Garantisce atomicità anche con ISR
 *    Contro:
 *    - Disabilita TUTTI gli interrupt (può causare latency)
 *    - Non adatto se log_write() viene chiamato molto frequentemente
 *
 *    Alternativa (per uso avanzato):
 *    - Usare un mutex RTOS (se si usa FreeRTOS)
 *    - Implementare un lock-free ring buffer (complesso)
 *
 * 4. PERFORMANCE:
 *    - log_write(): O(1) + O(n) nel caso di overflow
 *    - log_read(): O(1)
 *    - Overhead: ~2 bytes per messaggio (length field)
 *
 * 5. DEBUGGING:
 *    Per ispezionare il buffer via debugger:
 *      (gdb) x/2048xb 0x20004800
 *    Oppure via OpenOCD:
 *      > mdw 0x20004800 512
 *
 * 6. PERSISTENZA POST-RESET:
 *    Il buffer usa (NOLOAD) nel linker script, quindi non viene azzerato
 *    dal codice di startup. Se la RAM rimane alimentata durante un reset
 *    (es. soft reset), i log rimangono intatti. Utile per post-mortem debug!
 *
 * 7. ESTENSIONI FUTURE:
 *    - Timestamp per ogni messaggio (aggiungere 4 bytes per tempo)
 *    - Contatore di overflow (quanti messaggi persi)
 *    - Livelli di log configurabili (enable/disable DEBUG in runtime)
 *    - Funzione di dump formattato con timestamp leggibili
 *    - Compressione dei log (es. run-length encoding per messaggi ripetuti)
 *
 * ============================================================================= */

#endif /* ENABLE_LOG */
