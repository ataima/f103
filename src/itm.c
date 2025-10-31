/**
 ******************************************************************************
 * @file           : itm.c
 * @brief          : Implementazione ITM per console via SWO
 * @author         : STM32F103 Project
 ******************************************************************************
 * @attention
 *
 * Questo file implementa il supporto per l'ITM (Instrumentation Trace Macrocell)
 * che permette di inviare messaggi di debug attraverso il pin SWO dello ST-Link.
 *
 * REGISTRI ITM (ARM Cortex-M3):
 * =============================
 *
 * L'ITM è una periferica standard del core ARM Cortex-M, documentata nel
 * "ARM CoreSight Components Technical Reference Manual".
 *
 * Indirizzi base:
 * - ITM base:  0xE0000000
 * - TPIU base: 0xE0040000  (Trace Port Interface Unit)
 * - DWT base:  0xE0001000  (Data Watchpoint and Trace)
 * - CoreDebug: 0xE000EDF0
 *
 ******************************************************************************
 */
#include "common.h"
#include "itm.h"
#include "log.h"
#include <stddef.h>

#if ENABLE_ITM
/* =============================================================================
 * DEFINIZIONI REGISTRI ITM E TPIU
 * ============================================================================= */

/* Core Debug Registers */
#define DEMCR           (*(volatile u32 *)0xE000EDFCUL)  /* Debug Exception and Monitor Control Register */
#define DEMCR_TRCENA    BIT(24)  /* Trace enable bit */

/* ITM Registers (Instrumentation Trace Macrocell) */
#define ITM_BASE        0xE0000000UL

/* ITM Stimulus Port Registers (32 ports: 0-31) */
/* Ogni port ha un registro a 32 bit per scrivere dati */
#define ITM_STIM(port)  (*(volatile u32 *)(ITM_BASE + 0x000UL + ((port) * 4)))

/* ITM Trace Enable Register - abilita i port desiderati */
#define ITM_TER         (*(volatile u32 *)(ITM_BASE + 0xE00UL))

/* ITM Trace Privilege Register - configura i livelli di privilegio */
#define ITM_TPR         (*(volatile u32 *)(ITM_BASE + 0xE40UL))

/* ITM Trace Control Register - controllo generale dell'ITM */
#define ITM_TCR         (*(volatile u32 *)(ITM_BASE + 0xE80UL))
#define ITM_TCR_ITMENA  BIT(0)   /* ITM Enable */
#define ITM_TCR_TSENA   BIT(1)   /* Timestamp enable */
#define ITM_TCR_SYNCENA BIT(2)   /* Sync packet enable */
#define ITM_TCR_TXENA   BIT(3)   /* DWT stimulus enable */
#define ITM_TCR_SWOENA  BIT(4)   /* SWO enable */
#define ITM_TCR_TraceBusID_SHIFT  16
#define ITM_TCR_TraceBusID_MASK   (0x7FUL << 16)

/* ITM Lock Access Register - sblocca i registri ITM per scrittura */
#define ITM_LAR         (*(volatile u32 *)(ITM_BASE + 0xFB0UL))
#define ITM_LAR_UNLOCK  0xC5ACCE55UL  /* Magic key per sbloccare */

/* ITM Lock Status Register - verifica se i registri sono bloccati */
#define ITM_LSR         (*(volatile u32 *)(ITM_BASE + 0xFB4UL))

/* TPIU Registers (Trace Port Interface Unit) */
#define TPIU_BASE       0xE0040000UL

/* TPIU Async Clock Prescaler Register - divisore per clock SWO */
#define TPIU_ACPR       (*(volatile u32 *)(TPIU_BASE + 0x010UL))

/* TPIU Selected Pin Protocol Register - seleziona protocollo (NRZ/Manchester) */
#define TPIU_SPPR       (*(volatile u32 *)(TPIU_BASE + 0x0F0UL))
#define TPIU_SPPR_NRZ   0x00000002UL  /* NRZ encoding (standard UART-like) */
#define TPIU_SPPR_MANCHESTER 0x00000001UL  /* Manchester encoding */

/* TPIU Formatter and Flush Control Register */
#define TPIU_FFCR       (*(volatile u32 *)(TPIU_BASE + 0x304UL))
#define TPIU_FFCR_TrigIn BIT(8)

/* DWT (Data Watchpoint and Trace) Control Register */
#define DWT_CTRL        (*(volatile u32 *)0xE0001000UL)
#define DWT_CTRL_CYCCNTENA BIT(0)  /* Cycle counter enable */

/* =============================================================================
 * VARIABILI GLOBALI PRIVATE
 * ============================================================================= */

/* Flag di inizializzazione */
static volatile bool itm_initialized = false;

/* =============================================================================
 * FUNZIONI PRIVATE
 * ============================================================================= */

/**
 * @brief  Verifica se lo stimulus port è pronto per la scrittura
 *
 * @details Il registro ITM_STIM ha il bit 0 che indica se il FIFO è libero.
 *          Se bit[0] == 1, possiamo scrivere un nuovo byte.
 *          Se bit[0] == 0, il FIFO è pieno e dobbiamo aspettare.
 *
 * @param  port - Numero dello stimulus port (0-31)
 * @retval bool - true se pronto, false se occupato
 */
static inline bool itm_port_ready(u8 port)
{
    return (ITM_STIM(port) & BIT(0)) != 0;
}

/* =============================================================================
 * FUNZIONI PUBBLICHE - IMPLEMENTAZIONE
 * ============================================================================= */

/**
 * @brief  Inizializza l'ITM per l'output via SWO
 */
int itm_init(void)
{
    /* =========================================================================
     * STEP 1: ABILITA IL TRACE MODULE (DEMCR.TRCENA)
     * =========================================================================
     *
     * Prima di poter usare ITM, DWT, o qualsiasi altra periferica di debug,
     * dobbiamo abilitare il trace enable bit nel Debug Exception and Monitor
     * Control Register (DEMCR).
     */
    DEMCR |= DEMCR_TRCENA;

    /* =========================================================================
     * STEP 2: SBLOCCA I REGISTRI ITM (ITM LAR - Lock Access Register)
     * =========================================================================
     *
     * I registri ITM sono protetti da scrittura per default. Per sbloccarli,
     * dobbiamo scrivere una "magic key" (0xC5ACCE55) nel Lock Access Register.
     */
    ITM_LAR = ITM_LAR_UNLOCK;

    /* =========================================================================
     * STEP 3: CONFIGURA IL PRESCALER SWO NEL TPIU
     * =========================================================================
     *
     * Il TPIU (Trace Port Interface Unit) è responsabile di serializzare
     * i dati ITM sul pin SWO. Dobbiamo configurare il prescaler per ottenere
     * la frequenza SWO desiderata.
     *
     * Formula: ACPR = (SYSCLK / SWO_FREQ) - 1
     *
     * Esempio con SYSCLK = 72 MHz e SWO = 2 MHz:
     * ACPR = (72000000 / 2000000) - 1 = 36 - 1 = 35
     */
    u32 prescaler = (ITM_SYSCLK_HZ / ITM_SWO_FREQ_HZ) - 1;
    TPIU_ACPR = prescaler;

    /* =========================================================================
     * STEP 4: SELEZIONA IL PROTOCOLLO NRZ (UART-LIKE)
     * =========================================================================
     *
     * Il TPIU supporta due protocolli:
     * - NRZ (Non-Return-to-Zero): simile a UART, più semplice
     * - Manchester: encoding più robusto ma richiede doppia bandwidth
     *
     * Usiamo NRZ perché è lo standard e più compatibile.
     */
    TPIU_SPPR = TPIU_SPPR_NRZ;

    /* =========================================================================
     * STEP 5: DISABILITA IL FORMATTER (OPZIONALE)
     * =========================================================================
     *
     * Il formatter aggiunge frame intorno ai dati. Per l'ITM semplice non serve.
     */
    TPIU_FFCR = 0;

    /* =========================================================================
     * STEP 6: ABILITA GLI STIMULUS PORT DESIDERATI (ITM TER)
     * =========================================================================
     *
     * Il Trace Enable Register (TER) è un bitmask dove ogni bit abilita
     * il corrispondente stimulus port.
     *
     * Esempio:
     * - Bit 0 = 1: abilita port 0
     * - Bit 1 = 1: abilita port 1
     * - ...
     *
     * Abilitiamo il port ITM_LOG_PORT (tipicamente 0).
     */
    ITM_TER = BIT(ITM_LOG_PORT);

    /* =========================================================================
     * STEP 7: CONFIGURA IL TRACE PRIVILEGE REGISTER (OPZIONALE)
     * =========================================================================
     *
     * TPR controlla quali port sono accessibili in modalità user vs privileged.
     * Per semplicità, permettiamo accesso a tutti i port da tutti i livelli.
     */
    ITM_TPR = 0x00000000;  /* Tutti i port accessibili */

    /* =========================================================================
     * STEP 8: ABILITA L'ITM GLOBALMENTE (ITM TCR)
     * =========================================================================
     *
     * Il Trace Control Register (TCR) è il "master enable" dell'ITM.
     * Dobbiamo impostare:
     * - ITMENA = 1: abilita ITM
     * - SYNCENA = 1: abilita sync packets (opzionale, per debug)
     * - TraceBusID: ID univoco per questo trace stream (opzionale)
     */
    ITM_TCR = ITM_TCR_ITMENA | ITM_TCR_SYNCENA;

    /* =========================================================================
     * STEP 9: OPZIONALE - ABILITA IL CYCLE COUNTER DEL DWT
     * =========================================================================
     *
     * Il DWT (Data Watchpoint and Trace) ha un cycle counter che può essere
     * usato per timestamping. Opzionale ma utile per profiling.
     */
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;

    /* Marca come inizializzato */
    itm_initialized = true;

    return ITM_OK;
}

/**
 * @brief  Deinizializza l'ITM
 */
int itm_finit(void)
{
    /* Disabilita l'ITM */
    ITM_TCR = 0;

    /* Disabilita tutti gli stimulus port */
    ITM_TER = 0;

    /* Blocca i registri ITM */
    ITM_LAR = 0;

    /* Marca come non inizializzato */
    itm_initialized = false;

    return ITM_OK;
}

/**
 * @brief  Invia un singolo carattere via ITM
 */
int itm_putchar(char ch)
{
    /* Verifica se ITM è inizializzato */
    if (!itm_initialized) {
        return ITM_ERROR_INIT;
    }

    /* =========================================================================
     * SCRITTURA NEL REGISTRO STIMULUS
     * =========================================================================
     *
     * Per inviare un byte:
     * 1. Attendiamo che il FIFO sia libero (bit 0 del registro STIM == 1)
     * 2. Scriviamo il byte nel registro STIM[port]
     *
     * IMPORTANTE: Se il debugger NON è connesso, il bit 0 rimarrà sempre 0
     * e la funzione si bloccherebbe. Per evitare deadlock, aggiungiamo un timeout.
     */

    /* Timeout per evitare deadlock se debugger non connesso */
    u32 timeout = 10000;  /* ~10ms @ 72 MHz se ogni loop è ~1us */

    /* Attendi che il port sia pronto */
    while (!itm_port_ready(ITM_LOG_PORT) && timeout > 0) {
        timeout--;
    }

    /* Se timeout scaduto, il debugger probabilmente non è connesso */
    if (timeout == 0) {
        return ITM_ERROR_TIMEOUT;
    }

    /* Scrivi il carattere nel registro stimulus */
    ITM_STIM(ITM_LOG_PORT) = (u8)ch;

    return ITM_OK;
}

/**
 * @brief  Scrive una stringa via ITM
 */
int itm_write(const char *str)
{
    /* Validazione parametri */
    if (str == NULL) {
        return ITM_ERROR_INVALID;
    }

    /* Verifica se ITM è inizializzato */
    if (!itm_initialized) {
        return ITM_ERROR_INIT;
    }

    /* Invia carattere per carattere */
    while (*str != '\0') {
        int result = itm_putchar(*str);

        /* Se errore (timeout o altro), ritorna subito */
        if (result != ITM_OK) {
            return result;
        }

        str++;
    }

    return ITM_OK;
}

/**
 * @brief  Dumpa tutto il buffer circolare di log via ITM
 */
int log_via_itm(void)
{
#if ENABLE_LOG
    /* Buffer temporaneo per leggere i messaggi dal log */
    char buffer[LOG_MAX_MESSAGE_SIZE];

    /* Conta quanti messaggi ci sono nel buffer */
    u16 total_messages = log_get_count();
    u16 sent_count = 0;

    /* Verifica se ci sono messaggi da inviare */
    if (total_messages == 0) {
        itm_write("[LOG] Buffer vuoto - nessun messaggio da inviare\n");
        return 0;
    }

    /* Header del dump */
    itm_write("=======================================================\n");
    itm_write("[LOG DUMP] Inizio dump del buffer circolare\n");
    itm_write("[LOG DUMP] Messaggi presenti: ");

    /* Converti il numero in stringa e invialo */
    /* Nota: usiamo un approccio semplice senza sprintf per evitare dipendenze */
    char count_str[16];
    u16 temp = total_messages;
    int digits = 0;

    /* Conta le cifre */
    if (temp == 0) {
        count_str[digits++] = '0';
    } else {
        /* Estrai le cifre (in ordine inverso) */
        char temp_str[16];
        int i = 0;
        while (temp > 0) {
            temp_str[i++] = '0' + (temp % 10);
            temp /= 10;
        }
        /* Inverti le cifre */
        for (int j = i - 1; j >= 0; j--) {
            count_str[digits++] = temp_str[j];
        }
    }
    count_str[digits] = '\0';

    itm_write(count_str);
    itm_write("\n");
    itm_write("=======================================================\n\n");

    /* Leggi e invia tutti i messaggi uno per uno */
    while (log_get_count() > 0) {
        /* Legge il prossimo messaggio dal buffer (operazione POP) */
        int result = log_read(buffer);

        if (result == LOG_OK) {
            /* Invia il messaggio via ITM */
            itm_write(buffer);
            itm_write("\n");  /* Aggiungi newline per leggibilità */
            sent_count++;
        } else if (result == LOG_ERROR_EMPTY) {
            /* Buffer vuoto - finito */
            break;
        } else {
            /* Errore durante la lettura - segnala e continua */
            itm_write("[LOG DUMP ERROR] Errore lettura messaggio #");
            /* TODO: stampa numero messaggio */
            itm_write("\n");
            break;
        }
    }

    /* Footer del dump */
    itm_write("\n=======================================================\n");
    itm_write("[LOG DUMP] Fine dump - Messaggi inviati: ");

    /* Converti sent_count in stringa */
    temp = sent_count;
    digits = 0;
    if (temp == 0) {
        count_str[digits++] = '0';
    } else {
        char temp_str[16];
        int i = 0;
        while (temp > 0) {
            temp_str[i++] = '0' + (temp % 10);
            temp /= 10;
        }
        for (int j = i - 1; j >= 0; j--) {
            count_str[digits++] = temp_str[j];
        }
    }
    count_str[digits] = '\0';

    itm_write(count_str);
    itm_write("\n");
    itm_write("=======================================================\n");

    return sent_count;
#else
    itm_write("[LOG] Sistema di logging disabilitato (ENABLE_LOG=0)\n");
    return 0;
#endif
}

/* =============================================================================
 * NOTE IMPLEMENTATIVE
 * ============================================================================= */

/*
 * PERCHÉ SERVE IL TIMEOUT IN itm_putchar()?
 * ==========================================
 *
 * Se il debugger NON è connesso, il registro ITM_STIM[port] avrà sempre
 * il bit 0 = 0 (FIFO pieno), perché non c'è nessuno a leggere i dati.
 * Senza timeout, la funzione si bloccherebbe in un loop infinito.
 *
 * Con il timeout, se il debugger non è connesso, la funzione ritorna
 * ITM_ERROR_TIMEOUT dopo ~10ms e il programma può continuare.
 *
 *
 * ALTERNATIVE ALL'ITM PER DEBUG:
 * ===============================
 *
 * 1. UART tradizionale:
 *    Pro: Funziona sempre, non serve debugger
 *    Contro: Occupa pin GPIO, serve convertitore USB-Serial
 *
 * 2. Semihosting:
 *    Pro: printf() funziona direttamente
 *    Contro: MOLTO lento, blocca il programma, serve debugger
 *
 * 3. ITM (questo modulo):
 *    Pro: Veloce, non occupa GPIO, zero overhead
 *    Contro: Serve debugger connesso
 *
 * 4. Buffer circolare in RAM (log.c):
 *    Pro: Velocissimo, non serve nulla, persiste dopo reset
 *    Contro: Limitato a 2KB, serve tool per leggerlo
 *
 * SOLUZIONE IDEALE: Combinare ITM + buffer circolare!
 * - Durante debug: usa ITM per vedere log in tempo reale
 * - In produzione: usa buffer circolare, leggi via UART o debugger quando serve
 */


#endif
