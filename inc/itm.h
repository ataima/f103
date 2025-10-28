/**
 ******************************************************************************
 * @file           : itm.h
 * @brief          : ITM (Instrumentation Trace Macrocell) per console via SWO
 * @author         : STM32F103 Project
 ******************************************************************************
 * @attention
 *
 * Questo modulo implementa il supporto per l'ITM, che permette di inviare
 * messaggi di debug attraverso il pin SWO (Serial Wire Output) dello ST-Link.
 *
 * L'ITM è una periferica del core ARM Cortex-M3 che permette di inviare dati
 * attraverso il debugger senza usare pin GPIO o UART dedicati.
 *
 * VANTAGGI:
 * - Non occupa UART hardware
 * - Non occupa pin GPIO
 * - Alta velocità (fino a diversi Mbit/s)
 * - Zero overhead sulla CPU (DMA-like)
 *
 * CONFIGURAZIONE NECESSARIA:
 * 1. Il clock SWO deve essere configurato nel debugger (vedere note sotto)
 * 2. Il pin SWO (PB3 su STM32F103) deve essere lasciato libero
 * 3. Il debugger deve essere connesso (ST-Link V2 o superiore)
 *
 * COME VISUALIZZARE I MESSAGGI:
 * - STM32CubeIDE: Window → Show View → SWV → SWV ITM Data Console
 * - OpenOCD: monitor tpiu config internal - uart off <SYSCLK_Hz>
 * - ST-Link Utility: ST-Link → Printf via SWO viewer
 *
 ******************************************************************************
 */

#ifndef ITM_H_
#define ITM_H_

#include "common.h"

#define ENABLE_ITM 0


#if ENABLE_ITM
/* =============================================================================
 * CONFIGURAZIONE ITM
 * ============================================================================= */

/**
 * @brief Stimulus port utilizzato per i messaggi di log
 *
 * L'ITM ha 32 stimulus port (0-31). Il port 0 è tipicamente usato per printf.
 * Possiamo usare il port 1 per i nostri log dedicati.
 */
#define ITM_LOG_PORT    0    /* Port 0 per compatibilità con printf via ITM */

/**
 * @brief Frequenza SYSCLK in Hz (deve corrispondere al clock effettivo)
 *
 * IMPORTANTE: Questo valore deve essere uguale a quello configurato in clock.c!
 * Per STM32F103 a massima velocità: 72 MHz
 */
#define ITM_SYSCLK_HZ   72000000UL

/**
 * @brief Frequenza SWO desiderata in Hz
 *
 * Frequenze tipiche:
 * - 2000000 (2 MHz): Veloce, richiede debugger di qualità
 * - 1000000 (1 MHz): Compromesso buono
 * - 500000 (500 kHz): Più stabile, compatibilità migliore
 *
 * NOTA: Deve essere un divisore di SYSCLK!
 * SWO_PRESCALER = (SYSCLK / SWO_FREQ) - 1
 *
 * IMPORTANTE: Se hai problemi con "failed to init SWV", prova:
 * 1. Ridurre a 500000 Hz (500 kHz)
 * 2. Verificare che il debugger sia configurato con lo stesso valore
 * 3. Aggiungere un delay dopo SystemClock_Config() nel main
 */
#define ITM_SWO_FREQ_HZ 500000UL  /* 500 kHz per massima compatibilità */

/* =============================================================================
 * CODICI DI RITORNO
 * ============================================================================= */

#define ITM_OK              0       /* Operazione riuscita */
#define ITM_ERROR_INIT      -1      /* Errore durante inizializzazione */
#define ITM_ERROR_TIMEOUT   -2      /* Timeout durante scrittura */
#define ITM_ERROR_INVALID   -3      /* Parametro invalido */

/* =============================================================================
 * FUNZIONI PUBBLICHE
 * ============================================================================= */

/**
 * @brief  Inizializza l'ITM per l'output via SWO
 *
 * @details Configura i registri dell'ITM e del TPIU (Trace Port Interface Unit)
 *          per permettere l'output dei messaggi attraverso il pin SWO.
 *
 *          Sequenza di inizializzazione:
 *          1. Abilita accesso ai registri di debug (DEMCR)
 *          2. Sblocca i registri ITM (Lock Access Register)
 *          3. Configura il prescaler SWO nel TPIU
 *          4. Configura il protocollo (NRZ/Manchester)
 *          5. Abilita l'ITM e gli stimulus port desiderati
 *
 * @param  None
 *
 * @retval int - ITM_OK se successo, codice errore altrimenti
 *
 * @note   Chiamare DOPO SystemClock_Config() perché usa SYSCLK_FREQ!
 * @note   Funziona solo se il debugger è connesso
 *
 * @example
 *   SystemClock_Config();
 *   itm_init();
 *   itm_write("Hello from ITM!\n");
 */
int itm_init(void);

/**
 * @brief  Deinizializza l'ITM e disabilita l'output SWO
 *
 * @details Disabilita l'ITM e i suoi stimulus port. Utile per risparmiare
 *          energia o liberare risorse quando l'ITM non è più necessario.
 *
 * @param  None
 * @retval int - ITM_OK se successo
 *
 * @note   Dopo questa chiamata, itm_write() non farà nulla
 */
int itm_finit(void);

/**
 * @brief  Scrive una stringa sull'ITM
 *
 * @details Invia una stringa null-terminated attraverso lo stimulus port ITM.
 *          Ogni carattere viene scritto nel registro ITM_STIM[port].
 *
 *          IMPORTANTE: Questa funzione è BLOCCANTE! Se il buffer ITM è pieno,
 *          aspetta che si svuoti. Evitare di chiamarla da ISR critiche.
 *
 * @param  str - Puntatore alla stringa null-terminated da inviare
 *
 * @retval int - ITM_OK se successo
 *             - ITM_ERROR_INVALID se str è NULL
 *             - ITM_ERROR_TIMEOUT se timeout durante la scrittura
 *
 * @note   Thread-safe: Sì (l'ITM gestisce la concorrenza internamente)
 * @note   Interrupt-safe: No (funzione bloccante, non usare in ISR critiche)
 *
 * @example
 *   itm_write("Temperatura: ");
 *   itm_write("25.3 C\n");
 */
int itm_write(const char *str);

/**
 * @brief  Invia un singolo carattere sull'ITM
 *
 * @details Scrive un byte nel registro stimulus dell'ITM. Utile per
 *          implementazioni custom di printf o per output binario.
 *
 * @param  ch - Carattere da inviare
 *
 * @retval int - ITM_OK se successo, ITM_ERROR_TIMEOUT se timeout
 *
 * @note   Funzione bloccante (attende che il FIFO ITM sia libero)
 */
int itm_putchar(char ch);

/**
 * @brief  Dumpa tutto il buffer circolare di log via ITM
 *
 * @details Legge tutti i messaggi presenti nel buffer circolare di log
 *          (implementato in log.c) e li invia sequenzialmente via ITM.
 *
 *          Utile per:
 *          - Ispezionare i log accumulati durante l'esecuzione
 *          - Debug post-mortem (se il log sopravvive al reset)
 *          - Scaricare tutti i log prima di una operazione critica
 *
 *          Funzionamento:
 *          1. Legge il numero di messaggi nel buffer (log_get_count())
 *          2. Per ogni messaggio:
 *             a. Estrae il messaggio con log_read()
 *             b. Invia via ITM con itm_write()
 *             c. Aggiunge un newline per leggibilità
 *
 * @param  None
 *
 * @retval int - Numero di messaggi inviati, o codice errore negativo
 *
 * @note   ATTENZIONE: Questa funzione SVUOTA il buffer di log!
 *         I messaggi vengono rimossi man mano che vengono letti.
 *         Se vuoi preservare il log, considera di modificare la funzione
 *         per usare un metodo di lettura non-distruttivo.
 *
 * @note   Può essere lenta se ci sono molti messaggi nel buffer!
 *         Evitare di chiamarla in contesti time-critical.
 *
 * @example
 *   // Scenario 1: Dump periodico
 *   if (button_pressed) {
 *       int count = log_via_itm();
 *       itm_write("--- Dumped ");
 *       // ... stampa count ...
 *       itm_write(" messages ---\n");
 *   }
 *
 *   // Scenario 2: Dump all'avvio dopo reset
 *   SystemClock_Config();
 *   itm_init();
 *   itm_write("=== POST-MORTEM LOG DUMP ===\n");
 *   log_via_itm();
 *   itm_write("=== END LOG DUMP ===\n");
 *   log_clear();  // Svuota per iniziare con log freschi
 */
int log_via_itm(void);

/* =============================================================================
 * NOTE DI UTILIZZO
 * ============================================================================= */

/*
 * CONFIGURAZIONE DEL DEBUGGER:
 * ============================
 *
 * STM32CubeIDE:
 * -------------
 * 1. Run → Debug Configurations
 * 2. Seleziona la tua configurazione debug
 * 3. Tab "Debugger"
 * 4. Sezione "Serial Wire Viewer (SWV)":
 *    - Enable: ✓
 *    - Core Clock: 72.0 MHz  (deve corrispondere a SYSCLK!)
 *    - SWO Clock: 2000 kHz   (deve corrispondere a ITM_SWO_FREQ_HZ!)
 * 5. Apply e chiudi
 * 6. Durante il debug: Window → Show View → SWV → SWV ITM Data Console
 * 7. Nel pannello SWV ITM Data Console:
 *    - Configure Trace: abilita Port 0 (o quello che usi)
 *    - Start Trace
 *
 * OpenOCD:
 * --------
 * Nel file di configurazione OpenOCD (.cfg):
 *   tpiu config internal - uart off 72000000
 *
 * Oppure dalla telnet console:
 *   > monitor tpiu config internal - uart off 72000000
 *   > monitor itm port 0 on
 *
 * ST-Link Utility:
 * ----------------
 * 1. ST-Link → Printf via SWO viewer
 * 2. System clock: 72 MHz
 * 3. Stimulus port: 0
 * 4. Start
 *
 *
 * ESEMPIO DI UTILIZZO COMPLETO:
 * ==============================
 *
 * int main(void)
 * {
 *     // 1. Configura il clock a 72 MHz
 *     SystemClock_Config();
 *
 *     // 2. Inizializza il logging in RAM
 *     log_init();
 *
 *     // 3. Inizializza l'ITM
 *     if (itm_init() == ITM_OK) {
 *         itm_write("ITM initialized successfully!\n");
 *     }
 *
 *     // 4. Logga alcuni messaggi
 *     log_info("Sistema avviato");
 *     log_debug("Clock: 72 MHz");
 *
 *     // 5. Dumpa i log via ITM
 *     itm_write("\n=== Current Logs ===\n");
 *     int count = log_via_itm();
 *     itm_write("=== End ===\n");
 *
 *     // 6. Main loop con logging continuo
 *     while(1) {
 *         // Log normale in RAM
 *         log_info("Heartbeat");
 *
 *         // Output diretto via ITM (più veloce, ma non persistente)
 *         itm_write(".");
 *
 *         delay_ms(1000);
 *     }
 * }
 *
 *
 * TROUBLESHOOTING:
 * ================
 *
 * Problema: Non vedo output nell'ITM console
 * Soluzione:
 *   1. Verifica che il debugger sia connesso
 *   2. Verifica che la freq SWO nel debugger corrisponda a ITM_SWO_FREQ_HZ
 *   3. Verifica che SYSCLK nel debugger corrisponda a ITM_SYSCLK_HZ
 *   4. Assicurati che lo stimulus port sia abilitato nel debugger
 *   5. Verifica che itm_init() sia stato chiamato DOPO SystemClock_Config()
 *
 * Problema: Output ITM parziale o corrotto
 * Soluzione:
 *   1. Riduci ITM_SWO_FREQ_HZ (es. da 2 MHz a 500 kHz)
 *   2. Verifica che il cavo ST-Link sia di buona qualità
 *   3. Riduci la lunghezza del cavo debugger
 *
 * Problema: Sistema si blocca dopo itm_write()
 * Soluzione:
 *   1. Il debugger deve essere connesso prima di chiamare itm_init()
 *   2. Aggiungi un timeout nella funzione itm_putchar() (vedi implementazione)
 */

#endif /** ENABLE_ITM**/

#endif /* ITM_H_ */

