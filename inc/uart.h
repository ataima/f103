/**
 ******************************************************************************
 * @file           : uart.h
 * @brief          : UART driver per console seriale
 * @author         : STM32F103 Project
 ******************************************************************************
 * @attention
 *
 * Questo modulo implementa un driver UART per comunicazione seriale.
 * Utile per debug, logging, e comunicazione con PC o altri dispositivi.
 *
 * CONFIGURAZIONE DEFAULT:
 * - UART1 (USART1)
 * - Baud rate: 115200
 * - Data bits: 8
 * - Parity: None
 * - Stop bits: 1
 * - Hardware flow control: None
 *
 * PIN UTILIZZATI:
 * - TX: PA9  (USART1_TX)
 * - RX: PA10 (USART1_RX) - opzionale se serve solo TX
 *
 * COLLEGAMENTO CON PC:
 * - Usa un adattatore USB-UART (es. CP2102, FTDI, CH340)
 * - Collega: GND-GND, TX(PA9)->RX(adapter), RX(PA10)->TX(adapter)
 * - Apri terminale seriale: minicom, putty, screen, etc.
 *
 ******************************************************************************
 */

#ifndef UART_H_
#define UART_H_

#include "common.h"

/* =============================================================================
 * CONFIGURAZIONE UART
 * ============================================================================= */

/**
 * @brief Quale USART utilizzare (1, 2, o 3)
 *
 * USART1: APB2 @ 72 MHz - Pin TX=PA9, RX=PA10
 * USART2: APB1 @ 36 MHz - Pin TX=PA2, RX=PA3
 * USART3: APB1 @ 36 MHz - Pin TX=PB10, RX=PB11
 *
 * Default: USART1 (più veloce, pin comodi sulla Blue Pill)
 */
#define UART_USE_USART      1

/**
 * @brief Baud rate per la comunicazione seriale
 *
 * Valori tipici:
 * - 9600    : Standard basso, compatibilità massima
 * - 38400   : Veloce per debug
 * - 115200  : Standard moderno (DEFAULT)
 * - 230400  : Molto veloce
 * - 460800  : Altissima velocità (richiede UART di qualità)
 * - 921600  : Massima velocità pratica
 */
#define UART_BAUDRATE       115200

/**
 * @brief Abilita supporto RX (ricezione)
 *
 * Se 0: solo TX (trasmissione) - risparmia risorse
 * Se 1: TX + RX - permette comunicazione bidirezionale
 */
#define UART_ENABLE_RX      0

/**
 * @brief Abilita interrupts per RX
 *
 * Se 1: usa interrupt per ricezione (non-blocking)
 * Se 0: polling (blocking) - più semplice ma meno efficiente
 *
 * NOTA: Richiede UART_ENABLE_RX = 1
 */
#define UART_ENABLE_RX_IRQ  0

/**
 * @brief Dimensione buffer RX (se UART_ENABLE_RX_IRQ = 1)
 */
#define UART_RX_BUFFER_SIZE 128

/* =============================================================================
 * CODICI DI RITORNO
 * ============================================================================= */

#define UART_OK             0       /* Operazione riuscita */
#define UART_ERROR_INIT     -1      /* Errore durante inizializzazione */
#define UART_ERROR_TIMEOUT  -2      /* Timeout durante trasmissione */
#define UART_ERROR_INVALID  -3      /* Parametro invalido */
#define UART_ERROR_BUSY     -4      /* UART occupato */

/* =============================================================================
 * FUNZIONI PUBBLICHE
 * ============================================================================= */

/**
 * @brief  Inizializza l'UART con configurazione 115200,8,N,1
 *
 * @details Configura i GPIO, abilita il clock USART, imposta baud rate,
 *          formato dati (8 bit, no parity, 1 stop), e abilita TX (e RX se configurato).
 *
 *          Sequenza di inizializzazione:
 *          1. Abilita clock GPIO porta A (per PA9/PA10)
 *          2. Configura PA9 come alternate function push-pull (TX)
 *          3. Configura PA10 come floating input (RX) se UART_ENABLE_RX = 1
 *          4. Abilita clock USART1 (APB2)
 *          5. Calcola e imposta baud rate register (BRR)
 *          6. Configura formato: 8 bit, no parity, 1 stop bit
 *          7. Abilita USART e TX (e RX se configurato)
 *
 * @param  None
 *
 * @retval int - UART_OK se successo, codice errore altrimenti
 *
 * @note   Chiamare DOPO SystemClock_Config() perché usa PCLK2!
 * @note   Baud rate effettivo dipende da PCLK2 (72 MHz per USART1)
 *
 * @example
 *   SystemClock_Config();
 *   uart_init();
 *   uart_write("Hello UART!\n");
 */
int uart_init(void);

/**
 * @brief  Deinizializza l'UART
 *
 * @details Disabilita USART e rilascia i pin GPIO.
 *
 * @param  None
 * @retval int - UART_OK se successo
 */
int uart_finit(void);

/**
 * @brief  Scrive una stringa sull'UART
 *
 * @details Invia una stringa null-terminated attraverso l'UART.
 *          Ogni carattere viene scritto nel registro USART_DR e aspetta
 *          che il flag TXE (Transmit Data Register Empty) sia settato
 *          prima di inviare il successivo.
 *
 *          IMPORTANTE: Questa funzione è BLOCCANTE! Aspetta che ogni
 *          byte sia trasmesso prima di ritornare.
 *
 * @param  str - Puntatore alla stringa null-terminated da inviare
 *
 * @retval int - UART_OK se successo
 *             - UART_ERROR_INVALID se str è NULL
 *             - UART_ERROR_TIMEOUT se timeout durante la trasmissione
 *
 * @note   Thread-safe: No (non usa lock)
 * @note   Interrupt-safe: Sì (ma può causare latency lunga)
 *
 * @example
 *   uart_write("Temperatura: ");
 *   uart_write("25.3 C\n");
 */
int uart_write(const char *str);

/**
 * @brief  Invia un singolo carattere sull'UART
 *
 * @details Scrive un byte nel registro USART_DR e aspetta che venga
 *          trasmesso (flag TXE).
 *
 * @param  ch - Carattere da inviare
 *
 * @retval int - UART_OK se successo, UART_ERROR_TIMEOUT se timeout
 *
 * @note   Funzione bloccante (attende che il registro sia libero)
 */
int uart_putchar(char ch);

/**
 * @brief  Legge un carattere dall'UART (se RX abilitato)
 *
 * @details Aspetta finché il flag RXNE (Receive Data Register Not Empty)
 *          è settato, poi legge il byte dal registro USART_DR.
 *
 * @param  ch - Puntatore dove salvare il carattere ricevuto
 *
 * @retval int - UART_OK se successo
 *             - UART_ERROR_INVALID se ch è NULL
 *             - UART_ERROR_TIMEOUT se timeout
 *
 * @note   Funzione bloccante (aspetta fino a ricezione)
 * @note   Disponibile solo se UART_ENABLE_RX = 1
 */
#if UART_ENABLE_RX
int uart_getchar(char *ch);
#endif

/**
 * @brief  Dumpa tutto il buffer circolare di log via UART
 *
 * @details Legge tutti i messaggi presenti nel buffer circolare di log
 *          (implementato in log.c) e li invia sequenzialmente via UART.
 *
 *          Utile per:
 *          - Ispezionare i log accumulati durante l'esecuzione
 *          - Debug post-mortem (se il log sopravvive al reset)
 *          - Scaricare tutti i log via seriale
 *
 *          Funzionamento:
 *          1. Legge il numero di messaggi nel buffer (log_get_count())
 *          2. Per ogni messaggio:
 *             a. Estrae il messaggio con log_read()
 *             b. Invia via UART con uart_write()
 *             c. Aggiunge un newline per leggibilità
 *
 * @param  None
 *
 * @retval int - Numero di messaggi inviati, o codice errore negativo
 *
 * @note   ATTENZIONE: Questa funzione SVUOTA il buffer di log!
 *         I messaggi vengono rimossi man mano che vengono letti.
 *
 * @note   Può essere lenta se ci sono molti messaggi nel buffer!
 *         A 115200 baud, ogni carattere richiede ~87 μs.
 *
 * @example
 *   // Dump periodico via pulsante
 *   if (button_pressed) {
 *       uart_write("\n=== LOG DUMP ===\n");
 *       int count = log_via_uart();
 *       uart_write("=== END ===\n");
 *   }
 *
 *   // Dump all'avvio dopo reset
 *   SystemClock_Config();
 *   uart_init();
 *   uart_write("=== POST-MORTEM LOG DUMP ===\n");
 *   log_via_uart();
 *   uart_write("=== END LOG DUMP ===\n");
 *   log_clear();  // Svuota per iniziare con log freschi
 */
int log_via_uart(void);

/**
 * @brief  Scrive un numero intero sull'UART (helper function)
 *
 * @details Converte un numero intero in stringa decimale e lo invia via UART.
 *          Utile per debug veloce senza sprintf.
 *
 * @param  num - Numero da stampare
 *
 * @retval int - UART_OK se successo
 *
 * @example
 *   uart_write("Counter: ");
 *   uart_write_int(counter);
 *   uart_write("\n");
 */
int uart_write_int(i32 num);

/**
 * @brief  Scrive un numero in esadecimale sull'UART (helper function)
 *
 * @details Converte un numero in stringa esadecimale (0xABCD1234) e lo invia via UART.
 *
 * @param  num - Numero da stampare in hex
 *
 * @retval int - UART_OK se successo
 *
 * @example
 *   uart_write("Address: ");
 *   uart_write_hex((u32)&variable);
 *   uart_write("\n");
 */
int uart_write_hex(u32 num);

/* =============================================================================
 * NOTE DI UTILIZZO
 * ============================================================================= */

/*
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
 *     // 3. Inizializza l'UART
 *     if (uart_init() == UART_OK) {
 *         uart_write("UART initialized @ 115200 baud\n");
 *     }
 *
 *     // 4. Logga alcuni messaggi
 *     log_info("Sistema avviato");
 *     log_debug("Clock: 72 MHz");
 *
 *     // 5. Dumpa i log via UART
 *     uart_write("\n=== Current Logs ===\n");
 *     int count = log_via_uart();
 *     uart_write("=== End ===\n");
 *
 *     // 6. Main loop con logging
 *     while(1) {
 *         // Log normale in RAM
 *         log_info("Heartbeat");
 *
 *         // Output diretto via UART (più lento, ma persistente)
 *         uart_write(".");
 *
 *         delay_ms(1000);
 *     }
 * }
 *
 *
 * CONFIGURAZIONE TERMINALE SERIALE:
 * ==================================
 *
 * Linux (minicom):
 * ----------------
 * sudo minicom -D /dev/ttyUSB0 -b 115200
 *
 * Linux (screen):
 * ---------------
 * screen /dev/ttyUSB0 115200
 *
 * Windows (PuTTY):
 * ----------------
 * Serial line: COM3 (o quello del tuo adattatore)
 * Speed: 115200
 * Data bits: 8
 * Stop bits: 1
 * Parity: None
 * Flow control: None
 *
 * Python (pyserial):
 * ------------------
 * import serial
 * ser = serial.Serial('/dev/ttyUSB0', 115200)
 * while True:
 *     print(ser.readline().decode('utf-8'), end='')
 *
 *
 * TROUBLESHOOTING:
 * ================
 *
 * Problema: Nessun output sul terminale
 * Soluzione:
 *   1. Verifica che i cavi siano collegati (GND comune!)
 *   2. Verifica che TX micro → RX adattatore
 *   3. Verifica baud rate corretto (115200)
 *   4. Prova a invertire TX/RX se hai dubbi
 *   5. Verifica che uart_init() ritorni UART_OK
 *
 * Problema: Output corrotto o caratteri strani
 * Soluzione:
 *   1. Verifica baud rate (deve essere uguale su entrambi i lati)
 *   2. Verifica che SystemClock_Config() sia stato chiamato prima
 *   3. Verifica che il clock sia a 72 MHz
 *   4. Prova a ridurre il baud rate a 38400
 *
 * Problema: Output lento o mancante
 * Soluzione:
 *   1. uart_write() è bloccante - evita chiamate in ISR critiche
 *   2. Riduci la quantità di dati inviati
 *   3. Aumenta il baud rate a 230400 o superiore
 */

#endif /* UART_H_ */
