/**
 ******************************************************************************
 * @file           : uart.c
 * @brief          : Implementazione driver UART per console seriale
 * @author         : STM32F103 Project
 ******************************************************************************
 * @attention
 *
 * Questo file implementa un driver UART bare-metal per STM32F103.
 * Supporta USART1/2/3 con configurazione 115200,8,N,1 (customizzabile).
 *
 * REGISTRI USART (STM32F103):
 * ============================
 *
 * I registri principali sono già definiti in stm32f103_regs.h:
 * - USART_SR: Status Register (flag TXE, RXNE, TC, etc.)
 * - USART_DR: Data Register (TX/RX buffer)
 * - USART_BRR: Baud Rate Register
 * - USART_CR1: Control Register 1 (enable, data format, interrupts)
 * - USART_CR2: Control Register 2 (stop bits, clock)
 * - USART_CR3: Control Register 3 (flow control)
 *
 ******************************************************************************
 */
#include "common.h"
#include "uart.h"
#include "log.h"
#include "clock.h"
#include "utils.h"
#include <stddef.h>

/* =============================================================================
 * SELEZIONE USART E PIN
 * ============================================================================= */

#if UART_USE_USART == 1
    /* USART1: APB2 @ 72 MHz */
    #define UART_USART_BASE     USART1_BASE
    #define UART_RCC_EN_BIT     RCC_APB2ENR_USART1EN
    #define UART_RCC_ENR        RCC_APB2ENR
    #define UART_PCLK_HZ        PCLK2_FREQ_HZ  /* 72 MHz */

    /* GPIO: PA9=TX, PA10=RX */
    #define UART_GPIO_PORT      GPIOA_BASE
    #define UART_TX_PIN         9
    #define UART_RX_PIN         10
    #define UART_GPIO_RCC_BIT   RCC_APB2ENR_IOPAEN

#elif UART_USE_USART == 2
    /* USART2: APB1 @ 36 MHz */
    #define UART_USART_BASE     USART2_BASE
    #define UART_RCC_EN_BIT     RCC_APB1ENR_USART2EN
    #define UART_RCC_ENR        RCC_APB1ENR
    #define UART_PCLK_HZ        PCLK1_FREQ_HZ  /* 36 MHz */

    /* GPIO: PA2=TX, PA3=RX */
    #define UART_GPIO_PORT      GPIOA_BASE
    #define UART_TX_PIN         2
    #define UART_RX_PIN         3
    #define UART_GPIO_RCC_BIT   RCC_APB2ENR_IOPAEN

#elif UART_USE_USART == 3
    /* USART3: APB1 @ 36 MHz */
    #define UART_USART_BASE     USART3_BASE
    #define UART_RCC_EN_BIT     RCC_APB1ENR_USART3EN
    #define UART_RCC_ENR        RCC_APB1ENR
    #define UART_PCLK_HZ        PCLK1_FREQ_HZ  /* 36 MHz */

    /* GPIO: PB10=TX, PB11=RX */
    #define UART_GPIO_PORT      GPIOB_BASE
    #define UART_TX_PIN         10
    #define UART_RX_PIN         11
    #define UART_GPIO_RCC_BIT   RCC_APB2ENR_IOPBEN

#else
    #error "UART_USE_USART deve essere 1, 2, o 3"
#endif

/* =============================================================================
 * VARIABILI GLOBALI PRIVATE
 * ============================================================================= */

/* Flag di inizializzazione */
static volatile bool uart_initialized = false;

#if UART_ENABLE_RX_IRQ
/* Buffer circolare per RX (se interrupt abilitati) */
static volatile char uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile u16 uart_rx_head = 0;
static volatile u16 uart_rx_tail = 0;
#endif

/* =============================================================================
 * FUNZIONI HELPER PRIVATE
 * ============================================================================= */

/**
 * @brief  Configura un pin GPIO come alternate function (per USART TX)
 *
 * @details Per USART TX, il pin deve essere configurato come:
 *          - Mode: Output 50 MHz
 *          - CNF: Alternate function push-pull
 *
 * @param  gpio_base - Base address della porta GPIO (es. GPIOA_BASE)
 * @param  pin - Numero del pin (0-15)
 */
static void uart_gpio_config_af_pp(u32 gpio_base, u8 pin)
{
    volatile u32 *cr_reg;
    u32 cr_value;
    u8 bit_offset;

    /* Scegli CRL (pin 0-7) o CRH (pin 8-15) */
    if (pin < 8) {
        cr_reg = &GPIO_CRL(gpio_base);
        bit_offset = pin * 4;
    } else {
        cr_reg = &GPIO_CRH(gpio_base);
        bit_offset = (pin - 8) * 4;
    }

    /* Leggi il registro corrente */
    cr_value = *cr_reg;

    /* Pulisci i 4 bit del pin */
    cr_value &= ~(0xFUL << bit_offset);

    /* Imposta: MODE=11 (50MHz), CNF=10 (AF push-pull) = 0b1011 = 0xB */
    cr_value |= (0xBUL << bit_offset);

    /* Scrivi il registro */
    *cr_reg = cr_value;
}

/**
 * @brief  Configura un pin GPIO come input floating (per USART RX)
 *
 * @details Per USART RX, il pin deve essere configurato come:
 *          - Mode: Input
 *          - CNF: Floating input
 *
 * @param  gpio_base - Base address della porta GPIO
 * @param  pin - Numero del pin (0-15)
 */
__attribute__((unused))
static void uart_gpio_config_input_floating(u32 gpio_base, u8 pin)
{
    volatile u32 *cr_reg;
    u32 cr_value;
    u8 bit_offset;

    /* Scegli CRL (pin 0-7) o CRH (pin 8-15) */
    if (pin < 8) {
        cr_reg = &GPIO_CRL(gpio_base);
        bit_offset = pin * 4;
    } else {
        cr_reg = &GPIO_CRH(gpio_base);
        bit_offset = (pin - 8) * 4;
    }

    /* Leggi il registro corrente */
    cr_value = *cr_reg;

    /* Pulisci i 4 bit del pin */
    cr_value &= ~(0xFUL << bit_offset);

    /* Imposta: MODE=00 (input), CNF=01 (floating) = 0b0100 = 0x4 */
    cr_value |= (0x4UL << bit_offset);

    /* Scrivi il registro */
    *cr_reg = cr_value;
}

/**
 * @brief  Calcola il valore del Baud Rate Register (BRR)
 *
 * @details Formula: BRR = PCLK / BAUDRATE
 *          Il BRR è diviso in mantissa (12 bit) e frazione (4 bit).
 *
 * @param  pclk_hz - Frequenza del bus APB (Hz)
 * @param  baudrate - Baud rate desiderato
 * @retval u16 - Valore da scrivere in USART_BRR
 */
static u16 uart_calculate_brr(u32 pclk_hz, u32 baudrate)
{
    /* Formula semplificata: BRR = (PCLK + BAUDRATE/2) / BAUDRATE
     * Il +BAUDRATE/2 serve per arrotondare correttamente */
    u32 brr = (pclk_hz + (baudrate / 2)) / baudrate;
    return (u16)brr;
}

/* =============================================================================
 * FUNZIONI PUBBLICHE - IMPLEMENTAZIONE
 * ============================================================================= */

/**
 * @brief  Inizializza l'UART
 */
int uart_init(void)
{
    /* =========================================================================
     * STEP 1: ABILITA CLOCK GPIO
     * =========================================================================
     *
     * Prima di configurare i pin, dobbiamo abilitare il clock della porta GPIO.
     */
    RCC_APB2ENR |= UART_GPIO_RCC_BIT;

    /* =========================================================================
     * STEP 2: CONFIGURA PIN GPIO
     * =========================================================================
     *
     * TX: Alternate function push-pull, 50 MHz
     * RX: Floating input (se abilitato)
     */

    /* Configura TX */
    uart_gpio_config_af_pp(UART_GPIO_PORT, UART_TX_PIN);

    /* Configura RX (se abilitato) */
#if UART_ENABLE_RX
    uart_gpio_config_input_floating(UART_GPIO_PORT, UART_RX_PIN);
#endif

    /* =========================================================================
     * STEP 3: ABILITA CLOCK USART
     * =========================================================================
     */
    UART_RCC_ENR |= UART_RCC_EN_BIT;

    /* =========================================================================
     * STEP 4: CONFIGURA BAUD RATE
     * =========================================================================
     *
     * Il Baud Rate Register (BRR) determina la velocità di trasmissione.
     * BRR = PCLK / BAUDRATE
     *
     * Esempio: PCLK2=72MHz, BAUD=115200
     * BRR = 72000000 / 115200 = 625 (0x271)
     */
    u16 brr = uart_calculate_brr(UART_PCLK_HZ, UART_BAUDRATE);
    USART_BRR(UART_USART_BASE) = brr;

    /* =========================================================================
     * STEP 5: CONFIGURA FORMATO DATI
     * =========================================================================
     *
     * Control Register 1 (CR1):
     * - UE: USART Enable
     * - TE: Transmitter Enable
     * - RE: Receiver Enable (se UART_ENABLE_RX)
     * - M: Word length (0 = 8 bit data)
     * - PCE: Parity control (0 = no parity)
     *
     * Default: 8 bit, no parity
     */
    u32 cr1 = 0;
    cr1 |= USART_CR1_TE;   /* Abilita trasmettitore */

#if UART_ENABLE_RX
    cr1 |= USART_CR1_RE;   /* Abilita ricevitore */
#endif

#if UART_ENABLE_RX_IRQ
    cr1 |= USART_CR1_RXNEIE;  /* Abilita interrupt RX */
#endif

    USART_CR1(UART_USART_BASE) = cr1;

    /* =========================================================================
     * STEP 6: CONFIGURA STOP BITS
     * =========================================================================
     *
     * Control Register 2 (CR2):
     * - STOP[1:0]: Stop bits (00 = 1 stop bit)
     *
     * Default: 1 stop bit
     */
    USART_CR2(UART_USART_BASE) = 0;  /* 1 stop bit */

    /* =========================================================================
     * STEP 7: CONFIGURA FLOW CONTROL
     * =========================================================================
     *
     * Control Register 3 (CR3):
     * - CTSE: CTS enable (hardware flow control)
     * - RTSE: RTS enable (hardware flow control)
     *
     * Default: No flow control
     */
    USART_CR3(UART_USART_BASE) = 0;  /* No flow control */

    /* =========================================================================
     * STEP 8: ABILITA USART
     * =========================================================================
     */
    USART_CR1(UART_USART_BASE) |= USART_CR1_UE;

    /* Marca come inizializzato */
    uart_initialized = true;

    return UART_OK;
}

/**
 * @brief  Deinizializza l'UART
 */
int uart_finit(void)
{
    /* Disabilita USART */
    USART_CR1(UART_USART_BASE) = 0;

    /* Disabilita clock USART */
    UART_RCC_ENR &= ~UART_RCC_EN_BIT;

    /* Marca come non inizializzato */
    uart_initialized = false;

    return UART_OK;
}

/**
 * @brief  Invia un singolo carattere via UART
 */
int uart_putchar(char ch)
{
    /* Verifica se UART è inizializzato */
    if (!uart_initialized) {
        return UART_ERROR_INIT;
    }

    /* =========================================================================
     * TRASMISSIONE BYTE
     * =========================================================================
     *
     * Per trasmettere un byte:
     * 1. Attendiamo che il flag TXE (Transmit Data Register Empty) sia 1
     * 2. Scriviamo il byte in USART_DR
     * 3. L'hardware trasmette automaticamente
     *
     * IMPORTANTE: TXE = 1 significa che il registro DR è vuoto e pronto
     * per un nuovo byte, ma il byte precedente potrebbe essere ancora
     * in trasmissione (verificare TC per completamento totale).
     */

    /* Timeout per evitare deadlock */
    u32 timeout = 100000;  /* ~10ms @ 72 MHz */

    /* Attendi che TXE sia settato (registro DR vuoto) */
    while (!(USART_SR(UART_USART_BASE) & USART_SR_TXE) && timeout > 0) {
        timeout--;
    }

    /* Se timeout scaduto, errore */
    if (timeout == 0) {
        return UART_ERROR_TIMEOUT;
    }

    /* Scrivi il byte nel registro dati */
    USART_DR(UART_USART_BASE) = (u8)ch;

    return UART_OK;
}

/**
 * @brief  Scrive una stringa via UART
 */
int uart_write(const char *str)
{
    /* Validazione parametri */
    if (str == NULL) {
        return UART_ERROR_INVALID;
    }

    /* Verifica se UART è inizializzato */
    if (!uart_initialized) {
        return UART_ERROR_INIT;
    }

    /* Invia carattere per carattere */
    while (*str != '\0') {
        int result = uart_putchar(*str);

        /* Se errore (timeout o altro), ritorna subito */
        if (result != UART_OK) {
            return result;
        }

        str++;
    }

    return UART_OK;
}

/**
 * @brief  Legge un carattere dall'UART (se RX abilitato)
 */
#if UART_ENABLE_RX
int uart_getchar(char *ch)
{
    /* Validazione parametri */
    if (ch == NULL) {
        return UART_ERROR_INVALID;
    }

    /* Verifica se UART è inizializzato */
    if (!uart_initialized) {
        return UART_ERROR_INIT;
    }

    /* Timeout per evitare deadlock */
    u32 timeout = 1000000;  /* ~100ms @ 72 MHz */

    /* Attendi che RXNE sia settato (dato disponibile) */
    while (!(USART_SR(UART_USART_BASE) & USART_SR_RXNE) && timeout > 0) {
        timeout--;
    }

    /* Se timeout scaduto, errore */
    if (timeout == 0) {
        return UART_ERROR_TIMEOUT;
    }

    /* Leggi il byte dal registro dati */
    *ch = (char)(USART_DR(UART_USART_BASE) & 0xFF);

    return UART_OK;
}
#endif

/**
 * @brief  Dumpa tutto il buffer circolare di log via UART
 * @note   Disponibile SOLO se LOG_TO_MEMORY=1
 */
int log_via_uart(void)
{
#if ENABLE_LOG && LOG_TO_MEMORY
    /* Buffer temporaneo per leggere i messaggi dal log */
    char buffer[LOG_MAX_MESSAGE_SIZE];

    /* Conta quanti messaggi ci sono nel buffer */
    u16 total_messages = log_get_count();
    u16 sent_count = 0;

    /* Verifica se ci sono messaggi da inviare */
    if (total_messages == 0) {
        uart_write("[LOG] Buffer vuoto - nessun messaggio da inviare\r\n");
        return 0;
    }

    /* Header del dump */
    uart_write("=======================================================\r\n");
    uart_write("[LOG DUMP] Inizio dump del buffer circolare\r\n");
    uart_write("\r\n");
    uart_write("=======================================================\r\n\r\n");

    /* Leggi e invia tutti i messaggi uno per uno */
    while (log_get_count() > 0) {
        /* Legge il prossimo messaggio dal buffer (operazione POP) */
        int result = log_read(buffer);

        if (result == LOG_OK) {
            /* Invia il messaggio via UART */
            uart_write(buffer);
            uart_write("\r\n");  /* CRLF per compatibilità terminali */
            sent_count++;
        } else if (result == LOG_ERROR_EMPTY) {
            /* Buffer vuoto - finito */
            break;
        } else {
            /* Errore durante la lettura - segnala e continua */
            uart_write("[LOG DUMP ERROR] Errore lettura messaggio\r\n");
            break;
        }
    }

    u32_to_dec_pad(sent_count, buffer, LOG_MAX_MESSAGE_SIZE);
    /* Footer del dump */
    uart_write("\r\n=======================================================\r\n");
    uart_write("[LOG DUMP] Fine dump - Messaggi inviati: ");
    uart_write(buffer);
    uart_write("\r\n");
    uart_write("=======================================================\r\n");

    return sent_count;
#else
    uart_write("[LOG] log_via_uart() disponibile solo con LOG_TO_MEMORY=1\r\n");
    return 0;
#endif
}



/* =============================================================================
 * NOTE IMPLEMENTATIVE
 * ============================================================================= */

/*
 * CALCOLO BAUD RATE:
 * ==================
 *
 * Il baud rate è determinato dal valore del registro BRR:
 *
 *   Baud = PCLK / (16 * USARTDIV)
 *
 * Dove USARTDIV è il valore contenuto in BRR (mantissa + frazione).
 *
 * Esempio per USART1 (PCLK2 = 72 MHz, Baud = 115200):
 *   USARTDIV = 72000000 / (16 * 115200) = 39.0625
 *   BRR = 39 + (0.0625 * 16) = 39 + 1 = 40 (in realtà 625 in formato BRR)
 *
 * Formula semplificata usata nel codice:
 *   BRR = PCLK / BAUDRATE
 *
 * Questa è una semplificazione che funziona perché il fattore 16 è
 * già gestito internamente dall'hardware USART.
 *
 *
 * TIMING A 115200 BAUD:
 * =====================
 *
 * Ogni carattere (8 bit + start + stop = 10 bit) richiede:
 *   T = 10 / 115200 ≈ 86.8 μs
 *
 * Quindi uart_write() di una stringa da 100 caratteri blocca per:
 *   T = 100 * 86.8 μs ≈ 8.7 ms
 *
 * Se serve output non-bloccante, considera:
 * - Usare DMA per UART TX
 * - Implementare un buffer TX con interrupt
 * - Ridurre la quantità di dati trasmessi
 *
 *
 * CONFRONTO UART vs ITM:
 * ======================
 *
 * UART:
 * - Pro: Funziona sempre, anche senza debugger
 * - Pro: Pin standard, facile da collegare
 * - Pro: Lunga distanza (metri con RS232)
 * - Contro: Occupa 2 pin GPIO (TX/RX)
 * - Contro: Più lento di ITM (max ~921600 baud pratici)
 * - Contro: Bloccante se non usi DMA
 *
 * ITM:
 * - Pro: Zero pin GPIO (usa SWO del debugger)
 * - Pro: Molto veloce (fino a 2 MHz)
 * - Pro: Non-bloccante (FIFO hardware)
 * - Contro: Richiede debugger connesso
 * - Contro: Distanza limitata (cavo debugger)
 * - Contro: Non disponibile in produzione
 *
 * SOLUZIONE IDEALE: Usa entrambi!
 * - Debug: ITM per velocità
 * - Produzione: UART per diagnostica remota
 */
