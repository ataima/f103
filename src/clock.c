/**
 ******************************************************************************
 * @file           : clock.c
 * @brief          : Configurazione del sistema di clock a 72 MHz
 * @author         : Generato per STM32F103C8T6
 ******************************************************************************
 * @attention
 *
 * Questo file implementa la configurazione del clock di sistema per portare
 * l'STM32F103C8T6 alla sua massima frequenza operativa di 72 MHz.
 *
 * Il processo di configurazione segue questi passi critici:
 * 1. Abilita HSE (oscillatore esterno da 8 MHz)
 * 2. Configura i prescaler dei bus PRIMA di aumentare la frequenza
 * 3. Configura i wait state della Flash per operare a 72 MHz
 * 4. Configura il PLL per moltiplicare HSE x9 (8 MHz * 9 = 72 MHz)
 * 5. Abilita il PLL e attende il lock
 * 6. Commuta SYSCLK dal HSI al PLL
 *
 ******************************************************************************
 */
#include "common.h"
#include "clock.h"

/* =============================================================================
 * IMPLEMENTAZIONE DELLE FUNZIONI
 * ============================================================================= */

/**
 * @brief  Configura il sistema di clock a 72 MHz
 * @details Questa è la funzione principale che esegue tutti i passi necessari
 *          per portare il microcontrollore alla massima velocità.
 */
int SystemClock_Config(void)
{
    u32 timeout;

    /* =========================================================================
     * STEP 1: ABILITAZIONE HSE (HIGH SPEED EXTERNAL OSCILLATOR)
     * =========================================================================
     *
     * L'HSE è il quarzo esterno da 8 MHz presente sulla Blue Pill.
     * È più stabile e preciso dell'HSI interno, quindi è preferito per
     * applicazioni che richiedono timing accurati (USB, UART ad alta velocità).
     */

    /* Abilita l'oscillatore esterno HSE */
    RCC_CR |= RCC_CR_HSEON;

    /* Attendi che HSE sia stabile e pronto (flag HSERDY) */
    timeout = HSE_STARTUP_TIMEOUT;
    while (!(RCC_CR & RCC_CR_HSERDY) && (timeout > 0))
    {
        timeout--;
    }

    /* Verifica che HSE si sia avviato correttamente */
    if (timeout == 0)
    {
        /* HSE non si è avviato - possibile problema hardware:
         * - Quarzo difettoso
         * - Condensatori di carico errati
         * - Tracce PCB interrotte
         */
        return CLOCK_ERROR_HSE;
    }

    /* =========================================================================
     * STEP 2: CONFIGURAZIONE FLASH WAIT STATES
     * =========================================================================
     *
     * CRITICO: Quando aumentiamo la frequenza a 72 MHz, la Flash memory non
     * riesce a tenere il passo senza wait states. Dobbiamo configurare 2 WS
     * PRIMA di aumentare il clock, altrimenti il micro potrebbe crashare!
     *
     * Tabella wait states per STM32F103:
     * 0 WS: 0 MHz < SYSCLK <= 24 MHz
     * 1 WS: 24 MHz < SYSCLK <= 48 MHz
     * 2 WS: 48 MHz < SYSCLK <= 72 MHz
     */

    /* Configura 2 wait states per 72 MHz */
    FLASH_ACR &= ~FLASH_ACR_LATENCY_MASK;      /* Pulisci bits latency */
    FLASH_ACR |= FLASH_ACR_LATENCY_2;          /* Imposta 2 wait states */

    /* Abilita prefetch buffer per migliorare performance */
    FLASH_ACR |= FLASH_ACR_PRFTBE;

    /* =========================================================================
     * STEP 3: CONFIGURAZIONE PRESCALER DEI BUS
     * =========================================================================
     *
     * IMPORTANTE: Configurare i prescaler PRIMA di attivare il PLL!
     * Questo previene che i bus periferici vengano sovraclockati durante
     * la transizione.
     *
     * Configurazione target:
     * - HCLK (AHB bus):  72 MHz (prescaler /1) - CPU, memoria, DMA
     * - PCLK1 (APB1 bus): 36 MHz (prescaler /2) - I2C, UART, Timer2-7
     * - PCLK2 (APB2 bus): 72 MHz (prescaler /1) - ADC, USART1, Timer1
     * - ADCCLK:          12 MHz (prescaler /6) - ADC converter
     *
     * NOTA CRITICA: APB1 ha un limite massimo di 36 MHz!
     * Se superiamo questo limite, i periferici su APB1 non funzioneranno
     * correttamente o potrebbero danneggiarsi!
     */

    u32 cfgr_temp = RCC_CFGR;

    /* AHB Prescaler: HCLK = SYSCLK / 1 = 72 MHz */
    cfgr_temp &= ~RCC_CFGR_HPRE_MASK;          /* Pulisci bits HPRE */
    cfgr_temp |= RCC_CFGR_HPRE_DIV1;           /* Nessuna divisione */

    /* APB1 Prescaler: PCLK1 = HCLK / 2 = 36 MHz (MASSIMO CONSENTITO!) */
    cfgr_temp &= ~RCC_CFGR_PPRE1_MASK;         /* Pulisci bits PPRE1 */
    cfgr_temp |= RCC_CFGR_PPRE1_DIV2;          /* Divide per 2 */

    /* APB2 Prescaler: PCLK2 = HCLK / 1 = 72 MHz */
    cfgr_temp &= ~RCC_CFGR_PPRE2_MASK;         /* Pulisci bits PPRE2 */
    cfgr_temp |= RCC_CFGR_PPRE2_DIV1;          /* Nessuna divisione */

    /* ADC Prescaler: ADCCLK = PCLK2 / 6 = 12 MHz */
    /* L'ADC dell'STM32F103 ha un massimo di 14 MHz, quindi 12 MHz è sicuro */
    cfgr_temp &= ~RCC_CFGR_ADCPRE_MASK;        /* Pulisci bits ADCPRE */
    cfgr_temp |= RCC_CFGR_ADCPRE_DIV6;         /* Divide per 6 */

    /* Scrivi la configurazione nel registro */
    RCC_CFGR = cfgr_temp;

    /* =========================================================================
     * STEP 4: CONFIGURAZIONE DEL PLL
     * =========================================================================
     *
     * Il PLL (Phase-Locked Loop) è un moltiplicatore di frequenza.
     * Configurazione scelta:
     * - Input: HSE = 8 MHz (quarzo esterno)
     * - Moltiplicatore: x9
     * - Output: 8 MHz * 9 = 72 MHz
     *
     * Alternative (non usate):
     * - HSI/2 * 18 = 4 MHz * 18 = 72 MHz (meno preciso, HSI ha ±1% tolleranza)
     * - HSE/2 * 18 = 4 MHz * 18 = 72 MHz (inutilmente complesso)
     */

    /* Assicurati che il PLL sia disabilitato prima di configurarlo */
    RCC_CR &= ~RCC_CR_PLLON;

    /* Attendi che il PLL sia completamente spento */
    while (RCC_CR & RCC_CR_PLLRDY);

    cfgr_temp = RCC_CFGR;

    /* PLL source = HSE (8 MHz) */
    cfgr_temp |= RCC_CFGR_PLLSRC_HSE;

    /* HSE non diviso per il PLL (usa HSE direttamente) */
    cfgr_temp &= ~RCC_CFGR_PLLXTPRE_HSE_DIV2;

    /* PLL Multiplier = x9 per ottenere 72 MHz */
    cfgr_temp &= ~RCC_CFGR_PLLMUL_MASK;        /* Pulisci bits PLLMUL */
    cfgr_temp |= RCC_CFGR_PLLMUL9;             /* 8 MHz * 9 = 72 MHz */

    /* Scrivi la configurazione del PLL */
    RCC_CFGR = cfgr_temp;

    /* =========================================================================
     * STEP 5: ABILITAZIONE E LOCK DEL PLL
     * =========================================================================
     *
     * Dopo aver configurato il PLL, dobbiamo abilitarlo e attendere che
     * raggiunga il "lock" (stabilità). Il lock avviene quando il PLL ha
     * sincronizzato la sua uscita con l'input moltiplicato.
     */

    /* Abilita il PLL */
    RCC_CR |= RCC_CR_PLLON;

    /* Attendi il PLL lock (flag PLLRDY diventa 1) */
    timeout = PLL_STARTUP_TIMEOUT;
    while (!(RCC_CR & RCC_CR_PLLRDY) && (timeout > 0))
    {
        timeout--;
    }

    /* Verifica che il PLL abbia fatto lock */
    if (timeout == 0)
    {
        /* PLL non ha fatto lock - possibile problema:
         * - HSE non stabile
         * - Configurazione PLL errata
         * - Problema hardware
         */
        return CLOCK_ERROR_PLL;
    }

    /* =========================================================================
     * STEP 6: SWITCH DEL SYSCLK AL PLL
     * =========================================================================
     *
     * MOMENTO CRITICO: Ora commutiamo la sorgente del clock di sistema
     * dall'HSI (8 MHz default) al PLL (72 MHz). Questo è il "big switch"!
     *
     * La sequenza è:
     * 1. Scriviamo nel campo SW (System clock Switch) per selezionare PLL
     * 2. Leggiamo il campo SWS (System clock Switch Status) finché non
     *    conferma che il PLL è la sorgente attiva
     *
     * Durante questa transizione il micro continua a funzionare normalmente.
     */

    /* Seleziona PLL come sorgente di SYSCLK */
    cfgr_temp = RCC_CFGR;
    cfgr_temp &= ~RCC_CFGR_SW_MASK;            /* Pulisci bits SW */
    cfgr_temp |= RCC_CFGR_SW_PLL;              /* Seleziona PLL */
    RCC_CFGR = cfgr_temp;

    /* Attendi che SYSCLK sia effettivamente commutato al PLL */
    /* Leggiamo SWS (System Clock Switch Status) - è read-only e viene
     * impostato automaticamente dall'hardware quando lo switch è completo */
    timeout = 0x5000;
    while (((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) && (timeout > 0))
    {
        timeout--;
    }

    if (timeout == 0)
    {
        /* Switch non completato - molto raro, possibile glitch hardware */
        return CLOCK_ERROR_PLL;
    }

    /* =========================================================================
     * STEP 7: CONFIGURAZIONE OPZIONALE - CSS (CLOCK SECURITY SYSTEM)
     * =========================================================================
     *
     * Il CSS monitora l'HSE e, se rileva un failure (quarzo si ferma),
     * commuta automaticamente il sistema su HSI per evitare il crash.
     * È utile per applicazioni critiche, ma può essere omesso.
     *
     * NOTA: Abilitiamolo per sicurezza!
     */

    /* Abilita il Clock Security System */
    RCC_CR |= RCC_CR_CSSON;

    /* =========================================================================
     * SUCCESSO!
     * =========================================================================
     *
     * A questo punto il sistema sta funzionando a 72 MHz!
     *
     * Riepilogo configurazione finale:
     * - SYSCLK: 72 MHz (PLL da HSE)
     * - HCLK:   72 MHz (bus AHB)
     * - PCLK1:  36 MHz (bus APB1)
     * - PCLK2:  72 MHz (bus APB2)
     * - ADCCLK: 12 MHz
     * - Flash:  2 wait states + prefetch enabled
     * - CSS:    Abilitato per sicurezza
     */

    return CLOCK_OK;
}

/**
 * @brief  Restituisce la frequenza corrente di SYSCLK
 * @details Legge il registro RCC_CFGR per determinare quale sorgente di clock
 *          è attualmente in uso e restituisce la relativa frequenza.
 */
u32 SystemClock_GetFreq(void)
{
    /* Leggi quale clock è la sorgente di SYSCLK (campo SWS) */
    u32 sysclk_source = RCC_CFGR & RCC_CFGR_SWS_MASK;

    switch (sysclk_source)
    {
        case RCC_CFGR_SWS_HSI:
            /* HSI = 8 MHz interno */
            return 8000000UL;

        case RCC_CFGR_SWS_HSE:
            /* HSE = 8 MHz esterno (quarzo sulla Blue Pill) */
            return 8000000UL;

        case RCC_CFGR_SWS_PLL:
            /* PLL = 72 MHz (dopo la nostra configurazione) */
            return SYSCLK_FREQ_HZ;

        default:
            /* Caso impossibile, ma per sicurezza */
            return 8000000UL;
    }
}

/**
 * @brief  Restituisce la frequenza corrente del bus AHB (HCLK)
 */
u32 SystemClock_GetHCLK(void)
{
    /* Dopo la nostra configurazione, HCLK = SYSCLK / 1 */
    return HCLK_FREQ_HZ;
}

/**
 * @brief  Restituisce la frequenza corrente del bus APB1 (PCLK1)
 */
u32 SystemClock_GetPCLK1(void)
{
    /* Dopo la nostra configurazione, PCLK1 = HCLK / 2 = 36 MHz */
    return PCLK1_FREQ_HZ;
}

/**
 * @brief  Restituisce la frequenza corrente del bus APB2 (PCLK2)
 */
u32 SystemClock_GetPCLK2(void)
{
    /* Dopo la nostra configurazione, PCLK2 = HCLK / 1 = 72 MHz */
    return PCLK2_FREQ_HZ;
}

/* =============================================================================
 * NOTE FINALI E CONSIDERAZIONI
 * =============================================================================
 *
 * 1. CONSUMO ENERGETICO:
 *    A 72 MHz, l'STM32F103C8T6 consuma circa 36 mA in run mode.
 *    Se serve risparmio energetico, si può ridurre la frequenza o usare
 *    le modalità sleep/stop.
 *
 * 2. TIMING DEI PERIFERICI:
 *    I timer su APB1 hanno il clock a 36 MHz, ma il loro clock interno
 *    è 72 MHz grazie a un moltiplicatore x2 automatico quando APB1 prescaler != 1.
 *    Lo stesso vale per i timer su APB2.
 *
 * 3. USB:
 *    Per usare l'USB, serve un clock a 48 MHz esatti. Con PLL a 72 MHz,
 *    l'USB prescaler deve essere /1.5 (72/1.5 = 48 MHz).
 *
 * 4. OVERCLOCKING:
 *    Tecnicamente si può andare oltre 72 MHz (alcuni arrivano a 128 MHz),
 *    ma è fuori specifica e può causare:
 *    - Instabilità
 *    - Errori di calcolo
 *    - Danneggiamento del chip
 *    - Invalidazione della garanzia
 *    NON RACCOMANDATO per applicazioni di produzione!
 *
 * 5. DEBUGGING:
 *    Se il codice va in HardFault dopo aver chiamato questa funzione,
 *    verifica:
 *    - Che l'HSE sia presente e funzionante (quarzo da 8 MHz)
 *    - Che i condensatori di carico siano corretti (tipicamente 20-30 pF)
 *    - Che i wait states della Flash siano configurati correttamente
 *
 * ============================================================================= */
