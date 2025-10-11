/**
 ******************************************************************************
 * @file           : clock.h
 * @brief          : Header per la configurazione del sistema di clock
 * @author         : Generato per STM32F103C8T6
 ******************************************************************************
 * @attention
 *
 * Questo file contiene le definizioni e le dichiarazioni per configurare
 * il clock di sistema dell'STM32F103C8T6 alla massima frequenza di 72 MHz.
 *
 ******************************************************************************
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include "common.h"

/* =============================================================================
 * COSTANTI DI CONFIGURAZIONE CLOCK
 * =============================================================================
 *
 * STM32F103C8T6 - Configurazione clock massima:
 *
 *   HSE (High Speed External) = 8 MHz   (quarzo esterno sulla Blue Pill)
 *   PLL (Phase-Locked Loop)   = x9      (moltiplicatore PLL)
 *   SYSCLK                    = 72 MHz  (HSE * PLL = 8MHz * 9 = 72MHz)
 *   AHB                       = 72 MHz  (prescaler = 1)
 *   APB1                      = 36 MHz  (prescaler = 2, max 36MHz!)
 *   APB2                      = 72 MHz  (prescaler = 1)
 *   ADC                       = 12 MHz  (prescaler = 6, max 14MHz)
 *
 * NOTA: APB1 deve essere <= 36 MHz per rispettare le specifiche del chip!
 */

/* Frequenze di clock target (in Hz) */
#define SYSCLK_FREQ_HZ      72000000UL   /* Frequenza SYSCLK = 72 MHz */
#define HCLK_FREQ_HZ        72000000UL   /* Frequenza AHB bus = 72 MHz */
#define PCLK1_FREQ_HZ       36000000UL   /* Frequenza APB1 bus = 36 MHz */
#define PCLK2_FREQ_HZ       72000000UL   /* Frequenza APB2 bus = 72 MHz */
#define ADC_FREQ_HZ         12000000UL   /* Frequenza ADC = 12 MHz */

/* Valori di timeout per il clock */
#define HSE_STARTUP_TIMEOUT 0x5000       /* Timeout per avvio HSE */
#define PLL_STARTUP_TIMEOUT 0x5000       /* Timeout per lock del PLL */

/* Codici di ritorno */
#define CLOCK_OK            0            /* Inizializzazione riuscita */
#define CLOCK_ERROR_HSE     -1           /* HSE non si avvia */
#define CLOCK_ERROR_PLL     -2           /* PLL non fa lock */

/* =============================================================================
 * FUNZIONI PUBBLICHE
 * ============================================================================= */

/**
 * @brief  Inizializza il sistema di clock a 72 MHz
 *
 * @details Questa funzione configura il sistema di clock nel seguente modo:
 *          1. Abilita l'oscillatore esterno HSE (8 MHz)
 *          2. Configura il PLL per moltiplicare HSE x9 (72 MHz)
 *          3. Configura i prescaler dei bus:
 *             - AHB: /1 (72 MHz)
 *             - APB1: /2 (36 MHz - massimo consentito)
 *             - APB2: /1 (72 MHz)
 *             - ADC: /6 (12 MHz)
 *          4. Configura i wait state della Flash (2 WS per 72 MHz)
 *          5. Commuta SYSCLK su PLL
 *
 * @param  None
 * @retval int - CLOCK_OK se successo, codice errore altrimenti
 */
int SystemClock_Config(void);

/**
 * @brief  Ottiene la frequenza corrente del SYSCLK
 * @param  None
 * @retval u32 - Frequenza in Hz
 */
u32 SystemClock_GetFreq(void);

/**
 * @brief  Ottiene la frequenza corrente del bus AHB
 * @param  None
 * @retval u32 - Frequenza in Hz
 */
u32 SystemClock_GetHCLK(void);

/**
 * @brief  Ottiene la frequenza corrente del bus APB1
 * @param  None
 * @retval u32 - Frequenza in Hz
 */
u32 SystemClock_GetPCLK1(void);

/**
 * @brief  Ottiene la frequenza corrente del bus APB2
 * @param  None
 * @retval u32 - Frequenza in Hz
 */
u32 SystemClock_GetPCLK2(void);

#endif /* CLOCK_H_ */
