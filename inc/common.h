/**
 ******************************************************************************
 * @file           : common.h
 * @brief          : Header comune per tutto il progetto
 * @author         : STM32F103 Project
 ******************************************************************************
 * @attention
 *
 * Questo file deve essere incluso da TUTTI i file .c e .h del progetto.
 * Contiene:
 * - Include standard della libreria C
 * - Typedef brevi per tipi interi (i8, u8, i16, u16, ecc.)
 * - Macro comuni per controllo interrupt
 * - Include dei registri del processore (stm32f103_regs.h)
 *
 ******************************************************************************
 */

#ifndef COMMON_H_
#define COMMON_H_

/* =============================================================================
 * INCLUDE STANDARD LIBRARY C
 * ============================================================================= */
#include <stdint.h>      /* int8_t, uint8_t, int16_t, uint16_t, ecc. */
#include <stdbool.h>     /* bool, true, false */
#include <stddef.h>      /* size_t, NULL, offsetof */
#include <string.h>      /* memcpy, memset, strlen, ecc. */

/* =============================================================================
 * TYPEDEF BREVI PER TIPI INTERI
 * ============================================================================= */

/* Tipi signed */
typedef int8_t   i8;     /* Signed 8-bit  (-128 to 127) */
typedef int16_t  i16;    /* Signed 16-bit (-32768 to 32767) */
typedef int32_t  i32;    /* Signed 32-bit (-2147483648 to 2147483647) */
typedef int64_t  i64;    /* Signed 64-bit */

/* Tipi unsigned */
typedef uint8_t  u8;     /* Unsigned 8-bit  (0 to 255) */
typedef uint16_t u16;    /* Unsigned 16-bit (0 to 65535) */
typedef uint32_t u32;    /* Unsigned 32-bit (0 to 4294967295) */
typedef uint64_t u64;    /* Unsigned 64-bit */

/* =============================================================================
 * MACRO PER CONTROLLO INTERRUPT (ARM CORTEX-M)
 * ============================================================================= */

/**
 * @brief  Disabilita gli interrupt globali (PRIMASK = 1)
 * @note   Usa istruzione assembly CPSID I
 */
#if defined(__GNUC__)
#define __disable_irq() __asm volatile ("cpsid i" : : : "memory")
#else
static inline void __disable_irq(void) {
    __asm volatile ("cpsid i" : : : "memory");
}
#endif

/**
 * @brief  Abilita gli interrupt globali (PRIMASK = 0)
 * @note   Usa istruzione assembly CPSIE I
 */
#if defined(__GNUC__)
#define __enable_irq()  __asm volatile ("cpsie i" : : : "memory")
#else
static inline void __enable_irq(void) {
    __asm volatile ("cpsie i" : : : "memory");
}
#endif

/**
 * @brief  No Operation - ciclo vuoto
 */
#define __NOP()  __asm volatile ("nop")

/**
 * @brief  Wait For Interrupt - CPU in sleep fino a interrupt
 */
#define __WFI()  __asm volatile ("wfi")

/**
 * @brief  Wait For Event - CPU in sleep fino a evento
 */
#define __WFE()  __asm volatile ("wfe")

/* =============================================================================
 * MACRO COMUNI DI UTILITÀ
 * ============================================================================= */

/**
 * @brief  Restituisce il numero di elementi in un array statico
 */
#define ARRAY_SIZE(arr)  (sizeof(arr) / sizeof((arr)[0]))

/**
 * @brief  Marca una variabile come non usata (evita warning del compilatore)
 */
#define UNUSED(x)  ((void)(x))

/**
 * @brief  Min/Max macros
 */
#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif

/**
 * @brief  Bit manipulation macros
 */
#define BIT(n)              (1UL << (n))
#define SET_BIT(reg, bit)   ((reg) |= (bit))
#define CLR_BIT(reg, bit)   ((reg) &= ~(bit))
#define READ_BIT(reg, bit)  ((reg) & (bit))
#define TOGGLE_BIT(reg, bit) ((reg) ^= (bit))

/* =============================================================================
 * INCLUDE REGISTRI PROCESSORE STM32F103
 * ============================================================================= */
#include "stm32f103_regs.h"

#define ENABLE_LOG 1


#endif /* COMMON_H_ */
