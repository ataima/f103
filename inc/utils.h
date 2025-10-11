/**
 ******************************************************************************
 * @file           : utils.h
 * @brief          : Utility functions per conversioni e formattazioni
 * @author         : STM32F103 Project
 ******************************************************************************
 * @attention
 *
 * Questo modulo fornisce funzioni di utility per conversioni numeriche
 * e formattazioni, utili in ambienti embedded senza printf completo.
 *
 * Funzionalità principali:
 * - Conversione interi → stringa (decimale, esadecimale, binario)
 * - Formattazione con padding e zero-fill
 * - Versioni ottimizzate per microcontrollori
 *
 ******************************************************************************
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "common.h"
#include <stdarg.h>

/* =============================================================================
 * FUNZIONI DI CONVERSIONE DECIMALE (equivalente %d, %u)
 * ============================================================================= */

/**
 * @brief  Converte un numero unsigned 32-bit in stringa decimale
 *
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione (minimo 11 bytes: 10 cifre + '\0')
 *
 * @retval u8 - Numero di caratteri scritti (escluso '\0')
 *
 * @note   Buffer deve essere almeno 11 bytes per u32 (max "4294967295\0")
 *
 * @example
 *   char buf[12];
 *   u8 len = u32_to_dec(1234, buf);  // buf = "1234", len = 4
 */
u8 u32_to_dec(u32 value, char *buffer);

/**
 * @brief  Converte un numero signed 32-bit in stringa decimale
 *
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione (minimo 12 bytes: segno + 10 cifre + '\0')
 *
 * @retval u8 - Numero di caratteri scritti (escluso '\0')
 *
 * @note   Buffer deve essere almeno 12 bytes per i32 (max "-2147483648\0")
 *
 * @example
 *   char buf[12];
 *   u8 len = i32_to_dec(-1234, buf);  // buf = "-1234", len = 5
 */
u8 i32_to_dec(i32 value, char *buffer);

/**
 * @brief  Converte u16 in stringa decimale
 * @param  value - Valore da convertire (0-65535)
 * @param  buffer - Buffer di destinazione (minimo 6 bytes)
 * @retval u8 - Numero di caratteri scritti
 */
u8 u16_to_dec(u16 value, char *buffer);

/**
 * @brief  Converte i16 in stringa decimale
 * @param  value - Valore da convertire (-32768 a 32767)
 * @param  buffer - Buffer di destinazione (minimo 7 bytes)
 * @retval u8 - Numero di caratteri scritti
 */
u8 i16_to_dec(i16 value, char *buffer);

/**
 * @brief  Converte u8 in stringa decimale
 * @param  value - Valore da convertire (0-255)
 * @param  buffer - Buffer di destinazione (minimo 4 bytes)
 * @retval u8 - Numero di caratteri scritti
 */
u8 u8_to_dec(u8 value, char *buffer);

/**
 * @brief  Converte i8 in stringa decimale
 * @param  value - Valore da convertire (-128 a 127)
 * @param  buffer - Buffer di destinazione (minimo 5 bytes)
 * @retval u8 - Numero di caratteri scritti
 */
u8 i8_to_dec(i8 value, char *buffer);

/* =============================================================================
 * FUNZIONI DI CONVERSIONE ESADECIMALE (equivalente %x, %X)
 * ============================================================================= */

/**
 * @brief  Converte un numero unsigned 32-bit in stringa esadecimale
 *
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione (minimo 9 bytes: 8 hex + '\0')
 * @param  uppercase - true per maiuscole (A-F), false per minuscole (a-f)
 *
 * @retval u8 - Numero di caratteri scritti (sempre 8)
 *
 * @note   Scrive sempre 8 cifre hex con zero-padding (es: "0000ABCD")
 *
 * @example
 *   char buf[9];
 *   u32_to_hex(0xABCD, buf, true);   // buf = "0000ABCD"
 *   u32_to_hex(0xABCD, buf, false);  // buf = "0000abcd"
 */
u8 u32_to_hex(u32 value, char *buffer, bool uppercase);

/**
 * @brief  Converte u16 in stringa esadecimale (4 cifre)
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione (minimo 5 bytes)
 * @param  uppercase - true per A-F, false per a-f
 * @retval u8 - Numero di caratteri scritti (sempre 4)
 */
u8 u16_to_hex(u16 value, char *buffer, bool uppercase);

/**
 * @brief  Converte u8 in stringa esadecimale (2 cifre)
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione (minimo 3 bytes)
 * @param  uppercase - true per A-F, false per a-f
 * @retval u8 - Numero di caratteri scritti (sempre 2)
 */
u8 u8_to_hex(u8 value, char *buffer, bool uppercase);

/* =============================================================================
 * FUNZIONI DI CONVERSIONE BINARIA (equivalente %b - non standard C)
 * ============================================================================= */

/**
 * @brief  Converte un numero unsigned 32-bit in stringa binaria
 *
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione (minimo 33 bytes: 32 bit + '\0')
 *
 * @retval u8 - Numero di caratteri scritti (sempre 32)
 *
 * @note   Scrive sempre 32 bit (es: "00000000000000000000000000001010")
 *
 * @example
 *   char buf[33];
 *   u32_to_bin(10, buf);  // buf = "00000000000000000000000000001010"
 */
u8 u32_to_bin(u32 value, char *buffer);

/**
 * @brief  Converte u16 in stringa binaria (16 bit)
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione (minimo 17 bytes)
 * @retval u8 - Numero di caratteri scritti (sempre 16)
 */
u8 u16_to_bin(u16 value, char *buffer);

/**
 * @brief  Converte u8 in stringa binaria (8 bit)
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione (minimo 9 bytes)
 * @retval u8 - Numero di caratteri scritti (sempre 8)
 */
u8 u8_to_bin(u8 value, char *buffer);

/* =============================================================================
 * FUNZIONI DI FORMATTAZIONE CON PADDING
 * ============================================================================= */

/**
 * @brief  Converte u32 in decimale con zero-padding a sinistra
 *
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione
 * @param  width - Larghezza totale (riempie con '0' a sinistra)
 *
 * @retval u8 - Numero di caratteri scritti
 *
 * @example
 *   char buf[6];
 *   u32_to_dec_pad(42, buf, 5);  // buf = "00042"
 */
u8 u32_to_dec_pad(u32 value, char *buffer, u8 width);

/**
 * @brief  Converte u32 in hex senza zero-padding (solo cifre significative)
 *
 * @param  value - Valore da convertire
 * @param  buffer - Buffer di destinazione (minimo 9 bytes)
 * @param  uppercase - true per A-F, false per a-f
 *
 * @retval u8 - Numero di caratteri scritti (minimo 1)
 *
 * @example
 *   char buf[9];
 *   u32_to_hex_compact(0xAB, buf, true);  // buf = "AB" (non "000000AB")
 */
u8 u32_to_hex_compact(u32 value, char *buffer, bool uppercase);

/* =============================================================================
 * MINI SNPRINTF - Formattazione stile printf minimale
 * ============================================================================= */

/**
 * @brief  Versione minimale di vsnprintf per embedded systems (con va_list)
 *
 * @details Supporta i seguenti format specifier:
 *          - %d  : signed decimal (i32)
 *          - %u  : unsigned decimal (u32)
 *          - %x  : hexadecimal lowercase (u32, senza zero-padding)
 *          - %X  : hexadecimal uppercase (u32, senza zero-padding)
 *          - %b  : binary (u32, senza zero-padding)
 *          - %c  : character (char)
 *          - %s  : string (char*)
 *          - %%  : literal '%'
 *
 * @param  buffer - Buffer di destinazione
 * @param  size - Dimensione massima del buffer (incluso '\0')
 * @param  format - Stringa di formato
 * @param  args - Lista di argomenti variabili (va_list)
 *
 * @retval int - Numero di caratteri scritti (escluso '\0'), o -1 se errore
 *
 * @note   La funzione garantisce sempre null-termination se size > 0
 * @note   Se size è insufficiente, la stringa viene troncata
 * @note   NON supporta: width, precision, flags, modificatori di lunghezza
 * @note   Questa è la versione "interna" usata da mini_snprintf
 */
int mini_vsnprintf(char *buffer, u32 size, const char *format, va_list args);

/**
 * @brief  Versione minimale di snprintf per embedded systems
 *
 * @details Supporta i seguenti format specifier:
 *          - %d  : signed decimal (i32)
 *          - %u  : unsigned decimal (u32)
 *          - %x  : hexadecimal lowercase (u32, senza zero-padding)
 *          - %X  : hexadecimal uppercase (u32, senza zero-padding)
 *          - %b  : binary (u32, senza zero-padding)
 *          - %c  : character (char)
 *          - %s  : string (char*)
 *          - %%  : literal '%'
 *
 * @param  buffer - Buffer di destinazione
 * @param  size - Dimensione massima del buffer (incluso '\0')
 * @param  format - Stringa di formato
 * @param  ... - Argomenti variabili
 *
 * @retval int - Numero di caratteri scritti (escluso '\0'), o -1 se errore
 *
 * @note   La funzione garantisce sempre null-termination se size > 0
 * @note   Se size è insufficiente, la stringa viene troncata
 * @note   NON supporta: width, precision, flags, modificatori di lunghezza
 *
 * @example
 *   char buf[64];
 *   mini_snprintf(buf, sizeof(buf), "Temp: %d C, Addr: 0x%X", temp, addr);
 *   // Output: "Temp: 25 C, Addr: 0x1234ABCD"
 *
 * @example
 *   char buf[32];
 *   mini_snprintf(buf, sizeof(buf), "Pin %u = %b", pin, state);
 *   // Output: "Pin 5 = 00000000000000000000000000000001"
 */
int mini_snprintf(char *buffer, u32 size, const char *format, ...);

#endif /* UTILS_H_ */
