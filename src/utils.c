/**
 ******************************************************************************
 * @file           : utils.c
 * @brief          : Implementazione utility functions
 * @author         : STM32F103 Project
 ******************************************************************************
 */

#include "utils.h"
#include <stdarg.h>  /* Per va_list, va_start, va_arg, va_end */

/* =============================================================================
 * FUNZIONI DI CONVERSIONE DECIMALE
 * ============================================================================= */

/**
 * @brief  Converte u32 in stringa decimale
 */
u8 u32_to_dec(u32 value, char *buffer)
{
    /* Caso speciale: zero */
    if (value == 0)
    {
        buffer[0] = '0';
        buffer[1] = '\0';
        return 1;
    }

    /* Converti cifra per cifra (dal meno significativo) */
    u8 i = 0;
    u32 temp = value;

    while (temp > 0)
    {
        buffer[i++] = '0' + (temp % 10);
        temp /= 10;
    }

    /* Null terminator */
    buffer[i] = '\0';

    /* Inverti la stringa (le cifre sono al contrario) */
    u8 len = i;
    for (u8 j = 0; j < len / 2; j++)
    {
        char tmp = buffer[j];
        buffer[j] = buffer[len - 1 - j];
        buffer[len - 1 - j] = tmp;
    }

    return len;
}

/**
 * @brief  Converte i32 in stringa decimale (con segno)
 */
u8 i32_to_dec(i32 value, char *buffer)
{
    /* Gestisci segno negativo */
    if (value < 0)
    {
        buffer[0] = '-';
        /* Converti valore assoluto (attenzione a INT32_MIN) */
        u32 abs_value = (value == -2147483648) ? 2147483648U : (u32)(-value);
        u8 len = u32_to_dec(abs_value, buffer + 1);
        return len + 1;
    }
    else
    {
        return u32_to_dec((u32)value, buffer);
    }
}

/**
 * @brief  Converte u16 in stringa decimale
 */
u8 u16_to_dec(u16 value, char *buffer)
{
    return u32_to_dec((u32)value, buffer);
}

/**
 * @brief  Converte i16 in stringa decimale
 */
u8 i16_to_dec(i16 value, char *buffer)
{
    return i32_to_dec((i32)value, buffer);
}

/**
 * @brief  Converte u8 in stringa decimale
 */
u8 u8_to_dec(u8 value, char *buffer)
{
    return u32_to_dec((u32)value, buffer);
}

/**
 * @brief  Converte i8 in stringa decimale
 */
u8 i8_to_dec(i8 value, char *buffer)
{
    return i32_to_dec((i32)value, buffer);
}

/* =============================================================================
 * FUNZIONI DI CONVERSIONE ESADECIMALE
 * ============================================================================= */

/**
 * @brief  Converte u32 in stringa esadecimale (8 cifre con zero-padding)
 */
u8 u32_to_hex(u32 value, char *buffer, bool uppercase)
{
    const char *hex_digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";

    /* Converti ogni nibble (4 bit) in una cifra hex */
    for (i8 i = 7; i >= 0; i--)
    {
        buffer[7 - i] = hex_digits[(value >> (i * 4)) & 0xF];
    }

    buffer[8] = '\0';
    return 8;
}

/**
 * @brief  Converte u16 in stringa esadecimale (4 cifre)
 */
u8 u16_to_hex(u16 value, char *buffer, bool uppercase)
{
    const char *hex_digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";

    for (i8 i = 3; i >= 0; i--)
    {
        buffer[3 - i] = hex_digits[(value >> (i * 4)) & 0xF];
    }

    buffer[4] = '\0';
    return 4;
}

/**
 * @brief  Converte u8 in stringa esadecimale (2 cifre)
 */
u8 u8_to_hex(u8 value, char *buffer, bool uppercase)
{
    const char *hex_digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";

    buffer[0] = hex_digits[(value >> 4) & 0xF];
    buffer[1] = hex_digits[value & 0xF];
    buffer[2] = '\0';

    return 2;
}

/* =============================================================================
 * FUNZIONI DI CONVERSIONE BINARIA
 * ============================================================================= */

/**
 * @brief  Converte u32 in stringa binaria (32 bit)
 */
u8 u32_to_bin(u32 value, char *buffer)
{
    for (i8 i = 31; i >= 0; i--)
    {
        buffer[31 - i] = ((value >> i) & 1) ? '1' : '0';
    }

    buffer[32] = '\0';
    return 32;
}

/**
 * @brief  Converte u16 in stringa binaria (16 bit)
 */
u8 u16_to_bin(u16 value, char *buffer)
{
    for (i8 i = 15; i >= 0; i--)
    {
        buffer[15 - i] = ((value >> i) & 1) ? '1' : '0';
    }

    buffer[16] = '\0';
    return 16;
}

/**
 * @brief  Converte u8 in stringa binaria (8 bit)
 */
u8 u8_to_bin(u8 value, char *buffer)
{
    for (i8 i = 7; i >= 0; i--)
    {
        buffer[7 - i] = ((value >> i) & 1) ? '1' : '0';
    }

    buffer[8] = '\0';
    return 8;
}

/* =============================================================================
 * FUNZIONI DI FORMATTAZIONE CON PADDING
 * ============================================================================= */

/**
 * @brief  Converte u32 in decimale con zero-padding
 */
u8 u32_to_dec_pad(u32 value, char *buffer, u8 width)
{
    /* Prima converti normalmente */
    char temp[12];
    u8 len = u32_to_dec(value, temp);

    /* Se lunghezza è già >= width, copia direttamente */
    if (len >= width)
    {
        for (u8 i = 0; i <= len; i++)
        {
            buffer[i] = temp[i];
        }
        return len;
    }

    /* Altrimenti aggiungi zero-padding a sinistra */
    u8 padding = width - len;
    for (u8 i = 0; i < padding; i++)
    {
        buffer[i] = '0';
    }

    /* Copia le cifre */
    for (u8 i = 0; i <= len; i++)
    {
        buffer[padding + i] = temp[i];
    }

    return width;
}

/**
 * @brief  Converte u32 in hex senza zero-padding (solo cifre significative)
 */
u8 u32_to_hex_compact(u32 value, char *buffer, bool uppercase)
{
    const char *hex_digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";

    /* Caso speciale: zero */
    if (value == 0)
    {
        buffer[0] = '0';
        buffer[1] = '\0';
        return 1;
    }

    /* Trova il primo nibble non-zero */
    i8 start_nibble = 7;
    while (start_nibble >= 0 && ((value >> (start_nibble * 4)) & 0xF) == 0)
    {
        start_nibble--;
    }

    /* Converti solo i nibble significativi */
    u8 idx = 0;
    for (i8 i = start_nibble; i >= 0; i--)
    {
        buffer[idx++] = hex_digits[(value >> (i * 4)) & 0xF];
    }

    buffer[idx] = '\0';
    return idx;
}

/* =============================================================================
 * MINI SNPRINTF - Versione minimale per embedded
 * ============================================================================= */

/**
 * @brief  Helper: scrive un carattere nel buffer con controllo bounds
 */
static inline bool write_char(char **buf, u32 *remaining, char c)
{
    if (*remaining > 1)  /* Lascia spazio per '\0' */
    {
        **buf = c;
        (*buf)++;
        (*remaining)--;
        return true;
    }
    return false;
}

/**
 * @brief  Helper: scrive una stringa nel buffer con controllo bounds
 */
static u32 write_string(char **buf, u32 *remaining, const char *str)
{
    u32 written = 0;

    while (*str && *remaining > 1)
    {
        **buf = *str++;
        (*buf)++;
        (*remaining)--;
        written++;
    }

    return written;
}

/**
 * @brief  Implementazione mini_vsnprintf (versione con va_list)
 */
int mini_vsnprintf(char *buffer, u32 size, const char *format, va_list args)
{
    /* Validazione parametri */
    if (buffer == NULL || format == NULL || size == 0)
    {
        return -1;
    }

    char *buf_ptr = buffer;
    u32 remaining = size;
    u32 total_written = 0;

    /* Scansiona la stringa di formato */
    while (*format && remaining > 1)
    {
        if (*format == '%')
        {
            format++;  /* Salta il '%' */

            /* Gestisci format specifier */
            switch (*format)
            {
                case 'd':  /* Signed decimal */
                {
                    i32 value = va_arg(args, i32);
                    char temp[12];
                    i32_to_dec(value, temp);
                    u32 written = write_string(&buf_ptr, &remaining, temp);
                    total_written += written;
                    break;
                }

                case 'u':  /* Unsigned decimal */
                {
                    u32 value = va_arg(args, u32);
                    char temp[11];
                    u32_to_dec(value, temp);
                    u32 written = write_string(&buf_ptr, &remaining, temp);
                    total_written += written;
                    break;
                }

                case 'x':  /* Hexadecimal lowercase */
                {
                    u32 value = va_arg(args, u32);
                    char temp[9];
                    u32_to_hex_compact(value, temp, false);
                    u32 written = write_string(&buf_ptr, &remaining, temp);
                    total_written += written;
                    break;
                }

                case 'X':  /* Hexadecimal uppercase */
                {
                    u32 value = va_arg(args, u32);
                    char temp[9];
                    u32_to_hex_compact(value, temp, true);
                    u32 written = write_string(&buf_ptr, &remaining, temp);
                    total_written += written;
                    break;
                }

                case 'b':  /* Binary */
                {
                    u32 value = va_arg(args, u32);
                    char temp[33];
                    u32_to_bin(value, temp);
                    u32 written = write_string(&buf_ptr, &remaining, temp);
                    total_written += written;
                    break;
                }

                case 'c':  /* Character */
                {
                    /* char viene promosso a int nel varargs */
                    char value = (char)va_arg(args, int);
                    if (write_char(&buf_ptr, &remaining, value))
                    {
                        total_written++;
                    }
                    break;
                }

                case 's':  /* String */
                {
                    const char *str = va_arg(args, const char*);
                    if (str == NULL)
                    {
                        str = "(null)";
                    }
                    u32 written = write_string(&buf_ptr, &remaining, str);
                    total_written += written;
                    break;
                }

                case '%':  /* Literal '%' */
                {
                    if (write_char(&buf_ptr, &remaining, '%'))
                    {
                        total_written++;
                    }
                    break;
                }

                default:  /* Format specifier non riconosciuto - stampa letteralmente */
                {
                    if (write_char(&buf_ptr, &remaining, '%'))
                    {
                        total_written++;
                    }
                    if (write_char(&buf_ptr, &remaining, *format))
                    {
                        total_written++;
                    }
                    break;
                }
            }

            format++;
        }
        else
        {
            /* Carattere normale - copia letteralmente */
            if (write_char(&buf_ptr, &remaining, *format))
            {
                total_written++;
            }
            format++;
        }
    }

    /* Null-terminator finale */
    *buf_ptr = '\0';

    return (int)total_written;
}

/**
 * @brief  Implementazione mini_snprintf (wrapper di mini_vsnprintf)
 */
int mini_snprintf(char *buffer, u32 size, const char *format, ...)
{
    va_list args;
    va_start(args, format);

    int result = mini_vsnprintf(buffer, size, format, args);

    va_end(args);

    return result;
}
