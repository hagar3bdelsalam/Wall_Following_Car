#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize UART0 @ 9600 8N1
 * Pins: D0 (RX), D1 (TX)
 * Ring buffer size: 64 bytes
 */
void uart_init(void);

/**
 * @brief Write a single byte to UART
 * @param b byte to write
 */
void uart_write(uint8_t b);

/**
 * @brief Check if data available in RX buffer
 * @return true if at least one byte is available
 */
bool uart_available(void);

/**
 * @brief Read a byte from UART (blocking)
 * @return next byte from RX buffer
 */
uint8_t uart_read_blocking(void);

/**
 * @brief Peek at next byte without consuming (non-blocking)
 * @return next byte, or -1 if buffer empty
 */
int16_t uart_peek(void);

/**
 * @brief Peek with timeout
 * @param timeout_us timeout in microseconds
 * @return next byte, or -1 if timeout
 */
int16_t uart_peek_wait_us(uint32_t timeout_us);

/**
 * @brief Write null-terminated string
 * @param s pointer to string
 */
void uart_print_str(const char* s);

/**
 * @brief Write string with CRLF
 * @param s pointer to string
 */
void uart_println(const char* s);

/**
 * @brief Write long integer
 * @param n value
 */
void uart_print_long(long n);

/**
 * @brief Write unsigned long integer
 * @param n value
 */
void uart_print_ulong(unsigned long n);

/**
 * @brief Write floating point
 * @param f value
 * @param decimals decimal places
 */
void uart_print_float(double f, uint8_t decimals);

/**
 * @brief Parse long integer from UART
 * Waits for valid digit or minus sign, timeout 2s between bytes
 * @return parsed value
 */
long uart_parse_long(void);

/**
 * @brief Parse float from UART
 * Waits for valid digit, minus, or dot, timeout 2s between bytes
 * @return parsed value
 */
float uart_parse_float(void);

#endif // UART_H
