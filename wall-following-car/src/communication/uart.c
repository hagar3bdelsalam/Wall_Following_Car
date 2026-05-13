#include "uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "../utils/timing.h"

#define UART_BAUD      9600UL
#define UBRR_VALUE     ((16000000UL / 16UL / UART_BAUD) - 1UL)
#define UART_RX_CAP    64

static volatile uint8_t uart_rx_buf[UART_RX_CAP];
static volatile uint8_t uart_rx_head = 0;
static volatile uint8_t uart_rx_tail = 0;

ISR(USART_RX_vect) {
    uint8_t b  = UDR0;
    uint8_t nh = (uint8_t)((uart_rx_head + 1U) % UART_RX_CAP);
    if (nh != uart_rx_tail) {
        uart_rx_buf[uart_rx_head] = b;
        uart_rx_head = nh;
    }
}

static bool uart_rx_empty(void) {
    uint8_t sreg = SREG;
    cli();
    bool empty = (uart_rx_head == uart_rx_tail);
    SREG = sreg;
    return empty;
}

void uart_init(void) {
    uart_rx_head = 0;
    uart_rx_tail = 0;
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);
    UCSR0A = 0;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_write(uint8_t b) {
    while (!(UCSR0A & (1 << UDRE0))) { }
    UDR0 = b;
}

bool uart_available(void) {
    return !uart_rx_empty();
}

uint8_t uart_read_blocking(void) {
    while (uart_rx_empty()) { }
    uint8_t sreg = SREG;
    cli();
    uint8_t b = uart_rx_buf[uart_rx_tail];
    uart_rx_tail = (uint8_t)((uart_rx_tail + 1U) % UART_RX_CAP);
    SREG = sreg;
    return b;
}

int16_t uart_peek(void) {
    uint8_t sreg = SREG;
    cli();
    if (uart_rx_head == uart_rx_tail) {
        SREG = sreg;
        return -1;
    }
    uint8_t b = uart_rx_buf[uart_rx_tail];
    SREG = sreg;
    return (int16_t)b;
}

int16_t uart_peek_wait_us(uint32_t timeout_us) {
    uint32_t dl = myMicros() + timeout_us;
    for (;;) {
        int16_t c = uart_peek();
        if (c != -1) return c;
        if ((int32_t)(dl - myMicros()) <= 0) return -1;
    }
}

void uart_print_str(const char* s) {
    while (*s) uart_write((uint8_t)*s++);
}

void uart_println(const char* s) {
    uart_print_str(s);
    uart_print_str("\r\n");
}

void uart_print_long(long n) {
    char buf[12];
    ltoa(n, buf, 10);
    uart_print_str(buf);
}

void uart_print_ulong(unsigned long n) {
    char buf[12];
    ultoa(n, buf, 10);
    uart_print_str(buf);
}

void uart_print_float(double f, uint8_t decimals) {
    char buf[16];
    dtostrf(f, 0, decimals, buf);
    uart_print_str(buf);
}

#define UART_PARSE_INTER_BYTE_US 2000000UL

long uart_parse_long(void) {
    uint32_t deadline = myMicros() + UART_PARSE_INTER_BYTE_US;
    int16_t  c;

    while ((int32_t)(deadline - myMicros()) > 0) {
        c = uart_peek();
        if (c == -1) continue;
        if (c == '-' || (c >= '0' && c <= '9')) break;
        uart_read_blocking();
    }

    bool negative = false;
    c = uart_peek();
    if (c == '-') { uart_read_blocking(); negative = true; }

    long val = 0;
    deadline = myMicros() + UART_PARSE_INTER_BYTE_US;
    while ((int32_t)(deadline - myMicros()) > 0) {
        c = uart_peek();
        if (c == -1) continue;
        if (c >= '0' && c <= '9') {
            uart_read_blocking();
            val = val * 10 + (c - '0');
            deadline = myMicros() + UART_PARSE_INTER_BYTE_US;
        } else {
            break;
        }
    }
    return negative ? -val : val;
}

float uart_parse_float(void) {
    uint32_t deadline = myMicros() + UART_PARSE_INTER_BYTE_US;
    int16_t  c;

    while ((int32_t)(deadline - myMicros()) > 0) {
        c = uart_peek();
        if (c == -1) continue;
        if (c == '-' || c == '.' || (c >= '0' && c <= '9')) break;
        uart_read_blocking();
    }

    bool  negative = false;
    c = uart_peek();
    if (c == '-') { uart_read_blocking(); negative = true; }

    float val      = 0.0f;
    float frac_div = 1.0f;
    bool  in_frac  = false;

    deadline = myMicros() + UART_PARSE_INTER_BYTE_US;
    while ((int32_t)(deadline - myMicros()) > 0) {
        c = uart_peek();
        if (c == -1) continue;
        if (c >= '0' && c <= '9') {
            uart_read_blocking();
            if (in_frac) {
                frac_div *= 10.0f;
                val += (float)(c - '0') / frac_div;
            } else {
                val = val * 10.0f + (float)(c - '0');
            }
            deadline = myMicros() + UART_PARSE_INTER_BYTE_US;
        } else if (c == '.' && !in_frac) {
            uart_read_blocking();
            in_frac = true;
            deadline = myMicros() + UART_PARSE_INTER_BYTE_US;
        } else {
            break;
        }
    }
    return negative ? -val : val;
}
