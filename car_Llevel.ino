// ============================================================
// Wall-Following Robot - LOW LEVEL (register-level) version
// Same logic as car_Hlevel.ino, no Arduino HL APIs.
// MCU: ATmega328P @ 16 MHz (Arduino UNO, HC-05 on D0/D1).
// ============================================================

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
// No <util/delay.h> — all timing is timer-managed (Timer0 CTC + Timer1 TCNT1L).
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ============================================================
// --- BIT HELPERS ---
// ============================================================
#define SBI(reg, bit) ((reg) |=  (1 << (bit)))
#define CBI(reg, bit) ((reg) &= ~(1 << (bit)))
#define RBI(reg, bit) (((reg) >> (bit)) & 1)

// ============================================================
// --- PIN -> PORT/BIT MAP (ATmega328P) ---
// ============================================================
// Motor 1 (was D11 EN, D12 IN1, D13 IN2)
#define M1_EN_BIT     PB3   // OC2A
#define M1_IN1_BIT    PB4
#define M1_IN2_BIT    PB5

// Motor 2 (was D10 EN, D9 IN1, D8 IN2)
#define M2_EN_BIT     PB2   // OC1B
#define M2_IN1_BIT    PB1
#define M2_IN2_BIT    PB0

// Ultrasonic FRONT (TRIG D5/PD5, ECHO D4/PD4)
#define TRIG_FRONT_BIT  PD5
#define ECHO_FRONT_BIT  PD4

// Ultrasonic RIGHT (TRIG D7/PD7, ECHO D6/PD6)
#define TRIG_RIGHT_BIT  PD7
#define ECHO_RIGHT_BIT  PD6

// Ultrasonic LEFT  (TRIG A4/PC4, ECHO A5/PC5)
#define TRIG_LEFT_BIT   PC0
#define ECHO_LEFT_BIT   PC1

// Encoders (RIGHT D2/PD2 INT0, LEFT D3/PD3 INT1)
#define ENC_RIGHT_BIT   PD2
#define ENC_LEFT_BIT    PD3

// Sensor descriptor (used by getDistance)
typedef struct {
    volatile uint8_t* trigPort;
    volatile uint8_t* echoPin;
    uint8_t           trigBit;
    uint8_t           echoBit;
} UltrasonicSensor;

static const UltrasonicSensor SENSOR_FRONT = { &PORTD, &PIND, TRIG_FRONT_BIT, ECHO_FRONT_BIT };
static const UltrasonicSensor SENSOR_RIGHT = { &PORTD, &PIND, TRIG_RIGHT_BIT, ECHO_RIGHT_BIT };
static const UltrasonicSensor SENSOR_LEFT  = { &PORTC, &PINC, TRIG_LEFT_BIT,  ECHO_LEFT_BIT  };

// ============================================================
// --- DEBUG FLAG + REPORT BUFFERS ---
// ============================================================
bool DEBUG = true;          // <<< set false to silence everything except final report

#define MAX_TURNS 64
char turnSequence[MAX_TURNS];
uint8_t turnCount = 0;

// ============================================================
// --- LIVE TUNABLE VARIABLES (same defaults as HL) ---
// ============================================================
int   BASE_SPEED       = 100;
float Kp               = 0.3;//0.5;
float Ki               = 0.0;
float Kd               = 3.5;//5.0;
int   TURN_TICKS_RIGHT = 25;
int   TURN_TICKS_LEFT  = 40;
float LEFT_TRIM        = 0.87;
float RIGHT_TRIM       = 1.0;
int   TURN_SPEED_RIGHT = 100;  // pivot turn: left wheel PWM (M1) when turning right
int   TURN_SPEED_LEFT  = 113;  // pivot turn: right wheel PWM (M2) when turning left (tune via ML)
int   STOP_DISTANCE    = 30;   // front obstacle: trigger turn/stop below this (cm); tune via O
int   BACK_SPEED       = 100;  // reverse PWM (both wheels); tune via BS
int   BACK_TIME_MS     = 1000; // reverse duration in ms for dead-end escape; tune via BT

// ============================================================
// --- HARDWARE SETTINGS ---
// ============================================================
const int   MAX_SPEED            = 200;
const int   MIN_SPEED            = 0;
const float TARGET_CENTER        = 15.0;
const int   DEAD_DISTANCE        = 20;   // stricter than STOP; front must be closer to count as dead band
const float SPEED_OF_SOUND       = 0.0343;
const float RIGHT_WALL_THRESHOLD = 30.0;
const float LEFT_WALL_THRESHOLD  = 30.0;
const float THRESHOLD_DIST       = 50.0;
const unsigned long DEBOUNCE_TIME = 500UL;   // microseconds

// ============================================================
// --- STATE ---
// ============================================================
float previous_error = 0;
float integral       = 0;
float lastDistR      = -1.0;
float lastDistL      = -1.0;
bool  isRunning      = false;
bool  warmupDone     = false;

static bool deadReverseTrialUsed = false;

volatile unsigned long leftTicks       = 0;
volatile unsigned long rightTicks      = 0;
volatile unsigned long lastLeftMicros  = 0;
volatile unsigned long lastRightMicros = 0;

// ============================================================
// --- TIME (Timer1 free-running 8-bit Fast PWM, prescaler 64) ---
// Ticks at 4 us, overflow every 1024 us. Reused for OC1B PWM.
// ============================================================
volatile uint32_t timer1_ovf_count = 0;

ISR(TIMER1_OVF_vect) {
    timer1_ovf_count++;
}

static uint32_t myMicros(void) {
    uint32_t ovf;
    uint8_t  cnt;
    uint8_t  sreg = SREG;
    cli();
    ovf = timer1_ovf_count;
    cnt = TCNT1L;
    if ((TIFR1 & (1 << TOV1)) && (cnt < 255)) {
        ovf++;
    }
    SREG = sreg;
    return ((ovf << 8) + cnt) * 4UL;
}

// ============================================================
// --- TIMER0: SYSTEM MILLISECOND CLOCK (CTC mode) ---
// ============================================================
// CTC mode (WGM01): counter counts 0 -> OCR0A -> auto-reset -> 0
// Prescaler 64: tick = 64 / 16 MHz = 4 us
// OCR0A = 249: (249 + 1) * 4 us = 1000 us = 1 ms per compare match
//
// Pin note: OC0A (PD6) and OC0B (PD5) are NOT connected to the
// timer output (COM0A1:0 = 00, COM0B1:0 = 00), so PD5/PD6 remain
// normal GPIO (Trig Front / Echo Right). No pin conflict.
// ============================================================
volatile uint32_t sys_millis = 0;

ISR(TIMER0_COMPA_vect) {
    sys_millis++;
}

static uint32_t myMillis(void) {
    uint32_t ms;
    uint8_t sreg = SREG;
    cli();              // disable interrupts for atomic 4-byte read
    ms = sys_millis;
    SREG = sreg;        // restore interrupt state
    return ms;
}

static void timer0_init(void) {
    TCCR0A = (1 << WGM01);                  // CTC mode (clear on compare match)
    TCCR0B = (1 << CS01) | (1 << CS00);      // prescaler 64
    OCR0A  = 249;                             // compare match every 1 ms
    TIMSK0 = (1 << OCIE0A);                   // enable compare match A interrupt
}

// Millisecond delay using Timer0's sys_millis (CTC interrupt).
// Busy-waits by polling the timer-driven counter.
static void myDelayMs(uint32_t ms) {
    uint32_t start = myMillis();
    while (myMillis() - start < ms) { }
}

// ============================================================
// --- MICROSECOND DELAY (Timer1 tick-based) ---
// ============================================================
// Timer1 prescaler = 64, so each TCNT1L tick = 4 us.
// Reads the timer counter register directly to busy-wait.
// Resolution: 4 us (rounds up). Max: 1020 us.
// ============================================================
static void myDelayUs(uint16_t us) {
    uint8_t ticks = (uint8_t)((us + 3) / 4);   // round up to next 4us
    if (ticks == 0) ticks = 1;                  // minimum 1 tick = 4 us
    uint8_t start = TCNT1L;                     // snapshot current counter
    while ((uint8_t)(TCNT1L - start) < ticks) { }
}

// pulseIn replacement (timeout in microseconds).
static uint32_t myPulseIn(volatile uint8_t* pinReg, uint8_t bitMask, uint32_t timeout_us) {
    uint32_t start = myMicros();

    // 1) wait for any prior pulse to end (echo LOW).
    while (*pinReg & bitMask) {
        if (myMicros() - start > timeout_us) return 0;
    }
    // 2) wait for pulse to begin (echo HIGH).
    while (!(*pinReg & bitMask)) {
        if (myMicros() - start > timeout_us) return 0;
    }
    uint32_t pulseStart = myMicros();
    // 3) wait for pulse to end (echo LOW).
    while (*pinReg & bitMask) {
        if (myMicros() - pulseStart > timeout_us) return 0;
    }
    return myMicros() - pulseStart;
}

// ============================================================
// --- UART (USART0 @ 9600 8N1; pins D0/D1 -> HC-05) ---
// RX ISR ring: HW FIFO is 2 bytes; blocking ultrasonic reads can overrun RX
// and drop multi-byte commands (V100->1, TR50 mis-parsed) while D7 still works.
// ============================================================
#define UART_BAUD   9600UL
#define UBRR_VALUE  ((F_CPU / 16UL / UART_BAUD) - 1UL)

#define UART_RX_CAP 64
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

static void uart_init(void) {
    uart_rx_head = 0;
    uart_rx_tail = 0;
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);
    UCSR0A = 0;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 8 data, 1 stop, no parity
}

static void uart_write(uint8_t b) {
    while (!(UCSR0A & (1 << UDRE0))) { }
    UDR0 = b;
}

static bool uart_available(void) {
    return !uart_rx_empty();
}

static uint8_t uart_read_blocking(void) {
    while (uart_rx_empty()) { /* spin */ }
    uint8_t sreg = SREG;
    cli();
    uint8_t b = uart_rx_buf[uart_rx_tail];
    uart_rx_tail = (uint8_t)((uart_rx_tail + 1U) % UART_RX_CAP);
    SREG = sreg;
    return b;
}

// Non-blocking peek: next byte in ring, or -1 (does not consume).
static int16_t uart_peek(void) {
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

// Wait for RX byte or timeout (HL uses delay+peek; HC-05 often gaps > 5 ms).
static int16_t uart_peek_wait_us(uint32_t timeout_us) {
    uint32_t dl = myMicros() + timeout_us;
    for (;;) {
        int16_t c = uart_peek();
        if (c != -1) return c;
        if ((int32_t)(dl - myMicros()) <= 0) return -1;
    }
}

static void uart_print_str(const char* s) {
    while (*s) uart_write((uint8_t)*s++);
}

static void uart_println(const char* s) {
    uart_print_str(s);
    uart_write('\r');
    uart_write('\n');
}

// Flash-string (PROGMEM) print helpers — reads string from program
// memory one byte at a time via pgm_read_byte(), saving ~1800 bytes of RAM.
static void uart_print_P(const char* pstr) {
    char c;
    while ((c = pgm_read_byte(pstr++)) != '\0') {
        uart_write((uint8_t)c);
    }
}

static void uart_println_P(const char* pstr) {
    uart_print_P(pstr);
    uart_write('\r');
    uart_write('\n');
}

static void uart_print_long(long n) {
    char buf[12];
    ltoa(n, buf, 10);
    uart_print_str(buf);
}

static void uart_print_ulong(unsigned long n) {
    char buf[12];
    ultoa(n, buf, 10);
    uart_print_str(buf);
}

static void uart_print_float(double f, uint8_t decimals) {
    char buf[16];
    dtostrf(f, 0, decimals, buf);
    uart_print_str(buf);
}

// Drop-in replacements for Serial.parseInt / Serial.parseFloat.
// Lead-in skip + inter-digit gaps both use UART_PARSE_INTER_BYTE_US (2 s; HC-05 + BT).
#define UART_PARSE_INTER_BYTE_US 2000000UL
static long uart_parse_long(void) {
    uint32_t deadline = myMicros() + UART_PARSE_INTER_BYTE_US;
    int16_t  c;

    while ((int32_t)(deadline - myMicros()) > 0) {
        c = uart_peek();
        if (c == -1) continue;
        if (c == '-' || (c >= '0' && c <= '9')) break;
        uart_read_blocking();   // discard
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

static float uart_parse_float(void) {
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
            if (in_frac) { frac_div *= 10.0f; val += (c - '0') / frac_div; }
            else         { val = val * 10.0f + (c - '0'); }
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

// ============================================================
// --- TIMERS / PWM ---
// ============================================================
// Timer1: 8-bit Fast PWM (mode 5), prescaler 64. OC1B drives M2_EN (PB2).
// TIMER1_OVF_vect feeds myMicros (one overflow = 1024 us).
static void timer1_init(void) {
    TCCR1A = (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12)  | (1 << CS11) | (1 << CS10);
    OCR1B  = 0;
    TIMSK1 = (1 << TOIE1);
}

// Timer2: 8-bit Fast PWM (mode 3), prescaler 64. OC2A drives M1_EN (PB3).
static void timer2_init(void) {
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);
    OCR2A  = 0;
}

static inline void pwm_set_M1(uint8_t duty) { OCR2A = duty; }
static inline void pwm_set_M2(uint8_t duty) { OCR1B = duty; }

// ============================================================
// --- GPIO ---
// ============================================================
static void gpio_init(void) {
    // PORTB: motors all outputs, default low.
    DDRB  |= (1 << M1_EN_BIT) | (1 << M1_IN1_BIT) | (1 << M1_IN2_BIT)
          |  (1 << M2_EN_BIT) | (1 << M2_IN1_BIT) | (1 << M2_IN2_BIT);
    PORTB &= ~((1 << M1_IN1_BIT) | (1 << M1_IN2_BIT)
            |  (1 << M2_IN1_BIT) | (1 << M2_IN2_BIT));

    // PORTD: trigs out, echoes in, encoders in w/ pull-ups.
    DDRD  |=  (1 << TRIG_FRONT_BIT) | (1 << TRIG_RIGHT_BIT);
    DDRD  &= ~((1 << ECHO_FRONT_BIT) | (1 << ECHO_RIGHT_BIT)
            |  (1 << ENC_LEFT_BIT)   | (1 << ENC_RIGHT_BIT));
    PORTD |=  (1 << ENC_LEFT_BIT) | (1 << ENC_RIGHT_BIT);   // pull-ups
    PORTD &= ~((1 << TRIG_FRONT_BIT) | (1 << TRIG_RIGHT_BIT));

    // PORTC: trig left out, echo left in.
    DDRC  |=  (1 << TRIG_LEFT_BIT);
    DDRC  &= ~(1 << ECHO_LEFT_BIT);
    PORTC &= ~(1 << TRIG_LEFT_BIT);
}

// ============================================================
// --- ENCODERS (INT0 right, INT1 left, on CHANGE) ---
// ============================================================
static void encoders_init(void) {
    // ISC01:0 = 01 (CHANGE), ISC11:0 = 01 (CHANGE).
    EICRA = (1 << ISC10) | (1 << ISC00);
    EIMSK = (1 << INT0)  | (1 << INT1);
}

ISR(INT0_vect) {  // ENCODER_RIGHT (PD2)
    uint32_t now = myMicros();
    if (now - lastRightMicros > DEBOUNCE_TIME) {
        rightTicks++;
        lastRightMicros = now;
    }
}

ISR(INT1_vect) {  // ENCODER_LEFT (PD3)
    uint32_t now = myMicros();
    if (now - lastLeftMicros > DEBOUNCE_TIME) {
        leftTicks++;
        lastLeftMicros = now;
    }
}

// ============================================================
// --- ULTRASONIC ---
// ============================================================
static float getDistance(const UltrasonicSensor* s) {
    *(s->trigPort) &= ~(1 << s->trigBit);
    myDelayUs(2);                               // 4 us (1 tick, >= 2 us required)
    *(s->trigPort) |=  (1 << s->trigBit);
    myDelayUs(10);                              // 12 us (3 ticks, >= 10 us per HC-SR04 spec)
    *(s->trigPort) &= ~(1 << s->trigBit);

    uint32_t duration = myPulseIn(s->echoPin, (1 << s->echoBit), 30000UL);
    if (duration == 0) return 0.0f;
    return duration * SPEED_OF_SOUND / 2.0f;
}

// ============================================================
// --- MOTORS ---
// ============================================================
static void runMotor(int motorIndex, int speed, bool forward) {
    uint8_t s8 = (speed < 0) ? 0 : (speed > 255 ? 255 : (uint8_t)speed);

    if (motorIndex == 1) {
        if (s8 == 0) {
            pwm_set_M1(0);
            PORTB &= ~((1 << M1_IN1_BIT) | (1 << M1_IN2_BIT));
        } else if (forward) {
            PORTB |=  (1 << M1_IN1_BIT);
            PORTB &= ~(1 << M1_IN2_BIT);
            pwm_set_M1(s8);
        } else {
            PORTB &= ~(1 << M1_IN1_BIT);
            PORTB |=  (1 << M1_IN2_BIT);
            pwm_set_M1(s8);
        }
    } else if (motorIndex == 2) {
        if (s8 == 0) {
            pwm_set_M2(0);
            PORTB &= ~((1 << M2_IN1_BIT) | (1 << M2_IN2_BIT));
        } else if (forward) {
            PORTB |=  (1 << M2_IN1_BIT);
            PORTB &= ~(1 << M2_IN2_BIT);
            pwm_set_M2(s8);
        } else {
            PORTB &= ~(1 << M2_IN1_BIT);
            PORTB |=  (1 << M2_IN2_BIT);
            pwm_set_M2(s8);
        }
    }
}

static void stopMotors(void) { runMotor(1, 0, true); runMotor(2, 0, true); }

// ============================================================
// --- TURN-LOG / DEAD-END REPORT ---
// ============================================================
static void recordTurn(char dir) {
    if (turnCount < MAX_TURNS) turnSequence[turnCount++] = dir;
}

// Always prints (even when DEBUG is false). Format from the project doc:
//   Turns: N
//   Sequence: L, R, R, L, L
static void printDeadEndReport(void) {
    uart_print_P(PSTR("Turns: "));
    uart_print_ulong(turnCount);
    uart_print_P(PSTR("\r\nSequence: "));
    for (uint8_t i = 0; i < turnCount; i++) {
        if (i) uart_print_P(PSTR(", "));
        uart_write((uint8_t)turnSequence[i]);
    }
    uart_print_P(PSTR("\r\n"));
}

// ============================================================
// --- MOVEMENT SEQUENCES & TURNS ---
// ============================================================
static void goForward(int seconds) {
    if (DEBUG) {
        uart_print_P(PSTR(">>> Moving Forward for "));
        uart_print_long(seconds);
        uart_println_P(PSTR("s."));
    }
    runMotor(1, (int)(BASE_SPEED * LEFT_TRIM),  true);
    runMotor(2, (int)(BASE_SPEED * RIGHT_TRIM), true);
    myDelayMs((uint32_t)seconds * 1000UL);
    stopMotors();
    if (DEBUG) uart_println_P(PSTR(">>> Done. Stopped."));
}

static void goBackwardMs(uint32_t ms) {
    /* Uses tunable BACK_SPEED — adjust via BS command over serial. */
    int l = (int)(BACK_SPEED * LEFT_TRIM);
    int r = (int)(BACK_SPEED * RIGHT_TRIM);
    if (l < 1) l = 1;
    if (r < 1) r = 1;
    runMotor(1, l, false);
    runMotor(2, r, false);
    myDelayUs(200);                             // 200 us (50 ticks) -- motor direction settle
    uint32_t end = myMicros() + ms * 1000UL;
    while ((int32_t)(end - myMicros()) > 0) { }
    stopMotors();
}

// Pivot Turn: Left wheel drives, right wheel dead.
static void turnRight90(void) {
    if (DEBUG) uart_println_P(PSTR(">>> Turning RIGHT (Pivot Turn)"));
    stopMotors(); myDelayMs(200);
    cli(); leftTicks = 0; sei();

    runMotor(1, TURN_SPEED_RIGHT, true);
    runMotor(2, 0,          true);

    while (leftTicks < (unsigned long)TURN_TICKS_RIGHT) {
        if (DEBUG) {
            uart_print_P(PSTR("Turning Right - Left Ticks: "));
            uart_print_ulong(leftTicks);
            uart_print_P(PSTR("\r\n"));
        }
        myDelayMs(20);
    }

    stopMotors(); myDelayMs(10);
    integral = 0; previous_error = 0; lastDistR = -1.0; lastDistL = -1.0;
    recordTurn('R');
}

// Pivot Turn: Right wheel drives, left wheel dead.
static void turnLeft90(void) {
    if (DEBUG) uart_println_P(PSTR(">>> Turning LEFT (Pivot Turn)"));
    stopMotors(); myDelayMs(200);
    cli(); rightTicks = 0; sei();

    runMotor(1, 0,          true);
    runMotor(2, TURN_SPEED_LEFT, true);

    while (rightTicks < (unsigned long)TURN_TICKS_LEFT) {
        if (DEBUG) {
            uart_print_P(PSTR("Turning Left - Right Ticks: "));
            uart_print_ulong(rightTicks);
            uart_print_P(PSTR("\r\n"));
        }
        myDelayMs(20);
    }

    stopMotors(); myDelayMs(10);
    integral = 0; previous_error = 0; lastDistR = -1.0; lastDistL = -1.0;
    recordTurn('L');
}

static void runSequenceRight(void) {
    int leftSpeed  = (int)(BASE_SPEED * LEFT_TRIM);
    int rightSpeed = (int)(BASE_SPEED * RIGHT_TRIM);

    // if (DEBUG) uart_println_P(PSTR(">>> RIGHT Sequence: 1. Moving Forward (2s)"));
    // runMotor(1, leftSpeed,  true);
    // runMotor(2, rightSpeed, true);
    // myDelayMs(2000);

     //stopMotors(); myDelayMs(200);
    turnRight90();

    // if (DEBUG) uart_println_P(PSTR(">>> RIGHT Sequence: 3. Moving Forward (1s)"));
    // runMotor(1, leftSpeed,  true);
    // runMotor(2, rightSpeed, true);
    // myDelayMs(1000);

    // if (DEBUG) uart_println_P(PSTR(">>> Sequence Complete. Stopped."));
    //stopMotors();
}

static void runSequenceLeft(void) {
    int leftSpeed  = (int)(BASE_SPEED * LEFT_TRIM);
    int rightSpeed = (int)(BASE_SPEED * RIGHT_TRIM);

    //if (DEBUG) uart_println_P(PSTR(">>> LEFT Sequence: 1. Moving Forward (2s)"));
    // runMotor(1, leftSpeed,  true);
    // runMotor(2, rightSpeed, true);
    // myDelayMs(2000);

   // stopMotors(); myDelayMs(200);
    turnLeft90();

    // if (DEBUG) uart_println_P(PSTR(">>> LEFT Sequence: 3. Moving Forward (1s)"));
    // runMotor(1, leftSpeed,  true);
    // runMotor(2, rightSpeed, true);
    // myDelayMs(1000);

    // if (DEBUG) uart_println_P(PSTR(">>> Sequence Complete. Stopped."));
    // stopMotors();
}

static void warmupSensors(void) {
    if (DEBUG) uart_println_P(PSTR("Warming up sensors..."));
    for (int i = 0; i < 20; i++) {
        float r = getDistance(&SENSOR_RIGHT);
        float l = getDistance(&SENSOR_LEFT);
        if (r > 0) lastDistR = r;
        if (l > 0) lastDistL = l;
        myDelayMs(10);
    }
    if (DEBUG) uart_println_P(PSTR("Sensors ready. Send commands."));
}

// ============================================================
// --- SETUP ---
// ============================================================
void setup(void) {
    cli();
    gpio_init();
    uart_init();
    timer0_init();      // System ms clock (CTC mode, 1ms COMPA interrupt)
    timer1_init();      // PWM for M2 + free-running counter for myMicros
    timer2_init();      // PWM for M1
    encoders_init();
    sei();

    if (DEBUG) {
        uart_println_P(PSTR("--- MASTER CAR SCRIPT READY (LL) ---"));
        uart_println_P(PSTR("'1' = Run Continuous PID | '0' = Stop"));
        uart_println_P(PSTR("'R' = Run Right Sequence | 'L' = Run Left Sequence"));
        uart_println_P(PSTR("'TR45' = Right Ticks to 45 | 'TL38' = Left Ticks to 38"));
        uart_println_P(PSTR("'T40' = Set BOTH Ticks to 40"));
        uart_println_P(PSTR("'ML120' = Left pivot turn PWM (M2) | 'MR100' = Right pivot PWM (M1)"));
        uart_println_P(PSTR("'P2.5' = Set Kp to 2.5 | 'D15' = Set Kd to 15"));
        uart_println_P(PSTR("'dcr0.85' = Set Right Trim | 'dcl0.88' = Set Left Trim"));
        uart_println_P(PSTR("'F3' = Move Forward 3s | 'V100' = Base speed | 'O30' = Front stop distance (cm)"));
        uart_println_P(PSTR("'B500' = Reverse 500ms | 'BS120' = Set back speed | 'BT500' = Set back time(ms)"));
        uart_println_P(PSTR("'?' = Settings"));
    }
    myDelayMs(3000);
}

// ============================================================
// --- MAIN LOOP ---
// ============================================================
void loop(void) {

    // 1. Serial commands
    if (uart_available()) {
        char cmd = (char)uart_read_blocking();

        if (cmd == '1' || cmd == 'S' || cmd == 's') {
            isRunning  = true; warmupDone = false;
            integral   = 0;    previous_error = 0;
            lastDistR  = -1.0; lastDistL      = -1.0;
            cli(); leftTicks = 0; rightTicks = 0; sei();
            turnCount  = 0;
            if (DEBUG) uart_println_P(PSTR(">>> Running Continuous PID..."));
        }
        else if (cmd == '0' || cmd == 'X' || cmd == 'x') {
            isRunning = false; stopMotors();
            if (DEBUG) uart_println_P(PSTR(">>> Stopped."));
        }
        else if (cmd == 'R' || cmd == 'r') { isRunning = false; runSequenceRight(); }
        else if (cmd == 'L' || cmd == 'l') { isRunning = false; runSequenceLeft();  }
        else if (cmd == 'B' || cmd == 'b') {
            myDelayMs(5);
            int16_t nextChar = uart_peek_wait_us(UART_PARSE_INTER_BYTE_US);
            while (nextChar == '\r' || nextChar == '\n') {
                uart_read_blocking();
                nextChar = uart_peek_wait_us(UART_PARSE_INTER_BYTE_US);
            }
            if (nextChar == 'S' || nextChar == 's') {
                // BS<val>: set back speed
                uart_read_blocking();
                int v = (int)uart_parse_long();
                if (v < 0) v = 0;
                if (v > MAX_SPEED) v = MAX_SPEED;
                BACK_SPEED = v;
                if (DEBUG) {
                    uart_print_P(PSTR(">>> Back Speed updated to: "));
                    uart_print_long(BACK_SPEED);
                    uart_println_P(PSTR(""));
                }
            } else if (nextChar == 'T' || nextChar == 't') {
                // BT<ms>: set back time for dead-end reverse
                uart_read_blocking();
                int v = (int)uart_parse_long();
                if (v < 50) v = 50;
                if (v > 5000) v = 5000;
                BACK_TIME_MS = v;
                if (DEBUG) {
                    uart_print_P(PSTR(">>> Back Time updated to: "));
                    uart_print_long(BACK_TIME_MS);
                    uart_println_P(PSTR("ms"));
                }
            } else {
                // B<ms>: reverse for given milliseconds
                isRunning = false;
                int ms = (int)uart_parse_long();
                if (ms > 0) {
                    if (DEBUG) {
                        uart_print_P(PSTR(">>> Reversing for "));
                        uart_print_long(ms);
                        uart_print_P(PSTR("ms at PWM "));
                        uart_print_long(BACK_SPEED);
                        uart_println_P(PSTR(""));
                    }
                    goBackwardMs((uint32_t)ms);
                    if (DEBUG) uart_println_P(PSTR(">>> Done. Stopped."));
                }
            }
        }
        else if (cmd == 'F' || cmd == 'f') {
            isRunning = false;
            int seconds = (int)uart_parse_long();
            if (seconds > 0) goForward(seconds);
        }
        else if (cmd == 'T' || cmd == 't') {
            myDelayMs(5);
            int16_t nextChar = uart_peek_wait_us(UART_PARSE_INTER_BYTE_US);
            while (nextChar == '\r' || nextChar == '\n') {
                uart_read_blocking();
                nextChar = uart_peek_wait_us(UART_PARSE_INTER_BYTE_US);
            }
            if (nextChar == 'R' || nextChar == 'r') {
                uart_read_blocking();
                TURN_TICKS_RIGHT = (int)uart_parse_long();
                if (DEBUG) {
                    uart_print_P(PSTR(">>> Right Turn Ticks updated to: "));
                    uart_print_long(TURN_TICKS_RIGHT);
                    uart_println_P(PSTR(""));
                }
            } else if (nextChar == 'L' || nextChar == 'l') {
                uart_read_blocking();
                TURN_TICKS_LEFT = (int)uart_parse_long();
                if (DEBUG) {
                    uart_print_P(PSTR(">>> Left Turn Ticks updated to: "));
                    uart_print_long(TURN_TICKS_LEFT);
                    uart_println_P(PSTR(""));
                }
            } else {
                int val = (int)uart_parse_long();
                TURN_TICKS_RIGHT = val;
                TURN_TICKS_LEFT  = val;
                if (DEBUG) {
                    uart_print_P(PSTR(">>> BOTH Turn Ticks updated to: "));
                    uart_print_long(val);
                    uart_println_P(PSTR(""));
                }
            }
        }
        else if (cmd == 'M' || cmd == 'm') { 
            myDelayMs(5);
            int16_t nextChar = uart_peek_wait_us(UART_PARSE_INTER_BYTE_US);
            while (nextChar == '\r' || nextChar == '\n') {
                uart_read_blocking();
                nextChar = uart_peek_wait_us(UART_PARSE_INTER_BYTE_US);
            }
            if (nextChar == 'L' || nextChar == 'l') {
                uart_read_blocking();
                int v = (int)uart_parse_long();
                if (v < 0) v = 0;
                if (v > MAX_SPEED) v = MAX_SPEED;
                TURN_SPEED_LEFT = v;
                if (DEBUG) {
                    uart_print_P(PSTR(">>> Left pivot PWM (turn speed) updated to: "));
                    uart_print_long(TURN_SPEED_LEFT);
                    uart_println_P(PSTR(""));
                }
            } else if (nextChar == 'R' || nextChar == 'r') {
                uart_read_blocking();
                int v = (int)uart_parse_long();
                if (v < 0) v = 0;
                if (v > MAX_SPEED) v = MAX_SPEED;
                TURN_SPEED_RIGHT = v;
                if (DEBUG) {
                    uart_print_P(PSTR(">>> Right pivot PWM (turn speed) updated to: "));
                    uart_print_long(TURN_SPEED_RIGHT);
                    uart_println_P(PSTR(""));
                }
            }
        }
        else if (cmd == 'O' || cmd == 'o') {
            myDelayMs(5);
            int v = (int)uart_parse_long();
            if (v < 5)   v = 5;
            if (v > 150) v = 150;
            STOP_DISTANCE = v;
            if (DEBUG) {
                uart_print_P(PSTR(">>> Front stop distance updated to: "));
                uart_print_long(STOP_DISTANCE);
                uart_println_P(PSTR(" cm"));
            }
        }
        else if (cmd == 'P' || cmd == 'p') {
            Kp = uart_parse_float();
            if (DEBUG) {
                uart_print_P(PSTR(">>> Kp updated to: "));
                uart_print_float(Kp, 2);
                uart_println_P(PSTR(""));
            }
        }
        else if (cmd == 'D' || cmd == 'd') {
            myDelayMs(5);
            int16_t nextChar = uart_peek_wait_us(UART_PARSE_INTER_BYTE_US);
            if (nextChar == 'C' || nextChar == 'c') {
                uart_read_blocking();
                myDelayMs(5);
                char thirdChar = (char)uart_read_blocking();
                if (thirdChar == 'R' || thirdChar == 'r') {
                    RIGHT_TRIM = uart_parse_float();
                    if (DEBUG) {
                        uart_print_P(PSTR(">>> Right Trim updated to: "));
                        uart_print_float(RIGHT_TRIM, 3);
                        uart_println_P(PSTR(""));
                    }
                } else if (thirdChar == 'L' || thirdChar == 'l') {
                    LEFT_TRIM = uart_parse_float();
                    if (DEBUG) {
                        uart_print_P(PSTR(">>> Left Trim updated to: "));
                        uart_print_float(LEFT_TRIM, 3);
                        uart_println_P(PSTR(""));
                    }
                }
            } else {
                Kd = uart_parse_float();
                if (DEBUG) {
                    uart_print_P(PSTR(">>> Kd updated to: "));
                    uart_print_float(Kd, 2);
                    uart_println_P(PSTR(""));
                }
            }
        }
        else if (cmd == 'V' || cmd == 'v') {
            myDelayMs(5);
            BASE_SPEED = (int)uart_parse_long();
            if (DEBUG) {
                uart_print_P(PSTR(">>> Base Speed updated to: "));
                uart_print_long(BASE_SPEED);
                uart_println_P(PSTR(""));
            }
        }
        else if (cmd == '?') {
            if (DEBUG) {
                uart_print_P(PSTR(">>> SETTINGS -> Kp:"));   uart_print_float(Kp, 2);
                uart_print_P(PSTR(" | Kd:"));                uart_print_float(Kd, 2);
                uart_print_P(PSTR(" | Speed:"));             uart_print_long(BASE_SPEED);
                uart_print_P(PSTR(" | Ticks Right:"));       uart_print_long(TURN_TICKS_RIGHT);
                uart_print_P(PSTR(" | Ticks Left:"));        uart_print_long(TURN_TICKS_LEFT);
                uart_print_P(PSTR(" | TurnPWM R:"));         uart_print_long(TURN_SPEED_RIGHT);
                uart_print_P(PSTR(" L:"));                   uart_print_long(TURN_SPEED_LEFT);
                uart_print_P(PSTR(" | L_Trim:"));            uart_print_float(LEFT_TRIM, 3);
                uart_print_P(PSTR(" | R_Trim:"));            uart_print_float(RIGHT_TRIM, 3);
                uart_print_P(PSTR(" | Stop(cm):"));           uart_print_long(STOP_DISTANCE);
                uart_print_P(PSTR(" | BackPWM:"));            uart_print_long(BACK_SPEED);
                uart_print_P(PSTR(" | BackMs:"));             uart_print_long(BACK_TIME_MS);
                uart_println_P(PSTR(""));
            }
        }
    }

    if (!isRunning) {
        stopMotors();
        myDelayMs(100);
        return;
    }

    if (!warmupDone) { warmupSensors(); warmupDone = true; }

    // 2. Read sensors
    float distRight = getDistance(&SENSOR_RIGHT);
    float distLeft  = getDistance(&SENSOR_LEFT);
    float distFront = getDistance(&SENSOR_FRONT);

    if (distFront > 0 && distFront >= STOP_DISTANCE) {
        deadReverseTrialUsed = false;
    }

    // 3. Front obstacle avoidance (triggers pivot turns)
    if (distFront > 0 && distFront < STOP_DISTANCE) {
        stopMotors(); myDelayMs(100);
        float freshRight = getDistance(&SENSOR_RIGHT);
        float freshLeft  = getDistance(&SENSOR_LEFT);
        float freshFront = getDistance(&SENSOR_FRONT);

        bool rightOpen = (freshRight <= 0 || freshRight > RIGHT_WALL_THRESHOLD);
        bool leftOpen  = (freshLeft  <= 0 || freshLeft  > LEFT_WALL_THRESHOLD);

        if (rightOpen) {
            deadReverseTrialUsed = false;
            turnRight90();
        } else if (leftOpen) {
            deadReverseTrialUsed = false;
            turnLeft90();
        } else {
            bool inDeadRange = (freshFront > 0 && freshFront < DEAD_DISTANCE);
            if (!inDeadRange) {
                if (DEBUG) uart_println_P(PSTR(">>> Stop+blocked, not dead range; resume PID."));
                return;
            }
            if (!deadReverseTrialUsed) {
                if (DEBUG) {
                    uart_print_P(PSTR(">>> Dead range: reverse "));
                    uart_print_long(BACK_TIME_MS);
                    uart_println_P(PSTR("ms."));
                }
                goBackwardMs((uint32_t)BACK_TIME_MS);
                deadReverseTrialUsed = true;
                return;
            }
            if (DEBUG) uart_println_P(PSTR("Dead end! Stopping."));
            isRunning = false;
            stopMotors();
            printDeadEndReport();   // ALWAYS printed, regardless of DEBUG
        }
        return;
    }

    // 4. PID center tracking
    float error = 0.0f;
    bool rightValid = (distRight > 0 && distRight < 45.0f);
    bool leftValid  = (distLeft  > 0 && distLeft  < 45.0f);

    if      (leftValid && rightValid) error = (distLeft - distRight) / 2.0f;
    else if (rightValid)              error = TARGET_CENTER - distRight;
    else if (leftValid)               error = distLeft - TARGET_CENTER;

    integral += error;
    if (integral >  100.0f) integral =  100.0f;
    if (integral < -100.0f) integral = -100.0f;
    float derivative = error - previous_error;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previous_error   = error;

    int leftMotorSpeed  = (int)(BASE_SPEED * LEFT_TRIM)  - (int)correction;
    int rightMotorSpeed = (int)(BASE_SPEED * RIGHT_TRIM) + (int)correction;

    if (leftMotorSpeed  < MIN_SPEED) leftMotorSpeed  = MIN_SPEED;
    if (leftMotorSpeed  > MAX_SPEED) leftMotorSpeed  = MAX_SPEED;
    if (rightMotorSpeed < MIN_SPEED) rightMotorSpeed = MIN_SPEED;
    if (rightMotorSpeed > MAX_SPEED) rightMotorSpeed = MAX_SPEED;

    // 5. Debug print (gated)
    if (DEBUG) {
        uart_print_P(PSTR("F: "));        uart_print_float(distFront, 2);
        uart_print_P(PSTR(" | L: "));     uart_print_float(distLeft,  2);
        uart_print_P(PSTR(" | R: "));     uart_print_float(distRight, 2);
        uart_print_P(PSTR(" | Err: "));   uart_print_float(error,     2);
        uart_print_P(PSTR(" | Corr: "));  uart_print_float(correction,2);
        uart_print_P(PSTR(" | L_PWM: ")); uart_print_long(leftMotorSpeed);
        uart_print_P(PSTR(" | R_PWM: ")); uart_print_long(rightMotorSpeed);
        uart_print_P(PSTR(" | L_Tick: "));uart_print_ulong(leftTicks);
        uart_print_P(PSTR(" | R_Tick: "));uart_print_ulong(rightTicks);
        uart_println_P(PSTR(""));
    }

    // 6. Drive motors
    runMotor(1, leftMotorSpeed,  true);
    runMotor(2, rightMotorSpeed, true);
    myDelayMs(50);
}
