#include "system_init.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Motor 1 (PB3=EN/OC2A, PB4=IN1, PB5=IN2)
#define M1_EN_BIT     PB3
#define M1_IN1_BIT    PB4
#define M1_IN2_BIT    PB5

// Motor 2 (PB2=EN/OC1B, PB1=IN1, PB0=IN2)
#define M2_EN_BIT     PB2
#define M2_IN1_BIT    PB1
#define M2_IN2_BIT    PB0

// Ultrasonic FRONT (PD5=TRIG, PD4=ECHO)
#define TRIG_FRONT_BIT  PD5
#define ECHO_FRONT_BIT  PD4

// Ultrasonic RIGHT (PD7=TRIG, PD6=ECHO)
#define TRIG_RIGHT_BIT  PD7
#define ECHO_RIGHT_BIT  PD6

// Ultrasonic LEFT (PC0=TRIG, PC1=ECHO)
#define TRIG_LEFT_BIT   PC0
#define ECHO_LEFT_BIT   PC1

// Encoders (PD2=RIGHT/INT0, PD3=LEFT/INT1)
#define ENC_RIGHT_BIT   PD2
#define ENC_LEFT_BIT    PD3

/**
 * @brief Initialize GPIO ports (motors, sensors, encoders)
 */
static void gpio_init(void) {
    // PORTB: motors all outputs, default low
    DDRB  |= (1 << M1_EN_BIT)  | (1 << M1_IN1_BIT) | (1 << M1_IN2_BIT)
          |  (1 << M2_EN_BIT)  | (1 << M2_IN1_BIT) | (1 << M2_IN2_BIT);
    PORTB &= ~((1 << M1_IN1_BIT) | (1 << M1_IN2_BIT)
            |  (1 << M2_IN1_BIT) | (1 << M2_IN2_BIT));

    // PORTD: trigs out, echoes in, encoders in w/ pull-ups
    DDRD  |=  (1 << TRIG_FRONT_BIT) | (1 << TRIG_RIGHT_BIT);
    DDRD  &= ~((1 << ECHO_FRONT_BIT) | (1 << ECHO_RIGHT_BIT)
            |  (1 << ENC_LEFT_BIT)   | (1 << ENC_RIGHT_BIT));
    PORTD |=  (1 << ENC_LEFT_BIT) | (1 << ENC_RIGHT_BIT);
    PORTD &= ~((1 << TRIG_FRONT_BIT) | (1 << TRIG_RIGHT_BIT));

    // PORTC: trig left out, echo left in
    DDRC  |=  (1 << TRIG_LEFT_BIT);
    DDRC  &= ~(1 << ECHO_LEFT_BIT);
    PORTC &= ~(1 << TRIG_LEFT_BIT);
}

/**
 * @brief Initialize encoder interrupts (INT0 for right, INT1 for left)
 */
static void encoders_init(void) {
    // ISC01:0 = 01 (CHANGE), ISC11:0 = 01 (CHANGE)
    EICRA = (1 << ISC10) | (1 << ISC00);
    EIMSK = (1 << INT0)  | (1 << INT1);
}

void system_init(void) {
    cli();  // Disable interrupts globally
    
    gpio_init();
    encoders_init();
    
    // Timer and UART initialization handled by their respective modules
    // (called after system_init in main setup)
    
    sei();  // Enable interrupts globally
}
