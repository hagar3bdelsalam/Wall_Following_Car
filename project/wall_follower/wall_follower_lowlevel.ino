#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

/* --- Hardware Mapping --- */
/* Motor 1 (Left): EN=PB3(OC2A), IN1=PB5, IN2=PB4 */
/* Motor 2 (Right): EN=PB2(OC1B), IN1=PB0, IN2=PB1 */
/* Ultrasonic: TRIG_F=PD5, ECHO_F=PD4 | TRIG_R=PD3, ECHO_R=PD2 | TRIG_L=PD7, ECHO_L=PD6 */

#define BASE_SPEED        70
#define MAX_SPEED         200
#define MIN_SPEED         0
#define TARGET_MARGIN_MIN 2.0f
#define TARGET_MARGIN_MAX 7.0f
#define STOP_DISTANCE     5
#define SPEED_OF_SOUND    0.0343f

/* PID Constants */
const float Kp = 3.0f;
const float Ki = 0.0f;
const float Kd = 0.7f;

/* Global Variables */
float previousError = 0;
float integral = 0;
bool isRunning = false;

/* --- Function Prototypes --- */
void initUART(unsigned int baud);
void uartTransmit(char data);
void uartPrint(const char* s);
void initMotors(void);
void initSensors(void);
void setMotorSpeeds(int left, int right);
float getDistance(uint8_t trigPin, uint8_t echoPin);

int main(void)
{
    initUART(103); // 9600 baud for 16MHz
    initMotors();
    initSensors();
    
    uartPrint("System Initialized. Send '1' to Start.\r\n");

    while (1)
    {
        /* 1. UART Command Handling */
        if (UCSR0A & (1 << RXC0))
        {
            char cmd = UDR0;
            if (cmd == '1' || cmd == 'S' || cmd == 's') isRunning = true;
            else if (cmd == '0' || cmd == 'X' || cmd == 'x') isRunning = false;
        }

        if (!isRunning)
        {
            setMotorSpeeds(0, 0);
            _delay_ms(100);
            continue;
        }

        /* 2. Sensor Readings */
        float distRight = getDistance(PD3, PD2);
        float distFront = getDistance(PD5, PD4);

        if (distFront > 0 && distFront < STOP_DISTANCE)
        {
            setMotorSpeeds(0, 0);
            _delay_ms(500);
            continue;
        }

        /* 3. PID Logic */
        float error = 0.0f;
        if (distRight > 0)
        {
            error = (distRight < TARGET_MARGIN_MIN) ? (TARGET_MARGIN_MIN - distRight) :
                    (distRight > TARGET_MARGIN_MAX) ? (TARGET_MARGIN_MAX - distRight) : 0.0f;
        }

        float derivative = error - previousError;
        integral += error;
        float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
        previousError = error;

        int leftSpeed = (int)(BASE_SPEED - correction);
        int rightSpeed = (int)(BASE_SPEED + correction);

        /* 4. Apply Speeds */
        setMotorSpeeds(leftSpeed, rightSpeed);
        
        _delay_ms(50);
    }
}

/* --- Hardware Initialization --- */

void initUART(unsigned int baud)
{
    UBRR0H = (unsigned char)(baud >> 8);
    UBRR0L = (unsigned char)baud;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void initMotors(void)
{
    /* Set Motor Pins as Output */
    DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5);

    /* Timer 1 (16-bit) for Motor 2 PWM on PB2 (OC1B) */
    TCCR1A = (1 << COM1B1) | (1 << WGM10); // Phase Correct PWM 8-bit
    TCCR1B = (1 << CS11);                  // Prescaler 8

    /* Timer 2 (8-bit) for Motor 1 PWM on PB3 (OC2A) */
    TCCR2A = (1 << COM2A1) | (1 << WGM20); // Phase Correct PWM
    TCCR2B = (1 << CS21);                  // Prescaler 8
}

void initSensors(void)
{
    /* Trig: Output, Echo: Input */
    DDRD |= (1 << DDD3) | (1 << DDD5) | (1 << DDD7);
    DDRD &= ~((1 << DDD2) | (1 << DDD4) | (1 << DDD6));
}

/* --- Helper Logic --- */

float getDistance(uint8_t trigPin, uint8_t echoPin)
{
    PORTD &= ~(1 << trigPin);
    _delay_us(2);
    PORTD |= (1 << trigPin);
    _delay_us(10);
    PORTD &= ~(1 << trigPin);

    uint32_t count = 0;
    uint32_t max_ticks = 40000; // Timeout approx 20-30ms

    while (!(PIND & (1 << echoPin)) && count < max_ticks) count++;
    if (count >= max_ticks) return 0;

    count = 0;
    while ((PIND & (1 << echoPin)) && count < max_ticks) count++;
    
    return (count * 0.06) * SPEED_OF_SOUND / 2.0; // Rough conversion for 16MHz loop
}

void setMotorSpeeds(int left, int right)
{
    /* Constrain values */
    if (left > MAX_SPEED) left = MAX_SPEED;
    if (left < MIN_SPEED) left = MIN_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;
    if (right < MIN_SPEED) right = MIN_SPEED;

    /* Left Motor (Motor 1) Direction */
    if (left > 0) { PORTB |= (1 << PB5); PORTB &= ~(1 << PB4); }
    OCR2A = left;

    /* Right Motor (Motor 2) Direction */
    if (right > 0) { PORTB |= (1 << PB0); PORTB &= ~(1 << PB1); }
    OCR1B = right;
}

void uartPrint(const char* s)
{
    while (*s)
    {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *s++;
    }
}