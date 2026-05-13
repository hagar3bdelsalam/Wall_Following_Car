#include "movement.h"
#include "../motors/motor.h"
#include "../sensors/encoders.h"
#include "../utils/timing.h"
#include "../communication/uart.h"
#include <avr/interrupt.h>

void movement_init(MovementConfig* mc) {
    mc->baseSpeed       = 110;
    mc->turnSpeedRight  = 100;
    mc->turnSpeedLeft   = 113;
    mc->turnTicksRight  = 27;
    mc->turnTicksLeft   = 72;
    mc->backSpeed       = 100;
    mc->backTimeMs      = 1000;
    mc->leftTrim        = 0.9f;
    mc->rightTrim       = 1.0f;
    mc->debugEnabled    = true;
}

void goForward(const MovementConfig* mc, int seconds) {
    if (mc->debugEnabled) {
        uart_print_str(">>> Moving Forward for ");
        uart_print_long(seconds);
        uart_println("s.");
    }
    runMotor(1, (int)(mc->baseSpeed * mc->leftTrim),  true);
    runMotor(2, (int)(mc->baseSpeed * mc->rightTrim), true);
    myDelayMs((uint32_t)seconds * 1000UL);
    stopMotors();
    if (mc->debugEnabled) uart_println(">>> Done. Stopped.");
}

void goBackwardMs(const MovementConfig* mc, uint32_t ms) {
    int l = (int)(mc->backSpeed * mc->leftTrim);
    int r = (int)(mc->backSpeed * mc->rightTrim);
    if (l < 1) l = 1;
    if (r < 1) r = 1;
    runMotor(1, l, false);
    runMotor(2, r, false);
    _delay_us(200);
    uint32_t end = myMicros() + ms * 1000UL;
    while ((int32_t)(end - myMicros()) > 0) { }
    stopMotors();
}

void turnRight90(const MovementConfig* mc) {
    if (mc->debugEnabled) uart_println(">>> Turning RIGHT (Pivot Turn)");
    stopMotors();
    myDelayMs(200);
    encoders_reset_ticks();

    volatile unsigned long* leftTicks = encoders_get_left_ticks_ptr();
    
    runMotor(1, mc->turnSpeedRight, true);
    runMotor(2, 0, true);

    while (*leftTicks < (unsigned long)mc->turnTicksRight) {
        if (mc->debugEnabled) {
            uart_print_str("Turning Right - Left Ticks: ");
            uart_print_ulong(*leftTicks);
            uart_print_str("\r\n");
        }
        myDelayMs(20);
    }

    stopMotors();
    myDelayMs(10);
}

void turnLeft90(const MovementConfig* mc) {
    if (mc->debugEnabled) uart_println(">>> Turning LEFT (Pivot Turn)");
    stopMotors();
    myDelayMs(200);
    encoders_reset_ticks();

    volatile unsigned long* rightTicks = encoders_get_right_ticks_ptr();
    
    runMotor(1, 0, true);
    runMotor(2, mc->turnSpeedLeft, true);

    while (*rightTicks < (unsigned long)mc->turnTicksLeft) {
        if (mc->debugEnabled) {
            uart_print_str("Turning Left - Right Ticks: ");
            uart_print_ulong(*rightTicks);
            uart_print_str("\r\n");
        }
        myDelayMs(20);
    }

    stopMotors();
    myDelayMs(10);
}
