# Build & Deployment Guide

## Quick Start (Arduino IDE)

1. **Open File**  
   File → Open → `wall_following_car.ino`

2. **Configure Board**  
   Tools → Board → Arduino AVR Boards → Arduino Uno  
   Tools → Port → /dev/ttyUSB0 (or COM3, etc.)

3. **Verify Compilation**  
   Sketch → Verify/Compile (Ctrl+R)

4. **Upload**  
   Sketch → Upload (Ctrl+U)

5. **Monitor Serial**  
   Tools → Serial Monitor (set 9600 baud)

---

## Compile via Command Line (AVR-GCC)

```bash
# Set environment
export F_CPU=16000000UL
export MCU=atmega328p

# Compile all modules
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/utils/timing.c -o timing.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/system/system_init.c -o system_init.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/motors/pwm_timer.c -o pwm_timer.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/motors/motor.c -o motor.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/sensors/ultrasonic.c -o ultrasonic.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/sensors/encoders.c -o encoders.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/control/pid.c -o pid.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/control/wall_follow.c -o wall_follow.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/navigation/movement.c -o movement.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/navigation/obstacle_avoidance.c -o obstacle_avoidance.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/communication/uart.c -o uart.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/communication/command_parser.c -o command_parser.o
avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -Wall -Os -c src/robot.c -o robot.o

# Link
avr-gcc -mmcu=$MCU -o firmware.elf *.o -lm

# Convert to hex
avr-objcopy -O ihex firmware.elf firmware.hex

# Optional: Check size
avr-size firmware.elf

# Upload (requires avrdude)
avrdude -p m328p -c usbasp -U flash:w:firmware.hex:i
```

---

## File Organization Reference

```
wall-following-car/
│
├── wall_following_car.ino        ← Arduino IDE entry point
│   - Contains: setup() and loop()
│   - Calls: robot_init() and robot_loop()
│
├── src/
│
│   ├── robot.c / robot.h         ← Main coordinator
│   │   - Initializes all subsystems
│   │   - robot_loop() = FSM execution
│   │
│   ├── utils/
│   │   ├── timing.c / .h         ← Timer1 ISR, myMicros(), myDelayMs()
│   │   └── utils.h               ← Bit macros (SBI, CBI, RBI)
│   │
│   ├── system/
│   │   └── system_init.c / .h    ← GPIO port setup, encoder interrupts
│   │
│   ├── motors/
│   │   ├── pwm_timer.c / .h      ← Timer2 PWM init, OCR access
│   │   └── motor.c / .h          ← runMotor(), stopMotors()
│   │
│   ├── sensors/
│   │   ├── ultrasonic.c / .h     ← getDistance() with pulse timing
│   │   └── encoders.c / .h       ← Encoder ISRs, tick counters
│   │
│   ├── control/
│   │   ├── pid.c / .h            ← Generic PID controller
│   │   └── wall_follow.c / .h    ← Wall centering logic
│   │
│   ├── navigation/
│   │   ├── movement.c / .h       ← Movement sequences (forward, turns)
│   │   └── obstacle_avoidance.c/.h ← Obstacle detection & avoidance FSM
│   │
│   └── communication/
│       ├── uart.c / .h           ← UART driver with ring buffer
│       └── command_parser.c / .h ← Serial command dispatch
│
├── docs/
│   └── (mechanical design image, FSM diagram, etc.)
│
└── README.md                      ← Comprehensive documentation
```

---

## Dependency Chain

```
wall_following_car.ino
    ↓
robot.c (robot_init, robot_loop)
    ↓
system_init   timing   pwm_timer   uart   encoders
    ↓              ↓         ↓        ↓         ↓
  GPIO      Timer1 ISR  Timer2 PWM  UART_RX  INT0/INT1
```

**Compilation Order** (satisfies dependencies):
1. utils/timing.c (lowest level - used by many)
2. system/system_init.c
3. motors/pwm_timer.c
4. motors/motor.c
5. sensors/ultrasonic.c
6. sensors/encoders.c
7. control/pid.c
8. control/wall_follow.c
9. communication/uart.c
10. navigation/movement.c
11. navigation/obstacle_avoidance.c
12. communication/command_parser.c
13. robot.c
14. wall_following_car.ino (links all)

---

## Memory Usage Estimate

**Flash (Program Memory):**
- Total available: 30 KB (on ATmega328P)
- This firmware: ~6-7 KB (modular + aggressive -Os)
- Remaining: ~23 KB (room for extensions)

**SRAM (Data Memory):**
- Total available: 2 KB
- Global state: ~300 bytes
- Stack: ~1.7 KB (mostly motor ISRs + sensor reads)
- Dynamic: <100 bytes

**EEPROM:**
- Not used (firmware doesn't store calibration)
- Available: 1 KB

---

## Troubleshooting

### Compilation Errors

**Error: `undefined reference to 'uart_init'`**
- Solution: Ensure all .c files compiled before linking

**Error: `iso c99 requires...` at ISR**
- Solution: Add `-std=c99` to avr-gcc command

**Error: `multiple definition of 'motor_1'`**
- Solution: Use `static` for file-scope globals, remove from headers

### Runtime Issues

**Motors don't move**
1. Check UART output (should print welcome banner)
2. Send command `1` to start
3. Verify motor speeds: `V150` then `1` (start with higher speed)

**No UART output**
1. Check baud rate: 9600
2. Check pin D0/D1 connections (RX/TX)
3. Verify UART_BAUD macro = 9600

**Sensor reads all zeros**
1. Check pin connections (D4-D7, A4-A5)
2. Power ultrasonic boards separately
3. Verify getDistance() called after warmup

---

## Flashing via avrdude (if using AVR programmer)

```bash
# List connected devices
avrdude -p m328p -c usbasp

# Erase and write
avrdude -p m328p -c usbasp -e -U flash:w:firmware.hex:i

# Verify
avrdude -p m328p -c usbasp -U flash:v:firmware.hex:i

# Read fuses (check F_CPU clock)
avrdude -p m328p -c usbasp -U lfuse:r:lfuse.txt:h
```

---

## Testing Sequence

1. **Compile & Upload**
   ```
   ✓ No compilation errors
   ✓ Upload successful
   ```

2. **Serial Monitor Test**
   ```
   ✓ "MASTER CAR SCRIPT READY" message
   ✓ Commands echo back (e.g., send '?' → see settings)
   ```

3. **Motor Test**
   ```
   Send: 1 (start)
   ✓ Both motors spin forward
   Send: 0 (stop)
   ✓ Motors stop
   ```

4. **Sensor Test**
   ```
   Send: 1 (start)
   ✓ Serial output shows: "F: 50.25 | L: 25.50 | R: 26.30 ..."
   ```

5. **Obstacle Avoidance Test**
   ```
   Bring obstacle to front sensor
   ✓ Robot either turns or backs away
   ```

---

## IDE Configuration (Optional)

Create `.vscode/settings.json` in project root:

```json
{
    "C_Cpp.default.includePath": [
        "${workspaceFolder}/src",
        "/usr/lib/avr/include"
    ],
    "C_Cpp.default.compilerPath": "/usr/bin/avr-gcc",
    "C_Cpp.default.cStandard": "c99"
}
```

---

## Next Steps

1. Compile and upload
2. Verify serial communication
3. Test motor control
4. Calibrate PID gains (see README.md tuning guide)
5. Adjust turn tick counts for 90° precision

---

**Questions?** See README.md for detailed architecture, FSM diagram, and control logic.
