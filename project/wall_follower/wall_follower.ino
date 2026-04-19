#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// ─────────────────────────────────────────────
//  CONFIG
// ─────────────────────────────────────────────

// Distance thresholds (cm)
#define WALL_FRONT_STOP_CM 2   // Ultrasonic: wall ahead → stop / turn
#define WALL_FRONT_CLEAR_CM 4  // Ultrasonic: path is clear

// Motor base speed (0–255 PWM duty equivalent)
#define BASE_SPEED 180
#define TURN_SPEED 160

// Turn duration estimate (ms) — refine with encoder counts later
#define TURN_90_TICKS 200

// IR sensor logic (depends on your module: 0 = wall detected, 1 = clear)
#define IR_WALL_DETECTED 0
#define IR_CLEAR 1

// ─────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────

// Left motor
#define PIN_MOTOR_L_IN1 26
#define PIN_MOTOR_L_IN2 27
#define PIN_MOTOR_L_PWM 14
#define PIN_ENC_L_A 34
#define PIN_ENC_L_B 35

// Right motor
#define PIN_MOTOR_R_IN3 25
#define PIN_MOTOR_R_IN4 33
#define PIN_MOTOR_R_PWM 32
#define PIN_ENC_R_A 36
#define PIN_ENC_R_B 39

// Ultrasonic
#define PIN_TRIG 5
#define PIN_ECHO 18

// IR sensors
#define PIN_IR_LEFT 19
#define PIN_IR_RIGHT 21

// ─────────────────────────────────────────────
//  FSM STATE DEFINITIONS
// ─────────────────────────────────────────────

typedef enum {
  STATE_IDLE,        // Robot powered on, waiting to start
  STATE_FORWARD,     // Moving straight along the wall
  STATE_TURN_LEFT,   // Executing a 90° left turn
  STATE_TURN_RIGHT,  // Executing a 90° right turn
  STATE_DONE         // Track completed
} RobotState;

// ─────────────────────────────────────────────
//  GLOBAL STATE
// ─────────────────────────────────────────────

static RobotState current_state = STATE_IDLE;

static int turn_count = 0;
static char turn_sequence[64];  // e.g. "L,R,R,L,L"
static int seq_index = 0;

static int32_t turn_start_left = 0;
static int32_t turn_start_right = 0;
static uint32_t last_report_ms = 0;

// Encoder tick counters (incremented in ISR)
volatile int32_t enc_left_ticks = 0;
volatile int32_t enc_right_ticks = 0;

// ─────────────────────────────────────────────
//  DRIVER STUBS
//  Replace each function body with your
//  register-level ESP32 implementation.
// ─────────────────────────────────────────────

/**
 * motor_init()
 * Configure GPIO and LEDC PWM channels for both motors.
 * Register-level: set IO_MUX, GPIO_ENABLE_REG, LEDC timer/channel regs.
 */
void motor_init(void) {
  // TODO: register-level GPIO + LEDC PWM init
  // Placeholder: uses Arduino-style for compilation
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_IN3, OUTPUT);
  pinMode(PIN_MOTOR_R_IN4, OUTPUT);
  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
}

/**
 * motor_set(left_speed, right_speed)
 * speed: -255 (full reverse) to +255 (full forward), 0 = brake
 * Register-level: write IN1/IN2 direction bits + LEDC duty register.
 */
void motor_set(int left_speed, int right_speed) {
  // TODO: replace with register-level writes

  // LEFT motor direction
  if (left_speed > 0) {
    digitalWrite(PIN_MOTOR_L_IN1, HIGH);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
  } else if (left_speed < 0) {
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, HIGH);
  } else {
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);  // brake
  }
  analogWrite(PIN_MOTOR_L_PWM, abs(left_speed));

  // RIGHT motor direction
  if (right_speed > 0) {
    digitalWrite(PIN_MOTOR_R_IN3, HIGH);
    digitalWrite(PIN_MOTOR_R_IN4, LOW);
  } else if (right_speed < 0) {
    digitalWrite(PIN_MOTOR_R_IN3, LOW);
    digitalWrite(PIN_MOTOR_R_IN4, HIGH);
  } else {
    digitalWrite(PIN_MOTOR_R_IN3, LOW);
    digitalWrite(PIN_MOTOR_R_IN4, LOW);  // brake
  }
  analogWrite(PIN_MOTOR_R_PWM, abs(right_speed));
}

/**
 * ultrasonic_read_cm()
 * Returns distance in cm measured by HC-SR04.
 * Register-level: toggle TRIG GPIO via GPIO_OUT_REG, measure ECHO pulse
 * width using hardware timer capture — avoid blocking pulseIn().
 */
float ultrasonic_read_cm(void) {
  // TODO: replace with non-blocking timer-based measurement
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 30000);  // 30 ms timeout
  return duration * 0.034f / 2.0f;
}

/**
 * ir_read_left() / ir_read_right()
 * Returns IR_WALL_DETECTED or IR_CLEAR.
 * Register-level: read GPIO_IN_REG bit for the IR pin.
 */
int ir_read_left(void) {
  // TODO: register-level: return (GPIO_IN_REG >> PIN_IR_LEFT) & 1
  return digitalRead(PIN_IR_LEFT);
}

int ir_read_right(void) {
  // TODO: register-level: return (GPIO_IN_REG >> PIN_IR_RIGHT) & 1
  return digitalRead(PIN_IR_RIGHT);
}

void IRAM_ATTR isr_enc_left(void) {
  if (digitalRead(PIN_ENC_L_B))
    enc_left_ticks++;
  else
    enc_left_ticks--;
}
void IRAM_ATTR isr_enc_right(void) {
  if (digitalRead(PIN_ENC_R_B))
    enc_right_ticks++;
  else
    enc_right_ticks--;
}

/**
 * encoder_init()
 * Configure encoder pins as inputs with interrupt on CHANGE.
 * Register-level: GPIO_ENABLE_REG, GPIO_PIN_REG interrupt type,
 * attach ISR via intr_matrix_set().
 */
void encoder_init(void) {
  // TODO: register-level GPIO interrupt config
  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_R_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isr_enc_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isr_enc_right, CHANGE);
}

/**
 * wifi_send_report()
 * Sends turn summary over WiFi (TCP/UDP) or Serial for Bluetooth.
 * Format required by project spec:
 *   Turns: 5
 *   Sequence: L, R, R, L, L
 */
void wifi_send_report(void) {
  // TODO: replace with actual WiFi TCP send using ESP-IDF socket API
  // For now, send over Serial (can wire to BT module UART)
  Serial.printf("\n--- TURN REPORT ---\n");
  Serial.printf("Turns: %d\n", turn_count);
  Serial.printf("Sequence: %s\n", turn_sequence);
  Serial.printf("-------------------\n");
}

// ─────────────────────────────────────────────
//  HELPER: RECORD A TURN
// ─────────────────────────────────────────────

void record_turn(char direction) {
  turn_count++;
  if (seq_index > 0) {
    turn_sequence[seq_index++] = ',';
    turn_sequence[seq_index++] = ' ';
  }
  turn_sequence[seq_index++] = direction;
  turn_sequence[seq_index] = '\0';
  Serial.printf("[TURN #%d] Direction: %c\n", turn_count, direction);
}

// ─────────────────────────────────────────────
//  FSM TRANSITION LOGIC
// ─────────────────────────────────────────────

void fsm_update(void) {
  float dist_cm = ultrasonic_read_cm();
  int ir_left = ir_read_left();
  int ir_right = ir_read_right();
  uint32_t now_ms = millis();

  bool wall_ahead = (dist_cm > 0 && dist_cm < WALL_FRONT_STOP_CM);
  bool left_open = (ir_left == IR_CLEAR);
  bool right_open = (ir_right == IR_CLEAR);

  int32_t left_diff = abs(enc_left_ticks - turn_start_left);
  int32_t right_diff = abs(enc_right_ticks - turn_start_right);

  switch (current_state) {

    // ── IDLE ──────────────────────────────────────────────
    case STATE_IDLE:
      motor_set(0, 0);
      // Wait for Serial START command or button press
      if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "START") {
          Serial.println("[FSM] Starting!");
          current_state = STATE_FORWARD;
        }
      }
      break;

    // ── FORWARD ───────────────────────────────────────────
    case STATE_FORWARD:
      motor_set(BASE_SPEED, BASE_SPEED);

      if (wall_ahead) {
        // Wall directly ahead → need to turn
        motor_set(0, 0);

        if (right_open) {
          // Right is the default if right open
          record_turn('R');
          current_state = STATE_TURN_RIGHT;
        } else if (left_open) {
          // Only left is open
          record_turn('L');
          current_state = STATE_TURN_LEFT;
        } else {
          // Dead end
          current_state = STATE_DONE;
        }
      } else if (right_open && dist_cm > WALL_FRONT_CLEAR_CM) {
        // Robot should take the right turn to keep following wall
        record_turn('R');
        current_state = STATE_TURN_RIGHT;
      }
      turn_start_left = enc_left_ticks;
      turn_start_right = enc_right_ticks;
      break;

    // ── TURN LEFT ─────────────────────────────────────────
    case STATE_TURN_LEFT:
      // Left wheel back, right wheel forward → pivot left
      motor_set(-TURN_SPEED, TURN_SPEED);

      if (left_diff >= TURN_90_TICKS || right_diff >= TURN_90_TICKS) {
        motor_set(0, 0);
        current_state = STATE_FORWARD;
      }

      break;

    // ── TURN RIGHT ────────────────────────────────────────
    case STATE_TURN_RIGHT:
      // Left wheel forward, right wheel back → pivot right
      motor_set(TURN_SPEED, -TURN_SPEED);

      if (left_diff >= TURN_90_TICKS || right_diff >= TURN_90_TICKS) {
        motor_set(0, 0);
        current_state = STATE_FORWARD;
      }

      break;

    // ── DONE ─────────────────────────────────────────────
    case STATE_DONE:
      motor_set(0, 0);
      wifi_send_report();
      // Stay in DONE — do nothing
      break;
  }
}

// ─────────────────────────────────────────────
//  ARDUINO ENTRY POINTS
// ─────────────────────────────────────────────

void setup(void) {
  Serial.begin(115200);

  // Sensor pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  motor_init();
  encoder_init();

  // Initialize turn sequence buffer
  turn_sequence[0] = '\0';

  Serial.println("=== Wall-Following Robot READY ===");
  Serial.println("Send 'START' via Serial to begin.");
  current_state = STATE_IDLE;
}

void loop(void) {
  fsm_update();
}
