// --- Motor Pins ---
const int M2_EN  = 10;
const int M2_IN1 = 9;
const int M2_IN2 = 8;

const int M1_EN  = 11;
const int M1_IN1 = 12;
const int M1_IN2 = 13;

// --- Encoder Pins ---
const int ENCODER_LEFT  = 2;
const int ENCODER_RIGHT = 3;

// --- Ultrasonic Pins ---
const int TRIG_FRONT = 5;  const int ECHO_FRONT = 4;
const int TRIG_RIGHT = 7;  const int ECHO_RIGHT = 6;
const int TRIG_LEFT  = A4; const int ECHO_LEFT  = A5;

// --- Base Speed & Limits ---
const int BASE_SPEED = 150;
const int MAX_SPEED  = 200;
const int MIN_SPEED  = 0;

// --- Motor Trim ---
const float LEFT_TRIM  = 0.97;
const float RIGHT_TRIM = 0.87;

// --- Wall Following Target ---
const float TARGET_MARGIN_MIN = 4.0;
const float TARGET_MARGIN_MAX = 10.0;

// --- Wall PID Constants ---
const float Kp = 0.7;
const float Ki = 0.0;
const float Kd = 0.35;

// --- Wall PID State ---
float previous_error = 0;
float integral       = 0;

// --- Sensor Spike Filter ---
float lastDistR = -1.0;
float lastDistL = -1.0;
const float THRESHOLD_DIST = 50.0;

// --- Encoder Variables ---
volatile unsigned long leftTicks  = 0;
volatile unsigned long rightTicks = 0;
unsigned long lastEncoderTime = 0;
long leftTicksPerSec  = 0;
long rightTicksPerSec = 0;

const int   PPR = 20*2;
const float SPEED_CORRECT_KP = 0.5;

// --- Runtime Flags & Safety ---
bool isRunning = false;
const int   STOP_DISTANCE  = 10;
const float SPEED_OF_SOUND = 0.0343;
volatile unsigned long lastLeftMicros  = 0;
volatile unsigned long lastRightMicros = 0;
bool warmupDone = false;

// --- Turn Settings ---
// Distance threshold: if right wall is farther than this, "no wall on right" → turn right
const float RIGHT_WALL_THRESHOLD = 15.0;
// Distance threshold: if left wall is farther than this, "no wall on left" → turn left
const float LEFT_WALL_THRESHOLD  = 15.0;

// How long to spin in place for ~90° (tune this for your robot's geometry & speed)
const int TURN_SPEED    = 150;   // PWM for both motors during turn
const int TURN_DURATION = 650;   // ms — increase if robot undershoots 90°

// ============================================================
// ISRs for Encoders
// ============================================================
void countLeft() {
    unsigned long now = micros();
    if (now - lastLeftMicros > 300) {
        leftTicks++;
        lastLeftMicros = now;
    }
}

void countRight() {
    unsigned long now = micros();
    if (now - lastRightMicros > 300) {
        rightTicks++;
        lastRightMicros = now;
    }
}

// ============================================================
// Turn Execution  (pivot-in-place, encoder-assisted)
// ============================================================
void turnRight90() {
    Serial.println(">>> Turning RIGHT");

    // Reset encoder ticks so we can measure the turn
    noInterrupts();
    leftTicks  = 0;
    rightTicks = 0;
    interrupts();

    // Pivot right: left motor forward, right motor backward
    runMotor(1, TURN_SPEED, true);   // Left wheel forward
    runMotor(2, TURN_SPEED, false);  // Right wheel backward

    // Use encoder ticks as a safety backup; primarily time-based
    // Target ticks for 90° = (wheel_base_circumference / 4) / wheel_circumference * PPR
    // Until you tune that, we rely on TURN_DURATION ms
    unsigned long start = millis();
    while (millis() - start < TURN_DURATION) {
        // Optional: break early if encoder target is reached
        delay(5);
    }

    stopMotors();
    delay(200); // brief settle before resuming wall follow

    // Reset PID state after turn so it doesn't carry stale error
    integral       = 0;
    previous_error = 0;
    lastDistR      = -1.0;
    lastDistL      = -1.0;
}

void turnLeft90() {
    Serial.println(">>> Turning LEFT");

    noInterrupts();
    leftTicks  = 0;
    rightTicks = 0;
    interrupts();

    // Pivot left: right motor forward, left motor backward
    runMotor(1, TURN_SPEED, false);  // Left wheel backward
    runMotor(2, TURN_SPEED, true);   // Right wheel forward

    unsigned long start = millis();
    while (millis() - start < TURN_DURATION) {
        delay(5);
    }

    stopMotors();
    delay(200);

    integral       = 0;
    previous_error = 0;
    lastDistR      = -1.0;
    lastDistL      = -1.0;
}

// ============================================================
void warmupSensors() {
    Serial.println("Warming up sensors...");
    for (int i = 0; i < 20; i++) {
        float r = getDistance(TRIG_RIGHT, ECHO_RIGHT);
        float l = getDistance(TRIG_LEFT,  ECHO_LEFT);
        float f = getDistance(TRIG_FRONT, ECHO_FRONT);
        if (r > 0) lastDistR = r;
        if (l > 0) lastDistL = l;
        Serial.print("Warmup "); Serial.print(i + 1);
        Serial.print("/20 | R:"); Serial.print(r);
        Serial.print(" L:"); Serial.print(l);
        Serial.print(" F:"); Serial.println(f);
        delay(100);
    }
    Serial.println("Sensors ready. Driving...");
}

// ============================================================
void setup() {
    Serial.begin(9600);

    pinMode(M1_EN, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
    pinMode(M2_EN, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);

    pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
    pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);

    pinMode(ENCODER_LEFT,  INPUT_PULLUP);
    pinMode(ENCODER_RIGHT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT),  countLeft,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countRight, CHANGE);

    Serial.println("Ready. Send '1'/'S' to Start, '0'/'X' to Stop.");
    delay(3000);
}

// ============================================================
void loop() {

    // 1. Serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == '1' || cmd == 'S' || cmd == 's') {
            isRunning  = true;
            warmupDone = false;
            integral   = 0;
            previous_error = 0;
            lastDistR  = -1.0;
            lastDistL  = -1.0;
            noInterrupts();
            leftTicks  = 0;
            rightTicks = 0;
            interrupts();
            Serial.println("Starting...");
        } else if (cmd == '0' || cmd == 'X' || cmd == 'x') {
            isRunning = false;
            Serial.println("Stopping...");
        }
    }

    if (!isRunning) {
        stopMotors();
        delay(100);
        return;
    }

    if (!warmupDone) {
        warmupSensors();
        warmupDone = true;
    }

    // 2. Encoder Speed Calculation (every 100ms)
    unsigned long currentTime = millis();
    if (currentTime - lastEncoderTime >= 100) {
        noInterrupts();
        unsigned long currentLeftTicks  = leftTicks;
        unsigned long currentRightTicks = rightTicks;
        leftTicks  = 0;
        rightTicks = 0;
        interrupts();
        leftTicksPerSec  = currentLeftTicks  * 10;
        rightTicksPerSec = currentRightTicks * 10;
        lastEncoderTime  = currentTime;
    }

    // 3. Read sensors
    float distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    float distLeft  = getDistance(TRIG_LEFT,  ECHO_LEFT);
    float distFront = getDistance(TRIG_FRONT, ECHO_FRONT);

    // 4. Spike filter
    if (lastDistR >= 0 && distRight > 0 && distRight > THRESHOLD_DIST)
        distRight = lastDistR;
    if (distRight > 0) lastDistR = distRight;

    if (lastDistL >= 0 && distLeft > 0 && distLeft > THRESHOLD_DIST)
        distLeft = lastDistL;
    if (distLeft > 0) lastDistL = distLeft;

    // 5. Front obstacle → decide turn direction
    //    Priority: RIGHT > LEFT > STOP
    if (distFront > 0 && distFront < STOP_DISTANCE) {

        stopMotors();
        delay(100); // brief pause before deciding

        // Re-read side sensors fresh for the decision
        float freshRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
        float freshLeft  = getDistance(TRIG_LEFT,  ECHO_LEFT);

        Serial.print("Front wall! R:");
        Serial.print(freshRight);
        Serial.print(" L:");
        Serial.println(freshLeft);

        bool rightOpen = (freshRight <= 0 || freshRight > RIGHT_WALL_THRESHOLD);
        bool leftOpen  = (freshLeft  <= 0 || freshLeft  > LEFT_WALL_THRESHOLD);

        if (rightOpen) {
            // RIGHT has priority — no wall or wall is far enough
            turnRight90();
        } else if (leftOpen) {
            // Right is blocked, but left is open
            turnLeft90();
        } else {
            // Both sides blocked — full dead end, stop
            Serial.println("Dead end! No path available.");
            stopMotors();
            isRunning = false; // or add a 180° U-turn here
        }

        return; // skip wall-follow PID this iteration
    }

    // 6. Wall PID Error (right-wall following)
    float error = 0.0;
    if (distRight > 0) {
        if      (distRight < TARGET_MARGIN_MIN) error = TARGET_MARGIN_MIN - distRight;
        else if (distRight > TARGET_MARGIN_MAX) error = TARGET_MARGIN_MAX - distRight;
        else                                    error = 0.0;
    }

    // 7. PID computation
    integral        += error;
    integral         = constrain(integral, -100, 100);
    float derivative = error - previous_error;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previous_error   = error;

    // 8. Motor speeds
    int leftMotorSpeed  = (int)(BASE_SPEED * LEFT_TRIM)  - (int)correction;
    int rightMotorSpeed = (int)(BASE_SPEED * RIGHT_TRIM) + (int)correction;

    leftMotorSpeed  = constrain(leftMotorSpeed,  MIN_SPEED, MAX_SPEED);
    rightMotorSpeed = constrain(rightMotorSpeed, MIN_SPEED, MAX_SPEED);

    float leftRPM  = ((float)leftTicksPerSec  / PPR) * 60.0;
    float rightRPM = ((float)rightTicksPerSec / PPR) * 60.0;

    // 9. Debug output
    Serial.print("F:"); Serial.print(distFront);
    Serial.print(" R:"); Serial.print(distRight);
    Serial.print(" L:"); Serial.print(distLeft);
    Serial.print(" Err:"); Serial.print(error);
    Serial.print(" Corr:"); Serial.print(correction);
    Serial.print(" | L_Hz:"); Serial.print(leftTicksPerSec);
    Serial.print(" R_Hz:"); Serial.print(rightTicksPerSec);
    Serial.print(" | L_PWM:"); Serial.print(leftMotorSpeed);
    Serial.print(" R_PWM:"); Serial.print(rightMotorSpeed);
    Serial.print(" | L_RPM:"); Serial.print(leftRPM);
    Serial.print(" R_RPM:"); Serial.println(rightRPM);

    // 10. Drive
    runMotor(1, leftMotorSpeed,  true);
    runMotor(2, rightMotorSpeed, true);

    delay(50);
}

// ============================================================
float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 0.0;
    return duration * SPEED_OF_SOUND / 2.0;
}

void stopMotors() {
    runMotor(1, 0, true);
    runMotor(2, 0, true);
}

void runMotor(int motorIndex, int speed, bool forward) {
    int enPin, in1Pin, in2Pin;
    if      (motorIndex == 1) { enPin = M1_EN; in1Pin = M1_IN1; in2Pin = M1_IN2; }
    else if (motorIndex == 2) { enPin = M2_EN; in1Pin = M2_IN1; in2Pin = M2_IN2; }
    else return;

    analogWrite(enPin, speed);
    if (speed == 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
    } else {
        digitalWrite(in1Pin, forward ? HIGH : LOW);
        digitalWrite(in2Pin, forward ? LOW  : HIGH);
    }
}