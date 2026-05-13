
//  PINS 
const int M2_EN  = 10;
const int M2_IN1 = 9;
const int M2_IN2 = 8;

const int M1_EN  = 11;
const int M1_IN1 = 12;
const int M1_IN2 = 13;

const int TRIG_FRONT = 5;  const int ECHO_FRONT = 4;
const int TRIG_RIGHT = 7;  const int ECHO_RIGHT = 6;
const int TRIG_LEFT  = A4; const int ECHO_LEFT  = A5;

const int ENCODER_LEFT  = 3;
const int ENCODER_RIGHT = 2;

//  LIVE TUNABLE VARIABLES 
int BASE_SPEED = 120; 
float Kp = 2.0;       
float Ki = 0.0;
float Kd = 10.0;      

// Tweak these individually to get exactly 90 degrees on each side!
int TURN_TICKS_RIGHT = 35;  
int TURN_TICKS_LEFT  = 30;

float LEFT_TRIM  = 0.84; //0.885
float RIGHT_TRIM = 0.865; // 0.84

//  HARDWARE SETTINGS 
const int MAX_SPEED  = 200;
const int MIN_SPEED  = 0;

const float TARGET_CENTER = 15.0; 
const int   STOP_DISTANCE = 30;
const float SPEED_OF_SOUND = 0.0343;

const float RIGHT_WALL_THRESHOLD = 30.0;
const float LEFT_WALL_THRESHOLD  = 30.0;
const int TURN_SPEED = 100;

//  STATE & SENSOR VARIABLES 
float previous_error = 0;
float integral       = 0;

float lastDistR = -1.0;
float lastDistL = -1.0;
const float THRESHOLD_DIST = 50.0;

bool isRunning = false;
bool warmupDone = false;

volatile unsigned long leftTicks  = 0;
volatile unsigned long rightTicks = 0;
volatile unsigned long lastLeftMicros  = 0;
volatile unsigned long lastRightMicros = 0;
const unsigned long DEBOUNCE_TIME = 500; // 500 microseconds to stop jitter

//  INTERRUPTS (DEBOUNCED) 
void countLeft() {
    unsigned long now = micros();
    if (now - lastLeftMicros > DEBOUNCE_TIME) {
        leftTicks++;
        lastLeftMicros = now;
    }
}

void countRight() {
    unsigned long now = micros();
    if (now - lastRightMicros > DEBOUNCE_TIME) {
        rightTicks++;
        lastRightMicros = now;
    }
}

//  MOVEMENT SEQUENCES & TURNS 
void goForward(int seconds) {
    Serial.print(">>> Moving Forward for "); Serial.print(seconds); Serial.println("s.");
    runMotor(1, (int)(BASE_SPEED * LEFT_TRIM), true);
    runMotor(2, (int)(BASE_SPEED * RIGHT_TRIM), true);
    delay(seconds * 1000UL); 
    stopMotors();
    Serial.println(">>> Done. Stopped.");
}

// Pivot Turn: Left Wheel Drives, Right Wheel Dead
void turnRight90() {
    Serial.println(">>> Turning RIGHT (Pivot Turn)");
    noInterrupts(); leftTicks = 0; interrupts();

    runMotor(1, TURN_SPEED, true);
    runMotor(2, 0, true);

    while (leftTicks < TURN_TICKS_RIGHT) {
        // LOGGING ADDED HERE
        Serial.print("Turning Right - Left Ticks: ");
        Serial.println(leftTicks);
        delay(20); // 20ms pause so we don't spam the serial monitor too hard
    }

    stopMotors(); delay(200); 
    integral = 0; previous_error = 0; lastDistR = -1.0; lastDistL = -1.0;
}

// Pivot Turn: Right Wheel Drives, Left Wheel Dead
void turnLeft90() {
    Serial.println(">>> Turning LEFT (Pivot Turn)");
    noInterrupts(); rightTicks = 0; interrupts();

    runMotor(1, 0, true);
    runMotor(2, TURN_SPEED, true);

    while (rightTicks < TURN_TICKS_LEFT) {
        // LOGGING ADDED HERE
        Serial.print("Turning Left - Right Ticks: ");
        Serial.println(rightTicks);
        delay(20); 
    }

    stopMotors(); delay(200);
    integral = 0; previous_error = 0; lastDistR = -1.0; lastDistL = -1.0;
}

// Full Sequences
void runSequenceRight() {
    int leftSpeed  = (int)(BASE_SPEED * LEFT_TRIM);
    int rightSpeed = (int)(BASE_SPEED * RIGHT_TRIM);

    Serial.println(">>> RIGHT Sequence: 1. Moving Forward (2s)");
    runMotor(1, leftSpeed, true); runMotor(2, rightSpeed, true);
    delay(2000);

    stopMotors(); delay(200);
    turnRight90(); 

    Serial.println(">>> RIGHT Sequence: 3. Moving Forward (1s)");
    runMotor(1, leftSpeed, true); runMotor(2, rightSpeed, true);
    delay(1000);

    Serial.println(">>> Sequence Complete. Stopped.");
    stopMotors();
}

void runSequenceLeft() {
    int leftSpeed  = (int)(BASE_SPEED * LEFT_TRIM);
    int rightSpeed = (int)(BASE_SPEED * RIGHT_TRIM);

    Serial.println(">>> LEFT Sequence: 1. Moving Forward (2s)");
    runMotor(1, leftSpeed, true); runMotor(2, rightSpeed, true);
    delay(2000);

    stopMotors(); delay(200);
    turnLeft90(); 

    Serial.println(">>> LEFT Sequence: 3. Moving Forward (1s)");
    runMotor(1, leftSpeed, true); runMotor(2, rightSpeed, true);
    delay(1000);

    Serial.println(">>> Sequence Complete. Stopped.");
    stopMotors();
}

void warmupSensors() {
    Serial.println("Warming up sensors...");
    for (int i = 0; i < 20; i++) {
        float r = getDistance(TRIG_RIGHT, ECHO_RIGHT);
        float l = getDistance(TRIG_LEFT,  ECHO_LEFT);
        if (r > 0) lastDistR = r;
        if (l > 0) lastDistL = l;
        delay(10); 
    }
    Serial.println("Sensors ready. Send commands.");
}

//  SETUP 
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

    Serial.println(" MASTER CAR SCRIPT READY ");
    Serial.println("'1' = Run Continuous PID | '0' = Stop");
    Serial.println("'R' = Run Right Sequence | 'L' = Run Left Sequence");
    Serial.println("'TR45' = Right Ticks to 45 | 'TL38' = Left Ticks to 38");
    Serial.println("'T40' = Set BOTH Ticks to 40");
    Serial.println("'P2.5' = Set Kp to 2.5 | 'D15' = Set Kd to 15");
    Serial.println("'dcr0.85' = Set Right Trim | 'dcl0.88' = Set Left Trim");
    Serial.println("'F3' = Move Forward 3s | '?' = Show Current Settings");
    delay(3000);
}

//  MAIN LOOP 
void loop() {

    // 1. Serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        if (cmd == '1' || cmd == 'S' || cmd == 's') {
            isRunning = true; warmupDone = false;
            integral = 0; previous_error = 0; lastDistR = -1.0; lastDistL = -1.0;
            noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts(); // Reset ticks for forward driving
            Serial.println(">>> Running Continuous PID...");
        } 
        else if (cmd == '0' || cmd == 'X' || cmd == 'x') {
            isRunning = false; stopMotors(); Serial.println(">>> Stopped.");
        } 
        else if (cmd == 'R' || cmd == 'r') {
            isRunning = false; runSequenceRight(); 
        }
        else if (cmd == 'L' || cmd == 'l') {
            isRunning = false; runSequenceLeft(); 
        }
        else if (cmd == 'F' || cmd == 'f') {
            isRunning = false; 
            int seconds = Serial.parseInt();
            if (seconds > 0) goForward(seconds);
        }
        else if (cmd == 'T' || cmd == 't') {
            delay(5); // Give the Arduino a millisecond to read the next letter
            char nextChar = Serial.peek();
            
            if (nextChar == 'R' || nextChar == 'r') {
                Serial.read(); // consume the 'R'
                TURN_TICKS_RIGHT = Serial.parseInt();
                Serial.print(">>> Right Turn Ticks updated to: "); Serial.println(TURN_TICKS_RIGHT);
            } 
            else if (nextChar == 'L' || nextChar == 'l') {
                Serial.read(); // consume the 'L'
                TURN_TICKS_LEFT = Serial.parseInt();
                Serial.print(">>> Left Turn Ticks updated to: "); Serial.println(TURN_TICKS_LEFT);
            } 
            else {
                int val = Serial.parseInt();
                TURN_TICKS_RIGHT = val;
                TURN_TICKS_LEFT = val;
                Serial.print(">>> BOTH Turn Ticks updated to: "); Serial.println(val);
            }
        }
        else if (cmd == 'P' || cmd == 'p') {
            Kp = Serial.parseFloat();
            Serial.print(">>> Kp updated to: "); Serial.println(Kp);
        }
        else if (cmd == 'D' || cmd == 'd') {
            delay(5); // Wait for potential 'c' to enter serial buffer
            char nextChar = Serial.peek();
            
            if (nextChar == 'C' || nextChar == 'c') {
                Serial.read(); // consume 'c'
                delay(5);
                char thirdChar = Serial.read(); // consume 'r' or 'l'
                
                if (thirdChar == 'R' || thirdChar == 'r') {
                    RIGHT_TRIM = Serial.parseFloat();
                    Serial.print(">>> Right Trim updated to: "); Serial.println(RIGHT_TRIM, 3);
                } 
                else if (thirdChar == 'L' || thirdChar == 'l') {
                    LEFT_TRIM = Serial.parseFloat();
                    Serial.print(">>> Left Trim updated to: "); Serial.println(LEFT_TRIM, 3);
                }
            } 
            else {
                // Keep backward compatibility for standard 'D' command to update Kd
                Kd = Serial.parseFloat();
                Serial.print(">>> Kd updated to: "); Serial.println(Kd);
            }
        }
        else if (cmd == 'V' || cmd == 'v') {
            BASE_SPEED = Serial.parseInt();
            Serial.print(">>> Base Speed updated to: "); Serial.println(BASE_SPEED);
        }
        else if (cmd == '?') {
            Serial.print(">>> SETTINGS -> Kp:"); Serial.print(Kp);
            Serial.print(" | Kd:"); Serial.print(Kd);
            Serial.print(" | Speed:"); Serial.print(BASE_SPEED);
            Serial.print(" | Ticks Right:"); Serial.print(TURN_TICKS_RIGHT);
            Serial.print(" | Ticks Left:"); Serial.print(TURN_TICKS_LEFT);
            Serial.print(" | L_Trim:"); Serial.print(LEFT_TRIM, 3);
            Serial.print(" | R_Trim:"); Serial.println(RIGHT_TRIM, 3);
        }
    }

    if (!isRunning) {
        stopMotors();
        delay(100);
        return;
    }

    if (!warmupDone) { warmupSensors(); warmupDone = true; }

    // 2. Read sensors
    float distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    float distLeft  = getDistance(TRIG_LEFT,  ECHO_LEFT);
    float distFront = getDistance(TRIG_FRONT, ECHO_FRONT);

    // 3. Spike filter
    if (lastDistR >= 0 && distRight > 0 && distRight > THRESHOLD_DIST) distRight = lastDistR;
    if (distRight > 0) lastDistR = distRight;
    if (lastDistL >= 0 && distLeft > 0 && distLeft > THRESHOLD_DIST) distLeft = lastDistL;
    if (distLeft > 0) lastDistL = distLeft;

    // 4. Front obstacle avoidance (Triggers Pivot Turns)
    if (distFront > 0 && distFront < STOP_DISTANCE) {
        stopMotors(); delay(100); 
        float freshRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
        float freshLeft  = getDistance(TRIG_LEFT,  ECHO_LEFT);
        
        bool rightOpen = (freshRight <= 0 || freshRight > RIGHT_WALL_THRESHOLD);
        bool leftOpen  = (freshLeft  <= 0 || freshLeft  > LEFT_WALL_THRESHOLD);

        if (rightOpen) turnRight90();
        else if (leftOpen) turnLeft90();
        else { Serial.println("Dead end! Stopping."); isRunning = false; }
        return; 
    }

    // 5. PID Center Tracking Math
    float error = 0.0;
    bool rightValid = (distRight > 0 && distRight < 45.0);
    bool leftValid  = (distLeft > 0 && distLeft < 45.0);

    if (leftValid && rightValid) { error = (distLeft - distRight) / 2.0; } 
    else if (rightValid)         { error = TARGET_CENTER - distRight; } 
    else if (leftValid)          { error = distLeft - TARGET_CENTER; }

    integral += error; integral = constrain(integral, -100, 100);
    float derivative = error - previous_error;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previous_error = error;

    int leftMotorSpeed  = (int)(BASE_SPEED * LEFT_TRIM)  - (int)correction;
    int rightMotorSpeed = (int)(BASE_SPEED * RIGHT_TRIM) + (int)correction;
    
    leftMotorSpeed  = constrain(leftMotorSpeed,  MIN_SPEED, MAX_SPEED);
    rightMotorSpeed = constrain(rightMotorSpeed, MIN_SPEED, MAX_SPEED);

    // 6. Debug Print
    Serial.print("F: "); Serial.print(distFront);
    Serial.print(" | L: "); Serial.print(distLeft);
    Serial.print(" | R: "); Serial.print(distRight);
    Serial.print(" | Err: "); Serial.print(error);
    Serial.print(" | Corr: "); Serial.print(correction);
    Serial.print(" | L_PWM: "); Serial.print(leftMotorSpeed);
    Serial.print(" | R_PWM: "); Serial.print(rightMotorSpeed);
    Serial.print(" | L_Tick: "); Serial.print(leftTicks);
    Serial.print(" | R_Tick: "); Serial.println(rightTicks);

    // 7. Drive Motors
    runMotor(1, leftMotorSpeed,  true);
    runMotor(2, rightMotorSpeed, true);
    delay(50);
}

//  HELPERS 
float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 0.0;
    return duration * SPEED_OF_SOUND / 2.0;
}

void stopMotors() { runMotor(1, 0, true); runMotor(2, 0, true); }

void runMotor(int motorIndex, int speed, bool forward) {
    int enPin, in1Pin, in2Pin;
    if      (motorIndex == 1) { enPin = M1_EN; in1Pin = M1_IN1; in2Pin = M1_IN2; }
    else if (motorIndex == 2) { enPin = M2_EN; in1Pin = M2_IN1; in2Pin = M2_IN2; }
    else return;
    
    analogWrite(enPin, speed);
    if (speed == 0) { 
        digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, LOW); 
    } else { 
        digitalWrite(in1Pin, forward ? HIGH : LOW); digitalWrite(in2Pin, forward ? LOW : HIGH); 
    }
}