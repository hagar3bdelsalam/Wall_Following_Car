#include <stdint.h>

/* ─────────────────────────────────────────────
   CONFIGURATIONS & THRESHOLDS
───────────────────────────────────────────── */

constexpr float WALL_FRONT_STOP_CM  = 15.0f;
constexpr float WALL_FRONT_CLEAR_CM = 20.0f;
constexpr float WALL_SIDE_OPEN_CM   = 20.0f;

constexpr int BASE_SPEED  = 180;
constexpr int TURN_SPEED  = 160;

constexpr unsigned long TURN_90_MS = 500UL;  // tune this for your robot

/* ─────────────────────────────────────────────
   PIN DEFINITIONS
───────────────────────────────────────────── */

// Left motor
constexpr uint8_t pinMotorLIn1 = 6;
constexpr uint8_t pinMotorLIn2 = 7;
constexpr uint8_t pinMotorLEn  = 9;

// Right motor
constexpr uint8_t pinMotorRIn1 = 8;
constexpr uint8_t pinMotorRIn2 = 12;
constexpr uint8_t pinMotorREn  = 10;

// Ultrasonic sensors
constexpr uint8_t pinTrigFront = 11;
constexpr uint8_t pinEchoFront = 13;

constexpr uint8_t pinTrigLeft  = A2;
constexpr uint8_t pinEchoLeft  = A3;

constexpr uint8_t pinTrigRight = A0;
constexpr uint8_t pinEchoRight = A1;

/* ─────────────────────────────────────────────
   TYPES
───────────────────────────────────────────── */

enum class RobotState : uint8_t
{
    Idle,
    Forward,
    TurnLeft,
    TurnRight,
    Done
};

/* ─────────────────────────────────────────────
   GLOBAL STATE
───────────────────────────────────────────── */

static RobotState currentState = RobotState::Idle;

static int  turnCount         = 0;
static char turnSequence[128] = {0};
static int  seqIndex          = 0;

static unsigned long turnStartMs = 0;  // timestamp when a turn began

/* ─────────────────────────────────────────────
   MOTOR CONTROL
───────────────────────────────────────────── */

void motorInit()
{
    pinMode(pinMotorLIn1, OUTPUT);
    pinMode(pinMotorLIn2, OUTPUT);
    pinMode(pinMotorLEn,  OUTPUT);
    pinMode(pinMotorRIn1, OUTPUT);
    pinMode(pinMotorRIn2, OUTPUT);
    pinMode(pinMotorREn,  OUTPUT);
}

void motorDrive(uint8_t in1, uint8_t in2, uint8_t en, int speed)
{
    if (speed > 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if (speed < 0)
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    analogWrite(en, abs(speed));
}

void motorSet(int leftSpeed, int rightSpeed)
{
    motorDrive(pinMotorLIn1, pinMotorLIn2, pinMotorLEn,  leftSpeed);
    motorDrive(pinMotorRIn1, pinMotorRIn2, pinMotorREn, rightSpeed);
}

/* ─────────────────────────────────────────────
   ULTRASONIC
───────────────────────────────────────────── */

float ultrasonicReadCm(uint8_t trigPin, uint8_t echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    const long duration = pulseIn(echoPin, HIGH, 30000UL);
    if (duration == 0) return 999.0f;
    return duration * 0.034f / 2.0f;
}

/* ─────────────────────────────────────────────
   TURN LOGGING
───────────────────────────────────────────── */

void recordTurn(char direction)
{
    turnCount++;
    if (seqIndex > 0)
    {
        turnSequence[seqIndex++] = ',';
        turnSequence[seqIndex++] = ' ';
    }
    turnSequence[seqIndex++] = direction;
    turnSequence[seqIndex]   = '\0';

    Serial.print("[TURN #");
    Serial.print(turnCount);
    Serial.print("] Direction: ");
    Serial.println(direction);
}

void sendReport()
{
    Serial.println("\n--- TURN REPORT ---");
    Serial.print("Total turns : "); Serial.println(turnCount);
    Serial.print("Sequence    : "); Serial.println(turnSequence);
    Serial.println("-------------------");
}

/* ─────────────────────────────────────────────
   FSM
───────────────────────────────────────────── */

void fsmUpdate()
{
    const float distFront = ultrasonicReadCm(pinTrigFront, pinEchoFront);
    const float distLeft  = ultrasonicReadCm(pinTrigLeft,  pinEchoLeft);
    const float distRight = ultrasonicReadCm(pinTrigRight, pinEchoRight);

    const bool wallAhead = (distFront < WALL_FRONT_STOP_CM);
    const bool leftOpen  = (distLeft  > WALL_SIDE_OPEN_CM);
    const bool rightOpen = (distRight > WALL_SIDE_OPEN_CM);

    switch (currentState)
    {
    case RobotState::Idle:
    {
        motorSet(0, 0);
        if (Serial.available())
        {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if (cmd == "START")
            {
                Serial.println("[FSM] Starting!");
                currentState = RobotState::Forward;
            }
        }
        break;
    }

    case RobotState::Forward:
    {
        motorSet(BASE_SPEED, BASE_SPEED);

        if (wallAhead)
        {
            motorSet(0, 0);
            delay(100);

            if (rightOpen)
            {
                recordTurn('R');
                motorSet(0, 0);
                turnStartMs  = millis();
                currentState = RobotState::TurnRight;
            }
            else if (leftOpen)
            {
                recordTurn('L');
                motorSet(0, 0);
                turnStartMs  = millis();
                currentState = RobotState::TurnLeft;
            }
            else
            {
                currentState = RobotState::Done;
            }
        }
        else if (rightOpen && distFront > WALL_FRONT_CLEAR_CM)
        {
            recordTurn('R');
            motorSet(0, 0);
            delay(100);
            turnStartMs  = millis();
            currentState = RobotState::TurnRight;
        }
        break;
    }

    case RobotState::TurnLeft:
    {
        motorSet(-TURN_SPEED, TURN_SPEED);

        if (millis() - turnStartMs >= TURN_90_MS)
        {
            motorSet(0, 0);
            delay(100);
            currentState = RobotState::Forward;
        }
        break;
    }

    case RobotState::TurnRight:
    {
        motorSet(TURN_SPEED, -TURN_SPEED);

        if (millis() - turnStartMs >= TURN_90_MS)
        {
            motorSet(0, 0);
            delay(100);
            currentState = RobotState::Forward;
        }
        break;
    }

    case RobotState::Done:
    {
        motorSet(0, 0);
        sendReport();
        while (true) { delay(100); }
        break;
    }
    }
}

/* ─────────────────────────────────────────────
   ENTRY POINTS
───────────────────────────────────────────── */

void setup()
{
    Serial.begin(115200);

    pinMode(pinTrigFront, OUTPUT); pinMode(pinEchoFront, INPUT);
    pinMode(pinTrigLeft,  OUTPUT); pinMode(pinEchoLeft,  INPUT);
    pinMode(pinTrigRight, OUTPUT); pinMode(pinEchoRight, INPUT);

    motorInit();

    Serial.println("=== Wall-Following Robot READY ===");
    Serial.println("Send 'START' via Serial to begin.");
    currentState = RobotState::Idle;
}

void loop()
{
    fsmUpdate();
}