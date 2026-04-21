#include <stdint.h>
#include <math.h>

/* ─────────────────────────────────────────────
   CONFIGURATIONS & THRESHOLDS
───────────────────────────────────────────── */

constexpr float WALL_FRONT_STOP_CM = 2.0f;
constexpr float WALL_FRONT_CLEAR_CM = 4.0f;

constexpr int BASE_SPEED = 180;
constexpr int TURN_SPEED = 160;

constexpr int32_t TURN_90_TICKS = 200;

constexpr int IR_WALL_DETECTED = 0;
constexpr int IR_CLEAR = 1;

/* ─────────────────────────────────────────────
   PIN DEFINITIONS (Adjust for your specific AVR board)
───────────────────────────────────────────── */

constexpr uint8_t pinMotorLIn1 = 6;
constexpr uint8_t pinMotorLIn2 = 7;
constexpr uint8_t pinMotorLPwm = 9;
constexpr uint8_t pinEncLA = 2;
constexpr uint8_t pinEncLB = 4;

constexpr uint8_t pinMotorRIn3 = 8;
constexpr uint8_t pinMotorRIn4 = 12;
constexpr uint8_t pinMotorRPwm = 10;
constexpr uint8_t pinEncRA = 3;
constexpr uint8_t pinEncRB = 5;

constexpr uint8_t pinTrig = 11;
constexpr uint8_t pinEcho = 13;

constexpr uint8_t pinIrLeft = A0;
constexpr uint8_t pinIrRight = A1;

/* ─────────────────────────────────────────────
   TYPES & ALIASES
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

static int turnCount = 0;
static char turnSequence[64];
static int seqIndex = 0;

static int32_t turnStartLeft = 0;
static int32_t turnStartRight = 0;

volatile int32_t encLeftTicks = 0;
volatile int32_t encRightTicks = 0;

/* ─────────────────────────────────────────────
   INTERRUPT SERVICE ROUTINES
───────────────────────────────────────────── */

void isrEncLeft(void)
{
    digitalRead(pinEncLB) ? encLeftTicks++ : encLeftTicks--;
}

void isrEncRight(void)
{
    digitalRead(pinEncRB) ? encRightTicks++ : encRightTicks--;
}

/* ─────────────────────────────────────────────
   HARDWARE ABSTRACTIONS
───────────────────────────────────────────── */

void motorInit(void)
{
    pinMode(pinMotorLIn1, OUTPUT);
    pinMode(pinMotorLIn2, OUTPUT);
    pinMode(pinMotorLPwm, OUTPUT);
    pinMode(pinMotorRIn3, OUTPUT);
    pinMode(pinMotorRIn4, OUTPUT);
    pinMode(pinMotorRPwm, OUTPUT);
}

void encoderInit(void)
{
    pinMode(pinEncLA, INPUT);
    pinMode(pinEncRA, INPUT);
    pinMode(pinEncLB, INPUT);
    pinMode(pinEncRB, INPUT);

    attachInterrupt(digitalPinToInterrupt(pinEncLA), isrEncLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncRA), isrEncRight, CHANGE);
}

void motorSet(const int leftSpeed, const int rightSpeed)
{
    digitalWrite(pinMotorLIn1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(pinMotorLIn2, leftSpeed < 0 ? HIGH : LOW);
    analogWrite(pinMotorLPwm, abs(leftSpeed));

    digitalWrite(pinMotorRIn3, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(pinMotorRIn4, rightSpeed < 0 ? HIGH : LOW);
    analogWrite(pinMotorRPwm, abs(rightSpeed));
}

float ultrasonicReadCm(void)
{
    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);

    const long duration = pulseIn(pinEcho, HIGH, 30000);
    return duration * 0.034f / 2.0f;
}

static inline int irReadLeft(void)
{
    return digitalRead(pinIrLeft);
}

static inline int irReadRight(void)
{
    return digitalRead(pinIrRight);
}

void sendReport(void)
{
    Serial.print("\n--- TURN REPORT ---\n");
    Serial.print("Turns: ");
    Serial.println(turnCount);
    Serial.print("Sequence: ");
    Serial.println(turnSequence);
    Serial.print("-------------------\n");
}

void recordTurn(const char direction)
{
    turnCount++;
    if (seqIndex > 0)
    {
        turnSequence[seqIndex++] = ',';
        turnSequence[seqIndex++] = ' ';
    }
    turnSequence[seqIndex++] = direction;
    turnSequence[seqIndex] = '\0';

    Serial.print("[TURN #");
    Serial.print(turnCount);
    Serial.print("] Direction: ");
    Serial.println(direction);
}

/* ─────────────────────────────────────────────
   CORE LOGIC
───────────────────────────────────────────── */

void fsmUpdate(void)
{
    const float distCm = ultrasonicReadCm();
    const int irLeft = irReadLeft();
    const int irRight = irReadRight();

    const bool wallAhead = (distCm > 0.0f && distCm < WALL_FRONT_STOP_CM);
    const bool leftOpen = (irLeft == IR_CLEAR);
    const bool rightOpen = (irRight == IR_CLEAR);

    const int32_t leftDiff = abs(encLeftTicks - turnStartLeft);
    const int32_t rightDiff = abs(encRightTicks - turnStartRight);

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

            currentState = rightOpen ? RobotState::TurnRight : leftOpen ? RobotState::TurnLeft
                                                                        : RobotState::Done;

            if (currentState != RobotState::Done)
            {
                recordTurn(currentState == RobotState::TurnRight ? 'R' : 'L');
            }
        }
        else if (rightOpen && distCm > WALL_FRONT_CLEAR_CM)
        {
            recordTurn('R');
            currentState = RobotState::TurnRight;
        }

        turnStartLeft = encLeftTicks;
        turnStartRight = encRightTicks;
        break;
    }

    case RobotState::TurnLeft:
    {
        motorSet(-TURN_SPEED, TURN_SPEED);

        if (leftDiff >= TURN_90_TICKS || rightDiff >= TURN_90_TICKS)
        {
            motorSet(0, 0);
            currentState = RobotState::Forward;
        }
        break;
    }

    case RobotState::TurnRight:
    {
        motorSet(TURN_SPEED, -TURN_SPEED);

        if (leftDiff >= TURN_90_TICKS || rightDiff >= TURN_90_TICKS)
        {
            motorSet(0, 0);
            currentState = RobotState::Forward;
        }
        break;
    }

    case RobotState::Done:
    {
        motorSet(0, 0);
        sendReport();

        /* Suspend execution after reporting */
        while (true)
        {
            delay(100);
        }
        break;
    }
    }
}

/* ─────────────────────────────────────────────
   ENTRY POINTS
───────────────────────────────────────────── */

void setup(void)
{
    Serial.begin(115200);

    pinMode(pinTrig, OUTPUT);
    pinMode(pinEcho, INPUT);
    pinMode(pinIrLeft, INPUT);
    pinMode(pinIrRight, INPUT);

    motorInit();
    encoderInit();

    turnSequence[0] = '\0';

    Serial.println("=== Wall-Following Robot READY ===");
    Serial.println("Send 'START' via Serial to begin.");
    currentState = RobotState::Idle;
}

void loop(void)
{
    fsmUpdate();
}