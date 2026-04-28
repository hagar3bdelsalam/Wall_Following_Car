// Motor control pins
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10;  // Must be PWM pin
const int IN3 = 13;
const int IN4 = 12;
const int ENB = 11;  // Must be PWM pin
int flag;
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  flag = 1;
}

void loop() {
  if (flag) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(10000);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 100);  // Speed (0–255)
    analogWrite(ENB, 100);  // Speed (0–255)
    delay(10000);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(2000);

    flag = 0;
  }
}