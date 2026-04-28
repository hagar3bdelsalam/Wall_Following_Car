// Motor control pins
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10; // Must be PWM pin

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
}

void loop() {
  // 🔁 Rotate forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200); // Speed (0–255)
  delay(3000);

  // ⛔ Stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(2000);

  // 🔄 Rotate backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);
  delay(3000);

  // ⛔ Stop again
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(2000);
}