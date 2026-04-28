// 3x HC-SR04 Ultrasonic Sensor Test
// Sensor 1: Trig=D2, Echo=D3
// Sensor 2: Trig=D4, Echo=D5
// Sensor 3: Trig=D6, Echo=D7

const int trigPin = 2;
const int echoPin = 3;

void setup() {
  Serial.begin(9600);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  Serial.println("HC-SR04 — Distance Test");
  Serial.println("----------------------------");
}

long getDistance(int trigPin, int echoPin) {
  // Send 10µs pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo duration (timeout at 30ms ~ 5m)
  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) return -1;  // No echo / out of range

  return duration * 0.034 / 2;  // Convert to cm
}

void loop() {
    long dist = getDistance(trigPin, echoPin);

    if (dist == -1) {
      Serial.println("Out of range");
    } else {
      Serial.print(dist);
      Serial.println(" cm");
    }

  Serial.println("----------------------------");
  delay(500);
}