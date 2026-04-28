// HC-05 Bluetooth Module Test
// Tests basic communication via Serial Monitor <-> HC-05 <-> Bluetooth device

// Wiring:
//   HC-05 VCC  -> 5V
//   HC-05 GND  -> GND
//   HC-05 TXD  -> Arduino Pin 10 (SoftwareSerial RX)
//   HC-05 RXD  -> Arduino Pin 11 (SoftwareSerial TX) via voltage divider
//   HC-05 STATE -> Optional, Pin 13 (LED indicator)

#include <SoftwareSerial.h>

#define BT_RX_PIN  10   // Arduino RX <- HC-05 TX
#define BT_TX_PIN  11   // Arduino TX -> HC-05 RX
#define STATE_PIN  13   // Optional: HC-05 STATE pin (built-in LED)

SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

void setup() {
  Serial.begin(9600);       // Serial Monitor baud rate
  BTSerial.begin(9600);     // HC-05 default baud rate (38400 in AT mode)

  pinMode(STATE_PIN, INPUT);

  Serial.println("=== HC-05 Bluetooth Module Test ===");
  Serial.println("Baud Rate : 9600");
  Serial.println("-----------------------------------");
  Serial.println("Type a message and press Enter to send via Bluetooth.");
  Serial.println("Incoming BT messages will appear below.");
  Serial.println("===================================\n");
}

void loop() {
  // Forward data from Serial Monitor -> HC-05 (TX to paired device)
  if (Serial.available()) {
    char c = Serial.read();
    BTSerial.write(c);
    Serial.write(c); // Echo back to monitor
  }

  // Forward data from HC-05 -> Serial Monitor (RX from paired device)
  if (BTSerial.available()) {
    char c = BTSerial.read();
    Serial.write(c);
  }

  // Optional: Show connection state via built-in LED
  // HC-05 STATE pin = HIGH when connected, blinks when waiting
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 1000) {
    lastCheck = millis();
    int state = digitalRead(STATE_PIN);
    Serial.print("[STATUS] HC-05 State Pin: ");
    Serial.println(state == HIGH ? "CONNECTED" : "NOT CONNECTED / PAIRING");
  }
}