#include <SPI.h>
#include <RH_RF95.h>

// Singleton instance of the radio driver
RH_RF95 SX1278;  // <--- usually RH_RF95 rf95(10, 2); on Uno - check your pins!

#define ALARM 8
#define LIGHT 9
#define LED 6
#define LED1 5

void onSequence() {
  digitalWrite(LED, HIGH);
  digitalWrite(LED1, LOW);
  delay(200);
  digitalWrite(LED, LOW);
  digitalWrite(LED1, HIGH);
  delay(200);
  digitalWrite(LED, HIGH);
  digitalWrite(LED1, HIGH);
  delay(300);
  digitalWrite(LED, LOW);
  digitalWrite(LED1, LOW);
  delay(300);
  digitalWrite(LED, HIGH);
  digitalWrite(LED1, HIGH);
  delay(300);
  digitalWrite(LED, LOW);
  digitalWrite(LED1, LOW);
}

void setup() {
  Serial.begin(9600);
  delay(100);

  pinMode(LED, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(ALARM, OUTPUT);
  pinMode(LIGHT, OUTPUT);

  digitalWrite(LED, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(ALARM, LOW);
  digitalWrite(LIGHT, LOW);

  onSequence();

  Serial.println();
  Serial.println("LoRa Receiver starting...");

  // Most common pins on Arduino Uno/Nano: CS=10, IRQ=2
  // If you're using different pins → change here:
  // RH_RF95 SX1278(10, 2);     // CS, IRQ

  if (!SX1278.init()) {
    Serial.println("LoRa init failed !!! Check wiring / module");
    while (true) {
      digitalWrite(LED, !digitalRead(LED));
      delay(200);
    }
  }

  // Optional: set your frequency, power, etc.
  // SX1278.setFrequency(433.0);
  // SX1278.setTxPower(13);   // 5..23 dBm

  Serial.println("LoRa init OK");
  Serial.println("Waiting for packets...");
}

void loop() {
  digitalWrite(LED, HIGH);
  if (SX1278.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);  // ← important!

    if (SX1278.recv(buf, &len)) {  // ← pass pointer to length
      // ───────────────────────────────────────────────
      // Print EVERYTHING we received
      Serial.println("──────────────────────────────");
      Serial.print("Received ");
      Serial.print(len);
      Serial.print(" bytes   RSSI: ");
      Serial.print(SX1278.lastRssi());
      Serial.print(" dBm   SNR: ");
      Serial.print(SX1278.lastSNR());
      Serial.println(" dB");

      // Print as HEX + ASCII
      Serial.print("HEX: ");
      for (uint8_t i = 0; i < len; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      Serial.print("ASCII: ");
      for (uint8_t i = 0; i < len; i++) {
        if (buf[i] >= 32 && buf[i] <= 126) {
          Serial.print((char)buf[i]);
        } else {
          Serial.print(".");
        }
      }
      Serial.println();
      // ───────────────────────────────────────────────

      // Your original logic
      if (len >= 1 && buf[0] == '1') {
        Serial.println("→ Control signal '1' received!");

        digitalWrite(LED, HIGH);
        digitalWrite(LED1, HIGH);
        delay(1000);
        digitalWrite(LED, LOW);
        digitalWrite(LED1, LOW);
        delay(100);

        // Beep + light pattern (4 long + short sequence)
        for (int i = 0; i < 4; i++) {
          digitalWrite(ALARM, HIGH);
          digitalWrite(LIGHT, HIGH);
          delay(2000);
          digitalWrite(ALARM, LOW);
          digitalWrite(LIGHT, LOW);
          delay(100);
        }

        // Final short beep + light off
        digitalWrite(ALARM, HIGH);
        digitalWrite(LIGHT, LOW);
        delay(500);
        digitalWrite(ALARM, LOW);
        digitalWrite(LIGHT, LOW);

        digitalWrite(LED, LOW);
        digitalWrite(LED1, LOW);

        Serial.println("→ Sequence completed");
      } else {
        Serial.println("→ Message received but not '1'");
      }

      Serial.println("──────────────────────────────");
    } else {
      Serial.println("recv() failed (corrupt packet?)");
    }
  }

  // Optional: small delay or LowPower if you want to save energy
  // LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
}