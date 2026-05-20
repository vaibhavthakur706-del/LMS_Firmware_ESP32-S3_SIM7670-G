// ============================================================
//  RainGauge_ImmediateWake.ino
//  On EXT1 wake (rain tip) increment count immediately and sleep
// ============================================================

#include "driver/rtc_io.h"
#include <Arduino.h>

#define RAIN_PIN         GPIO_NUM_16
#define RAIN_MM_PER_TIP  0.2794
#define BAUD_RATE        115200

// Use deep sleep behavior
#define USE_DEEP_SLEEP   true

// RTC persistent counter
RTC_DATA_ATTR unsigned long rainCount = 0;

// Volatile ISR state (kept for compatibility)
volatile unsigned long isrCount = 0;
volatile unsigned long lastTipMicros = 0;
volatile bool newTip = false;

// Debounce for awake-mode ISR (keeps ISR safe if used)
#define DEBOUNCE_US 200000UL

void IRAM_ATTR rainISR() {
  unsigned long now = micros();
  unsigned long delta = now - lastTipMicros;
  if (delta > DEBOUNCE_US) {
    lastTipMicros = now;
    isrCount++;
    rainCount++;
    newTip = true;
  }
}

void printStatus(unsigned long count) {
  float mm = count * RAIN_MM_PER_TIP;
  Serial.println("─────────────────────────────");
  Serial.printf ("  Tips : %lu\n", count);
  Serial.printf ("  Rain : %.4f mm\n", mm);
  Serial.println("─────────────────────────────");
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(200);

  Serial.println("\n=== RainGauge Immediate Wake ===");
  Serial.printf("Pin: GPIO%d  Mode: %s\n", (int)RAIN_PIN, USE_DEEP_SLEEP ? "DEEP SLEEP" : "AWAKE");

  // Keep your requested pin settings exactly:
  pinMode(RAIN_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainISR, RISING);

  // Configure RTC pull to match pinMode so EXT1 sees same idle state
  rtc_gpio_pullup_dis((gpio_num_t)RAIN_PIN);
  rtc_gpio_pulldown_en((gpio_num_t)RAIN_PIN);

  if (USE_DEEP_SLEEP) {
    uint64_t mask = 1ULL << RAIN_PIN;
    // Wake when pin goes HIGH (RISING)
    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
      // Immediate increment on wake, then go back to deep sleep
      noInterrupts();
      rainCount++;            // increment persistent counter immediately
      interrupts();

      Serial.println("🌧️ Woke by rain (EXT1). Incremented count immediately.");
      printStatus(rainCount);

      // small optional delay to allow serial to flush (keep minimal)
      Serial.flush();
      delay(50);

      // go back to deep sleep waiting for next tip
      Serial.println("💤 Returning to deep sleep...");
      Serial.flush();
      esp_deep_sleep_start();
    } else {
      // Not woken by rain: enter deep sleep and wait for first tip
      Serial.println("🔌 Entering deep sleep, waiting for rain tip...");
      Serial.flush();
      esp_deep_sleep_start();
    }
  } else {
    // Awake debug mode
    Serial.println(">> Running in AWAKE mode (interrupts active).");
  }
}

void loop() {
  if (!USE_DEEP_SLEEP) {
    if (newTip) {
      newTip = false;
      noInterrupts();
      unsigned long count = isrCount;
      interrupts();
      Serial.println(">> TIP DETECTED!");
      printStatus(count);
    }
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat >= 5000) {
      lastHeartbeat = millis();
      noInterrupts();
      unsigned long count = isrCount;
      interrupts();
      Serial.printf("[Heartbeat] tips so far: %lu\n", count);
    }
    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'r' || cmd == 'R') {
        noInterrupts();
        isrCount = 0;
        rainCount = 0;
        interrupts();
        Serial.println(">> Count RESET to 0");
      } else if (cmd == 's' || cmd == 'S') {
        noInterrupts();
        unsigned long count = isrCount;
        interrupts();
        printStatus(count);
      }
    }
  } else {
    delay(1000);
  }
}
