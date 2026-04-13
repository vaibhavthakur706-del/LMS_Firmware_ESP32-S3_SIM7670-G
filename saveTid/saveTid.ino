#include <Preferences.h>

Preferences prefs;

String inputTID = "";


void getTID() {
  prefs.begin("device", true);  // READ-ONLY MODE
  inputTID = prefs.getString("tid", "");
  if (inputTID == "") {
    Serial.println("[ERROR] tid not found in flash");
  } else {
    Serial.print("[SUCCESS] Device tId = ");
    Serial.println(inputTID);
  }
  prefs.end();
  inputTID = "";
}

void saveTID(String tid) {
  Serial.println("\n[INFO] Saving TID...");

  // Open NVS in WRITE mode
  prefs.begin("device", false);

  // Save TID
  prefs.putString("tid", tid);

  prefs.end();

  Serial.print("[DONE] TID Saved Successfully: ");
  Serial.println(tid);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== TID Writer Firmware ===");

  getTID();

  Serial.println("\n[INFO] Enter new TID:");
}

void loop() {
  // Read serial input
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputTID.length() > 0) {
        saveTID(inputTID);
        inputTID = "";
        getTID();
        Serial.println("\n[INFO] Enter new TID:");
      }
    } else {
      inputTID += c;
    }
  }
}