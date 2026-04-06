#include <Preferences.h>

Preferences prefs;

String inputTID = "";

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== TID Writer Firmware ===");
  Serial.println("Enter TID and press ENTER:");

    // ===== NVS Triplet ID =====
  prefs.begin("device", true);  // READ-ONLY MODE
  inputTID = prefs.getString("tid", "");
  if (inputTID == "") {
    Serial.println("❌ ERROR: tid not found in flash");
  } else {
    Serial.print("✅ Device tId = ");
    Serial.println(inputTID);
  }
  prefs.end();
  Serial.println("Triplet ID (from flash): " + inputTID);
}

void loop() {
  // Read serial input
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputTID.length() > 0) {
        saveTID(inputTID);
        inputTID = "";
        Serial.println("\nEnter new TID:");
      }
    } else {
      inputTID += c;
    }
  }
}

void saveTID(String tid) {
  Serial.println("\nSaving TID...");

  // Open NVS in WRITE mode
  prefs.begin("device", false);

  // Save TID
  prefs.putString("tid", tid);

  prefs.end();

  Serial.print("✅ TID Saved Successfully: ");
  Serial.println(tid);
}