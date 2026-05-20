#include "driver/rtc_io.h"
#include <HardwareSerial.h>
#include <Preferences.h>

#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <BH1750.h>
#include <SPI.h>
#include <LoRa.h>
#include <math.h>

// ===== OTA LIB =====
#include <ESP32S3_SIM7670_OTA.h>
#define CURRENT_FIRMWARE_VERSION "1.0.2"

// ===================== Pin Definitions =====================

// I2C
#define SDA_PIN 6
#define SCL_PIN 7

// DHT22
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Soil
#define SOIL_MOISTURE 5
#define SOIL_TEMP_PIN 8

// MPU
#define MPU_INT_PIN 9

// LoRa SX1278
#define PIN_LORA_SCK 13
#define PIN_LORA_MISO 14
#define PIN_LORA_MOSI 15
#define PIN_LORA_CS 10
#define PIN_LORA_RST 11
#define PIN_LORA_DIO0 12
#define LORA_FREQUENCY 433E6

// ─── RH_RF95 Header bytes (broadcast: TO=0xFF, FROM=0xFF, ID=0x00, FLAGS=0x00)
#define RH_TO 0xFF
#define RH_FROM 0xFF
#define RH_ID 0x00
#define RH_FLAGS 0x00
uint8_t payload[] = { '1', 0x00 };

// Rain Sensor
#define RAIN_PIN GPIO_NUM_16
#define RAIN_MM_PER_TIP 0.2794
#define RAIN_DEBOUNCE_US 200000UL  // 200 ms debounce

// GSM Serial (SIM7670X)
#define GSM_RX 17
#define GSM_TX 18

// Sleep Timing
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP 600

// ===================== Object Definitions =====================
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
BH1750 lightMeter;
HardwareSerial SerialAt(2);

ESP32S3_SIM7670_OTA ota(
  SerialAt,
  GSM_RX, GSM_TX,
  "airtelgprs.com",
  "",
  "",
  "landslidemonitoring.in",
  443);

Preferences prefs;

// ===================== Globals =====================
String tId = "t1";
sensors_event_t a, g, temp;

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR unsigned long rainCount = 0;  // persistent across deep-sleep
RTC_DATA_ATTR volatile unsigned long motionCount = 0;

bool rainWake = false;
bool loRaAvailable = false;

// ===================== Function Declarations =====================
String sendGSMData(String url);
bool extractOTA(String payload, String &version, String &url);
void print_wakeup_reason();
void readGSM();
void initBH1750();
void initBMP180();
float readTemp();
float readHumidity();
bool initMPU();
void readMPU();
float readLight();
float readPressure();
float readSoilTemperature();
String getS5Reading();
String getGNSSLocation();
float getRainfallMM();

void sendRH(const uint8_t *payload, uint8_t payloadLen) {
  LoRa.beginPacket();
  LoRa.write(RH_TO);
  LoRa.write(RH_FROM);
  LoRa.write(RH_ID);
  LoRa.write(RH_FLAGS);
  LoRa.write(payload, payloadLen);
  LoRa.endPacket();
}

// ===================== ISR =====================
// Debounced rain ISR (used when awake; EXT1 handles counting during sleep)
volatile unsigned long lastTipMicros = 0;

void IRAM_ATTR rainISR() {
  unsigned long now = micros();
  unsigned long delta = now - lastTipMicros;
  if (delta > RAIN_DEBOUNCE_US) {
    lastTipMicros = now;
    rainCount++;
  }
}

void IRAM_ATTR mpuISR() {
  motionCount++;
}

// ===================== Setup =====================
void setup() {

  // --- EXT0: MPU wakeup ---
  esp_sleep_enable_ext0_wakeup((gpio_num_t)MPU_INT_PIN, 1);
  rtc_gpio_pullup_dis((gpio_num_t)MPU_INT_PIN);
  rtc_gpio_pulldown_en((gpio_num_t)MPU_INT_PIN);

  // --- EXT1: Rain gauge wakeup (RISING — pin goes HIGH on each tip) ---
  uint64_t rainMask = 1ULL << RAIN_PIN;
  esp_sleep_enable_ext1_wakeup(rainMask, ESP_EXT1_WAKEUP_ANY_HIGH);
  rtc_gpio_pullup_dis((gpio_num_t)RAIN_PIN);
  rtc_gpio_pulldown_en((gpio_num_t)RAIN_PIN);

  Serial.begin(115200);
  SerialAt.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(200);

  // ===== Check wakeup cause FIRST =====
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_EXT1) {
    // ── Rain tip wakeup: increment count immediately and go back to sleep ──
    noInterrupts();
    rainCount++;
    interrupts();

    Serial.printf("🌧️ Rain tip! Count: %lu  (%.4f mm)\n",
                  rainCount, rainCount * RAIN_MM_PER_TIP);
    Serial.flush();
    delay(50);  // let serial drain
    esp_deep_sleep_start();
    // execution never reaches here
  }

  // ── All other wakeups (timer, EXT0, power-on) continue normally ──

  SerialAt.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(1000);

  // ===== NVS Triplet ID =====
  prefs.begin("device", true);
  tId = prefs.getString("tid", "");
  if (tId == "") {
    Serial.println("❌ ERROR: tid not found in flash");
  } else {
    Serial.print("✅ Device tId = ");
    Serial.println(tId);
  }
  prefs.end();
  Serial.println("Triplet ID (from flash): " + tId);

  pinMode(SOIL_MOISTURE, INPUT);
  pinMode(SOIL_TEMP_PIN, INPUT);

  // Rain pin: INPUT_PULLDOWN + RISING (matches test sketch)
  pinMode(RAIN_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainISR, RISING);

  pinMode(MPU_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpuISR, RISING);

  ++bootCount;

  Wire.begin(SDA_PIN, SCL_PIN);

  initBH1750();
  initBMP180();
  dht.begin();
  initMPU();
  readMPU();

  // ===================== LoRa INIT =====================
  Serial.println("Initializing LoRa...");
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

  if (LoRa.begin(LORA_FREQUENCY)) {
    loRaAvailable = true;
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(8);
    LoRa.setSyncWord(0x39);
    LoRa.enableCrc();
    Serial.println("✅ LoRa init OK — 433 MHz | SF7 | BW125 | CR4/5 | SyncWord 0x39");
  } else {
    loRaAvailable = false;
    Serial.println("⚠️ LoRa not detected");
  }

  // ===== OTA INIT =====
  ota.begin();
  ota.setVersion(CURRENT_FIRMWARE_VERSION);

  print_wakeup_reason();

  // LoRa send on EXT0 (MPU motion wakeup)
  if (cause == ESP_SLEEP_WAKEUP_EXT0 && loRaAvailable) {
    Serial.println("📡 Sending LoRa packet (motion wake)");
    LoRa.beginPacket();
    LoRa.print("Motion detected at " + tId);
    LoRa.endPacket();
  }

  // Read accumulated rainfall, then reset counter
  float rainfall = getRainfallMM();

  String url =
    "https://landslidemonitoring.in/ota.php?api_key=3WU63XFVOKEC1VBM"
    "&triplet="
    + tId + "&" + tId + "s1=" + String(readTemp()) + "," + String(readHumidity()) + "&" + tId + "s2=" + String(readPressure()) + "&" + tId + "s3=" + String(rainfall) + "&" + tId + "s4=" + String(readLight()) + "&" + tId + "s5=" + getS5Reading() + "&" + tId + "s6=" + String(readSoilTemperature()) + "&" + tId + "s7=" + String(analogRead(SOIL_MOISTURE)) + "&" + tId + "s8=0" + "&" + tId + "s9=00.0000|00.0000";

  String response = sendGSMData(url);
  String otaVersion, otaUrl;

  if (!extractOTA(response, otaVersion, otaUrl)) {
    Serial.println("❌ OTA PARSE FAILED");
  } else {
    Serial.println("✅ OTA PARSE OK");
    Serial.println("Version : " + otaVersion);
    Serial.println("URL (raw): " + otaUrl);

    otaUrl.replace("\\/", "/");
    Serial.println("URL (fixed): " + otaUrl);

    if (otaVersion == CURRENT_FIRMWARE_VERSION) {
      Serial.println("Firmware already up to date");
    } else {
      Serial.println("🚀 STARTING OTA FLASH");
      ota.performOTA(otaUrl);
    }
  }

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("💤 Going to sleep for " + String(TIME_TO_SLEEP) + " sec...");
  Serial.flush();
  esp_deep_sleep_start();
}

void loop() {}

// ===================== OTA JSON Parser =====================
bool extractOTA(String payload, String &version, String &url) {
  int v = payload.indexOf("\"ota_version\"");
  int u = payload.indexOf("\"ota_url\"");
  if (v == -1 || u == -1) return false;

  int vs = payload.indexOf("\"", v + 13) + 1;
  int ve = payload.indexOf("\"", vs);
  version = payload.substring(vs, ve);

  int us = payload.indexOf("\"", u + 9) + 1;
  int ue = payload.indexOf("\"", us);
  url = payload.substring(us, ue);

  return true;
}

// ===================== GSM =====================
String sendGSMData(String url) {
  Serial.println("*************URL*************");
  Serial.println(url);
  Serial.println("*****************************");

  String response = "";
  String urcLine = "";

  SerialAt.println("AT");
  delay(200);
  SerialAt.println("AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"");
  delay(200);
  SerialAt.println("AT+CGACT=1,1");
  delay(1000);
  SerialAt.println("AT+HTTPINIT");
  delay(1000);
  SerialAt.print("AT+HTTPPARA=\"URL\",\"");
  SerialAt.print(url);
  SerialAt.println("\"");
  delay(500);
  SerialAt.println("AT+HTTPACTION=0");

  unsigned long start = millis();
  int contentLength = -1;

  while (millis() - start < 20000) {
    while (SerialAt.available()) {
      char c = SerialAt.read();
      if (c == '\n') {
        urcLine.trim();
        if (urcLine.length()) Serial.println(urcLine);
        if (urcLine.startsWith("+HTTPACTION:")) {
          int lastComma = urcLine.lastIndexOf(',');
          if (lastComma != -1)
            contentLength = urcLine.substring(lastComma + 1).toInt();
          goto READ_BODY;
        }
        urcLine = "";
      } else {
        urcLine += c;
      }
    }
  }

READ_BODY:
  if (contentLength <= 0) {
    Serial.println("❌ HTTPACTION failed or empty response");
    SerialAt.println("AT+HTTPTERM");
    return "";
  }

  SerialAt.print("AT+HTTPREAD=0,");
  SerialAt.println(contentLength);

  start = millis();
  while (millis() - start < 20000) {
    while (SerialAt.available()) response += (char)SerialAt.read();
    if (response.indexOf("}") != -1) break;
  }

  SerialAt.println("AT+HTTPTERM");
  delay(500);

  int js = response.indexOf("{");
  int je = response.lastIndexOf("}");
  if (js != -1 && je != -1 && je > js)
    response = response.substring(js, je + 1);
  else
    response = "";

  Serial.println("===== RAW RESPONSE =====");
  Serial.println(response);
  return response;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  switch (reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("🔔 Wakeup: MPU interrupt"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("🌧️ Wakeup: Rain tip"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("⏰ Wakeup: Timer (scheduled)"); break;
    default: Serial.printf("Wakeup cause: %d\n", reason); break;
  }
}

void readGSM() {
  delay(100);
  while (SerialAt.available()) Serial.write(SerialAt.read());
}

// ===================== Sensors =====================
void initBH1750() {
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23))
    Serial.println("❌ BH1750 not found!");
}

void initBMP180() {
  if (!bmp.begin())
    Serial.println("❌ BMP180 not found!");
}

float readLight() {
  float lux = lightMeter.readLightLevel();
  Serial.printf("💡 Light: %.2f lx\n", lux);
  return lux;
}

float readPressure() {
  uint32_t d = bmp.readPressure();
  float p = (d != 0) ? d / 100.0 : 0;
  Serial.printf("🌡️ Pressure: %.2f hPa\n", p);
  return p;
}

float readSoilTemperature() {
  int analogValue = analogRead(SOIL_TEMP_PIN);
  float voltage = analogValue * (3.3 / 4095.0);
  float temperatureC = (voltage / 3.3) * 100.0;
  Serial.printf("🌱 Soil Temp: %.2f °C\n", temperatureC);
  return temperatureC;
}

bool initMPU() {
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found!");
    return false;
  }
  Serial.println("✅ MPU6050 initialized.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(false);
  mpu.setMotionInterrupt(true);
  return true;
}

float readTemp() {
  float t = dht.readTemperature();
  if (isnan(t)) t = 255;
  Serial.printf("🌡️ Air Temp: %.2f°C\n", t);
  return t;
}

float readHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) h = 255;
  Serial.printf("💧 Humidity: %.2f%%\n", h);
  return h;
}

void readMPU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float Ax = a.acceleration.x;
  float Ay = a.acceleration.y;
  float Az = a.acceleration.z;
  float Wx = g.gyro.x * (180.0 / PI);
  float Wy = g.gyro.y * (180.0 / PI);
  float Wz = g.gyro.z * (180.0 / PI);

  float roll = atan2(Ay, Az) * 180.0 / PI;
  float pitch = atan(-Ax / sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;
  float yaw = atan2(Wy, Wx) * 180.0 / PI;

  Serial.println("--------- MPU DATA ---------");
  Serial.printf("Ax: %.2f  Ay: %.2f  Az: %.2f (m/s²)\n", Ax, Ay, Az);
  Serial.printf("Wx: %.2f  Wy: %.2f  Wz: %.2f (°/s)\n", Wx, Wy, Wz);
  Serial.printf("Roll:  %.2f°\nPitch: %.2f°\nYaw:   %.2f°\n", roll, pitch, yaw);
  Serial.println("----------------------------");
}

float getRainfallMM() {
  noInterrupts();
  unsigned long count = rainCount;
  rainCount = 0;  // reset after reading
  interrupts();

  float mm = count * RAIN_MM_PER_TIP;
  Serial.printf("🌧️ Rainfall: %.4f mm  (%lu tips)\n", mm, count);
  return mm;
}

String getGNSSLocation() {
  SerialAt.println("AT+CGNSINF");
  delay(1000);
  String response = "";
  while (SerialAt.available()) response += (char)SerialAt.read();
  int latStart = response.indexOf(",", 28);
  int lonStart = response.indexOf(",", latStart + 1);
  int lonEnd = response.indexOf(",", lonStart + 1);
  if (latStart > 0 && lonStart > 0 && lonEnd > 0) {
    String lat = response.substring(latStart + 1, lonStart);
    lat.trim();
    String lon = response.substring(lonStart + 1, lonEnd);
    lon.trim();
    return lat + "," + lon;
  }
  return "0.0,0.0";
}

String getS5Reading() {
  mpu.getEvent(&a, &g, &temp);
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan(-a.acceleration.x / sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float yaw = atan2(g.gyro.z, g.gyro.x) * 180.0 / PI;

  return String(a.acceleration.x, 2) + "," + String(a.acceleration.y, 2) + "," + String(a.acceleration.z, 2) + "," + String(g.gyro.x, 2) + "," + String(g.gyro.y, 2) + "," + String(g.gyro.z, 2) + "," + String(roll, 2) + "," + String(pitch, 2) + "," + String(yaw, 2) + "," + String(motionCount) + ",0,0";
}
/*
t1&t1s1=28.60,42.10
&t1s2=882.61
&t1s3=1.68
&t1s4=28.33
&t1s5=1.61,9.32,-0.73,-0.00,0.05,0.09,94.47,-9.78,90.50,0,0,0
&t1s6=22.34
&t1s7=2987
&t1s8=0
&t1s9=00.0000|00.0000
*/

/*
  AT+CGDCONT=1,"IP","airtelgprs.com"
  AT+CGACT=1,1
  AT+HTTPINIT
  AT+HTTPPARA="URL","https://landslidemonitoring.in/ota.php?api_key=3WU63XFVOKEC1VBM&triplet=t1&t1s1=255.00,255.00&t1s2=2.55&t1s3=0.00&t1s4=-2.00&t1s5=7.35,-82.69,0.62,-14.64,18.67,-14.92,-89.57,-5.08,-134.47,0,0,0&t1s6=32.50&t1s7=61&t1s8=0&t1s9=00.0000|00.0000"
  AT+HTTPACTION=0
  AT+HTTPREAD=0,Content Length(e.g. 677)
  */


/*
https://landslidemonitoring.in/ota.php?api_key=3WU63XFVOKEC1VBM&triplet=t1&t1s1=255.00,255.00&t1s2=2.55&t1s3=0.00&t1s4=-2.00&t1s5=7.35,-82.69,0.62,-14.64,18.67,-14.92,-89.57,-5.08,-134.47,0,0,0&t1s6=32.50&t1s7=61&t1s8=0&t1s9=00.0000|00.0000
*/

/*
http://landslidemonitoring.esy.es/ota.php?api_key=3WU63XFVOKEC1VBM&triplet=
t1&t1s1=20.50,57.90&
t1s2=892.71&
t1s3=0.00&
t1s4=10.00&
t1s5=0.00,0.00,0.00,0.00,0.00,0.00,0.00,nan,0.00,2200,0,0&
t1s6=16.73&
t1s7=2504&
t1s8=0&
t1s9=00.0000|00.0000
*/

/*
http://landslidemonitoring.esy.es/ota.php?api_key=3WU63XFVOKEC1VBM&triplet=t1&
t1s1=20.70,58.00&       //temp+humidity
t1s2=892.56&            //pressure
t1s3=0.00&              //rainfall
t1s4=9.17&              //light
t1s5=1.39,-1.04,10.04,0.10,0.04,-0.04,-5.94,-7.84,-23.38,0,0,0&   //a.x, a.y, a.z, g.x, g.y, g.z, roll, pitch, yaw, motionCount, 0, 0
t1s6=17.36&             //soil temp
t1s7=2507&              //soil moisture
t1s8=0&
t1s9=00.0000|00.0000
*/