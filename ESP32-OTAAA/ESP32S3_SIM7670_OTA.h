#ifndef ESP32S3_SIM7670_OTA_H
#define ESP32S3_SIM7670_OTA_H

#define TINY_GSM_MODEM_SIM7600

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <Update.h>

class ESP32S3_SIM7670_OTA
{
public:
    ESP32S3_SIM7670_OTA(
        HardwareSerial &serialPort,
        int rxPin, int txPin,
        const char *apn, const char *user, const char *pass,
        const char *server, int port = 80);

    void begin();                           // Initialize modem
    void setVersion(const String &version); // Set current firmware version
    void setDelay(unsigned long delayMs);   // Set delay for OTA check
    void setPaths(const String &versionPath, const String &firmwareBase);
    void checkAndUpdate(); // Run OTA logic
    bool checkForUpdate(String &newVersion);
    bool performOTA(const String &firmwareUrl);

private:
    HardwareSerial &_serial;
    int _rx, _tx;
    const char *_apn;
    const char *_user;
    const char *_pass;
    const char *_server;
    int _port;

    String _currentVersion;
    String _versionPath;
    String _firmwareBase;
    unsigned long _delayMs = 60000; // default 1 min

    TinyGsm *_modem;
    TinyGsmClient *_client;
    HttpClient *_http;
};

#endif
