#include "ESP32S3_SIM7670_OTA.h"

ESP32S3_SIM7670_OTA::ESP32S3_SIM7670_OTA(
    HardwareSerial &serialPort,
    int rxPin, int txPin,
    const char *apn, const char *user, const char *pass,
    const char *server, int port) : _serial(serialPort), _rx(rxPin), _tx(txPin),
                                    _apn(apn), _user(user), _pass(pass), _server(server), _port(port)
{
}

void ESP32S3_SIM7670_OTA::begin()
{
    Serial.println("🔹 Initializing SIM7670G modem...");
    _serial.begin(115200, SERIAL_8N1, _rx, _tx);

    _modem = new TinyGsm(_serial);
    _modem->restart();

    if (!_modem->waitForNetwork())
    {
        Serial.println("❌ Network not found!");
        return;
    }
    Serial.println("✅ Network connected");

    if (!_modem->gprsConnect(_apn, _user, _pass))
    {
        Serial.println("❌ GPRS connection failed!");
        return;
    }
    Serial.println("✅ GPRS connected");

    _client = new TinyGsmClient(*_modem);
    _http = new HttpClient(*_client, _server, _port);
}

void ESP32S3_SIM7670_OTA::setVersion(const String &version)
{
    _currentVersion = version;
}

void ESP32S3_SIM7670_OTA::setDelay(unsigned long delayMs)
{
    _delayMs = delayMs;
}

void ESP32S3_SIM7670_OTA::setPaths(const String &versionPath, const String &firmwareBase)
{
    _versionPath = versionPath;
    _firmwareBase = firmwareBase;
}

bool ESP32S3_SIM7670_OTA::checkForUpdate(String &newVersion)
{
    Serial.println("🔍 Checking for update...");

    if (_http->connect(_server, _port))
    {
        _http->get(_versionPath);
        int statusCode = _http->responseStatusCode();
        String payload = _http->responseBody();

        if (statusCode == 200 && payload.length() > 0)
        {
            payload.trim();
            newVersion = payload;
            Serial.printf("🆕 Online version: %s | Current: %s\n", newVersion.c_str(), _currentVersion.c_str());
            return (newVersion != _currentVersion);
        }
        else
        {
            Serial.printf("⚠️ Failed to fetch version.txt (%d)\n", statusCode);
        }
    }
    else
    {
        Serial.println("❌ Connection to server failed.");
    }

    _http->stop();
    return false;
}

bool ESP32S3_SIM7670_OTA::performOTA(const String &firmwareUrl)
{
    Serial.println("⬇️ Starting OTA download...");

    if (!_http->connect(_server, _port))
    {
        Serial.println("❌ Connection failed");
        return false;
    }

    _http->get(firmwareUrl);
    int statusCode = _http->responseStatusCode();

    if (statusCode != 200)
    {
        Serial.printf("❌ HTTP %d\n", statusCode);
        _http->stop();
        return false;
    }

    int contentLength = _http->contentLength();
    if (contentLength <= 0)
    {
        Serial.println("❌ Invalid Content-Length");
        _http->stop();
        return false;
    }

    Serial.printf("📦 Firmware size: %d bytes\n", contentLength);
    bool canBegin = Update.begin(contentLength);
    if (!canBegin)
    {
        Serial.println("❌ Not enough space for OTA");
        _http->stop();
        return false;
    }

    uint8_t buff[1024];
    int written = 0;

    while (_http->connected() && written < contentLength)
    {
        size_t len = _http->readBytes(buff, sizeof(buff));
        if (len > 0)
        {
            Update.write(buff, len);
            written += len;
            Serial.printf("\rProgress: %d / %d bytes", written, contentLength);
        }
    }

    Serial.println();
    _http->stop();

    if (Update.end())
    {
        if (Update.isFinished())
        {
            Serial.println("✅ OTA complete");
            return true;
        }
        else
        {
            Serial.println("⚠️ OTA not finished properly");
            return false;
        }
    }
    else
    {
        Serial.printf("❌ OTA error: %s\n", Update.errorString());
        return false;
    }
}

void ESP32S3_SIM7670_OTA::checkAndUpdate()
{
    String newVersion;
    if (checkForUpdate(newVersion))
    {
        Serial.printf("⚙️ New version available: %s\n", newVersion.c_str());
        String firmwareUrl = _firmwareBase + newVersion + ".bin";
        if (performOTA(firmwareUrl))
        {
            Serial.println("✅ OTA Successful! Rebooting...");
            ESP.restart();
        }
        else
        {
            Serial.println("❌ OTA Failed!");
        }
    }
    else
    {
        Serial.println("🔸 No update available.");
    }

    delay(_delayMs);
}
