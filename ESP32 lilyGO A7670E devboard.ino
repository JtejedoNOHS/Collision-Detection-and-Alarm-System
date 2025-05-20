// LilyGO T-A76XX Enhanced GNSS SMS Alert System with Web Interface
// Features:
// - GNSS location reporting
// - MPU6050 impact and orientation detection (modular fallback if unavailable)
// - Ultrasonic sensor for proximity warning (outputs HIGH when < 2m, modular fallback if unavailable)
// - Button for manual alert reset and SMS trigger
// - SMS includes G-force, GNSS location, car orientation, and (optionally) last known speed
// - Web interface to monitor sensor data and modify sensitivity/recipient

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NewPing.h>
#include <ESPmDNS.h>

#define MODEM_TX_PIN        26
#define MODEM_RX_PIN        27
#define BOARD_PWRKEY_PIN    4
#define BOARD_POWERON_PIN   12
#define MODEM_RESET_PIN     5
#define MODEM_BAUDRATE      115200

#define TRIGGER_PIN         14
#define ECHO_PIN            15
#define MAX_DISTANCE        300
#define ALERT_OUTPUT_PIN    2
#define BUTTON_PIN          13

float impactThreshold = 3.0;
int ultrasonicThreshold = 200; // centimeters

const char* ssid = "PLDTHOMEFIBRa6d98";
const char* password = "PLDTWIFI26umx";
HardwareSerial SerialAT(1);
WebServer server(80);
Adafruit_MPU6050 mpu;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

String defaultRecipient = "+639948244158";
const char* preDefinedMessage = "Crash Alert!";
String smsResult = "";
String lastRecipient = defaultRecipient;
bool crashDetected = false;
bool mpuPresent = false;
bool ultrasonicPresent = false;

void modemPowerOn();
String sendAT(const char* cmd, uint32_t timeout = 1500, bool waitForPrompt = false);
String getGNSS();
String sendSMS(String recipient, String message);
String getCrashReport();
void checkForCrash();
void handleProximityOutput();
void handleButton();
void setupWebRoutes();

void setup() {
    Serial.begin(115200);
    modemPowerOn();
    SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(500);

    if (MDNS.begin("gnss-alert")) {
        Serial.println("MDNS responder started");
    }

    setupWebRoutes();
    server.begin();

    pinMode(ALERT_OUTPUT_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    if (mpu.begin()) {
        mpuPresent = true;
    } else {
        Serial.println("MPU6050 not detected. Defaulting to simulated values.");
    }

    int testPing = sonar.ping_cm();
    ultrasonicPresent = (testPing > 0);
    if (!ultrasonicPresent) {
        Serial.println("Ultrasonic sensor not detected. Defaulting to simulated distance.");
    }
}

void loop() {
    server.handleClient();
    handleProximityOutput();
    handleButton();
    checkForCrash();
}

void setupWebRoutes() {
    server.on("/", []() {
        String html = "<h2>GNSS Crash Alert System</h2>";
        html += "<b>Last SMS:</b> " + smsResult + "<br>";
        html += "<form action='/set' method='POST'>";
        html += "<label>Ultrasonic Sensitivity (cm):</label><input name='threshold' value='" + String(ultrasonicThreshold) + "'><br>";
        html += "<label>Recipient Number:</label><input name='recipient' value='" + defaultRecipient + "'><br>";
        html += "<input type='submit' value='Update'>";
        html += "</form>";
        server.send(200, "text/html", html);
    });

    server.on("/set", HTTP_POST, []() {
        if (server.hasArg("threshold")) {
            ultrasonicThreshold = server.arg("threshold").toInt();
        }
        if (server.hasArg("recipient")) {
            defaultRecipient = server.arg("recipient");
        }
        server.sendHeader("Location", "/");
        server.send(303);
    });
}

void modemPowerOn() {
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);
    delay(100);

    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, LOW);
    delay(100);
    digitalWrite(MODEM_RESET_PIN, HIGH);
    delay(2600);
    digitalWrite(MODEM_RESET_PIN, LOW);

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(2000);
}

String sendAT(const char* cmd, uint32_t timeout, bool waitForPrompt) {
    while (SerialAT.available()) SerialAT.read();
    SerialAT.println(cmd);
    String resp = "";
    uint32_t start = millis();
    if (waitForPrompt) {
        while (millis() - start < timeout) {
            if (SerialAT.available()) {
                char c = SerialAT.read();
                resp += c;
                if (c == '>') break;
            }
        }
    } else {
        while (millis() - start < timeout) {
            if (SerialAT.available())
                resp += char(SerialAT.read());
            if (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0) break;
        }
    }
    return resp;
}

String getGNSS() {
    sendAT("AT+CGPS=1,1", 2000);
    String resp = sendAT("AT+CGPSINFO", 2000);
    int i = resp.indexOf("+CGPSINFO:");
    if (i >= 0) {
        int end = resp.indexOf('\n', i);
        String info = resp.substring(i + 10, end);
        info.trim();
        if (info.length() == 0 || info.indexOf(",") == 0) return "GNSS not fixed";
        int p1 = info.indexOf(',');
        if (p1 == -1) return "GNSS parse error";
        String lat = info.substring(0, p1);
        int p2 = info.indexOf(',', p1 + 1);
        String lat_dir = info.substring(p1 + 1, p2);
        int p3 = info.indexOf(',', p2 + 1);
        String lon = info.substring(p2 + 1, p3);
        int p4 = info.indexOf(',', p3 + 1);
        String lon_dir = info.substring(p3 + 1, p4);

        float lat_val = lat.toFloat();
        float lon_val = lon.toFloat();
        float lat_dd = int(lat_val / 100) + fmod(lat_val, 100) / 60.0;
        float lon_dd = int(lon_val / 100) + fmod(lon_val, 100) / 60.0;
        if (lat_dir == "S") lat_dd = -lat_dd;
        if (lon_dir == "W") lon_dd = -lon_dd;

        char buf[128];
        snprintf(buf, sizeof(buf), "https://maps.google.com/?q=%.6f,%.6f", lat_dd, lon_dd);
        return String(buf);
    }
    return "GNSS not fixed";
}

String getCrashReport() {
    String report;
    float impact = 0.0;
    String orientation = "Unknown";

    if (mpuPresent) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        impact = sqrt(a.acceleration.x * a.acceleration.x +
                      a.acceleration.y * a.acceleration.y +
                      a.acceleration.z * a.acceleration.z) / 9.81;

        orientation = (abs(a.acceleration.z) < 3) ? "Flipped or on side" : "Normal";
    } else {
        impact = 0.0;
        orientation = "Sensor Offline";
    }

    report = "G-Force: " + String(impact, 2) + " G\n";
    report += "Location: " + getGNSS() + "\n";
    report += "Orientation: " + orientation + "\n";
    report += "Status: Crash Detected!";
    return report;
}

String sendSMS(String recipient, String message) {
    String resp = sendAT("AT+CMGF=1", 1000);
    if (resp.indexOf("OK") == -1) return "Failed to set text mode: " + resp;
    resp = sendAT(("AT+CMGS=\"" + recipient + "\"").c_str(), 2000, true);
    if (resp.indexOf('>') == -1) return "No prompt: " + resp;
    SerialAT.print(message);
    SerialAT.write(0x1A);
    uint32_t start = millis();
    String result = "";
    while (millis() - start < 12000) {
        if (SerialAT.available()) {
            char c = SerialAT.read();
            result += c;
            if (result.indexOf("OK") >= 0 || result.indexOf("ERROR") >= 0) break;
        }
    }
    if (result.indexOf("OK") != -1) return "SMS Sent!";
    if (result.indexOf("ERROR") != -1) return "SMS Failed: " + result;
    return "Unknown result: " + result;
}

void checkForCrash() {
    if (!mpuPresent || crashDetected) return;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float impact = sqrt(a.acceleration.x * a.acceleration.x +
                        a.acceleration.y * a.acceleration.y +
                        a.acceleration.z * a.acceleration.z) / 9.81;
    if (impact > impactThreshold) {
        crashDetected = true;
        smsResult = sendSMS(defaultRecipient, getCrashReport());
        digitalWrite(ALERT_OUTPUT_PIN, HIGH);
        delay(1000);
        digitalWrite(ALERT_OUTPUT_PIN, LOW);
    }
}

void handleProximityOutput() {
    if (!ultrasonicPresent) return;

    int distance = sonar.ping_cm();
    if (distance > 0 && distance < ultrasonicThreshold) {
        digitalWrite(ALERT_OUTPUT_PIN, HIGH);
    } else {
        digitalWrite(ALERT_OUTPUT_PIN, LOW);
    }
}

void handleButton() {
    static bool lastState = HIGH;
    bool currentState = digitalRead(BUTTON_PIN);
    if (lastState == HIGH && currentState == LOW) {
        crashDetected = false;
        smsResult = sendSMS(defaultRecipient, getCrashReport());
    }
    lastState = currentState;
}