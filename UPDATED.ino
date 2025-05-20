/**
 * LilyGO T-A76XX (A7670X/SIM7670G) Local Web Status + SMS + GPS Example (V8)
 * 
 * - Shows SIM, network, signal, carrier, and firmware version.
 * - Displays GPS latitude and longitude (decimal degrees or "GPS not fixed").
 * - Has an editable recipient number field (pre-filled with default), and a single "Send" SMS button with a pre-defined message.
 * 
 * Compatible with LilyGO-T-A76XX boards (A7670X/SIM7670G).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#define MODEM_TX_PIN        26
#define MODEM_RX_PIN        27
#define BOARD_PWRKEY_PIN    4
#define BOARD_POWERON_PIN   12
#define MODEM_RESET_PIN     5
#define MODEM_BAUDRATE      115200

const char* ssid = "PLDTHOMEFIBRa6d98";
const char* password = "PLDTWIFI26umx";
HardwareSerial SerialAT(1);
WebServer server(80);

// Default recipient and message
const char* defaultRecipient = "+639948244158";
const char* preDefinedMessage = "Hello from LilyGO T-A76XX!";

String smsResult = "";
String lastRecipient = defaultRecipient; // To remember last used recipient

// Function declarations
String sendAT(const char* cmd, uint32_t timeout = 1500, bool waitForPrompt = false);
String getGPS();
String getSIMStatus();
String getNetworkStatus();
String getSignalQuality();
String getCarrier();
String getVersion();
String sendSMS(String recipient, String message);

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

String getGPS() {
    sendAT("AT+CGPS=1,1", 2000);
    String resp = sendAT("AT+CGPSINFO", 2000);
    int i = resp.indexOf("+CGPSINFO:");
    if (i >= 0) {
        int end = resp.indexOf('\n', i);
        String info = resp.substring(i + 10, end);
        info.trim();
        if (info.length() == 0 || info.indexOf(",") == 0) return "GPS not fixed";
        int p1 = info.indexOf(',');
        if (p1 == -1) return "GPS parse error";
        String lat = info.substring(0, p1);
        int p2 = info.indexOf(',', p1 + 1);
        String lat_dir = info.substring(p1 + 1, p2);
        int p3 = info.indexOf(',', p2 + 1);
        String lon = info.substring(p2 + 1, p3);
        int p4 = info.indexOf(',', p3 + 1);
        String lon_dir = info.substring(p3 + 1, p4);

        if (lat.length() > 0 && lon.length() > 0 && lat != "0.0" && lon != "0.0") {
            float lat_dd = 0, lon_dd = 0;
            if (lat.length() > 4 && lon.length() > 5) {
                float lat_val = lat.toFloat();
                float lon_val = lon.toFloat();
                lat_dd = int(lat_val / 100) + fmod(lat_val, 100) / 60.0;
                lon_dd = int(lon_val / 100) + fmod(lon_val, 100) / 60.0;
                if (lat_dir == "S") lat_dd = -lat_dd;
                if (lon_dir == "W") lon_dd = -lon_dd;
                char buf[96];
                snprintf(buf, sizeof(buf),
                    "Latitude: %.6f %s<br>Longitude: %.6f %s", lat_dd, lat_dir.c_str(), lon_dd, lon_dir.c_str());
                return String(buf);
            }
            return "Latitude: " + lat + " " + lat_dir + "<br>Longitude: " + lon + " " + lon_dir;
        } else {
            return "GPS not fixed";
        }
    }
    return "No GPS info";
}

String getSIMStatus() {
    String r = sendAT("AT+CPIN?");
    int i = r.indexOf("+CPIN:");
    if (i >= 0) {
        int eol = r.indexOf('\n', i);
        return r.substring(i, eol);
    }
    return "No SIM or unknown";
}
String getNetworkStatus() {
    String r = sendAT("AT+CGREG?");
    int i = r.indexOf("+CGREG:");
    if (i >= 0) {
        int eol = r.indexOf('\n', i);
        return r.substring(i, eol);
    }
    return "Unknown network status";
}
String getSignalQuality() {
    String r = sendAT("AT+CSQ");
    int i = r.indexOf("+CSQ:");
    if (i >= 0) {
        int eol = r.indexOf('\n', i);
        return r.substring(i, eol);
    }
    return "Unknown signal";
}
String getCarrier() {
    String r = sendAT("AT+COPS?");
    int i = r.indexOf("+COPS:");
    if (i >= 0) {
        int quote1 = r.indexOf('"', i);
        int quote2 = r.indexOf('"', quote1 + 1);
        if (quote1 >= 0 && quote2 > quote1)
            return r.substring(quote1 + 1, quote2);
        int eol = r.indexOf('\n', i);
        return r.substring(i, eol);
    }
    return "Unknown carrier";
}
String getVersion() {
    String r = sendAT("AT+SIMCOMATI");
    return r.length() > 0 ? r : "Unknown modem";
}

String sendSMS(String recipient, String message) {
    String resp = sendAT("AT+CMGF=1", 1000);
    // FIX: Only fail if "OK" is NOT present
    if (resp.indexOf("OK") == -1) {
        return "Failed to set text mode: " + resp;
    }
    resp = sendAT(("AT+CMGS=\"" + recipient + "\"").c_str(), 2000, true);
    if (resp.indexOf('>') == -1) return "No prompt for SMS text: " + resp;
    SerialAT.print(message);
    SerialAT.write(0x1A); // Ctrl+Z
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
    return "Unknown SMS result: " + result;
}

void handleRoot() {
    String html = "<h2>LilyGO T-A76XX Modem Status, GPS & SMS (V8)</h2>";
    html += "<b>SIM Status:</b> " + getSIMStatus() + "<br>";
    html += "<b>Network Status:</b> " + getNetworkStatus() + "<br>";
    html += "<b>Signal Quality:</b> " + getSignalQuality() + "<br>";
    html += "<b>Carrier:</b> " + getCarrier() + "<br>";
    html += "<b>Version:</b> <pre>" + getVersion() + "</pre>";
    html += "<b>GPS:</b><br>" + getGPS() + "<br>";
    html += "<hr>";
    html += "<h3>Send Predefined SMS</h3>";
    if (smsResult.length()) html += "<b>Result:</b> " + smsResult + "<br>";
    html += "<form action=\"/send_sms\" method=\"POST\">";
    html += "Recipient (number): <input name=\"recipient\" type=\"text\" value=\"" + (lastRecipient.length() ? lastRecipient : defaultRecipient) + "\" required pattern=\"[\\d\\+]+\"><br>";
    html += "<input type=\"submit\" value=\"Send\">";
    html += "</form>";
    html += "<hr><small>Refresh to update statuses. GPS requires outdoor antenna for fix.</small>";
    server.send(200, "text/html", html);
}

void handleSendSMS() {
    String recipient = defaultRecipient;
    if (server.hasArg("recipient") && server.arg("recipient").length() > 0) {
        recipient = server.arg("recipient");
        lastRecipient = recipient;
    }
    smsResult = sendSMS(recipient, String(preDefinedMessage));
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting T-A76XX Web Status + SMS + GPS Example (V8)");

    modemPowerOn();
    SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println(" Connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());

    server.on("/", handleRoot);
    server.on("/send_sms", HTTP_POST, handleSendSMS);
    server.begin();
}

void loop() {
    server.handleClient();
}