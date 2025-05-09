#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MPU6050 Accelerometer and Gyro
Adafruit_MPU6050 mpu;

// Ultrasonic Sensor Pins
#define TRIG_PIN 12
#define ECHO_PIN 13

// Buzzer and Button
#define BUZZER_PIN 25
#define RESET_PIN 26

// GSM & GPS UART
HardwareSerial sim800(1);
HardwareSerial gps(2);
#define SIM800_RX 16
#define SIM800_TX 17
#define GPS_RX 4
#define GPS_TX 5

// WiFi AP
const char *ssid = "ESP32-Accident-System";
const char *password = "12345678";

// Web server
WebServer server(80);
Preferences preferences;
TinyGPSPlus gpsParser;

// Configurable ESP32-CAM IP
IPAddress esp32CamIP(192, 168, 4, 2);
uint16_t esp32CamPort = 8080;

// Variables
float filteredGX = 0, filteredGY = 0, filteredGZ = 0, totalG = 0;
bool accidentDetected = false;
bool sendSMSAfterDelay = false;
unsigned long accidentDetectionTime = 0;
float gForceThreshold = 3.0; // Default threshold
String carOrientation = "Upright";
String phoneNumber;

// Timers
unsigned long pageCycleTime = 0;
const unsigned long pageSwitchInterval = 5000;
int currentPage = 1;

void setup() {
  // Serial Monitor
  Serial.begin(115200);

  // Initialize OLED Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    while (1);
  }
  display.clearDisplay();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 Failed!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Initialize Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);

  // Initialize SIM800L
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);

  // Initialize GPS
  gps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Retrieve Phone Number and Threshold from Preferences
  preferences.begin("config", false);
  phoneNumber = preferences.getString("phone", "+639287523354");
  gForceThreshold = preferences.getFloat("gForce", 3.0);

  // Setup WiFi AP
  WiFi.softAP(ssid, password);
  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started. Access via http://esp32.local");
  }

  // Setup Web Server
  server.on("/", handleRootPage);
  server.on("/update-config", handleUpdateConfig);
  server.on("/trigger-esp32cam", []() {
    sendTriggerToESP32CAM();
    server.send(200, "text/plain", "Trigger signal sent to ESP32-CAM.");
  });
  server.begin();
}

void loop() {
  server.handleClient();

  // Handle GPS Data
  while (gps.available() > 0) {
    gpsParser.encode(gps.read());
  }

  // Handle MPU6050 Data
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  filteredGX = 0.8 * filteredGX + 0.2 * accel.acceleration.x;
  filteredGY = 0.8 * filteredGY + 0.2 * accel.acceleration.y;
  filteredGZ = 0.8 * filteredGZ + 0.2 * accel.acceleration.z;
  totalG = sqrt(filteredGX * filteredGX + filteredGY * filteredGY + filteredGZ * filteredGZ);

  detectCarOrientation(accel);

  // Accident Detection SMS
  if (sendSMSAfterDelay && millis() - accidentDetectionTime >= 5000) {
    sendSMS(false);
    sendSMSAfterDelay = false;
  }

  // Display Pages
  if (millis() - pageCycleTime > pageSwitchInterval) {
    currentPage = (currentPage == 1) ? 2 : 1;
    pageCycleTime = millis();
  }
  displayPage(currentPage);

  delay(100);
}

void detectCarOrientation(sensors_event_t &accel) {
  const float uprightZ = 8.0, flippedZ = -8.0, sideAccel = 2.0;

  if (filteredGZ > uprightZ) {
    carOrientation = "Upright";
    accidentDetected = false;
    sendSMSAfterDelay = false;
    digitalWrite(BUZZER_PIN, LOW);
  } else if (filteredGZ < flippedZ || fabs(filteredGX) > sideAccel || fabs(filteredGY) > sideAccel) {
    carOrientation = (filteredGZ < flippedZ) ? "Flipped" : "On Side";
    triggerAccident();
  } else if (totalG > gForceThreshold) {
    carOrientation = "High G-Force";
    triggerAccident();
  }
}

void triggerAccident() {
  accidentDetected = true;
  digitalWrite(BUZZER_PIN, HIGH);
  if (!sendSMSAfterDelay) {
    accidentDetectionTime = millis();
    sendSMSAfterDelay = true;
  }
}

void sendSMS(bool manual) {
  String lat = gpsParser.location.isValid() ? String(gpsParser.location.lat(), 6) : "Unavailable";
  String lng = gpsParser.location.isValid() ? String(gpsParser.location.lng(), 6) : "Unavailable";
  String link = "https://www.google.com/maps?q=" + lat + "," + lng;
  String msg = (manual ? "Manual Alert!\n" : "Accident Detected!\n") + "Location: " + lat + ", " + lng + "\nLink: " + link;

  sim800.println("AT+CMGF=1");
  delay(100);
  sim800.println("AT+CMGS=\"" + phoneNumber + "\"");
  delay(100);
  sim800.print(msg);
  sim800.write(26);
}

void displayPage(int page) {
  display.clearDisplay();
  if (page == 1) {
    display.setCursor(0, 0);
    display.print("Orientation: ");
    display.print(carOrientation);
    display.setCursor(0, 10);
    display.print("Total G: ");
    display.print(totalG, 2);
  } else {
    display.setCursor(0, 0);
    display.print("WiFi: ");
    display.print(WiFi.softAPIP());
  }
  display.display();
}

void sendTriggerToESP32CAM() {
  WiFiUDP udp;
  udp.beginPacket(esp32CamIP, esp32CamPort);
  udp.print("START_RECORDING");
  udp.endPacket();
}

void handleRootPage() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head><title>ESP32 Configuration</title></head>
    <body>
      <h1>ESP32 Configuration</h1>
      <form action="/update-config" method="POST">
        <label for="phone">Phone Number:</label><br>
        <input type="text" id="phone" name="phone" value=")rawliteral" +
                  phoneNumber + R"rawliteral("><br><br>
        <label for="gForce">G-Force Threshold:</label><br>
        <input type="number" id="gForce" name="gForce" step="0.1" value=")rawliteral" +
                  String(gForceThreshold) + R"rawliteral("><br><br>
        <input type="submit" value="Update">
      </form>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleUpdateConfig() {
  if (server.hasArg("phone")) {
    phoneNumber = server.arg("phone");
    preferences.putString("phone", phoneNumber);
  }
  if (server.hasArg("gForce")) {
    gForceThreshold = server.arg("gForce").toFloat();
    preferences.putFloat("gForce", gForceThreshold);
  }
  server.send(200, "text/plain", "Configuration updated. Please go back.");
}