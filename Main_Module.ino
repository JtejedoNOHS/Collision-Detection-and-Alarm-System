#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADXL345_U.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <TinyGPS++.h> // Include TinyGPS++ library

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ADXL345 Accelerometer
Adafruit_ADXL345_Unified accel(12345);

// Ultrasonic Sensor Pins
#define TRIG_PIN 12
#define ECHO_PIN 13

// Buzzer and Reset Button
#define BUZZER_PIN 25
#define RESET_PIN 26

// GSM Module and GPS using HardwareSerial
HardwareSerial sim800(1); // UART1 for SIM800L
HardwareSerial gps(2);    // UART2 for Neo 6M GPS
#define SIM800_RX 16
#define SIM800_TX 17
#define GPS_RX 4
#define GPS_TX 5

// Access Point Configuration
const char *ssid = "ESP32-Accident-System"; // Replace with your desired SSID
const char *password = "12345678";          // Replace with your desired password

// Web Server
WebServer server(80);
Preferences preferences;

// TinyGPS++ Object
TinyGPSPlus gpsParser;

// Variables
float gX, gY, gZ, totalG;
String severity = "No Accident";
String gpsLocation = "GPS Unavailable";
float speedKmh = 0;
float ultrasonicDistance = 0;
bool accidentDetected = false;
String recipientNumber = "+639123456789"; // Default recipient number

void setup() {
  Serial.begin(115200);

  // Initialize OLED Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED initialization failed!");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize Accelerometer
  if (!accel.begin()) {
    Serial.println("Could not find ADXL345 sensor!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);

  // Initialize Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize Buzzer and Reset Button
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);

  // Initialize SIM800L UART
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);
  sim800.println("AT"); // Test connection
  delay(1000);

  // Initialize Neo 6M GPS UART
  gps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Initialize Preferences for Persistent Storage
  preferences.begin("sms-config", false);
  recipientNumber = preferences.getString("phone", "+639123456789");
  Serial.println("Loaded phone number: " + recipientNumber);

  // Configure Wi-Fi Access Point
  WiFi.softAP(ssid, password); // Start the ESP32 in Access Point mode
  IPAddress IP = WiFi.softAPIP(); // Get the IP address of the ESP32
  Serial.println("Access Point started!");
  Serial.print("AP IP Address: ");
  Serial.println(IP);

  // Configure Web Server
  server.on("/", []() {
    String html = "<h1>ESP32 Accident Detection System</h1>";
    html += "<form action=\"/set\" method=\"POST\">";
    html += "<label for=\"phone\">Enter Recipient Phone Number:</label><br>";
    html += "<input type=\"text\" id=\"phone\" name=\"phone\" value=\"" + recipientNumber + "\"><br><br>";
    html += "<button type=\"submit\">Save</button>";
    html += "</form><br>";
    html += "<a href=\"/gps\">Check GPS Status</a><br>";
    html += "<a href=\"/sensors\">View Sensor Data</a>";
    server.send(200, "text/html", html);
  });

  server.on("/set", HTTP_POST, []() {
    recipientNumber = server.arg("phone");
    preferences.putString("phone", recipientNumber);
    server.send(200, "text/plain", "Phone number updated to: " + recipientNumber);
    Serial.println("Updated phone number: " + recipientNumber);
  });

  server.on("/gps", []() {
    String gpsStatus = getGPSStatusHTML();
    server.send(200, "text/html", gpsStatus);
  });

  server.on("/sensors", []() {
    String sensorData = getSensorDataHTML();
    server.send(200, "text/html", sensorData);
  });

  server.begin();
  Serial.println("Web server started!");
}

void loop() {
  // Handle Web Server
  server.handleClient();

  // Process GPS Data
  while (gps.available() > 0) {
    gpsParser.encode(gps.read());
  }

  // Read Accelerometer Data
  sensors_event_t event;
  accel.getEvent(&event);
  gX = event.acceleration.x / 9.81;
  gY = event.acceleration.y / 9.81;
  gZ = event.acceleration.z / 9.81;
  totalG = sqrt(gX * gX + gY * gY + gZ * gZ);

  // Categorize Severity
  severity = "No Accident";
  if (totalG > 5.0) { // Change threshold for severity
    severity = "Total Wreck";
    accidentDetected = true;
  } else if (totalG > 3.0) {
    severity = "Moderate Impact";
    accidentDetected = true;
  } else if (totalG > 1.5) {
    severity = "Rolled Over";
    accidentDetected = true;
  }

  // Read Ultrasonic Distance
  ultrasonicDistance = getUltrasonicDistance();

  // Display Status on OLED
  displayStatusOnOLED();

  delay(1000);
}

// Function to Get Ultrasonic Distance
float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

// Function to Display Status on OLED
void displayStatusOnOLED() {
  display.clearDisplay();

  // Line 1: GPS Status
  if (gpsParser.location.isValid()) {
    display.setCursor(0, 0);
    display.print("GPS: ");
    display.print("Lat ");
    display.print(gpsParser.location.lat(), 2);
    display.print(", Lng ");
    display.print(gpsParser.location.lng(), 2);
  } else {
    display.setCursor(0, 0);
    display.print("GPS: Unavailable");
  }

  // Line 2: Total G
  display.setCursor(0, 10);
  display.print("Total G: ");
  display.print(totalG, 2);

  // Line 3: Ultrasonic Distance
  display.setCursor(0, 20);
  display.print("Distance: ");
  display.print(ultrasonicDistance, 2);
  display.print(" cm");

  // Line 4: Accident Status
  display.setCursor(0, 30);
  display.print("Status: ");
  display.print(severity);

  // Line 5: Satellites in View
  if (gpsParser.satellites.isValid()) {
    display.setCursor(0, 40);
    display.print("Satellites: ");
    display.print(gpsParser.satellites.value());
  }

  // Update Display
  display.display();
}

// Function to Get GPS Status as HTML
String getGPSStatusHTML() {
  String html = "<h1>GPS Status</h1>";

  if (gpsParser.location.isValid()) {
    html += "<p>Latitude: " + String(gpsParser.location.lat(), 6) + "</p>";
    html += "<p>Longitude: " + String(gpsParser.location.lng(), 6) + "</p>";
  } else {
    html += "<p>GPS Location: Unavailable</p>";
  }

  if (gpsParser.satellites.isValid()) {
    html += "<p>Satellites in view: " + String(gpsParser.satellites.value()) + "</p>";
  } else {
    html += "<p>Satellites: Unavailable</p>";
  }

  return html;
}

// Function to Get Sensor Data as HTML
String getSensorDataHTML() {
  String html = "<h1>Sensor Data</h1>";
  html += "<p>Accelerometer (G-Forces):</p>";
  html += "<ul>";
  html += "<li>X-axis: " + String(gX, 2) + " G</li>";
  html += "<li>Y-axis: " + String(gY, 2) + " G</li>";
  html += "<li>Z-axis: " + String(gZ, 2) + " G</li>";
  html += "<li>Total G: " + String(totalG, 2) + " G</li>";
  html += "</ul>";
  html += "<p>Ultrasonic Distance: " + String(ultrasonicDistance, 2) + " cm</p>";
  html += "<p>Accident Status: " + severity + "</p>";
  return html;
}