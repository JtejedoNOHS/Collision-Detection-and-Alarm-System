#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>

// OLED Display Pins and Config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MPU6050 Config
Adafruit_MPU6050 mpu;
bool mpuAvailable = false;

// Ultrasonic Sensor Pins
#define TRIG_PIN 12
#define ECHO_PIN 13

// Buzzer and Buttons
#define BUZZER_PIN 25
#define RESET_PIN 26

// GSM Module and GPS using Hardware Serial
#define SIM800_RX 16
#define SIM800_TX 17
#define GPS_RX 4
#define GPS_TX 5
TinyGPSPlus gpsData;

// Define hardware serial ports
HardwareSerial sim800(1); // Using Serial1 for GSM
HardwareSerial gps(2);    // Using Serial2 for GPS

// WiFi and Web Server Config
const char* ssid = "ESP32_AP";
const char* password = "password123";
WebServer server(80);

// UDP Config for ESP32-CAM Communication
WiFiUDP udp;
const char* esp32CamIP = "192.168.4.2";
const int esp32CamPort = 8888;

// Global State Variables
double impactThreshold = 2.5; // Threshold for high G-forces
bool accidentDetected = false;
unsigned long accidentStartTime = 0;
String recipientNumbers[5] = {"09171234567"}; // Dynamic editable via Web Server
int recipientCount = 1;

// Display Page Variables
int currentPage = 1;
unsigned long lastPageSwitchTime = 0;
const unsigned long pageSwitchInterval = 5000; // Switch pages every 5 seconds

// Function Prototypes
void setupOLED();
void setupMPU();
void setupUltrasonic();
void setupGSM();
void setupGPS();
void setupWiFi();
void setupWebServer();
void checkMPU();
void checkUltrasonic();
void sendAccidentAlert();
void triggerESP32CamRecording();
void handleResetButton();
void handleWebRequests();
void updateDisplay();
void displayPage1();
void displayPage2();

void setup() {
  Serial.begin(115200);

  // Initialize Components
  setupOLED();
  setupMPU();
  setupUltrasonic();
  setupGSM();
  setupGPS();
  setupWiFi();
  setupWebServer();

  // Start Web Server
  server.begin();
}

void loop() {
  server.handleClient(); // Handle Web Server Requests
  checkMPU();            // Check MPU6050 for Accidents
  checkUltrasonic();     // Check Ultrasonic Sensor
  handleResetButton();   // Handle Reset Button Actions
  updateDisplay();       // Update OLED Display
}

// OLED Setup
void setupOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED initialization failed!");
    while (1);
  }
  display.clearDisplay();
  display.display();
  Serial.println("OLED initialized successfully.");
}

// MPU6050 Setup
void setupMPU() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    mpuAvailable = false;
  } else {
    mpuAvailable = true;
    Serial.println("MPU6050 initialized successfully.");
  }
}

// Ultrasonic Sensor Setup
void setupUltrasonic() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

// GSM Setup
void setupGSM() {
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);
  Serial.println("GSM initialized on Serial1.");
  // Register SIM card logic here...
}

// GPS Setup
void setupGPS() {
  gps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS initialized on Serial2.");
}

// WiFi Setup
void setupWiFi() {
  WiFi.softAP(ssid, password);
  Serial.print("WiFi Access Point started. IP: ");
  Serial.println(WiFi.softAPIP());
}

// Web Server Setup
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/plain", "Welcome to the ESP32 Accident Detection System");
  });
  server.on("/addRecipient", HTTP_POST, []() {
    if (server.hasArg("number")) {
      if (recipientCount < 5) {
        recipientNumbers[recipientCount++] = server.arg("number");
        server.send(200, "text/plain", "Recipient added successfully");
      } else {
        server.send(400, "text/plain", "Recipient list full");
      }
    } else {
      server.send(400, "text/plain", "Invalid request");
    }
  });
}

// Check MPU6050 for Accidents
void checkMPU() {
  if (!mpuAvailable) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  double mag = sqrt(a.acceleration.x * a.acceleration.x +
                    a.acceleration.y * a.acceleration.y +
                    a.acceleration.z * a.acceleration.z);

  if (mag >= impactThreshold) {
    accidentDetected = true;
    accidentStartTime = millis();
    sendAccidentAlert();
    triggerESP32CamRecording();
    digitalWrite(BUZZER_PIN, HIGH);
    delay(5000); // Delay before resetting
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// Check Ultrasonic Sensor
void checkUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration / 2) * 0.0343; // Calculate distance in cm

  if (distance < 50) {
    triggerESP32CamRecording();
  }
}

// Send Accident Alert via GSM
void sendAccidentAlert() {
  String message = "Accident detected! Location: ";
  message += "https://maps.google.com/?q=" + String(gpsData.location.lat(), 6) + "," + String(gpsData.location.lng(), 6);
  
  for (int i = 0; i < recipientCount; i++) {
    sim800.println("AT+CMGS=\"" + recipientNumbers[i] + "\"");
    delay(100);
    sim800.println(message);
    delay(100);
    sim800.write(26); // End SMS with CTRL+Z
    delay(5000);
  }
}

// Trigger ESP32-CAM Recording
void triggerESP32CamRecording() {
  udp.beginPacket(esp32CamIP, esp32CamPort);
  udp.print("START_RECORDING");
  udp.endPacket();
}

// Handle Reset Button
void handleResetButton() {
  static unsigned long buttonPressTime = 0;
  static int buttonPressCount = 0;

  if (digitalRead(RESET_PIN) == LOW) {
    if (buttonPressTime == 0) {
      buttonPressTime = millis();
    } else if (millis() - buttonPressTime > 15000) {
      ESP.restart();
    }
  } else {
    if (buttonPressTime != 0 && millis() - buttonPressTime < 500) {
      buttonPressCount++;
      if (buttonPressCount == 2) {
        accidentDetected = true;
        sendAccidentAlert();
      }
    }
    buttonPressTime = 0;
    buttonPressCount = 0;
  }
}

// Update OLED Display
void updateDisplay() {
  unsigned long currentTime = millis();

  // Switch pages based on time
  if (currentTime - lastPageSwitchTime >= pageSwitchInterval) {
    currentPage = (currentPage == 1) ? 2 : 1;
    lastPageSwitchTime = currentTime;
  }

  // Display the current page
  if (currentPage == 1) {
    displayPage1();
  } else {
    displayPage2();
  }
}

// Page 1: GPS, Signal, and Distance
void displayPage1() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Page 1: GPS Data");
  display.setCursor(0, 10);
  display.print("Lat: ");
  display.println(gpsData.location.lat(), 6);
  display.setCursor(0, 20);
  display.print("Lon: ");
  display.println(gpsData.location.lng(), 6);
  display.setCursor(0, 30);
  display.print("Signal: OK"); // Add signal status checking
  display.setCursor(0, 40);
  display.print("Distance: N/A"); // Add ultrasonic distance data
  display.display();
}

// Page 2: Telemetry
void displayPage2() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Page 2: Telemetry");
  display.setCursor(0, 10);
  display.print("Accident: ");
  display.println(accidentDetected ? "YES" : "NO");
  display.setCursor(0, 20);
  display.print("Threshold: ");
  display.println(impactThreshold);
  display.setCursor(0, 30);
  display.print("Impact Time: ");
  display.println(accidentStartTime);
  display.display();
}