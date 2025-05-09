#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <TinyGPS++.h>

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MPU6050 Accelerometer and Gyro
Adafruit_MPU6050 mpu;
bool mpuAvailable = false; // Track MPU availability

// Ultrasonic Sensor Pins
#define TRIG_PIN 12
#define ECHO_PIN 13

// Buzzer and Buttons
#define BUZZER_PIN 25
#define RESET_PIN 26 // Combined reset and manual alert button

// GSM Module and GPS using HardwareSerial
HardwareSerial sim800(1);
HardwareSerial gps(2);
#define SIM800_RX 16
#define SIM800_TX 17
#define GPS_RX 4
#define GPS_TX 5

// Access Point Configuration
const char *ssid = "ESP32-Accident-System";
const char *password = "12345678";

// Web Server
WebServer server(80);
Preferences preferences;

// TinyGPS++ Object
TinyGPSPlus gpsParser;

// ESP-NOW Data Structure
typedef struct {
  bool recordVideo; // True if ESP32-CAM should start recording
} esp_now_message_t;

// ESP32-CAM MAC Address (replace with the actual MAC address of your ESP32-CAM)
uint8_t esp32camAddress[] = {0x24, 0x6F, 0x28, 0xXX, 0xXX, 0xXX};

// Variables
float filteredGX = 0, filteredGY = 0, filteredGZ = 0;
float totalG, ultrasonicDistance = 0;
String severity = "No Accident";
String gpsLocation = "GPS Unavailable";
String gpsTime = "Time: Unavailable";
bool accidentDetected = false;
String carOrientation = "Upright";
String sim800Status = "Checking...";
bool espNowInitialized = false;


// Timer Variables for Reset Button
unsigned long resetButtonPressTime = 0;
unsigned long resetButtonReleaseTime = 0; // To track release time for short press
const unsigned long resetHoldDuration = 30000; // 30 seconds
const unsigned long shortPressDuration = 500; // 500 milliseconds for short press detection

// Timer Variables for Accident SMS Delay
const unsigned long accidentSMSDelay = 5000; // 5 seconds
unsigned long accidentDetectionTime = 0;
bool sendSMSAfterDelay = false;

// Timer Variables for Page Switching
unsigned long pageCycleTime = 0; // Time to track page cycling
const unsigned long pageSwitchInterval = 5000; // Switch pages every 5 seconds
int currentPage = 1; // Start on Page 1

// G-Force Threshold
const float gForceThreshold = 3.0; // Threshold for accident detection

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

  // Attempt to Initialize MPU6050
  initializeMPU();

  // Initialize Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize Buzzer and Buttons
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);

  // Initialize SIM800L UART
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);
  delay(1000);
  checkSIM800L(); // Check if SIM800L is working

  // Initialize GPS UART
  gps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Initialize Preferences
  preferences.begin("sms-config", false);
  String recipientNumber = preferences.getString("phone", "+639123456789");
  Serial.println("Loaded phone number: " + recipientNumber);

  // Configure Wi-Fi Access Point
  WiFi.softAP(ssid, password);
  Serial.println("AP IP Address: " + WiFi.softAPIP().toString());
  
  // Configure Web Server
  server.on("/", []() {
    String html = "<h1>ESP32 Accident Detection System</h1>";
    server.send(200, "text/html", html);
  });
  server.begin();
}

void loop() {
  // Handle Web Server
  server.handleClient();

  // Check if the reset button is pressed
  if (digitalRead(RESET_PIN) == LOW) { // Assuming LOW indicates button press
    if (resetButtonPressTime == 0) {
      resetButtonPressTime = millis(); // Start timing the button press
    } else if (millis() - resetButtonPressTime >= resetHoldDuration) {
      disableSystem(); // Disable the whole system after holding for 30 seconds
    }
  } else if (resetButtonPressTime > 0) { // Button was released
    resetButtonReleaseTime = millis();
    if (resetButtonReleaseTime - resetButtonPressTime < resetHoldDuration &&
        resetButtonReleaseTime - resetButtonPressTime > shortPressDuration) {
      triggerManualAlert(); // Short press triggers manual alert
    }
    resetButtonPressTime = 0; // Reset the timer
  }

  // Attempt to reinitialize MPU6050 if it is not available
  if (!mpuAvailable) {
    initializeMPU();
  }

  // Process GPS Data
  while (gps.available() > 0) {
    gpsParser.encode(gps.read());
  }

  // Update GPS Time
  updateGPSTime();

  // Read Accelerometer and Gyro Data only if MPU is available
  if (mpuAvailable) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Apply Low-Pass Filter to Smooth Data
    filteredGX = 0.8 * filteredGX + 0.2 * accel.acceleration.x;
    filteredGY = 0.8 * filteredGY + 0.2 * accel.acceleration.y;
    filteredGZ = 0.8 * filteredGZ + 0.2 * accel.acceleration.z;

    // Compute Total G-Force
    totalG = sqrt(filteredGX * filteredGX + filteredGY * filteredGY + filteredGZ * filteredGZ);

    // Detect Car Orientation and Rollover
    detectCarOrientation(accel, gyro);
  }

  // Handle SMS Delay for Accident Detection
  if (sendSMSAfterDelay && millis() - accidentDetectionTime >= accidentSMSDelay) {
    sendAccidentAlert(); // Send SMS after delay
    sendSMSAfterDelay = false; // Reset the flag
  }

  // Measure Ultrasonic Distance
  ultrasonicDistance = getUltrasonicDistance();

  // If distance is between 2 and 3 meters, send message to ESP32-CAM
  if (ultrasonicDistance >= 200 && ultrasonicDistance <= 300) {
    message.recordVideo = true; // Set the recording flag
    esp_err_t result = esp_now_send(esp32camAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
      Serial.println("Instruction sent to ESP32-CAM to start recording.");
    } else {
      Serial.println("Error sending message to ESP32-CAM.");
    }
  }

  // Cycle Between Pages
  if (millis() - pageCycleTime > pageSwitchInterval) {
    currentPage = (currentPage == 1) ? 2 : 1; // Toggle between Page 1 and Page 2
    pageCycleTime = millis(); // Reset the cycle timer
  }

  // Display the Current Page
  displayPage(currentPage);

  delay(100);
}

// Function to Disable the Entire System
void disableSystem() {
  Serial.println("System Disabled!");

  // Turn off all outputs
  digitalWrite(BUZZER_PIN, LOW);

  // Display "System Disabled" on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("System Disabled!");
  display.display();

  while (1); // Stop the program
}

// Function to Reset System State
void resetSystem() {
  Serial.println("System Reset Triggered!");

  // Reset accident detection flags
  accidentDetected = false;
  sendSMSAfterDelay = false;
  carOrientation = "Upright";

  // Stop the buzzer
  digitalWrite(BUZZER_PIN, LOW);

  // Clear OLED display
  display.clearDisplay();
  display.display();
}

// Function to Trigger Manual Alert (SMS and Buzzer)
void triggerManualAlert() {
  Serial.println("Manual Alert Triggered!");

  // Activate the buzzer
  digitalWrite(BUZZER_PIN, HIGH);

  // Send an SMS alert
  sendManualSMS();

  // Keep the buzzer on for 5 seconds, then turn it off
  delay(5000);
  digitalWrite(BUZZER_PIN, LOW);
}

// Function to Detect Car Orientation and Trigger Alerts
void detectCarOrientation(sensors_event_t &accel, sensors_event_t &gyro) {
  const float uprightAccelZThreshold = 8.0;
  const float flippedAccelZThreshold = -8.0;
  const float sideAccelThreshold = 2.0;

  // Check Static Orientation with Accelerometer
  if (filteredGZ > uprightAccelZThreshold) {
    carOrientation = "Upright";
    accidentDetected = false;
    sendSMSAfterDelay = false;

    // Turn off the buzzer
    digitalWrite(BUZZER_PIN, LOW);
  } else if (filteredGZ < flippedAccelZThreshold || fabs(filteredGX) > sideAccelThreshold || fabs(filteredGY) > sideAccelThreshold) {
    carOrientation = (filteredGZ < flippedAccelZThreshold) ? "Flipped" : "On Side";
    accidentDetected = true;

    // Turn on the buzzer
    digitalWrite(BUZZER_PIN, HIGH);

    // Start the SMS delay timer
    if (!sendSMSAfterDelay) {
      accidentDetectionTime = millis();
      sendSMSAfterDelay = true;
    }
  } else {
    carOrientation = "Unknown";
    accidentDetected = false;
    sendSMSAfterDelay = false;

    // Turn off the buzzer
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// Function to Initialize MPU6050
void initializeMPU() {
  Serial.println("Attempting to initialize MPU6050...");
  if (mpu.begin()) {
    mpuAvailable = true;
    Serial.println("MPU6050 initialized successfully!");
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  } else {
    mpuAvailable = false;
    Serial.println("MPU6050 not detected. Retrying...");
    delay(2000); // Retry after 2 seconds
  }
}

// Function to Check if SIM800L is Working
void checkSIM800L() {
  sim800.println("AT"); // Send AT command
  delay(1000);

  if (sim800.available()) {
    String response = sim800.readString();
    if (response.indexOf("OK") != -1) {
      sim800Status = "Working";
    } else {
      sim800Status = "Not Detected";
    }
  } else {
    sim800Status = "Not Detected";
  }
  Serial.println("SIM800L Status: " + sim800Status);
}

// Function to Update GPS Time
void updateGPSTime() {
  if (gpsParser.time.isValid()) {
    char timeBuffer[16];
    sprintf(timeBuffer, "%02d:%02d:%02d", gpsParser.time.hour(), gpsParser.time.minute(), gpsParser.time.second());
    gpsTime = String("Time: ") + timeBuffer;
  } else {
    gpsTime = "Time: Unavailable";
  }
}

// Function to Send Manual SMS Alert
void sendManualSMS() {
  String latitude = gpsParser.location.isValid() ? String(gpsParser.location.lat(), 6) : "Unavailable";
  String longitude = gpsParser.location.isValid() ? String(gpsParser.location.lng(), 6) : "Unavailable";

  // Generate Google Maps link
  String googleMapsLink = "https://www.google.com/maps?q=" + latitude + "," + longitude;

  // Construct the message
  String alertMessage = String("Manual Alert!\n") +
                      "Orientation: " + carOrientation + "\n" +
                      "Location: Latitude: " + latitude + ", Longitude: " + longitude + "\n" +
                      "Time: " + gpsTime + "\n" +
                      "View location: " + googleMapsLink;

  // Send the SMS
  sim800.println("AT+CMGF=1"); // Set SMS to Text Mode
  delay(100);
  sim800.println("AT+CMGS=\"+639123456789\""); // Replace with recipient's number
  delay(100);
  sim800.print(alertMessage);
  delay(100);
  sim800.write(26); // Send Ctrl+Z to indicate the end of the message
  delay(5000); // Wait for the message to be sent

  // Log the message to Serial
  Serial.println("SMS Sent: " + alertMessage);
}

// Function to Send Accident SMS Alert
void sendAccidentAlert() {
  String latitude = gpsParser.location.isValid() ? String(gpsParser.location.lat(), 6) : "Unavailable";
  String longitude = gpsParser.location.isValid() ? String(gpsParser.location.lng(), 6) : "Unavailable";

  // Generate Google Maps link
  String googleMapsLink = "https://www.google.com/maps?q=" + latitude + "," + longitude;

  // Construct the message
  String alertMessage = String("Accident Detected!\n") +
                      "Orientation: " + carOrientation + "\n" +
                      "Location: Latitude: " + latitude + ", Longitude: " + longitude + "\n" +
                      "Time: " + gpsTime + "\n" +
                      "View location: " + googleMapsLink;

  // Send the SMS
  sim800.println("AT+CMGF=1"); // Set SMS to Text Mode
  delay(100);
  sim800.println("AT+CMGS=\"+639123456789\""); // Replace with recipient's number
  delay(100);
  sim800.print(alertMessage);
  delay(100);
  sim800.write(26); // Send Ctrl+Z to indicate the end of the message
  delay(5000); // Wait for the message to be sent

  // Log the message to Serial
  Serial.println("SMS Sent: " + alertMessage);
}

// Function to Get Ultrasonic Distance
float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return -1;
  float distance = duration * 0.034 / 2;
  if (distance < 2 || distance > 400) return -1;
  return distance;
}

// Function to Display the Current Page
void displayPage(int page) {
  display.clearDisplay();
  if (page == 1) {
    display.setCursor(0, 0);
    display.print(gpsTime);

    if (gpsParser.location.isValid()) {
      display.setCursor(0, 10);
      display.print("GPS: Lat ");
      display.print(gpsParser.location.lat(), 2);
      display.print(", Lng ");
      display.print(gpsParser.location.lng(), 2);
    } else {
      display.setCursor(0, 10);
      display.print("GPS: Unavailable");
    }

    display.setCursor(0, 20);
    display.print("SIM800L: ");
    display.print(sim800Status);

    display.setCursor(0, 30);
    display.print("Orientation: ");
    display.print(carOrientation);

  } else if (page == 2) {
    display.setCursor(0, 0);
    display.print("Total G: ");
    display.print(totalG, 2);

    display.setCursor(0, 10);
    display.print("Distance: ");
    display.print(ultrasonicDistance, 2);
    display.print(" cm");

    display.setCursor(0, 20);
    display.print("Accident: ");
    display.print(accidentDetected ? "YES" : "NO");
  }
  display.display();
}

// Function to Send Trigger Signal to ESP32-CAM
void sendTriggerToESP32CAM() {
  // Example data to send to ESP32-CAM
  String triggerMessage = "START_RECORDING";

  // Replace with ESP32-CAM's IP address (when connected to this ESP32's AP)
  IPAddress esp32CamIP(192, 168, 4, 2); // Example IP address
  uint16_t esp32CamPort = 8080; // Example port where ESP32-CAM listens

  // Create a UDP object for sending the message
  WiFiUDP udp;
  udp.beginPacket(esp32CamIP, esp32CamPort);
  udp.print(triggerMessage);
  if (udp.endPacket()) {
    Serial.println("Trigger signal sent to ESP32-CAM.");
  } else {
    Serial.println("Failed to send trigger signal to ESP32-CAM.");
  }
}

// Function to Respond to Web Command for Triggering ESP32-CAM
void setupTriggerEndpoint() {
  server.on("/trigger-esp32cam", []() {
    sendTriggerToESP32CAM();
    server.send(200, "text/plain", "Trigger signal sent to ESP32-CAM.");
  });
}

// Call this function in setup() to set up the trigger endpoint
void setupTriggerFunctionality() {
  setupTriggerEndpoint();
}

// Function to Trigger Manual Alert (SMS and Buzzer)
void triggerManualAlert() {
  Serial.println("Manual Alert Triggered!");

  // Activate the buzzer
  digitalWrite(BUZZER_PIN, HIGH);

  // Send an SMS alert
  sendManualSMS();

  // Keep the buzzer on for 5 seconds, then turn it off
  delay(5000);
  digitalWrite(BUZZER_PIN, LOW);
}