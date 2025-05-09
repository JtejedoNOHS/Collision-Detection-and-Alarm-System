#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <Ultrasonic.h>
#include <Preferences.h>

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MPU6050
Adafruit_MPU6050 mpu;
bool mpuAvailable = false;
sensors_event_t a, g, temp;

// Ultrasonic Sensor
#define TRIG_PIN 12
#define ECHO_PIN 13
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

// Buzzer & Button
#define BUZZER_PIN 25
#define RESET_PIN 26

// SIM800L and GPS
HardwareSerial sim800(1); // UART1
HardwareSerial gps(2);    // UART2
#define SIM800_RX 16
#define SIM800_TX 17
#define GPS_RX 4
#define GPS_TX 5

TinyGPSPlus gpsParser;

// Web server
WebServer server(80);

// Preferences for saving recipient numbers
Preferences prefs;

// Variables
float filteredGX = 0, filteredGY = 0, filteredGZ = 0;
float totalG, ultrasonicDistance = 0;
String severity = "No Accident";
String gpsLocation = "GPS Unavailable";
String gpsTime = "Time: Unavailable";
bool accidentDetected = false;
unsigned long flippedStartTime = 0;
bool flippedTimerStarted = false;
unsigned long lastButtonPress = 0;
int buttonPressCount = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();

  // MPU6050 init
  if (mpu.begin()) {
    mpuAvailable = true;
  } else {
    display.println("MPU6050 Failed!");
    display.display();
  }

  // Buzzer & Button
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(ESP32CAM_TRIGGER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(ESP32CAM_TRIGGER_PIN, LOW);

  // SIM800L
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);
  delay(1000);
  sim800.println("AT+CFUN=1");
  delay(500);
  sim800.println("AT+CREG?");
  delay(500);

  // GPS
  gps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Load recipients from preferences
  prefs.begin("recipients", false);
  numRecipients = prefs.getInt("count", 1);
  for (int i = 0; i < numRecipients; i++) {
    recipientNumbers[i] = prefs.getString(("num" + String(i)).c_str(), "+639XXXXXXXXX");
  }

  // WiFi AP
  WiFi.softAP("ESP32_ACCIDENT", "password123");
  server.on("/", handleRoot);
  server.on("/recipients", handleRecipients);
  server.begin();

  display.println("Setup complete.");
  display.display();
}

void loop() {
  server.handleClient();
  checkButton();
  readMPU();
  readUltrasonic();
  readGPS();
  updateDisplay();
}

void checkButton() {
  if (digitalRead(RESET_PIN) == LOW) {
    if (millis() - lastButtonPress < 400) {
      buttonPressCount++;
    } else {
      buttonPressCount = 1;
    }
    lastButtonPress = millis();

    if (buttonPressCount >= 2) {
      triggerAccident("Manual Trigger");
    }

    unsigned long holdTime = 0;
    while (digitalRead(RESET_PIN) == LOW) {
      delay(10);
      holdTime += 10;
      if (holdTime >= 15000) { // 15 sec
        ESP.restart();
      }
    }
  }
}

void readMPU() {
  if (!mpuAvailable) return;
  mpu.getEvent(&a, &g, &temp);

  float accMag = sqrt(a.acceleration.x * a.acceleration.x +
                      a.acceleration.y * a.acceleration.y +
                      a.acceleration.z * a.acceleration.z) / 9.8; // in G

  bool flipped = abs(a.acceleration.z) < 3;

  if (accMag >= IMPACT_THRESHOLD) {
    triggerAccident("Impact " + String(accMag, 2) + "G");
  }

  if (flipped) {
    if (!flippedTimerStarted) {
      flippedStartTime = millis();
      flippedTimerStarted = true;
    } else if (millis() - flippedStartTime >= FLIP_TIMEOUT) {
      triggerAccident("Flipped >15s");
    }
  } else {
    flippedTimerStarted = false;
  }
}

void triggerAccident(String reason) {
  if (accidentDetected) return;
  accidentDetected = true;

  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(ESP32CAM_TRIGGER_PIN, HIGH);

  String sms = composeSMS(reason);

  for (int i = 0; i < numRecipients; i++) {
    sendSMS(recipientNumbers[i], sms);
    delay(SMS_DELAY_BETWEEN);
  }

  digitalWrite(BUZZER_PIN, LOW);
}

String composeSMS(String reason) {
  String sms = "ALERT: Accident Detected!\n";
  sms += "Reason: " + reason + "\n";
  sms += "Orientation: X=" + String(a.acceleration.x, 1) +
         " Y=" + String(a.acceleration.y, 1) +
         " Z=" + String(a.acceleration.z, 1) + "\n";
  sms += "Impact: " + String(sqrt(a.acceleration.x*a.acceleration.x +
                                 a.acceleration.y*a.acceleration.y +
                                 a.acceleration.z*a.acceleration.z)/9.8, 2) + "G\n";
  if (gpsParser.location.isValid()) {
    sms += "Location: https://maps.google.com/?q=";
    sms += gpsParser.location.lat();
    sms += ",";
    sms += gpsParser.location.lng();
  } else {
    sms += "Location: Invalid";
  }
  return sms;
}

void sendSMS(String number, String message) {
  sim800.println("AT+CMGF=1");
  delay(500);
  sim800.println("AT+CMGS=\"" + number + "\"");
  delay(500);
  sim800.print(message);
  sim800.write(26); // Ctrl+Z
  delay(5000);
}

void readGPS() {
  while (gps.available()) {
    gpsParser.encode(gps.read());
  }
}

void readUltrasonic() {
  long distance = ultrasonic.read();
  if (distance > 0 && distance < ULTRASONIC_TRIGGER_DISTANCE) {
    digitalWrite(ESP32CAM_TRIGGER_PIN, HIGH);
  } else {
    digitalWrite(ESP32CAM_TRIGGER_PIN, LOW);
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("GPS: ");
  if (gpsParser.location.isValid()) {
    display.print(gpsParser.location.lat(), 4);
    display.print(",");
    display.print(gpsParser.location.lng(), 4);
  } else {
    display.print("No Fix");
  }

  display.setCursor(0,10);
  display.print("SIM: ");
  sim800.println("AT+CSQ");
  delay(100);
  while (sim800.available()) {
    String line = sim800.readStringUntil('\n');
    if (line.indexOf("+CSQ:") >= 0) {
      display.print(line);
    }
  }

  display.setCursor(0,20);
  display.print("Ultra: ");
  display.print(ultrasonic.read());
  display.print("cm");

  display.setCursor(0,30);
  display.print(accidentDetected ? "ACCIDENT!" : "OK");

  display.setCursor(0,40);
  display.print("Accel: X");
  display.print(a.acceleration.x, 1);
  display.print(" Y");
  display.print(a.acceleration.y, 1);
  display.print(" Z");
  display.print(a.acceleration.z, 1);

  display.display();
}

void handleRoot() {
  String html = "<h1>ESP Accident Detection</h1>";
  html += "<p><a href='/recipients'>Edit Recipients</a></p>";
  server.send(200, "text/html", html);
}

void handleRecipients() {
  if (server.hasArg("num")) {
    numRecipients = server.arg("num").toInt();
    if (numRecipients > 5) numRecipients = 5;
    prefs.putInt("count", numRecipients);
    for (int i = 0; i < numRecipients; i++) {
      String key = "num" + String(i);
      recipientNumbers[i] = server.arg(key);
      prefs.putString(key.c_str(), recipientNumbers[i]);
    }
    server.send(200, "text/html", "<p>Recipients saved!</p><a href='/'>Back</a>");
  } else {
    String html = "<form method='get'>";
    html += "Number of Recipients (1-5): <input name='num' value='" + String(numRecipients) + "'><br>";
    for (int i = 0; i < 5; i++) {
      html += "Recipient " + String(i+1) + ": <input name='num" + String(i) + "' value='" + recipientNumbers[i] + "'><br>";
    }
    html += "<input type='submit' value='Save'></form>";
    server.send(200, "text/html", html);
  }
}
