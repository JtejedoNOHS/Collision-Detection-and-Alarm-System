#include "WifiCam.hpp"
#include <WiFi.h>
#include <SD_MMC.h>
#include <WiFiUdp.h>
#include "esp_camera.h"

static const char* WIFI_SSID = "ESP32-Accident-System";
static const char* WIFI_PASS = "12345678";

esp32cam::Resolution initialResolution;

WiFiUDP udp;

// File for saving video
File videoFile;

// UDP Configuration
const uint16_t localUdpPort = 8080; // Port for receiving UDP packets
char incomingPacket[255]; // Buffer for incoming packets

// Function to start recording video
bool startRecording() {
  videoFile = SD_MMC.open("/video.avi", FILE_WRITE);
  if (!videoFile) {
    Serial.println("Failed to open video file for writing.");
    return false;
  }

  Serial.println("Recording started.");
  return true;
}

// Function to stop recording video
void stopRecording() {
  if (videoFile) {
    videoFile.close();
    Serial.println("Recording stopped.");
  }
}

// Function to capture a frame and save to video file
void captureFrame() {
  camera_fb_t* frame = esp_camera_fb_get();
  if (frame) {
    videoFile.write(frame->buf, frame->len);
    esp_camera_fb_return(frame);
  } else {
    Serial.println("Failed to capture frame.");
  }
}

// Function to record video
void recordVideo() {
  unsigned long startTime = millis();
  const unsigned long recordDuration = 10000; // Record for 10 seconds

  while (millis() - startTime < recordDuration) {
    captureFrame();
    delay(100); // Adjust delay for frame rate
  }
  stopRecording();
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  esp32cam::setLogger(Serial);
  delay(1000);

  // Initialize WiFi
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi failure %d\n", WiFi.status());
    delay(5000);
    ESP.restart();
  }
  Serial.println("WiFi connected");
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
  delay(1000);

  // Initialize SD card
  if (!SD_MMC.begin()) {
    Serial.println("SD card initialization failed.");
    delay(5000);
    ESP.restart();
  }
  Serial.println("SD card initialized.");

  // Initialize camera
  {
    using namespace esp32cam;

    initialResolution = Resolution::find(1024, 768);

    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(initialResolution);
    cfg.setJpeg(80);

    bool ok = Camera.begin(cfg);
    if (!ok) {
      Serial.println("Camera initialization failed.");
      delay(5000);
      ESP.restart();
    }
    Serial.println("Camera initialized.");
  }

  // Start UDP listener
  if (udp.begin(localUdpPort)) {
    Serial.printf("Listening for UDP messages on port %d\n", localUdpPort);
  } else {
    Serial.println("Failed to start UDP listener.");
  }
}

void loop() {
  listenForUdpCommand();
}

// Function to listen for UDP commands
void listenForUdpCommand() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Receive the UDP packet
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0'; // Null-terminate the string
    }
    Serial.printf("Received UDP packet: %s\n", incomingPacket);

    // Check if the command is "START_RECORDING"
    if (String(incomingPacket) == "START_RECORDING") {
      Serial.println("START_RECORDING command received.");
      if (startRecording()) {
        Serial.println("Recording started via UDP command.");
        recordVideo();
      } else {
        Serial.println("Failed to start recording via UDP command.");
      }
    }
  }
}