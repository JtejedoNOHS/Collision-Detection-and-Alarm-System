#include <WiFi.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <SD_MMC.h>
#include "esp_camera.h"

// Wi-Fi Configuration
const char* ssid = "ESP32_AP";
const char* password = "password123";

// UDP Configuration
WiFiUDP udp;
const int udpPort = 8888;
char incomingPacket[255];

// SD Card Configuration
#define SD_MMC_MOUNT_POINT "/sdcard"

// Camera Configuration
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// Global Variables
bool isRecording = false;
String videoFileName;

void setup() {
  Serial.begin(115200);

  // Initialize Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Wi-Fi connected.");

  // Start UDP
  udp.begin(udpPort);
  Serial.printf("Listening for UDP commands on port %d\n", udpPort);

  // Initialize SD Card
  if (!SD_MMC.begin()) {
    Serial.println("SD Card initialization failed.");
    while (1);
  }
  Serial.println("SD Card initialized.");

  // Initialize Camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame Size and Quality
  config.frame_size = FRAMESIZE_UXGA; // UXGA (1600x1200)
  config.jpeg_quality = 10; // Set quality (lower is better, range 0-63)
  config.fb_count = 2; // Buffers
  
  // Camera Init
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera initialization failed.");
    while (1);
  }
  Serial.println("Camera initialized successfully.");
}

void loop() {
  // Listen for UDP commands
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0; // Null-terminate the packet
    }
    Serial.printf("Received UDP packet: %s\n", incomingPacket);

    // Check for START_RECORDING command
    if (strcmp(incomingPacket, "START_RECORDING") == 0) {
      if (!isRecording) {
        isRecording = true;
        videoFileName = "/video_" + String(millis()) + ".avi";
        Serial.printf("Recording started: %s\n", videoFileName.c_str());
        recordVideo();
      }
    }
  }
}

// Function to Record Video
void recordVideo() {
  File videoFile = SD_MMC.open(videoFileName, FILE_WRITE);
  if (!videoFile) {
    Serial.println("Failed to create video file on SD card.");
    isRecording = false;
    return;
  }

  unsigned long startTime = millis();
  const unsigned long maxDuration = 60000; // Maximum recording duration (60 seconds)

  while (isRecording && (millis() - startTime < maxDuration)) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed.");
      break;
    }

    // Write frame to video file
    videoFile.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);
    delay(100); // Adjust delay for frame rate
  }

  videoFile.close();
  Serial.println("Recording stopped.");
  isRecording = false;
}