#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "FS.h"
#include "SD_MMC.h"

// Camera pin config for AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// WiFi config
const char* ssid = "ESP32-Accident-System";
const char* password = "12345678";

// UDP config
WiFiUDP udp;
const int udpPort = 12345;

// Video config
const int recordingDuration = 15; // seconds
const int frameInterval = 500; // ms between frames

void startRecording();
void saveFrame(fs::FS &fs, String path, camera_fb_t *fb);

void setup() {
  Serial.begin(115200);

  // Connect to WiFi (as station)
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());

  // Init camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x", err);
    return;
  }

  // Init SD card
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }
  Serial.println("SD Card initialized");

  // Start UDP listener
  udp.begin(udpPort);
  Serial.println("Listening UDP on port " + String(udpPort));
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = '\0';
    Serial.println("UDP packet: " + String(incomingPacket));
    if (String(incomingPacket) == "START_RECORDING") {
      Serial.println("Trigger received, start recording...");
      startRecording();
    }
  }
}

void startRecording() {
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  char filename[64];
  strftime(filename, sizeof(filename), "/video_%Y%m%d_%H%M%S.mjpeg", &timeinfo);

  File videoFile = SD_MMC.open(filename, FILE_WRITE);
  if (!videoFile) {
    Serial.println("Failed to open file for writing");
    return;
  }
  Serial.println("Recording: " + String(filename));

  unsigned long startMillis = millis();
  while (millis() - startMillis < recordingDuration * 1000) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      continue;
    }
    // Write JPEG frame to file
    videoFile.write(fb->buf, fb->len);
    videoFile.flush();
    Serial.println("Saved frame: " + String(fb->len) + " bytes");
    esp_camera_fb_return(fb);
    delay(frameInterval);
  }

  videoFile.close();
  Serial.println("Recording finished");
}

