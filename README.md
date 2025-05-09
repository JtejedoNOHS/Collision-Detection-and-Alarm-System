# Collision Detection and Alarm System

This project is an ESP32-based Collision Detection and Alarm System designed to enhance safety by detecting accidents, monitoring vehicle orientation, and sending alerts with GPS location. The system integrates various hardware components like accelerometers, GPS, GSM modules, ultrasonic sensors, and a camera module to provide comprehensive accident detection and reporting.

## Features

- **Accident Detection**:
  - Monitors vehicle orientation using MPU6050 accelerometer and gyroscope.
  - Detects accidents based on G-force thresholds and vehicle rollover.

```cpp
// Detects Car Orientation and Rollover
void detectCarOrientation(sensors_event_t &accel, sensors_event_t &gyro) {
  if (filteredGZ > uprightAccelZThreshold) {
    carOrientation = "Upright";
    accidentDetected = false;
  } else if (filteredGZ < flippedAccelZThreshold) {
    carOrientation = "Flipped";
    accidentDetected = true;
  }
}
```

- **Alert System**:
  - Sends SMS alerts with GPS location to a predefined number using the SIM800L GSM module.
  - Triggers a buzzer for audible alarms.

```cpp
// Example of Sending Accident SMS Alert
void sendAccidentAlert() {
  String alertMessage = String("Accident Detected!\n") +
                      "Orientation: " + carOrientation + "\n" +
                      "Location: Latitude: " + latitude + ", Longitude: " + longitude;
  sim800.println("AT+CMGS=\"+639123456789\"");
  sim800.print(alertMessage);
  sim800.write(26); // End of message
}
```

- **Camera Integration**:
  - Communicates with an ESP32-CAM module to start video recording upon accident detection.
  - Stores video recordings on an SD card.

```cpp
// Start Recording Video from ESP32-CAM
void recordVideo() {
  unsigned long startTime = millis();
  while (millis() - startTime < recordDuration) {
    captureFrame();
  }
  stopRecording();
}
```

- **Ultrasonic Sensor**:
  - Measures distance to detect nearby objects.
  - Sends instructions to the ESP32-CAM to start recording when objects are within a specified range.

```cpp
// Measure Distance Using Ultrasonic Sensor
float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}
```

- **Web Server**:
  - Provides a simple web interface for system status and control.

```cpp
// Web Server to View System Status
server.on("/", []() {
  String html = "<h1>ESP32 Accident Detection System</h1>";
  server.send(200, "text/html", html);
});
```

- **OLED Display**:
  - Displays real-time system information such as GPS location, SIM module status, vehicle orientation, and accident status.

```cpp
// Display Real-time Information on OLED
void displayPage(int page) {
  if (page == 1) {
    display.print("GPS: " + gpsLocation);
    display.print("Orientation: " + carOrientation);
  } else {
    display.print("Total G: " + String(totalG));
    display.print("Accident: " + String(accidentDetected ? "YES" : "NO"));
  }
}
```

## Hardware Components

- **ESP32**: Main controller for the system.
- **Adafruit MPU6050**: Accelerometer and gyroscope for detecting vehicle orientation and movement.
- **Adafruit SSD1306**: OLED display for system information.
- **SIM800L GSM Module**: Sends SMS alerts.
- **Ultrasonic Sensor**: Measures distance for obstacle detection.
- **ESP32-CAM**: Captures video recordings upon accident detection.
- **Buzzer and Buttons**: Provides audible alerts and manual controls.
- **GPS Module**: Tracks location and time.

## Installation and Setup

1. **Hardware Setup**:
   - Connect the ESP32 with the necessary peripherals (MPU6050, SIM800L, GPS, ultrasonic sensor, OLED display, and buzzer).
   - Connect the ESP32-CAM to the same network as the ESP32.

2. **Software Setup**:
   - Install the required libraries:
     - `Adafruit_SSD1306`
     - `Adafruit_MPU6050`
     - `Adafruit_Sensor`
     - `TinyGPS++`
   - Upload the main system code to the ESP32.
   - Upload the camera module code to the ESP32-CAM.

3. **Configuration**:
   - Update the phone number in the code for SMS alerts.
   - Configure the Wi-Fi credentials for both the ESP32 and ESP32-CAM.

4. **Testing**:
   - Power on the system and verify component initialization.
   - Test accident detection by simulating G-force thresholds.
   - Verify SMS alerts and video recording functionality.

## Usage

- The system automatically monitors vehicle status and detects accidents.
- Use the web interface to check system status and trigger the ESP32-CAM manually.
- Press the reset button to reset the system or trigger a manual alert.

## Future Improvements

- Add support for cloud integration to store accident data.
- Enhance the web interface with more features and controls.
- Add real-time video streaming from the ESP32-CAM.

## License

This project is licensed under the MIT License.