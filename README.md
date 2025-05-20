Here’s a GitHub-formatted `README.md` you can include in your repository for the **GNSS SMS Alert System** project:

---

````markdown
# 🚨 GNSS SMS Alert System with Web Interface

An embedded system built on the **LilyGO T-A76XX Enhanced** board to detect vehicle crashes using an MPU6050 accelerometer, locate position via GNSS, and send alerts via SMS. A built-in web interface allows live monitoring and configuration.

---

## 📡 Features

- **Crash Detection via MPU6050**
  - Detects significant G-force impacts
  - Modular fallback if MPU is missing

- **GNSS Location**
  - Sends coordinates as a Google Maps link via SMS

- **Ultrasonic Proximity Alert**
  - Outputs HIGH when obstacle < threshold (default: 200 cm)
  - Automatically disables if sensor is not detected

- **Web Interface**
  - View last SMS report
  - Change ultrasonic sensitivity
  - Update recipient phone number  
  - Accessible at `http://gnss-alert.local`

- **Button Input**
  - Triggers SMS manually and resets crash status

---

## 🧰 Hardware Required

- LilyGO T-A76XX Enhanced Board  
- MPU6050 Accelerometer  
- Ultrasonic Sensor (e.g., HC-SR04)  
- SIM-enabled GSM module (SIM inserted)  
- Push button  
- Wi-Fi connection

---

## 🔧 Configuration

Update the following fields in the source code:

### Wi-Fi
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
````

### Recipient Phone Number

```cpp
String defaultRecipient = "+639XXXXXXXXX";
```

### Ultrasonic Threshold (in cm)

```cpp
int ultrasonicThreshold = 200;
```

---

## 🌐 Web Interface

After connecting to Wi-Fi, access the configuration UI at:

```
http://gnss-alert.local
```

Available controls:

* Modify ultrasonic sensitivity (distance in cm)
* Set new recipient phone number
* View latest SMS result

---

## 🚀 Getting Started

1. Clone or download this repository.
2. Open the project in Arduino IDE or PlatformIO.
3. Install required libraries:

   * `ESPmDNS`
   * `WiFi`
   * `WebServer`
   * `Adafruit_MPU6050`
   * `Adafruit_Sensor`
   * `NewPing`
4. Upload to the LilyGO T-A76XX board.
5. Monitor logs using the Serial Monitor at 115200 baud.

---

## 📦 Project Structure

```
📁 GNSS-SMS-Alert/
├── 📄 GNSS_SMS_Alert.ino
├── 📄 README.md
└── 📦 lib/
    └── (required libraries)
```

---

## 🛠️ Notes

* GNSS module will retry on failure and return "GNSS not fixed" if unavailable.
* Both MPU6050 and ultrasonic sensor checks are modular — the system defaults to a safe state if either is missing.
* The crash alert is only triggered once per event until reset.

---

## 📃 License

This project is open-source and available under the MIT License.

```

Let me know if you'd like this saved as a downloadable file or auto-generated as a GitHub repo template.
```
