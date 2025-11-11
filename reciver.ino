/*
  HandController_ESP32_DevKitV1.ino
  --------------------------------
  • Board: ESP32 DevKit V1 (ESP32-WROOM-32)
  • Sensor: MPU6050 (I2C: SDA=21, SCL=22)
  • Control: Reads orientation (roll, pitch, yaw) and sends via ESP-NOW to Car Receiver
  • Compatible: ESP32 Core v3.x (IDF v5.x)
  • Optimized: Minimal Serial (optional debug)
  • Libs: MPU6050_tockn (install via Library Manager)
  • LED: On-board LED (GPIO 2) blinks during ESP-NOW connection setup, steady ON when connected
  --------------------------------
*/

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// =================== LED Pin ===================
#define LED_PIN 2  // On-board LED for ESP32 DevKit V1

// =================== Objects ===================
MPU6050 mpu6050(Wire);

// =================== Receiver MAC Address ===================
//  CAR ESP32 MAC ADDRESS 
uint8_t receiverMacAddress[] = {0x88, 0x57, 0x21, 0xB6, 0xD3, 0x50};
 

// =================== Struct (matches Car Receiver) ===================
typedef struct ControlData {
  int16_t x;  // Roll (left/right tilt)
  int16_t y;  // Pitch (forward/back tilt)
  int16_t z;  // Yaw (rotation)
} ControlData;

ControlData controlData;

// =================== Callbacks ===================
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
// For ESP32 Core v3.x
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
// For ESP32 Core v2.x
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  // Optional: Uncomment for debug
  // Serial.print("Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// =================== Setup ===================
void setup() {
  // Uncomment for debug: Serial.begin(115200);
  // Serial.println("=== ESP32 Hand Controller ===");

  // LED setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start off

  Wire.begin(21, 22);  // SDA, SCL

  // Initialize MPU6050
  mpu6050.begin();
  mpu6050.calcGyroOffsets();  // Calibrate - hold steady for ~5s
  // Serial.println("MPU6050 ready - Calibrating...");

  // WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);

  // Blink LED while "connecting" (before ESP-NOW init and peer add)
  for(int i = 0; i < 10; i++) {  // Blink 10 times (about 2 seconds)
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // Serial.println("ESP-NOW init failed!");
    // Error: Blink rapidly forever
    while (true) {
      for(int j = 0; j < 20; j++) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(50);
      }
      delay(500);  // Pause between error bursts
    }
  }
  // Serial.println("ESP-NOW initialized");

  // Register send callback
  esp_now_register_send_cb(onDataSent);

  // Add peer (Car Receiver)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  // Channel 0 (auto)
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    // Serial.println("Failed to add peer!");
    // Error: Blink rapidly forever
    while (true) {
      for(int j = 0; j < 20; j++) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(50);
      }
      delay(500);  // Pause between error bursts
    }
  }
  // Serial.println("Peer added - Ready to control!");

  // ESP-NOW connected successfully: Turn on LED steady
  digitalWrite(LED_PIN, HIGH);
}

// =================== Loop ===================
void loop() {
  mpu6050.update();  // Read MPU6050

  // Map angles to control data (-90 to +90 degrees)
  controlData.x = (int16_t)mpu6050.getAngleX();  // Roll for steering (left/right)
  controlData.y = (int16_t)mpu6050.getAngleY();  // Pitch for speed (forward/back)
  controlData.z = (int16_t)mpu6050.getAngleZ();  // Yaw for rotation (optional)

  // Optional debug print
  // Serial.printf("X:%d | Y:%d | Z:%d\n", controlData.x, controlData.y, controlData.z);

  // Send via ESP-NOW
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&controlData, sizeof(controlData));

  // Optional: Check result
  // if (result != ESP_OK) {
  //   Serial.printf("Send error: %d\n", result);
  // }

  delay(100);  // 10 Hz update rate - adjust for responsiveness
}