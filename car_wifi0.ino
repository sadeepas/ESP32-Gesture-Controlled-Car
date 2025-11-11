/*
  ESP32_4WD_Car_WebSocket.ino
  --------------------------------
  • Board: ESP32 DevKit V1
  • Motor Driver: L293D Shield
  • Control: ESP-NOW (hand) + WebSocket (web app)
  • WebSocket: ws://192.168.4.1/ws (JSON: {type: 'control', x: float, y: float})
  • Telemetry: Sent every 500ms: {type: 'telemetry', payload: {obstacle_dist: int, battery: int, speed: float}}
  • Commands: E-STOP ('S'), AUTOPILOT ('A1'/'A0')
  • Optimized: No Serial, minimal libs for size
  • Libs: WebSockets (Links2004) + ArduinoJson
  • LED: On-board LED (GPIO 2) blinks during setup, steady ON when ready
  • Wiring: Based on provided summary
    - Right Motor: EN1 (D10) -> GPIO25, IN1 (D8) -> GPIO26, IN2 (D9) -> GPIO27
    - Left Motor: EN2 (D5) -> GPIO13, IN3 (D2) -> GPIO12, IN4 (D3) -> GPIO14
    - GND shared, Battery + (6-9V) to Shield EXT VIN
  • PWM: Updated for ESP32 Core 3.0+ (ledcAttach instead of ledcSetup/ledcAttachPin)
  --------------------------------
*/

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <esp_now.h>
#include <ArduinoJson.h>

// =================== Pin Configuration ===================
// Right Motor (Shield: EN1=D10, IN1=D8, IN2=D9)
#define PIN_MOTOR_R_EN 25
#define PIN_MOTOR_R_IN1 26
#define PIN_MOTOR_R_IN2 27

// Left Motor (Shield: EN2=D5, IN3=D2, IN4=D3)
#define PIN_MOTOR_L_EN 13
#define PIN_MOTOR_L_IN1 12
#define PIN_MOTOR_L_IN2 14

#define LED_PIN 2  // On-board LED for ESP32 DevKit V1

// =================== Motor Settings ===================
#define PWM_FREQ 5000
#define PWM_RES 8

#define SPEED_L1 150
#define SPEED_L2 200
#define SPEED_L3 255
#define MAX_SPEED 255

// =================== WiFi AP Settings ===================
const char* ssid = "ESP32Car";
const char* password = "12345678";

// =================== WebSocket ===================
WebSocketsServer webSocket = WebSocketsServer(81);  // Port 81

// =================== Telemetry (placeholders) ===================
int obstacle_dist = 100;  // Simulate or connect HC-SR04
int battery = 85;         // Simulate or read ADC
float speed = 0.0;        // Calculate from motors
unsigned long lastTelemetrySend = 0;
bool autoPilot = false;
unsigned long lastAutoToggle = 0;

// =================== ESP-NOW Data ===================
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} ControlData;

ControlData espNowData;

// =================== Function Prototypes ===================
void setMotor(char dir, uint8_t speed);
void processControl(float x, float y);
void processCommand(char cmd);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
void sendTelemetry();
void blinkLED(int times, int delayMs);
void setupLED();

// =================== Setup ===================
void setup() {
  // LED setup
  setupLED();

  // Pin setup
  pinMode(PIN_MOTOR_R_IN1, OUTPUT);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT);
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);

  // PWM setup (UPDATED FOR ESP32 CORE 3.0+: Use ledcAttach for auto-channel assignment)
  ledcAttach(PIN_MOTOR_R_EN, PWM_FREQ, PWM_RES);  // Right motor EN pin
  ledcAttach(PIN_MOTOR_L_EN, PWM_FREQ, PWM_RES);  // Left motor EN pin

  setMotor('S', 0);

  // WiFi AP
  WiFi.softAP(ssid, password);

  // Blink 5 times for WiFi AP setup
  blinkLED(5, 200);

  // WebSocket setup
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Blink 1 time to signal WiFi/WebSocket ready
  blinkLED(1, 200);

  // ESP-NOW setup
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onDataRecv);
    // ESP-NOW success: Blink slowly 8 times, then turn on LED steady
    blinkLED(8, 500);
    digitalWrite(LED_PIN, HIGH);
  } else {
    // Error: Blink rapidly forever
    while (true) {
      for(int j = 0; j < 20; j++) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(50);
      }
      delay(500);  // Pause between error bursts
    }
  }
}

// =================== Main Loop ===================
void loop() {
  webSocket.loop();

  // Send telemetry every 500ms
  if (millis() - lastTelemetrySend > 500) {
    sendTelemetry();
    lastTelemetrySend = millis();
  }

  // Simulate telemetry updates (replace with real sensors)
  if (millis() - lastAutoToggle > 10000 && autoPilot) {
    obstacle_dist = random(50, 200);
    battery = constrain(battery - 1, 0, 100);
    speed = random(0, 2);
  }
}

// =================== LED Functions ===================
void setupLED() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start off
}

void blinkLED(int times, int delayMs) {
  for(int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

// =================== WebSocket Event Handler ===================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      break;
    case WStype_CONNECTED:
      sendTelemetry();  // Welcome telemetry
      break;
    case WStype_TEXT:
      {
        // Reduced JSON size for efficiency
        StaticJsonDocument<128> doc;  // Smaller buffer
        DeserializationError error = deserializeJson(doc, payload);
        if (error) return;

        const char* msgType = doc["type"] | "";
        if (strcmp(msgType, "control") == 0) {
          float x = doc["x"] | 0.0;
          float y = doc["y"] | 0.0;
          processControl(x, y);
        } else if (strcmp(msgType, "command") == 0) {
          const char* cmd = doc["command"] | "";
          if (strlen(cmd) > 0) processCommand(cmd[0]);
        } else if (strcmp(msgType, "ping") == 0) {
          webSocket.sendTXT(num, "{\"type\":\"pong\"}");
        }
      }
      break;
  }
}

// =================== ESP-NOW Callback ===================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(ControlData)) return;
  memcpy(&espNowData, incomingData, sizeof(espNowData));

  float x = espNowData.x / 90.0;  // Normalize -1 to 1 (±90°)
  float y = espNowData.y / 90.0;
  processControl(x, y);
}

// =================== Control Processing ===================
void processControl(float x, float y) {
  char dir = 'S';
  uint8_t pwm_speed = SPEED_L2;

  if (!autoPilot) {
    if (y > 0.2) { dir = 'F'; pwm_speed = SPEED_L1 + abs(y) * (SPEED_L3 - SPEED_L1); }
    else if (y < -0.2) { dir = 'B'; pwm_speed = SPEED_L1 + abs(y) * (SPEED_L3 - SPEED_L1); }
    else if (x > 0.2) dir = 'R';
    else if (x < -0.2) dir = 'L';
  } else {
    // Auto mode: simple forward with obstacle check
    if (obstacle_dist > 30) { dir = 'F'; pwm_speed = SPEED_L2; }
    else { dir = 'S'; }
  }

  setMotor(dir, pwm_speed);
  speed = (pwm_speed / 255.0) * 2.0;  // Normalize to m/s approx
}

// =================== Command Processing ===================
void processCommand(char cmd) {
  switch (cmd) {
    case 'S':  // E-STOP
      setMotor('S', 0);
      autoPilot = false;
      break;
    case 'A':
      autoPilot = !autoPilot;
      lastAutoToggle = millis();
      break;
  }
}

// =================== Motor Control ===================
void setMotor(char dir, uint8_t speed) {
  speed = constrain(speed, 0, MAX_SPEED);

  switch (dir) {
    case 'F':
      digitalWrite(PIN_MOTOR_R_IN1, HIGH); digitalWrite(PIN_MOTOR_R_IN2, LOW);
      digitalWrite(PIN_MOTOR_L_IN1, HIGH); digitalWrite(PIN_MOTOR_L_IN2, LOW);
      break;
    case 'B':
      digitalWrite(PIN_MOTOR_R_IN1, LOW); digitalWrite(PIN_MOTOR_R_IN2, HIGH);
      digitalWrite(PIN_MOTOR_L_IN1, LOW); digitalWrite(PIN_MOTOR_L_IN2, HIGH);
      break;
    case 'L':
      digitalWrite(PIN_MOTOR_R_IN1, HIGH); digitalWrite(PIN_MOTOR_R_IN2, LOW);
      digitalWrite(PIN_MOTOR_L_IN1, LOW); digitalWrite(PIN_MOTOR_L_IN2, HIGH);
      break;
    case 'R':
      digitalWrite(PIN_MOTOR_R_IN1, LOW); digitalWrite(PIN_MOTOR_R_IN2, HIGH);
      digitalWrite(PIN_MOTOR_L_IN1, HIGH); digitalWrite(PIN_MOTOR_L_IN2, LOW);
      break;
    case 'S':
    default:
      digitalWrite(PIN_MOTOR_R_IN1, LOW); digitalWrite(PIN_MOTOR_R_IN2, LOW);
      digitalWrite(PIN_MOTOR_L_IN1, LOW); digitalWrite(PIN_MOTOR_L_IN2, LOW);
      speed = 0;
      break;
  }

  // UPDATED FOR ESP32 CORE 3.0+: Use ledcWrite with pin (auto-channel)
  ledcWrite(PIN_MOTOR_R_EN, speed);  // Right motor EN pin
  ledcWrite(PIN_MOTOR_L_EN, speed);  // Left motor EN pin
}

// =================== Telemetry Send ===================
void sendTelemetry() {
  StaticJsonDocument<128> doc;
  doc["type"] = "telemetry";
  JsonObject payload = doc.createNestedObject("payload");
  payload["obstacle_dist"] = obstacle_dist;
  payload["battery"] = battery;
  payload["speed"] = speed;

  String jsonStr;
  serializeJson(doc, jsonStr);
  webSocket.broadcastTXT(jsonStr);
}