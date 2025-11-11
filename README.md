# ESP32-Gesture-Controlled-Car
A gesture-controlled 4WD car using ESP32, MPU6050, and ESP-NOW, with a real-time web interface for control and telemetry via WebSocket.

# ESP32 Gesture & Web Controlled Car


A feature-rich, gesture-controlled 4WD car built with the ESP32. This project combines low-latency **ESP-NOW** for hand-gesture control with a sophisticated **WebSocket-based web dashboard** for remote control and real-time telemetry monitoring.

The system consists of two main parts:
1.  **The Car**: An ESP32-powered 4WD chassis with an L293D motor driver that creates its own Wi-Fi AP.
2.  **The Hand Controller**: An ESP32 with an MPU6050 gyroscope that reads hand gestures and transmits them to the car via ESP-NOW.
3.  **The Web Interface**: A fully responsive and modern command center, accessible from any browser (PC or mobile), that connects to the car's WebSocket server for control and data visualization.

---

## ✨ Key Features

*   **Dual Control Modes**:
    *   **Gesture Control**: Intuitive, low-latency control using hand gestures via ESP-NOW.
    *   **Web Interface**: A beautiful and responsive dashboard for control via a virtual joystick, accessible from a PC or smartphone.
*   **Real-Time Telemetry**: The web dashboard displays live data from the car, including:
    *   Obstacle Distance (simulated, can be replaced with a real sensor)
    *   Battery Level (simulated)
    *   Current Speed
*   **Live Telemetry Graph**: A dynamic chart plots speed, battery, and obstacle data over time.
*   **Autopilot & E-STOP**: Critical safety and autonomous command features accessible from the web interface.
*   **Robust Connectivity**:
    *   The car hosts a Wi-Fi Access Point for a reliable, direct connection.
    *   The hand controller automatically pairs with the car using its MAC address.
*   **Optimized Code**: The Arduino code is optimized to be lightweight, avoiding the use of the Serial monitor to maximize performance.
*   **Responsive Design**: The HTML interface adapts seamlessly to both desktop and mobile screens for a great user experience on any device.

---

## 🛠️ Hardware & Software

### Hardware Components

| Component              | Quantity | Purpose                                  |
| ---------------------- | :------: | ---------------------------------------- |
| ESP32 DevKit V1        |    2     | 1 for the car, 1 for the hand controller |
| L293D Motor Driver Shield |    1     | To drive the 4 DC motors                   |
| MPU6050 Gyro/Accelerometer |    1     | To detect hand gestures                  |
| 4WD Car Chassis        |    1     | Base frame, motors, and wheels           |
| Battery Pack (6-9V)    |    1     | To power the motors and ESP32 on the car |
| Jumper Wires           |   Set    | For wiring the components                |

### Software & Libraries

*   **Arduino IDE** or **PlatformIO**
*   **ESP32 Board Support** for Arduino IDE
*   **Arduino Libraries**:
    *   `WebSockets` by Links2004
    *   `ArduinoJson`
    *   `MPU6050_tockn`

---

## 🔧 Setup and Installation

### 1. Configure Arduino IDE

1.  Make sure you have the **ESP32 board manager** installed in your Arduino IDE.
2.  Install the required libraries through the Arduino Library Manager:
    *   Go to **Sketch -> Include Library -> Manage Libraries...**
    *   Search for and install `WebSockets` by Markus Sattler.
    *   Search for and install `ArduinoJson`.
    *   Search for and install `MPU6050_tockn`.

### 2. Flash the Car Firmware (`car_wifi0.ino`)

1.  Open `car_wifi0.ino` in the Arduino IDE.
2.  Connect the ESP32 designated for the car to your computer.
3.  Select the correct board (e.g., "ESP32 Dev Module") and COM port.
4.  Upload the sketch. This ESP32 will now create a Wi-Fi network named **"ESP32Car"**.
5.  **Important**: After uploading, find the car ESP32's MAC address. You can do this by running a simple MAC address scanner sketch or checking your router if it connects to another network temporarily.

### 3. Flash the Hand Controller Firmware (`reciver.ino`)

1.  Open `reciver.ino` in the Arduino IDE.
2.  In the code, find the following line:
    ```cpp
    uint8_t receiverMacAddress[] = {0x88, 0x57, 0x21, 0xB6, 0xD3, 0x50};
    ```
3.  **Replace** this MAC address with the MAC address of your car's ESP32.
4.  Connect the ESP32 for the hand controller.
5.  Select the correct board and COM port.
6.  Upload the sketch.

### 4. Wiring

**Car Wiring (ESP32 to L293D Shield):**

| ESP32 Pin | L293D Shield Pin | Function        |
| :-------: | :--------------: | --------------- |
|   GPIO25  |       EN1        | Right Motor PWM |
|   GPIO26  |       IN1        | Right Motor Dir |
|   GPIO27  |       IN2        | Right Motor Dir |
|   GPIO13  |       EN2        | Left Motor PWM  |
|   GPIO12  |       IN3        | Left Motor Dir  |
|   GPIO14  |       IN4        | Left Motor Dir  |
|    GND    |       GND        | Common Ground   |

*   Power the L293D shield's `EXT_VIN` terminal with your 6-9V battery pack. Ensure the GNDs are connected.

**Hand Controller Wiring (ESP32 to MPU6050):**

| ESP32 Pin | MPU6050 Pin | Function |
| :-------: | :---------: | -------- |
|   GPIO21  |     SDA     | I2C Data |
|   GPIO22  |     SCL     | I2C Clock|
|    3V3    |     VCC     | Power    |
|    GND    |     GND     | Ground   |

---

## 🚀 How to Use

1.  **Power On**: Power up both the car and the hand controller.
    *   The car's onboard LED will blink during setup and turn solid ON when ready.
    *   The hand controller's LED will blink while connecting and turn solid ON once paired with the car.
2.  **Connect to Wi-Fi**: On your computer or smartphone, connect to the Wi-Fi network named **`ESP32Car`**. The password is **`12345678`**.
3.  **Open the Web Interface**:
    *   Open `hand controled car pc.html` (for desktop) or `hand controled car phone.html` (for mobile) in a web browser.
    *   Click the "Connect" button (or go to settings) and ensure the WebSocket URL is `ws://192.168.4.1/ws` (Port 81 is often blocked, so this code uses the default port 80, which is implied).
    *   Click "Connect". The status indicator should turn green, and telemetry data will start appearing.
4.  **Control the Car**:
    *   **With Hand Gestures**: Tilt the hand controller forward/backward to move and left/right to turn.
    *   **With the Web App**: Use the virtual joystick on the screen for manual control. You can also use the E-STOP and Autopilot buttons.

---

## 📁 Code Structure

*   **`car_wifi0.ino`**: The main firmware for the car. It handles motor control, sets up the Wi-Fi AP, runs the WebSocket server, and receives commands from both ESP-NOW (hand controller) and WebSocket (web app).
*   **`reciver.ino`**: The firmware for the hand controller. It initializes the MPU6050, reads gesture data, and transmits it to the car using the ESP-NOW protocol.
*   **`hand controled car pc.html`**: The command center interface designed for desktop browsers. It includes a large layout with joystick, telemetry panels, and a real-time graph.
*   **`hand controled car phone.html`**: A mobile-first, responsive version of the command center. It uses a tabbed navigation system to provide a seamless experience on smaller screens.

---

## 📜 License

This project is open-source. Feel free to modify, distribute, and use it for your own projects. Please give credit if you use it in your work.

---

## 🙏 Acknowledgments

*   The **ESP32 community** for providing excellent documentation and support.
*   The creators of the **Arduino libraries** that made this project possible.
