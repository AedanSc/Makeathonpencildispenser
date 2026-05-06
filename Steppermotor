#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include <QTRSensors.h>
#include "controller_callbacks.h"
#include <ESP32Servo.h>

#define LEFT_MOTOR_FWD  16  // Left motor forward (IN1)
#define LEFT_MOTOR_BCK  17  // Left motor backward (IN2)
#define RIGHT_MOTOR_FWD 18  // Right motor forward (IN3)
#define RIGHT_MOTOR_BCK 19  // Right motor backward (IN4)

#define SERVO_PIN 13

// Servo settings
Servo armServo;
int servoAngle = 90; // start centered
int servoDirection = 1; // 1 for forward, -1 for backward
const int SERVO_STEP = 20;   // degrees to move per update while A/B held (increased speed)
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
const unsigned long SERVO_MOVE_INTERVAL = 1; // ms between servo steps (faster updates)

// Line sensor setup
QTRSensors qtr;
uint16_t sensors[8];  // Array to hold 8 sensor values

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Function to interpret sensor value as color
const char* getColor(uint16_t sensorValue) {
    // You might need to adjust this threshold based on your calibration
    const int THRESHOLD = 500;  // Adjust this value based on your testing
    return (sensorValue > THRESHOLD) ? "BLACK" : "WHITE";
}

void handleController(ControllerPtr myController) {
    // ===== SERVO CONTROL =====
    // A button: spin forward, B button: spin backward
    static unsigned long lastServoMove = 0;
    if (myController->a() || myController->b()) {
        unsigned long now = millis();
        if (now - lastServoMove >= SERVO_MOVE_INTERVAL) {
            if (myController->a()) {
                servoDirection = 1;   // Forward
            } else if (myController->b()) {
                servoDirection = -1;  // Backward
            }
            servoAngle += servoDirection * SERVO_STEP;
            // Bounce direction at limits for continuous rotation
            if (servoAngle >= SERVO_MAX) {
                servoAngle = SERVO_MAX;
                servoDirection = -servoDirection;
            } else if (servoAngle <= SERVO_MIN) {
                servoAngle = SERVO_MIN;
                servoDirection = -servoDirection;
            }
            armServo.write(servoAngle);
            lastServoMove = now;
        }
    }
    // ===== MOTOR CONTROL =====
    // Mapping adapted to your controller (left stick is primary):
    // Observed behaviour: left/right on stick map to forward/back, and
    // up/down map to turning. To match your description we remap as:
    // - baseSpeed = -axisX (left -> forward, right -> backward)
    // - steering  = -axisY (up -> left, down -> right)
    
    int raw_lx = myController->axisX(); // left stick X
    int raw_ly = myController->axisY(); // left stick Y

    // Small deadzone to avoid jitter
    const int DEADZONE = 8;
    if (abs(raw_lx) <= DEADZONE) raw_lx = 0;
    if (abs(raw_ly) <= DEADZONE) raw_ly = 0;

    // Compute target speeds (float for smoothing)
    // Remap for your controller so:
    // - left/right (raw_lx): left -> FORWARD, right -> BACKWARD
    // - up/down (raw_ly): up -> turn LEFT, down -> turn RIGHT
    float targetBase = map(-raw_lx, -127, 127, -255, 255); // left -> forward
    float targetSteer = map(raw_ly, -127, 127, -100, 100); // up (neg) -> negative steer -> left

    // Smooth targets to avoid jerky motion (simple low-pass filter)
    static float smoothBase = 0.0f;
    static float smoothSteer = 0.0f;
    const float ALPHA = 0.5f; // smoothing factor (0..1). Higher = more responsive
    smoothBase += (targetBase - smoothBase) * ALPHA;
    smoothSteer += (targetSteer - smoothSteer) * ALPHA;

    int leftMotorSpeed = (int)round(smoothBase + smoothSteer);
    int rightMotorSpeed = (int)round(smoothBase - smoothSteer);

    // If both very small, stop to avoid tiny pwm jitter
    if (abs(leftMotorSpeed) < 3) leftMotorSpeed = 0;
    if (abs(rightMotorSpeed) < 3) rightMotorSpeed = 0;

    // Constrain to valid PWM range
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
    
    // Apply speeds to left motor
    if (leftMotorSpeed > 0) {
        analogWrite(LEFT_MOTOR_FWD, leftMotorSpeed);
        analogWrite(LEFT_MOTOR_BCK, 0);
        digitalWrite(LEFT_MOTOR_BCK, LOW);
    } else if (leftMotorSpeed < 0) {
        analogWrite(LEFT_MOTOR_BCK, -leftMotorSpeed);
        analogWrite(LEFT_MOTOR_FWD, 0);
        digitalWrite(LEFT_MOTOR_FWD, LOW);
    } else {
        analogWrite(LEFT_MOTOR_FWD, 0);
        analogWrite(LEFT_MOTOR_BCK, 0);
        digitalWrite(LEFT_MOTOR_FWD, LOW);
        digitalWrite(LEFT_MOTOR_BCK, LOW);
    }
    
    // Apply speeds to right motor
    if (rightMotorSpeed > 0) {
        analogWrite(RIGHT_MOTOR_FWD, rightMotorSpeed);
        analogWrite(RIGHT_MOTOR_BCK, 0);
        digitalWrite(RIGHT_MOTOR_BCK, LOW);
    } else if (rightMotorSpeed < 0) {
        analogWrite(RIGHT_MOTOR_BCK, -rightMotorSpeed);
        analogWrite(RIGHT_MOTOR_FWD, 0);
        digitalWrite(RIGHT_MOTOR_FWD, LOW);
    } else {
        analogWrite(RIGHT_MOTOR_FWD, 0);
        analogWrite(RIGHT_MOTOR_BCK, 0);
        digitalWrite(RIGHT_MOTOR_FWD, LOW);
        digitalWrite(RIGHT_MOTOR_BCK, LOW);
    }
}


void setup() {
    Serial.begin(115200);
    
    // Initialize Bluetooth controller
    uni_bt_allowlist_set_enabled(true); // Enable allowlist first
    BP32.setup(&onConnectedController, &onDisconnectedController);
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress info log spam from gpio_isr_service
    
    // Initialize motor pins
    pinMode(LEFT_MOTOR_FWD, OUTPUT);
    pinMode(LEFT_MOTOR_BCK, OUTPUT);
    pinMode(RIGHT_MOTOR_FWD, OUTPUT);
    pinMode(RIGHT_MOTOR_BCK, OUTPUT);
    
    // Initialize line sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]) {35, 32, 33, 25, 26, 27, 14, 12}, 8);  // 8 sensors from left to right
    
    // Calibrate line sensors
    Serial.println("Starting line sensor calibration...");
    for (uint8_t i = 0; i < 250; i++) {
        if (i % 10 == 0) {  // Print status every 10 iterations
            Serial.printf("Calibrating %d/250\n", i);
        }
        qtr.calibrate();
        delay(20);
    }
    Serial.println("Line sensor calibration complete!");
    // Attach servo and set to initial angle
    armServo.attach(SERVO_PIN);
    armServo.write(servoAngle);
}

void loop() {
    static unsigned long lastDebugTime = 0;
    static unsigned long lastSensorTime = 0;
    
    BP32.update(); // Update the gamepad state
    
    // Read line sensors every 100ms
    unsigned long currentTime = millis();
    if (currentTime - lastSensorTime >= 100) {
        qtr.readLineBlack(sensors);
        // Print all 8 sensor values
        Serial.printf("Sensors (L->R): ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%d:%d(%s) ", i+1, sensors[i], getColor(sensors[i]));
            if (i == 3) Serial.printf("| "); // Visual separator in middle
        }
        Serial.printf("\n");
        lastSensorTime = currentTime;
    }
    
    // Print debug info every 5 seconds if no controller is connected
    if (currentTime - lastDebugTime >= 5000) {
        bool anyConnected = false;
        for (auto myController : myControllers) {
            if (myController && myController->isConnected()) {
                anyConnected = true;
                Serial.printf("Controller connected - Battery: %d%%\n", myController->battery());
            }
        }
        if (!anyConnected) {
            Serial.println("Waiting for controller connection...");
        }
        lastDebugTime = currentTime;
    }
    
    // Handle connected controllers
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            handleController(myController);
        }
    }
    
    vTaskDelay(1); // Small delay to prevent watchdog issues
}
