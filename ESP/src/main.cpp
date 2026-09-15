#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"

// --- WiFi Access Point settings ---
const char* AP_SSID = "CarRobotPS4";
const char* AP_PASSWORD = "robot1234";  // must be at least 8 characters
const unsigned int UDP_PORT = 4210;

WiFiUDP udp;
char packetBuffer[64];
String serialBuffer = "";

// --- WiFi telemetry settings ---
// Broadcasts live speed readings on a separate UDP port so a laptop app can
// display them wirelessly without interfering with the control channel above.
const unsigned int TELEMETRY_PORT = 4211;
const unsigned long TELEMETRY_INTERVAL_MS = 100;  // ~10 updates/sec

WiFiUDP telemetryUdp;
IPAddress broadcastIP(192, 168, 4, 255);
unsigned long lastTelemetryMillis = 0;

// Safety watchdog: if no manual command arrives within this window, stop.
const unsigned long COMMAND_TIMEOUT_MS = 500;
unsigned long lastPacketMillis = 0;

// --- Operating mode ---
// MANUAL: driven by PS4 commands over WiFi/USB Serial.
// LINE_FOLLOW: driven by the line error received over UART2 from the AVR board.
enum RobotMode { MODE_MANUAL, MODE_LINE_FOLLOW };
RobotMode currentMode = MODE_LINE_FOLLOW;

// --- Line follower settings ---
// UART2 pins used to talk to the AVR sensor board.
const int LINE_RX_PIN = 16;
const int LINE_TX_PIN = 17;

const int LINE_BASE_RPM = 60;    // forward speed while following the line
const float LINE_KP = 0.80;      // how strongly the error steers the wheels
const unsigned long LINE_ERROR_TIMEOUT_MS = 300;  // stop if AVR link is lost

int lineError = 0;
unsigned long lastLineErrorMillis = 0;
String uartBuffer = "";
bool obstacleStop = false;   // true while the AVR reports something too close

// Right motor pins (both PWM-capable)
const int IN1 = 27;
const int IN2 = 26;
// Left motor pins (both PWM-capable)
const int IN3 = 25;
const int IN4 = 14;

// LEDC channels: one per pin
const int CH_IN1 = 0;
const int CH_IN2 = 1;
const int CH_IN3 = 2;
const int CH_IN4 = 3;

const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;  // 8-bit -> values 0 to 255

// Encoder pins
const int ENCODER_A_LEFT  = 32;
const int ENCODER_B_LEFT  = 33;
const int ENCODER_A_RIGHT = 19;
const int ENCODER_B_RIGHT = 21;

volatile long pulseCount_Left = 0;
volatile long pulseCount_Right = 0;

unsigned long Pervious_Millis = 0;
unsigned long Current_Millis = 0;
unsigned int Period = 20; // ms - control loop interval

float Rpm_Left_Wheel = 0;
float Rpm_Right_Whell = 0;

int leftSpeed = 0;
int rightSpeed = 0;

const int MAX_TARGET_RPM = 240;
int targetLeft_Rpm = 0;
int targetRight_Rpm = 0;

float rampStep = 5.0; // RPM change per control loop
float rampSetpoint_Left = 0.0;
float rampSetpoint_Right = 0.0;

float Kp = 3.0, Ki = 1.0, Kd = 0.0;

float integral_Left = 0, lastError_Left = 0;
float integral_Right = 0, lastError_Right = 0;

// RPM below this = "stopped", regardless of noise or leftover integral
const float STOP_THRESHOLD_RPM = 20.0;
// RPM below this = ignore as encoder noise (used before PID sees it)
const float NOISE_DEADBAND_RPM = 5.0;

void setMotor(int chA, int chB, int speed);
void stopMotors();
void parseAndApply(String line);
void updateRamp(float &ramped, float target);
float computePID(float setpoint, float actual, float &integral, float &lastError, float dt);
void readLineError();
void sendTelemetry();

void IRAM_ATTR InterruptFunction_Left_Wheel(void) {
  if (digitalRead(ENCODER_A_LEFT) == 1) pulseCount_Left--;
  else pulseCount_Left++;
}

void IRAM_ATTR InterruptFunction_Right_Wheel(void) {
  if (digitalRead(ENCODER_A_RIGHT) == 1) pulseCount_Right--;
  else pulseCount_Right++;
}

void updateRamp(float &ramped, float target) {
  if (ramped < target) {
    ramped += rampStep;
    if (ramped > target) ramped = target;
  } else if (ramped > target) {
    ramped -= rampStep;
    if (ramped < target) ramped = target;
  }
}

float computePID(float setpoint, float actual, float &integral, float &lastError, float dt) {
  float error = setpoint - actual;
  float derivative = (error - lastError) / dt;
  lastError = error;

  // Provisional output using current integral (before deciding whether to accumulate more)
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Conditional integration: only accumulate if not already saturated
  if (output < 255 && output > -255) {
    integral += error * dt;
  }

  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  output = constrain(output, -255, 255);
  return output;
}

void Get_Rpm(void)
{
    Current_Millis = millis();

    if (Current_Millis - Pervious_Millis >= Period) {

        float dt = (Current_Millis - Pervious_Millis) / 1000.0;
        Pervious_Millis = Current_Millis;

        Rpm_Left_Wheel  = ((pulseCount_Left  / 21.3)  / 11.0) * 60.0 / dt;
        Rpm_Right_Whell = ((pulseCount_Right / 21.30) / 11.0) * 60.0 / dt;
        pulseCount_Left = 0;
        pulseCount_Right = 0;

        // Ignore tiny encoder noise when robot is essentially still
        if (abs(Rpm_Left_Wheel) < NOISE_DEADBAND_RPM)  Rpm_Left_Wheel = 0;
        if (abs(Rpm_Right_Whell) < NOISE_DEADBAND_RPM) Rpm_Right_Whell = 0;

        // Serial.printf("RPM_Left: %.1f  RPM_Right: %.1f\n", Rpm_Left_Wheel, Rpm_Right_Whell);

        float pwmLeft, pwmRight;

        // ---- LEFT WHEEL ----
        if (targetLeft_Rpm == 0 && abs(Rpm_Left_Wheel) < STOP_THRESHOLD_RPM) {
            // Fully stopped: cut power and clear all PID state
            pwmLeft = 0;
            integral_Left = 0;
            lastError_Left = 0;
            rampSetpoint_Left = 0;
        } else {
            updateRamp(rampSetpoint_Left, targetLeft_Rpm);
            pwmLeft = computePID(rampSetpoint_Left, Rpm_Left_Wheel, integral_Left, lastError_Left, dt);
        }

        // ---- RIGHT WHEEL ----
        if (targetRight_Rpm == 0 && abs(Rpm_Right_Whell) < STOP_THRESHOLD_RPM) {
            pwmRight = 0;
            integral_Right = 0;
            lastError_Right = 0;
            rampSetpoint_Right = 0;
        } else {
            updateRamp(rampSetpoint_Right, targetRight_Rpm);
            pwmRight = computePID(rampSetpoint_Right, Rpm_Right_Whell, integral_Right, lastError_Right, dt);
        }

        setMotor(CH_IN3, CH_IN4, (int)constrain(pwmLeft, -255, 255));
        setMotor(CH_IN1, CH_IN2, (int)constrain(pwmRight, -255, 255));
    }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_AP);

  WiFi.softAPConfig(
    IPAddress(192, 168, 4, 1),
    IPAddress(192, 168, 4, 1),
    IPAddress(255, 255, 255, 0)
  );

  WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4);

  esp_wifi_set_ps(WIFI_PS_NONE);  // disable power save - fixes intermittent connection failures
  Serial.print("Access Point started. IP address: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(UDP_PORT);
  Serial.printf("Listening for UDP packets on port %d\n", UDP_PORT);
  lastPacketMillis = millis();

  telemetryUdp.begin(TELEMETRY_PORT);
  Serial.printf("Broadcasting telemetry on UDP port %d\n", TELEMETRY_PORT);

  ledcSetup(CH_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_IN3, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_IN4, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(IN1, CH_IN1);
  ledcAttachPin(IN2, CH_IN2);
  ledcAttachPin(IN3, CH_IN3);
  ledcAttachPin(IN4, CH_IN4);

  pinMode(ENCODER_A_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_B_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_A_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_B_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_B_LEFT), InterruptFunction_Left_Wheel, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_RIGHT), InterruptFunction_Right_Wheel, RISING);

  Serial2.begin(9600, SERIAL_8N1, LINE_RX_PIN, LINE_TX_PIN);
  Serial.println("Serial2 ready for AVR line-follower link.");

  stopMotors();
  Serial.println("Ready. LINE_FOLLOW mode. Send leftSpeed,rightSpeed via UDP/Serial, or MODE:TOGGLE to switch.");
}

void loop() {
  // --- Read manual commands (WiFi UDP) ---
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      parseAndApply(String(packetBuffer));
    }
  }

  // --- Read manual commands (USB Serial) ---
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      parseAndApply(serialBuffer);
      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;
    }
  }

  // --- Read line-follower error / obstacle status (UART2 from AVR) ---
  readLineError();

  // --- Decide targets based on current mode ---
  if (currentMode == MODE_MANUAL) {
    // Safety: if no manual command arrived recently (WiFi out of range,
    // laptop closed, etc.), stop instead of running on a stale command.
    if (millis() - lastPacketMillis > COMMAND_TIMEOUT_MS) {
      targetLeft_Rpm = 0;
      targetRight_Rpm = 0;
    }
  } else {  // MODE_LINE_FOLLOW
    if (obstacleStop) {
      // AVR reports something too close - stop immediately.
      targetLeft_Rpm = 0;
      targetRight_Rpm = 0;
    } else if (millis() - lastLineErrorMillis > LINE_ERROR_TIMEOUT_MS) {
      // AVR link lost - stop instead of steering blind on stale data.
      targetLeft_Rpm = 0;
      targetRight_Rpm = 0;
    } else {
      // Positive error = drifted right of the line -> speed up left wheel,
      // slow down right wheel to steer back left. Flip LINE_KP's sign if
      // your sensor layout gives the opposite convention.
      int correction = (int)(LINE_KP * lineError);
      targetLeft_Rpm = constrain(LINE_BASE_RPM + correction, -MAX_TARGET_RPM, MAX_TARGET_RPM);
      targetRight_Rpm = constrain(LINE_BASE_RPM - correction, -MAX_TARGET_RPM, MAX_TARGET_RPM);
    }
  }

  Get_Rpm();

  // --- Broadcast speed telemetry over WiFi at a throttled rate ---
  // Independent of the Period (20ms) control loop above; telemetry doesn't
  // need to be that frequent, and sending too often can flood the WiFi link.
  if (millis() - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
    sendTelemetry();
    lastTelemetryMillis = millis();
  }
}

void parseAndApply(String line) {
  line.trim();

  if (line == "MODE:TOGGLE") {
    currentMode = (currentMode == MODE_MANUAL) ? MODE_LINE_FOLLOW : MODE_MANUAL;
    Serial.println(currentMode == MODE_MANUAL ? "Mode: MANUAL" : "Mode: LINE_FOLLOW");
    // Reset targets and timers so switching modes never carries over a
    // stale command or triggers a false watchdog stop right away.
    targetLeft_Rpm = 0;
    targetRight_Rpm = 0;
    lastPacketMillis = millis();
    lastLineErrorMillis = millis();
    return;
  }

  int commaIndex = line.indexOf(',');
  if (commaIndex == -1) return;

  leftSpeed = line.substring(0, commaIndex).toInt();
  rightSpeed = line.substring(commaIndex + 1).toInt();

  leftSpeed = constrain(leftSpeed, -MAX_TARGET_RPM, MAX_TARGET_RPM);
  rightSpeed = constrain(rightSpeed, -MAX_TARGET_RPM, MAX_TARGET_RPM);

  lastPacketMillis = millis();

  // Only apply manual speed commands while in manual mode; in line-follow
  // mode the targets are computed from the sensor error instead.
  if (currentMode == MODE_MANUAL) {
    targetLeft_Rpm = leftSpeed;
    targetRight_Rpm = rightSpeed;
  }
}

// Reads whatever is available on UART2 and, once a full line arrives,
// updates lineError (from "E:<integer>\n") or obstacleStop (from "S:1\n").
void readLineError() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      uartBuffer.trim();
      if (uartBuffer.startsWith("E:")) {
        lineError = uartBuffer.substring(2).toInt();
        obstacleStop = false;   // a normal line reading means the path is clear
        lastLineErrorMillis = millis();
      } else if (uartBuffer.startsWith("S:")) {
        obstacleStop = true;
        lastLineErrorMillis = millis();  // link is alive, just reporting a stop
        Serial.println("Obstacle detected - stopping.");
      }
      uartBuffer = "";
    } else if (c != '\r') {
      uartBuffer += c;
    }
  }
}

void setMotor(int chA, int chB, int speed) {
  if (speed > 0) {
    ledcWrite(chA, speed);
    ledcWrite(chB, 0);
  } else if (speed < 0) {
    ledcWrite(chA, 0);
    ledcWrite(chB, -speed);
  } else {
    ledcWrite(chA, 0);
    ledcWrite(chB, 0);
  }
}

void stopMotors() {
  setMotor(CH_IN1, CH_IN2, 0);
  setMotor(CH_IN3, CH_IN4, 0);
}

// Broadcasts "L:<rpm> R:<rpm>" over UDP so any laptop connected to the AP
// can display live speed without a wired connection to the robot.
void sendTelemetry() {
  char message[32];
  snprintf(message, sizeof(message), "L:%.1f R:%.1f", Rpm_Left_Wheel, Rpm_Right_Whell);

  telemetryUdp.beginPacket(broadcastIP, TELEMETRY_PORT);
  telemetryUdp.print(message);
  telemetryUdp.endPacket();
}