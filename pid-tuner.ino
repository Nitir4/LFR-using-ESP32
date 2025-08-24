/*******************************************************
 * ESP32 Line Follower + Web UI Tuning (PID + Weights + Base Speed)
 * - WiFi SoftAP: ESP32_LineFollower / 12345678  ->  http://192.168.4.1/
 * - Tune Kp, Ki, Kd, Base Speed, and 8 sensor weights in real time
 *******************************************************/

#include <WiFi.h>
#include <WebServer.h>

/* ----------------- Motor control pins ----------------- */
#define rightMotorF   27  // Right motor forward
#define rightMotorB   14  // Right motor backward
#define rightMotorPWM 12  // Right motor PWM (ENA)
#define leftMotorF    25  // Left motor forward
#define leftMotorB    33  // Left motor backward
#define leftMotorPWM  32  // Left motor PWM (ENB)
#define stby          26  // Motor driver standby/enable (if used)

/* ----------------- IR sensor pins ----------------- */
const int numSensors = 8;
int irSensors[numSensors] = {19, 18, 5, 17, 16, 4, 2, 15};

/* ----------------- PID & Control ----------------- */
float kp = 25;     // defaults (editable via web)
float ki = 1;
float kd = 20;
int   baseSpeed = 200; // 0..255

// Default sensor weights (editable via web)
int weights[numSensors] = { -1000, -1000, -500, 0, 0, 500, 1000, 1000 };

// PID state
long integral = 0;
int  previousError = 0;

// Run toggle (via UI)
volatile bool runEnabled = false;

/* ----------------- WiFi / Web ----------------- */
const char* AP_SSID = "ESP32_LineFollower";
const char* AP_PASS = "12345678";
WebServer server(80);

/* ----------------- Forward Declarations ----------------- */
int  calculateError(int sensorStates[]);
int  calculatePID(int error);
void driveMotors(int leftSpeed, int rightSpeed);
String htmlPage();

/* ================== Setup ================== */
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby, OUTPUT);
  digitalWrite(stby, HIGH); // enable motor driver (HIGH usually enables)

  // Sensor pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(irSensors[i], INPUT);
  }

  // SoftAP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Routes
  server.on("/", []() { server.send(200, "text/html", htmlPage()); });

  server.on("/update", []() {
    // Parse PID/base
    if (server.hasArg("kp"))   kp = server.arg("kp").toFloat();
    if (server.hasArg("ki"))   ki = server.arg("ki").toFloat();
    if (server.hasArg("kd"))   kd = server.arg("kd").toFloat();
    if (server.hasArg("base")) baseSpeed = constrain(server.arg("base").toInt(), 0, 255);

    // Parse weights
    for (int i = 0; i < numSensors; i++) {
      String key = "w" + String(i);
      if (server.hasArg(key)) {
        weights[i] = server.arg(key).toInt();
      }
    }
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "Updated");
  });

  server.on("/start", []() {
    runEnabled = true;
    integral = 0;            // reset PID integrator when starting
    previousError = 0;
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "Started");
  });

  server.on("/stop", []() {
    runEnabled = false;
    driveMotors(0, 0);
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "Stopped");
  });

  server.on("/status", []() {
    String s = "{";
    s += "\"kp\":" + String(kp) + ",";
    s += "\"ki\":" + String(ki) + ",";
    s += "\"kd\":" + String(kd) + ",";
    s += "\"base\":" + String(baseSpeed) + ",";
    s += "\"run\":" + String(runEnabled ? 1 : 0) + ",";
    s += "\"weights\":[";
    for (int i = 0; i < numSensors; i++) {
      s += String(weights[i]);
      if (i < numSensors - 1) s += ",";
    }
    s += "]}";
    server.send(200, "application/json", s);
  });

  server.begin();
}

/* ================== Loop ================== */
void loop() {
  // Service web server fast
  server.handleClient();

  if (!runEnabled) {
    // When stopped, keep motors off and loop quickly
    driveMotors(0, 0);
    delay(2);
    return;
  }

  // --- Read sensors (digital) ---
  int sensorStates[numSensors];
  for (int i = 0; i < numSensors; i++) {
    sensorStates[i] = digitalRead(irSensors[i]); // HIGH on line, LOW off line (depends on your sensor)
  }

  // --- Compute error & PID ---
  int error = calculateError(sensorStates);
  int motorSpeedDifference = calculatePID(error);

  // --- Compute motor speeds ---
  int leftSpeed  = baseSpeed + motorSpeedDifference;
  int rightSpeed = baseSpeed - motorSpeedDifference;
  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // --- Drive motors ---
  driveMotors(leftSpeed, rightSpeed);

  // Small delay to keep loop snappy but stable
  delay(1);
}

/* ================== Helpers ================== */

// Weighted position error from active sensors
int calculateError(int sensorStates[]) {
  long weightedSum = 0;
  int  active = 0;

  for (int i = 0; i < numSensors; i++) {
    if (sensorStates[i] == HIGH) {
      weightedSum += weights[i];
      active++;
    }
  }

  if (active == 0) {
    // No line seen — keep last error (or implement search behavior here)
    return previousError;
  }
  return (int)(weightedSum / active);
}

int calculatePID(int error) {
  // Integrator
  integral += error;
  // Anti-windup
  integral = constrain(integral, -10000, 10000);

  // Derivative
  int derivative = error - previousError;

  // PID
  float out = (kp * error) + (ki * integral) + (kd * derivative);

  previousError = error;
  return (int)out;
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // LEFT
  if (leftSpeed > 0) {
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, -leftSpeed);
  } else {
    // stop
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
  }

  // RIGHT
  if (rightSpeed > 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, HIGH);
    analogWrite(rightMotorPWM, -rightSpeed);
  } else {
    // stop
    digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 0);
  }
}

/* ----------------- HTML UI ----------------- */
String htmlPage() {
  String page = "";
  page += "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += "<title>Line Follower Tuning</title>";
  page += "<style>body{font-family:system-ui,Arial;margin:16px;max-width:720px;}";
  page += "h1{margin:0 0 8px;}fieldset{margin:12px 0;padding:12px;border-radius:8px;}";
  page += "label{display:inline-block;width:110px;}input[type=number]{width:110px;}";
  page += ".row{margin:6px 0;}.btns a,.btns input[type=submit]{display:inline-block;";
  page += "padding:8px 14px;margin:4px 6px 0 0;border:1px solid #999;border-radius:8px;";
  page += "text-decoration:none;color:#000;}.run{background:#d1ffd1;}.stop{background:#ffd1d1;}</style>";
  page += "<script>async function refreshStatus(){try{const r=await fetch('/status');";
  page += "const j=await r.json();document.getElementById('runstate').textContent=j.run?'RUNNING':'STOPPED';}";
  page += "catch(e){}}setInterval(refreshStatus,1000);window.onload=refreshStatus;</script></head><body>";

  page += "<h1>ESP32 Line Follower</h1>";
  page += "<div>Status: <b id='runstate'>...</b></div>";
  page += "<div class='btns'><a class='run' href='/start'>Start</a>";
  page += "<a class='stop' href='/stop'>Stop</a></div>";

  page += "<form action='/update' method='get'>";
  page += "<fieldset><legend>PID</legend>";
  page += "<div class='row'><label for='kp'>Kp:</label><input type='number' id='kp' name='kp' step='0.1' value='" + String(kp) + "'></div>";
  page += "<div class='row'><label for='ki'>Ki:</label><input type='number' id='ki' name='ki' step='0.1' value='" + String(ki) + "'></div>";
  page += "<div class='row'><label for='kd'>Kd:</label><input type='number' id='kd' name='kd' step='0.1' value='" + String(kd) + "'></div>";
  page += "</fieldset>";

  page += "<fieldset><legend>Base Speed</legend>";
  page += "<div class='row'><label for='base'>Base:</label><input type='number' id='base' name='base' min='0' max='255' value='" + String(baseSpeed) + "'></div>";
  page += "</fieldset>";

  page += "<fieldset><legend>Sensor Weights (S0..S7)</legend>";
  for (int i = 0; i < numSensors; i++) {
    page += "<div class='row'><label for='w" + String(i) + "'>S" + String(i) + ":</label>";
    page += "<input type='number' id='w" + String(i) + "' name='w" + String(i) + "' step='1' value='" + String(weights[i]) + "'></div>";
  }
  page += "</fieldset>";

  page += "<input type='submit' value='Update Values'></form>";
  page += "</body></html>";

  return page;
}
