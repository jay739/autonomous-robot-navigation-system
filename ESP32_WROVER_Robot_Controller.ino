#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "config.h"  // You can put all the sensitive data in the config.h file

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Web server on configured port
WebServer server(SERVER_PORT);

// Create global variables to store the latest position data
int x_position = 0;
int y_position = 0;
int robot_x = 0;
int robot_y = 0;
float distance = 0.0;
String position = "";
float angle = 0.0;  // Add angle variable
unsigned long lastUpdateTime = 0;

// Motor pins
// Motor A (Front Left)
int ENA = 23;
int IN1 = 5;
int IN2 = 4;
// Motor B (Front Right)
int ENB = 22;
int IN3 = 2;
int IN4 = 15;
// Motor C (Rear Left)
int ENC = 13;
int IN5 = 19;
int IN6 = 21;
// Motor D (Rear Right)
int END = 14;
int IN7 = 27;
int IN8 = 26;

// Motor control variables (from config.h)
unsigned long lastMotorUpdateTime = 0;
const int MOTOR_UPDATE_INTERVAL = 500; // Update motor control every 500ms

void setup() {
  Serial.begin(115200);
  
  // Print configuration for debugging
  if (DEBUG_MODE) {
    Serial.println("=== ESP32-WROVER Configuration ===");
    Serial.print("WiFi SSID: ");
    Serial.println(WIFI_SSID);
    Serial.print("Server Port: ");
    Serial.println(SERVER_PORT);
    Serial.println("===================================");
  }
  
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(END, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  
  // Stop all motors initially
  stopAllMotors();
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
  Serial.print("ESP32-WROVER IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Define the routes for the web server
  server.on("/position", HTTP_POST, handlePositionUpdate);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleDataRequest);
  
  // Start the server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // Handle client requests
  server.handleClient();
  
  // Print latest position data every 2 seconds if data is available
  if (millis() - lastUpdateTime > 2000 && lastUpdateTime > 0) {
    printPositionData();
    lastUpdateTime = millis();
  }
  
  // Update motor control based on position data
  if (millis() - lastMotorUpdateTime > MOTOR_UPDATE_INTERVAL && lastUpdateTime > 0) { 
    updateMotorControlUsingAngle();
    lastMotorUpdateTime = millis();
  }
}

// Function to update motor control based on angle and distance
void updateMotorControlUsingAngle() {
  // Only control motors if we have recent position data (less than 5 seconds old)
  if (lastUpdateTime > 0 && (millis() - lastUpdateTime < 5000)) {
    Serial.print("Adjusting motors based on angle: ");
    Serial.print(angle);
    Serial.print("° and distance: ");
    Serial.println(distance);
    
    // Calculate speed based on distance from target - farther = faster
    int speed;
    if (distance < TARGET_DISTANCE - DISTANCE_THRESHOLD) {
      // Too close, move backward faster if further away
      speed = map(TARGET_DISTANCE - distance, 0, TARGET_DISTANCE, MIN_SPEED, MAX_SPEED);
      speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    } else if (distance > TARGET_DISTANCE + DISTANCE_THRESHOLD) {
      // Too far, move forward faster if further away
      speed = map(distance - TARGET_DISTANCE, 0, TARGET_DISTANCE, MIN_SPEED, MAX_SPEED);
      speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    } else {
      // Within distance threshold, use minimum speed for orientation only
      speed = MIN_SPEED;
    }
    
    // Normalized angle (0-359 degrees)
    float normalizedAngle = angle;
    if (normalizedAngle < 0) normalizedAngle += 360;
    
    // Control based on angle
    if (distance < TARGET_DISTANCE - DISTANCE_THRESHOLD) {
      // Too close, move backward with angle adjustment
      if (normalizedAngle >= 315 || normalizedAngle < 45) {
        // Target is in front, move straight backward
        moveBackward(speed);
      } else if (normalizedAngle >= 45 && normalizedAngle < 135) {
        // Target is to the right, move backward-right
        moveBackwardRight(speed);
      } else if (normalizedAngle >= 135 && normalizedAngle < 225) {
        // Target is behind, move straight backward
        moveBackward(speed);
      } else { // normalizedAngle >= 225 && normalizedAngle < 315
        // Target is to the left, move backward-left
        moveBackwardLeft(speed);
      }
    } else if (distance > TARGET_DISTANCE + DISTANCE_THRESHOLD) {
      // Too far, move forward with angle adjustment
      if (normalizedAngle >= 315 || normalizedAngle < 45) {
        // Target is in front, move straight forward
        moveForward(speed);
      } else if (normalizedAngle >= 45 && normalizedAngle < 135) {
        // Target is to the right, move forward-right
        moveForwardRight(speed);
      } else if (normalizedAngle >= 135 && normalizedAngle < 225) {
        // Target is behind, move forward
        moveForward(speed);
      } else { // normalizedAngle >= 225 && normalizedAngle < 315
        // Target is to the left, move forward-left
        moveForwardLeft(speed);
      }
    } else {
      // Distance is within threshold, adjust orientation only
      if (normalizedAngle >= 315 || normalizedAngle < 45) {
        // Target is in front, stop motors
        stopAllMotors();
        Serial.println("Target in front and at correct distance, stopping motors");
      } else if (normalizedAngle >= 45 && normalizedAngle < 135) {
        // Target is to the right, rotate right
        moveRight(speed);
      } else if (normalizedAngle >= 135 && normalizedAngle < 225) {
        // Target is behind, rotate right (arbitrary choice)
        moveRight(speed);
      } else { // normalizedAngle >= 225 && normalizedAngle < 315
        // Target is to the left, rotate left
        moveLeft(speed);
      }
    }
  } else if (millis() - lastUpdateTime >= 5000 && lastUpdateTime > 0) {
    // If position data is too old, stop motors
    stopAllMotors();
    Serial.println("Position data too old, stopping motors");
  }
}

// Motor control functions
void stopAllMotors() {
  // Stop all motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
  
  // Set enable pins to LOW
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
  analogWrite(END, 0);
}

void moveForward(int speed) {
  // Set direction pins
  // Motor A (Front Left)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor B (Front Right)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Motor C (Rear Left)
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  // Motor D (Rear Right)
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);

  // Enable all motors
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
  analogWrite(END, speed);

  Serial.println("Moving forward");
}

void moveBackward(int speed) {
  // Set direction pins
  // Motor A (Front Left)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Motor B (Front Right)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Motor C (Rear Left)
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  // Motor D (Rear Right)
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);

  // Enable all motors
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
  analogWrite(END, speed);

  Serial.println("Moving backward");
}

void turnLeft(int speed) {
  // Set direction pins
  // Motor A (Front Left) & Motor C (Rear Left) backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  // Motor B (Front Right) & Motor D (Rear Right) forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);

  // Enable all motors
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
  analogWrite(END, speed);

  Serial.println("Turning left");
}

void turnRight(int speed) {
  // Set direction pins
  // Motor A (Front Left) & Motor C (Rear Left) forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  // Motor B (Front Right) & Motor D (Rear Right) backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);

  // Enable all motors
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
  analogWrite(END, speed);

  Serial.println("Turning right");
}

void moveForwardLeft(int speed) {
  // Set direction pins
  // Motor A (Front Left) - slower or stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Motor B (Front Right) - forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Motor C (Rear Left) - slower or stop
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  // Motor D (Rear Right) - forward
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);

  // Enable motors
  analogWrite(ENA, 0);
  analogWrite(ENB, speed);
  analogWrite(ENC, 0);
  analogWrite(END, speed);

  Serial.println("Moving forward-left");
}

void moveForwardRight(int speed) {
  // Set direction pins
  // Motor A (Front Left) - forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor B (Front Right) - slower or stop
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Motor C (Rear Left) - forward
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  // Motor D (Rear Right) - slower or stop
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);

  // Enable motors
  analogWrite(ENA, speed);
  analogWrite(ENB, 0);
  analogWrite(ENC, speed);
  analogWrite(END, 0);

  Serial.println("Moving forward-right");
}

void moveBackwardLeft(int speed) {
  // Set direction pins
  // Motor A (Front Left) - slower or stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Motor B (Front Right) - backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Motor C (Rear Left) - slower or stop
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  // Motor D (Rear Right) - backward
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);

  // Enable motors
  analogWrite(ENA, 0);
  analogWrite(ENB, speed);
  analogWrite(ENC, 0);
  analogWrite(END, speed);

  Serial.println("Moving backward-left");
}

void moveBackwardRight(int speed) {
  // Set direction pins
  // Motor A (Front Left) - backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Motor B (Front Right) - slower or stop
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Motor C (Rear Left) - backward
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  // Motor D (Rear Right) - slower or stop
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);

  // Enable motors
  analogWrite(ENA, speed);
  analogWrite(ENB, 0);
  analogWrite(ENC, speed);
  analogWrite(END, 0);

  Serial.println("Moving backward-right");
}

void moveLeft(int speed) {
  // Set direction pins
  // Motor A (Front Left) & Motor C (Rear Left) backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  // Motor B (Front Right) & Motor D (Rear Right) forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);

  // Enable all motors
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
  analogWrite(END, speed);

  Serial.println("Moving left");
}

void moveRight(int speed) {
  // Set direction pins
  // Motor A (Front Left) & Motor C (Rear Left) forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  // Motor B (Front Right) & Motor D (Rear Right) backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);

  // Enable all motors
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
  analogWrite(END, speed);

  Serial.println("Moving right");
}

// Handle POST requests to /position endpoint
void handlePositionUpdate() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    
    // Parse the JSON data
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      server.send(400, "text/plain", "Bad Request: Failed to parse JSON");
      return;
    }
    
    // Extract data from JSON
    x_position = doc["x_position"];
    y_position = doc["y_position"];
    robot_x = doc["robot_x"];
    robot_y = doc["robot_y"];
    distance = doc["distance"];
    position = doc["position"].as<String>();
    angle = doc["angle"];
    
    lastUpdateTime = millis();
    
    // Print received data
    Serial.println("Received new position data:");
    printPositionData();
    
    // Send response
    server.send(200, "text/plain", "Data received successfully");
  } else {
    server.send(400, "text/plain", "Bad Request: No data received");
  }
}

// Handle GET requests to the root endpoint
void handleRoot() {
  String html = "<html><head>";
  html += "<title>ESP32-WROVER Robot Control</title>";
  html += "<meta http-equiv='refresh' content='2'>"; // Auto-refresh every 2 seconds
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; }";
  html += "h1 { color: #0066CC; }";
  html += ".data-container { background-color: #f0f0f0; padding: 15px; border-radius: 8px; margin-top: 20px; }";
  html += ".data-item { margin: 10px 0; }";
  html += ".label { font-weight: bold; display: inline-block; width: 150px; }";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>ESP32-WROVER Robot Control</h1>";
  html += "<div class='data-container'>";
  
  html += "<div class='data-item'><span class='label'>X Position:</span> " + String(x_position) + "</div>";
  html += "<div class='data-item'><span class='label'>Y Position:</span> " + String(y_position) + "</div>";
  html += "<div class='data-item'><span class='label'>Robot X:</span> " + String(robot_x) + "</div>";
  html += "<div class='data-item'><span class='label'>Robot Y:</span> " + String(robot_y) + "</div>";
  html += "<div class='data-item'><span class='label'>Distance:</span> " + String(distance) + " cm</div>";
  html += "<div class='data-item'><span class='label'>Position:</span> " + position + "</div>";
  html += "<div class='data-item'><span class='label'>Angle:</span> " + String(angle) + "°</div>";
  html += "<div class='data-item'><span class='label'>Last Update:</span> ";
  
  if (lastUpdateTime > 0) {
    html += String((millis() - lastUpdateTime) / 1000) + " seconds ago";
  } else {
    html += "No data received yet";
  }
  
  html += "</div></div>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

// Handle GET requests to /data endpoint (for JSON data)
void handleDataRequest() {
  DynamicJsonDocument doc(1024);
  
  doc["x_position"] = x_position;
  doc["y_position"] = y_position;
  doc["robot_x"] = robot_x;
  doc["robot_y"] = robot_y;
  doc["distance"] = distance;
  doc["position"] = position;
  doc["angle"] = angle;
  doc["last_update"] = lastUpdateTime > 0 ? (millis() - lastUpdateTime) / 1000 : -1;
  
  String jsonResponse;
  serializeJson(doc, jsonResponse);
  
  server.send(200, "application/json", jsonResponse);
}

// Helper function to print position data to serial
void printPositionData() {
  Serial.println("-----------------------------------");
  Serial.println("Current Position Data:");
  Serial.print("X Position: ");
  Serial.println(x_position);
  Serial.print("Y Position: ");
  Serial.println(y_position);
  Serial.print("Robot X: ");
  Serial.println(robot_x);
  Serial.print("Robot Y: ");
  Serial.println(robot_y);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  Serial.print("Position: ");
  Serial.println(position);
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.println("°");
  Serial.println("-----------------------------------");
}
