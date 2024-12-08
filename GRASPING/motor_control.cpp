#include <Dynamixel2Arduino.h>

// Define the UART port and pins
#define DXL_SERIAL Serial1        // Use Serial1 on Xiao ESP32-C6
#define DXL_DIR_PIN 22            // Dynamixel2Arduino requires a direction control pin
#define DXL_TX_PIN 16             // TX pin (D6 = GPIO16)
#define DXL_RX_PIN 17             // RX pin (D7 = GPIO17)
#define BAUDRATE 57600            // Dynamixel baud rate

// Test a single motor ID
const uint8_t MOTOR_ID = 15;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  DXL_SERIAL.begin(BAUDRATE, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  if (dxl.ping(MOTOR_ID)) {
    Serial.print("Motor ID ");
    Serial.print(MOTOR_ID);
    Serial.println(" is online.");
  } else {
    Serial.println("Motor not responding. Check wiring and ID.");
    while (true);
  }

  dxl.torqueOff(MOTOR_ID);  // Ensure torque is off for configuration

  // Set Operating Mode
  if (dxl.setOperatingMode(MOTOR_ID, OP_POSITION)) {
    Serial.println("Operating mode set to position control.");
  } else {
    Serial.println("Failed to set operating mode.");
  }

  // Verify operating mode
  int mode = dxl.readControlTableItem(ControlTableItem::OPERATING_MODE, MOTOR_ID);
  Serial.print("Current Operating Mode: ");
  Serial.println(mode);

  dxl.torqueOn(MOTOR_ID);  // Enable torque
  bool isTorqueOn = dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, MOTOR_ID);
  Serial.print("Torque Enable Status: ");
  Serial.println(isTorqueOn ? "ON" : "OFF");

  // Set velocity and acceleration
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, MOTOR_ID, 200);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, MOTOR_ID, 50);

  int velocity = dxl.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, MOTOR_ID);
  int acceleration = dxl.readControlTableItem(ControlTableItem::PROFILE_ACCELERATION, MOTOR_ID);
  Serial.print("Current Velocity: ");
  Serial.println(velocity);
  Serial.print("Current Acceleration: ");
  Serial.println(acceleration);

  Serial.println("Setup complete.");
}

void loop() {
  // Read current position
  int currentPosition = dxl.getPresentPosition(MOTOR_ID);
  Serial.print("Current RAW Position: ");
  Serial.println(currentPosition);

  // Move to a goal position
  int goalPosition = 2000;  // Example position
  Serial.print("Moving to RAW Position: ");
  Serial.println(goalPosition);

  if (dxl.setGoalPosition(MOTOR_ID, goalPosition)) {
    Serial.println("Goal position set successfully.");
  } else {
    Serial.println("Failed to set goal position.");
  }

  // Wait until the motor reaches the goal
  while (abs(goalPosition - currentPosition) > 10) {
    currentPosition = dxl.getPresentPosition(MOTOR_ID);
    Serial.print("Current RAW Position: ");
    Serial.println(currentPosition);
    delay(100);
  }

  Serial.println("Reached goal position.");
  delay(2000);
}
