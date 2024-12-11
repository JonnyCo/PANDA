#include <Dynamixel2Arduino.h>

// Define the UART port and pins
#define DXL_SERIAL Serial1        // Use Serial1 on Xiao ESP32-C6
#define DXL_DIR_PIN 22            // Dynamixel2Arduino requires a direction control pin
#define DXL_TX_PIN 16             // TX pin (D6 = GPIO16)
#define DXL_RX_PIN 17             // RX pin (D7 = GPIO17)
#define BAUDRATE 57600            // Dynamixel baud rate

// Servo IDs
int DXL_IDS[] = {15, 16, 17, 18, 19};

// Create Dynamixel2Arduino instance
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize Dynamixel Serial
  DXL_SERIAL.begin(BAUDRATE, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);  // Specify TX and RX pins
  dxl.begin(BAUDRATE);

  // Set communication protocol (e.g., 2.0 for newer Dynamixel models)
  dxl.setPortProtocolVersion(2.0);

  // Ping all motors to verify connection
  Serial.println("Pinging Dynamixel motors...");
  for (int id : DXL_IDS) {
    if (dxl.ping(id)) {
      Serial.print("Motor ID ");
      Serial.print(id);
      Serial.println(" is online.");
    } else {
      Serial.print("Motor ID ");
      Serial.print(id);
      Serial.println(" is not responding.");
    }
  }

  Serial.println("Dynamixel setup complete.");
}

void loop() {
  // Blink LEDs on each motor in sequence
  for (int id : DXL_IDS) {
    Serial.print("Blinking LED on Motor ID ");
    Serial.println(id);

    dxl.ledOn(id);  // Turn on LED
    delay(500);     // Wait 500 ms
    dxl.ledOff(id); // Turn off LED
    delay(500);     // Wait 500 ms
  }
}
