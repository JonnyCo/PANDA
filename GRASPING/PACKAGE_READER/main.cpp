#include <MFRC522.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define CS_PIN 17   // GPIO17 for Chip Select
#define RST_PIN -1  // Leave RST unconnected

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

MFRC522 mfrc522(CS_PIN, RST_PIN);

// Eye animation parameters
int eyeX = SCREEN_WIDTH / 2;  // X position of the eye
int eyeY = SCREEN_HEIGHT / 2; // Y position of the eye
int eyeRadius = 20;           // Radius of the eye
int pupilRadius = 8;          // Radius of the pupil
int blinkState = 0;           // Blink state (0 = open, 1 = closed)

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin(); // Initialize SPI bus
  mfrc522.PCD_Init(); // Initialize MFRC522 reader

  Serial.println(F("RC522 RFID Reader Initialized."));

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1); // Hang if the display fails to initialize
  }

  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();
  delay(2000);
}

void loop() {
  // Eye animation logic
  animateEye();

  // RFID scanning logic (optional, kept for demo purposes)
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    String uid = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      uid += String(mfrc522.uid.uidByte[i], HEX);
      if (i < mfrc522.uid.size - 1) {
        uid += ":";
      }
    }

    Serial.println(uid);
    displayEyeMessage("Tag Scanned!");
    delay(2000);
  }
}

void animateEye() {
  display.clearDisplay();

  // Draw the eye outline
  display.drawCircle(eyeX, eyeY, eyeRadius, SSD1306_WHITE);

  // Blink logic
  if (blinkState == 1) {
    // Eye closed (draw a line instead of a pupil)
    display.drawLine(eyeX - eyeRadius, eyeY, eyeX + eyeRadius, eyeY, SSD1306_WHITE);
  } else {
    // Eye open, draw the pupil
    int pupilX = eyeX + random(-5, 5); // Slight random movement
    int pupilY = eyeY + random(-3, 3); // Slight random movement
    display.fillCircle(pupilX, pupilY, pupilRadius, SSD1306_WHITE);
  }

  display.display();
  delay(300);

  // Change blink state occasionally
  if (random(0, 10) > 8) {
    blinkState = 1 - blinkState; // Toggle blink state
  }
}

void displayEyeMessage(const char *message) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print(message);
  display.display();
  delay(2000);

  // Reset to the eye animation
  display.clearDisplay();
  animateEye();
}
