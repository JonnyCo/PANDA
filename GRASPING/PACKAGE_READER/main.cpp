#include <MFRC522.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>


#define CS_PIN 17   // GPIO22 for Chip Select
#define RST_PIN -1  // Leave RST unconnected


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

MFRC522 mfrc522(CS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin(); // Initialize SPI bus
  mfrc522.PCD_Init(); // Initialize MFRC522 reader

  Serial.println(F("RC522 RFID Reader Initialized. Scan a tag..."));

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1);
  }

  display.setRotation(2);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready To Scan a Tag");
  display.display();
  delay(1000);
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  // Read UID
  String uid = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    uid += String(mfrc522.uid.uidByte[i], HEX);
    if (i < mfrc522.uid.size - 1) {
      uid += ":";
    }
  }

  // Send UID over serial
  display.println("uid");
  display.println(uid);
  display.display();
  Serial.println(uid);

  mfrc522.PICC_HaltA(); // Halt the card for the next read
}
