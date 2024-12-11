#include <MFRC522.h>
#include <SPI.h>

#define CS_PIN 22   // GPIO22 for Chip Select
#define RST_PIN -1  // Leave RST unconnected

MFRC522 mfrc522(CS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin(); // Initialize SPI bus
  mfrc522.PCD_Init(); // Initialize MFRC522 reader

  Serial.println(F("RC522 RFID Reader Initialized. Scan a tag..."));
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
  Serial.println(uid);

  mfrc522.PICC_HaltA(); // Halt the card for the next read
}
