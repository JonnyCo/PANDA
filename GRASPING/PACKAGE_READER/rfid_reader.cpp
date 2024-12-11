#include <MFRC522.h>
#include <SPI.h>

#define CS_PIN 22
#define RST_PIN -1

MFRC522 mfrc522(CS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println(F("RC522 initialized. Waiting for a card..."));
}

void loop() {
  Serial.println(F("Checking for a card..."));
  
  // Check if a card is present
  if (!mfrc522.PICC_IsNewCardPresent()) {
    Serial.println(F("No card detected."));
    delay(500);
    return;
  }

  Serial.println(F("Card detected! Trying to read..."));

  // Attempt to read the card
  if (!mfrc522.PICC_ReadCardSerial()) {
    Serial.println(F("Failed to read the card."));
    return;
  }

  Serial.println(F("Card successfully read."));
  Serial.print(F("Card UID: "));
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    if (i < mfrc522.uid.size - 1) {
      Serial.print(":");
    }
  }
  Serial.println();
  mfrc522.PICC_HaltA(); // Halt the card
}
