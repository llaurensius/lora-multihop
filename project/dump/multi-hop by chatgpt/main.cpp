#include <SPI.h>
#include <LoRa.h>

// LoRa pins for ESP32
#define SS 18
#define RST 14
#define DI0 26

// Network settings
#define LORA_FREQUENCY 915E6  // Ganti dengan frekuensi sesuai wilayah Anda
#define NODE_ID 1             // ID unik untuk setiap node
#define NEXT_HOP_ID 2         // ID dari node hop berikutnya

// Prototipe fungsi
void sendMessage(String message);
void receiveMessage();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Inisialisasi LoRa
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Initializing OK!");
}

void loop() {
  // Kirim pesan
  sendMessage("Hello from Node " + String(NODE_ID));

  // Tunggu pesan
  receiveMessage();

  delay(5000);  // Delay antara pesan
}

void sendMessage(String message) {
  LoRa.beginPacket();
  LoRa.write(NODE_ID);
  LoRa.write(NEXT_HOP_ID);
  LoRa.print(message);
  LoRa.endPacket();

  Serial.println("Message sent: " + message);
}

void receiveMessage() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    int senderId = LoRa.read();
    int receiverId = LoRa.read();

    if (receiverId == NODE_ID) {
      String message = LoRa.readString();
      Serial.println("Message received: " + message);

      // Meneruskan pesan jika bukan untuk node ini
      if (senderId != NODE_ID) {
        Serial.println("Forwarding message...");
        sendMessage(message);
      }
    }
  }
}
