#include <SPI.h>
#include <LoRa.h>

#define SCK     18
#define MISO    19
#define MOSI    23

/*// Pin definitions for ESP32-WROOM32
#define SS      5
#define RST     14
#define DIO0    2
//*/

/// Pin definitions for ESP32-MisRed
#define SS      15
#define RST     26
#define DIO0    27
//*/

#define NODE_ID 3
#define MAX_HOPS 3
#define RETRY_COUNT 3

struct Packet {
  uint8_t source;
  uint8_t destination;
  uint8_t hopCount;
  uint8_t messageId;
  String message;
};

bool processed[256] = {false};

// Function declarations
void initLoRa();
void sendPacket(Packet packet);
Packet receivePacket();
void handlePacket(Packet packet);

void initLoRa() {
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed");
    while (true);
  }
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setTxPower(20);
}

void setup() {
  Serial.begin(115200);
  initLoRa();
}

void sendPacket(Packet packet) {
  LoRa.beginPacket();
  LoRa.write(packet.source);
  LoRa.write(packet.destination);
  LoRa.write(packet.hopCount);
  LoRa.write(packet.messageId);
  LoRa.print(packet.message);
  LoRa.endPacket();
}

Packet receivePacket() {
  Packet packet;
  packet.source = LoRa.read();
  packet.destination = LoRa.read();
  packet.hopCount = LoRa.read();
  packet.messageId = LoRa.read();
  packet.message = LoRa.readString();
  return packet;
}

void handlePacket(Packet packet) {
  if (processed[packet.messageId]) return;
  processed[packet.messageId] = true;
  
  if (packet.destination == NODE_ID) {
    Serial.println("Message received: " + packet.message);
    return;
  }
  
  if (packet.hopCount < MAX_HOPS) {
    packet.hopCount++;
    for (int i = 0; i < RETRY_COUNT; i++) {
      sendPacket(packet);
      delay(random(100, 500));
    }
  }
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Packet packet = receivePacket();
    handlePacket(packet);
  }
  
  if (Serial.available()) {
    String message = Serial.readString();
    Packet packet = {
      NODE_ID,
      3,              // Sending to Node 3 or destination node
      0,
      (uint8_t)random(256),  // Fixed narrowing conversion
      message
    };
    sendPacket(packet);
  }
}