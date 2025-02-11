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

// Statistics for debugging
unsigned long packetsReceived = 0;
unsigned long packetsForwarded = 0;
unsigned long packetsDropped = 0;

String getDebugPrefix() {
  return "[Node " + String(NODE_ID) + "] ";
}

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
void printPacketInfo(Packet packet, String event);
void printLoRaStatus();
void printNetworkStats();

void initLoRa() {
  Serial.println(getDebugPrefix() + "Initializing LoRa...");
  
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println(getDebugPrefix() + "LoRa initialization FAILED!");
    while (true);
  }
  
  // Configure for maximum reliability
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(10);  // Increased for better range (7->10)
  LoRa.setSignalBandwidth(125E3);
  LoRa.setTxPower(20);
  LoRa.setSyncWord(0xF3);  // Define custom sync word for network isolation
  
  Serial.println(getDebugPrefix() + "LoRa initialized successfully");
  printLoRaStatus();
}

void printLoRaStatus() {
  Serial.println(getDebugPrefix() + "LoRa Configuration:");
  Serial.println("- Frequency: 915 MHz");
  Serial.println("- Spreading Factor: 10");
  Serial.println("- Bandwidth: 125 kHz");
  Serial.println("- TX Power: 20 dBm");
  Serial.println("- Sync Word: 0xF3");
  Serial.println("- CRC: Enabled");
}

void printNetworkStats() {
  Serial.println(getDebugPrefix() + "Network Statistics:");
  Serial.println("- Packets Received: " + String(packetsReceived));
  Serial.println("- Packets Forwarded: " + String(packetsForwarded));
  Serial.println("- Packets Dropped: " + String(packetsDropped));
}

void sendPacket(Packet packet) {
  Serial.println(getDebugPrefix() + "Sending packet...");
  printPacketInfo(packet, "TX");
  
  long startTime = millis();
  LoRa.beginPacket();
  LoRa.write(packet.source);
  LoRa.write(packet.destination);
  LoRa.write(packet.hopCount);
  LoRa.write(packet.messageId);
  LoRa.print(packet.message);
  bool success = LoRa.endPacket();
  long endTime = millis();
  
  if (success) {
    Serial.println(getDebugPrefix() + "Packet sent successfully in " + String(endTime - startTime) + "ms");
  } else {
    Serial.println(getDebugPrefix() + "Failed to send packet!");
  }
}

Packet receivePacket() {
  Packet packet;
  
  // Read with timeout
  unsigned long startTime = millis();
  while (LoRa.available() < 4) {  // Wait for header
    if (millis() - startTime > 1000) {
      Serial.println(getDebugPrefix() + "Timeout waiting for packet header!");
      return packet;
    }
  }
  
  packet.source = LoRa.read();
  packet.destination = LoRa.read();
  packet.hopCount = LoRa.read();
  packet.messageId = LoRa.read();
  packet.message = LoRa.readString();
  
  packetsReceived++;
  printPacketInfo(packet, "RX");
  return packet;
}

void printPacketInfo(Packet packet, String event) {
  Serial.println("----------------------------------------");
  Serial.println(getDebugPrefix() + event + " Packet Details:");
  Serial.println("- Source Node: " + String(packet.source));
  Serial.println("- Destination Node: " + String(packet.destination));
  Serial.println("- Hop Count: " + String(packet.hopCount));
  Serial.println("- Message ID: " + String(packet.messageId));
  Serial.println("- Message: " + packet.message);
  if (event == "RX") {
    Serial.println("- RSSI: " + String(LoRa.packetRssi()) + " dBm");
    Serial.println("- SNR: " + String(LoRa.packetSnr()) + " dB");
    Serial.println("- Frequency Error: " + String(LoRa.packetFrequencyError()) + " Hz");
  }
  Serial.println("----------------------------------------");
}

void handlePacket(Packet packet) {
  // Print routing decision header
  Serial.println(getDebugPrefix() + "Making routing decision for packet from Node " + 
                String(packet.source) + " to Node " + String(packet.destination));
  
  if (processed[packet.messageId]) {
    Serial.println(getDebugPrefix() + "Packet ID " + String(packet.messageId) + " already processed, skipping");
    packetsDropped++;
    return;
  }
  
  processed[packet.messageId] = true;
  Serial.println(getDebugPrefix() + "Processing packet ID " + String(packet.messageId));
  
  if (packet.destination == NODE_ID) {
    Serial.println(getDebugPrefix() + "Message received for this node: " + packet.message);
    return;
  }
  
  if (packet.hopCount < MAX_HOPS) {
    packet.hopCount++;
    Serial.println(getDebugPrefix() + "Forwarding packet (hop " + String(packet.hopCount) + "/" + String(MAX_HOPS) + ")");
    
    for (int i = 0; i < RETRY_COUNT; i++) {
      Serial.println(getDebugPrefix() + "Retry " + String(i + 1) + "/" + String(RETRY_COUNT));
      sendPacket(packet);
      packetsForwarded++;
      delay(random(100, 500));  // Random delay to avoid collisions
    }
  } else {
    Serial.println(getDebugPrefix() + "Max hop count reached, dropping packet");
    packetsDropped++;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to be ready
  }
  Serial.println();
  Serial.println(getDebugPrefix() + "Starting LoRa Node " + String(NODE_ID));
  initLoRa();
}

void loop() {
  static unsigned long lastStats = 0;
  
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println(getDebugPrefix() + "Incoming packet detected (" + String(packetSize) + " bytes)");
    Packet packet = receivePacket();
    handlePacket(packet);
  }
  
  if (Serial.available()) {
    String message = Serial.readString();
    message.trim();
    
    Serial.println(getDebugPrefix() + "New message from Serial: " + message);
    
    Packet packet = {
      NODE_ID,        // Source is this node
      2,              // Changed to Node 2 as destination
      0,              // Initial hop count
      (uint8_t)random(256),
      message
    };
    
    sendPacket(packet);
  }
  
  // Print statistics every 10 seconds
  if (millis() - lastStats > 10000) {
    printNetworkStats();
    lastStats = millis();
  }
}