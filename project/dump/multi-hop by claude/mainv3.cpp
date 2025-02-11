#include <SPI.h>
#include <LoRa.h>

#define SCK     18
#define MISO    19
#define MOSI    23


#define SS      15
#define RST     26
#define DIO0    27

#define NODE_ID 3  // Change this for each node (1, 2, or 3)
#define MAX_HOPS 3
#define RETRY_COUNT 3
#define GATEWAY_NODE 3
#define DATA_INTERVAL 30000  // Send data every 30 seconds

// Statistics for debugging
unsigned long packetsReceived = 0;
unsigned long packetsForwarded = 0;
unsigned long packetsDropped = 0;
unsigned long lastDataSent = 0;

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

struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  float voltage;
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
SensorData generateRandomData();
String formatSensorData(SensorData data);

void initLoRa() {
  Serial.println(getDebugPrefix() + "Initializing LoRa...");
  
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println(getDebugPrefix() + "LoRa initialization FAILED!");
    while (true);
  }
  
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setTxPower(20);
  LoRa.setSyncWord(0xF3);
  
  Serial.println(getDebugPrefix() + "LoRa initialized successfully");
  printLoRaStatus();
}

// Generate random sensor data
SensorData generateRandomData() {
  SensorData data;
  data.temperature = random(2000, 3500) / 100.0;  // 20.00 to 35.00 °C
  data.humidity = random(4000, 9000) / 100.0;     // 40.00 to 90.00 %
  data.pressure = random(98000, 102000) / 100.0;  // 980.00 to 1020.00 hPa
  data.voltage = random(300, 420) / 100.0;        // 3.00 to 4.20 V
  return data;
}

// Format sensor data as JSON-like string
String formatSensorData(SensorData data) {
  return "{\"temp\":" + String(data.temperature, 2) + 
         ",\"hum\":" + String(data.humidity, 2) + 
         ",\"pres\":" + String(data.pressure, 2) + 
         ",\"volt\":" + String(data.voltage, 2) + "}";
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
  
  unsigned long startTime = millis();
  while (LoRa.available() < 4) {
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

void handlePacket(Packet packet) {
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
    // If this is the gateway, print parsed data
    if (NODE_ID == GATEWAY_NODE) {
      Serial.println(getDebugPrefix() + "Gateway received data from Node " + String(packet.source));
      Serial.println("Data: " + packet.message);
    }
    return;
  }
  
  if (packet.hopCount < MAX_HOPS) {
    packet.hopCount++;
    Serial.println(getDebugPrefix() + "Forwarding packet (hop " + String(packet.hopCount) + "/" + String(MAX_HOPS) + ")");
    
    for (int i = 0; i < RETRY_COUNT; i++) {
      Serial.println(getDebugPrefix() + "Retry " + String(i + 1) + "/" + String(RETRY_COUNT));
      sendPacket(packet);
      packetsForwarded++;
      delay(random(100, 500));
    }
  } else {
    Serial.println(getDebugPrefix() + "Max hop count reached, dropping packet");
    packetsDropped++;
  }
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

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to be ready
  }
  Serial.println();
  Serial.println(getDebugPrefix() + "Starting LoRa Node " + String(NODE_ID));
  
  // Initialize random seed using analog noise
  randomSeed(analogRead(0));
  
  initLoRa();
}

void loop() {
  static unsigned long lastStats = 0;
  
  // For end nodes (1 and 2), send random data periodically
  if (NODE_ID != GATEWAY_NODE && millis() - lastDataSent >= DATA_INTERVAL) {
    SensorData sensorData = generateRandomData();
    String dataMessage = formatSensorData(sensorData);
    
    Packet packet = {
      NODE_ID,          // Source is this node
      GATEWAY_NODE,     // Destination is gateway (Node 3)
      0,                // Initial hop count
      (uint8_t)random(256),
      dataMessage
    };
    
    sendPacket(packet);
    lastDataSent = millis();
  }
  
  // Check for incoming packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println(getDebugPrefix() + "Incoming packet detected (" + String(packetSize) + " bytes)");
    Packet packet = receivePacket();
    handlePacket(packet);
  }
  
  // Handle manual messages from Serial
  if (Serial.available()) {
    String message = Serial.readString();
    message.trim();
    
    Serial.println(getDebugPrefix() + "New message from Serial: " + message);
    
    Packet packet = {
      NODE_ID,
      GATEWAY_NODE,
      0,
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