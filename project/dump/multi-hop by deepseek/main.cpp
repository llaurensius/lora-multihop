#include <SPI.h>
#include <LoRa.h>

// ******************** CONFIGURATION ********************
#define NODE_ID       1       // Unique for each node
#define IS_GATEWAY    false   // Set true for gateway node
#define LORA_FREQ     868E6   // EU 868MHz frequency
#define MAX_HOPS      4       // Maximum allowed hops
#define TX_INTERVAL   15000   // Data transmission interval (ms)
#define DEBUG_LEVEL   3       // 0:None, 1:Errors, 2:Info, 3:Verbose
#define MAX_RETRIES   3       // Transmission retries
#define RETRY_DELAY   2000    // ms between retries

// *********** PIN CONFIGURATION (ESP32 DevKit) ***********
#define SS_PIN    5
#define RST_PIN   14
#define DI0_PIN   2

/*// *********** PIN CONFIGURATION (ESP32 MiSRed) ***********
#define SS_PIN    15
#define RST_PIN   26
#define DI0_PIN   27
/*///
/*// *********** PIN CONFIGURATION (ESP32 TTGO) ***********
#define SS_PIN    18
#define RST_PIN   24
#define DI0_PIN   26
/*///

// ******************** PACKET STRUCTURE ********************
struct LoRaPacket {
  uint8_t source;
  uint8_t destination;
  uint8_t hopCount;
  uint16_t packetId;
  char payload[64];  // Fixed-size buffer for safety
};

// ******************** GLOBAL VARIABLES ********************
unsigned long lastTxTime = 0;
unsigned long lastStatTime = 0;
const uint8_t GATEWAY_ID = 0;
uint16_t packetCounter = 0;
uint16_t receivedCount = 0;
uint16_t forwardedCount = 0;
uint16_t dropCount = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000); // Wait up to 5s for serial
  
  initializeLoRa();
  printNodeInfo();
  debugPrint(2, "System initialization complete");
}

void loop() {
  // Handle transmissions for end nodes
  if (!IS_GATEWAY) {
    handleEndNodeBehavior();
  }
  
  // Receive incoming packets
  receiveData();
  
  // Periodic statistics reporting
  handleDiagnostics();
}

// ******************** DEBUG UTILITIES ********************
void debugPrint(int level, String message) {
  if (level > DEBUG_LEVEL) return;
  
  String prefix;
  switch(level) {
    case 1: prefix = "[ERROR] "; break;
    case 2: prefix = "[INFO]  "; break;
    case 3: prefix = "[DEBUG] "; break;
  }
  
  Serial.println(getTimestamp() + " " + prefix + message);
}

String getTimestamp() {
  unsigned long sec = millis() / 1000;
  return "[" + String(sec / 60) + "m" + String(sec % 60) + "s]";
}

void printNetworkStats() {
  debugPrint(2, "Network Statistics:");
  debugPrint(2, "Total Received: " + String(receivedCount));
  debugPrint(2, "Forwarded: " + String(forwardedCount));
  debugPrint(2, "Dropped: " + String(dropCount));
  debugPrint(2, "Total Transmitted: " + String(packetCounter));
}

// ******************** LORA INITIALIZATION ********************
void initializeLoRa() {
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  
  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    if (LoRa.begin(LORA_FREQ)) {
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(8);
      LoRa.enableCrc();
      LoRa.setTxPower(17);
      debugPrint(2, "LoRa initialized on attempt " + String(attempt));
      return;
    }
    debugPrint(1, "LoRa init failed attempt " + String(attempt));
    delay(RETRY_DELAY);
  }
  
  debugPrint(1, "LoRa init failed - system halted");
  while(1) { delay(1000); }
}

// ******************** TRANSMISSION HANDLING ********************
void handleEndNodeBehavior() {
  if (millis() - lastTxTime > TX_INTERVAL) {
    LoRaPacket packet = createDataPacket();
    
    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
      if (transmitPacket(packet)) {
        lastTxTime = millis();
        debugPrint(2, "Transmission succeeded after " + String(attempt+1) + " attempts");
        break;
      }
      debugPrint(1, "Transmission failed attempt " + String(attempt+1));
      delay(RETRY_DELAY);
    }
  }
}

LoRaPacket createDataPacket() {
  LoRaPacket packet;
  packet.source = NODE_ID;
  packet.destination = GATEWAY_ID;
  packet.hopCount = 0;
  packet.packetId = ++packetCounter;
  
  // Generate simulated sensor data
  snprintf(packet.payload, sizeof(packet.payload),
           "N:%d|T:%d|H:%d|V:%.2f", 
           NODE_ID, 
           random(20,30), 
           random(40,60),
           random(300,420)/100.0);
           
  return packet;
}

bool transmitPacket(LoRaPacket packet) {
  LoRa.beginPacket();
  LoRa.write(packet.source);
  LoRa.write(packet.destination);
  LoRa.write(packet.hopCount);
  LoRa.write((uint8_t*)&packet.packetId, sizeof(packet.packetId));
  LoRa.print(packet.payload);
  
  if (LoRa.endPacket()) {
    debugPrint(3, "TX | ID:" + String(packet.packetId) + 
               " | Size:" + String(sizeof(packet)) + 
               " | Hops:" + packet.hopCount + 
               " | Payload: " + String(packet.payload));
    return true;
  }
  return false;
}

// ******************** RECEPTION HANDLING ********************
void receiveData() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    LoRaPacket received;
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    
    // Read headers
    received.source = LoRa.read();
    received.destination = LoRa.read();
    received.hopCount = LoRa.read();
    LoRa.readBytes((uint8_t*)&received.packetId, sizeof(received.packetId));
    
    // Read payload safely
    int payloadLength = 0;
    while (LoRa.available() && payloadLength < sizeof(received.payload)-1) {
      received.payload[payloadLength++] = LoRa.read();
    }
    received.payload[payloadLength] = '\0';
    
    receivedCount++;
    
    debugPrint(3, "RX | ID:" + String(received.packetId) + 
               " | From:" + String(received.source) + 
               " | RSSI:" + String(rssi) + 
               " | SNR:" + String(snr) + 
               " | Hops:" + String(received.hopCount) + 
               " | Size:" + String(packetSize) + "B");
    
    processPacket(received, rssi, snr);
  }
}

void processPacket(LoRaPacket packet, int rssi, float snr) {
  if (packet.hopCount > MAX_HOPS) {
    dropCount++;
    debugPrint(2, "Dropped packet " + String(packet.packetId) + 
               " - Max hops exceeded (Current: " + 
               String(packet.hopCount) + ")");
    return;
  }

  if (packet.destination == NODE_ID || IS_GATEWAY) {
    handleLocalPacket(packet, rssi, snr);
  } else {
    forwardPacket(packet);
  }
}

void handleLocalPacket(LoRaPacket packet, int rssi, float snr) {
  debugPrint(2, "Received final packet " + String(packet.packetId) + 
             " | RSSI:" + String(rssi) + 
             " | SNR:" + String(snr) + 
             " | Hops:" + String(packet.hopCount));
  
  if (IS_GATEWAY) {
    debugPrint(1, "GW Processing: " + String(packet.payload));
    // Add your gateway processing logic here
  }
}

void forwardPacket(LoRaPacket packet) {
  if (packet.hopCount >= MAX_HOPS) {
    dropCount++;
    debugPrint(2, "Can't forward packet " + String(packet.packetId) + 
               " - Max hops reached");
    return;
  }

  packet.hopCount++;
  if (transmitPacket(packet)) {
    forwardedCount++;
    debugPrint(3, "Forwarded packet " + String(packet.packetId) + 
               " | New hop count:" + String(packet.hopCount));
  } else {
    dropCount++;
    debugPrint(1, "Forwarding failed for packet " + String(packet.packetId));
  }
}

// ******************** DIAGNOSTICS & UTILITIES ********************
void handleDiagnostics() {
  if (millis() - lastStatTime > 60000) { // Every 60 seconds
    printNetworkStats();
    lastStatTime = millis();
  }
}

void printNodeInfo() {
  debugPrint(2, "\n=== Node Configuration ===");
  debugPrint(2, "ID:          " + String(NODE_ID));
  debugPrint(2, "Role:        " + String(IS_GATEWAY ? "Gateway" : "Router"));
  debugPrint(2, "Frequency:   " + String(LORA_FREQ / 1E6) + " MHz");
  debugPrint(2, "Max Hops:    " + String(MAX_HOPS));
  debugPrint(2, "TX Interval: " + String(TX_INTERVAL/1000) + "s");
  debugPrint(2, "Debug Level: " + String(DEBUG_LEVEL));
  debugPrint(2, "===========================\n");
}