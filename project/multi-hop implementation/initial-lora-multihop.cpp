// LoRa Multi-Hop Implementation for ESP32 with Serial Debug
// Libraries
#include <SPI.h>
#include <LoRa.h>
#include <Arduino.h>

// Debug configuration
#define DEBUG_MODE true           
#define DEBUG_PRINT(x)     if(DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x)   if(DEBUG_MODE) { Serial.println(x); }
#define DEBUG_PRINTF(...)  if(DEBUG_MODE) { Serial.printf(__VA_ARGS__); }

// Pin definitions
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23

/*// Pin definitions for ESP32-WROOM32
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2
//*/
/// Pin definitions for ESP32-MisRed
#define LORA_SS 15
#define LORA_RST 26
#define LORA_DIO0 27
//*/
// Pin definitions for ESP32 TTGO

// LoRa parameters
#define FREQUENCY 922E6    
#define BANDWIDTH 125E3    
#define SPREADING_FACTOR 7
#define TX_POWER 20        
#define SYNC_WORD 0x12     

// Node configuration
#define NODE_ID 1          // Set to 1 for end node, 0 for gateway
#define GATEWAY_ID 0       
#define MAX_HOPS 3         
#define RETRY_COUNT 3      

// Message types
#define MSG_TYPE_DATA 1
#define MSG_TYPE_ROUTE_REQUEST 2
#define MSG_TYPE_ROUTE_RESPONSE 3
#define MSG_TYPE_ACK 4

// Debug timing
#define DEBUG_INTERVAL 10000  
unsigned long lastDebugTime = 0;

// Route discovery timing
#define ROUTE_DISCOVERY_INTERVAL 60000  // 60 seconds
unsigned long lastRouteDiscovery = 0;
bool hasValidRoute = false;

// Performance metrics
struct PerformanceMetrics {
    unsigned long messagesSent;
    unsigned long messagesReceived;
    unsigned long messagesForwarded;
    unsigned long messagesFailed;
    unsigned long lastRSSI;
    float lastSNR;
    unsigned long uptimeSeconds;
    unsigned long routingTableUpdates;
} metrics = {0};

// Message structure
struct LoRaMessage {
    uint8_t messageType;   
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t hopCount;
    uint8_t messageId;
    char payload[32];
};

// Routing table entry structure
struct RoutingEntry {
    uint8_t destinationId;
    uint8_t nextHopId;
    uint8_t hopCount;
    unsigned long lastUpdate;
    int8_t lastRSSI;    
    float lastSNR;      
};

// Global variables
RoutingEntry routingTable[10];  
uint8_t routingTableSize = 0;
uint8_t messageCounter = 0;

// Function declarations
void printLoRaParameters();
void printRoutingTable();
void updateMetrics();
void receiveMessage(int packetSize);
bool sendMessage(LoRaMessage& msg);
void generateRandomData(char* payload, int length);
bool findRoute(uint8_t destinationId, RoutingEntry& route);
void updateRoutingTable(uint8_t sourceId, uint8_t hopCount, int8_t rssi, float snr);
void printDebugInfo();
bool initLoRa();
void initiateRouteDiscovery();
bool waitForAck(uint8_t messageId);
void sendAck(uint8_t messageId, uint8_t destinationId);
void sendRouteResponse(uint8_t destinationId);

// Function to print LoRa parameters
void printLoRaParameters() {
    DEBUG_PRINTLN("\n=== LoRa Parameters ===");
    DEBUG_PRINTF("Frequency: %.2f MHz\n", FREQUENCY/1E6);
    DEBUG_PRINTF("Bandwidth: %.2f kHz\n", BANDWIDTH/1E3);
    DEBUG_PRINTF("Spreading Factor: %d\n", SPREADING_FACTOR);
    DEBUG_PRINTF("TX Power: %d dBm\n", TX_POWER);
    DEBUG_PRINTF("Sync Word: 0x%02X\n", SYNC_WORD);
    DEBUG_PRINTLN("=====================");
}

// Function to initialize LoRa
bool initLoRa() {
    DEBUG_PRINTLN("Initializing LoRa...");
    
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    
    if (!LoRa.begin(FREQUENCY)) {
        DEBUG_PRINTLN("Starting LoRa failed!");
        return false;
    }
    
    LoRa.setSpreadingFactor(SPREADING_FACTOR);
    LoRa.setSignalBandwidth(BANDWIDTH);
    LoRa.setTxPower(TX_POWER);
    LoRa.setSyncWord(SYNC_WORD);
    
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    
    DEBUG_PRINTLN("\n=== LoRa Multi-Hop Node Starting ===");
    DEBUG_PRINTF("Node ID: %d\n", NODE_ID);
    
    if (!initLoRa()) {
        DEBUG_PRINTLN("LoRa initialization failed!");
        while (1) {
            delay(1000);
        }
    }
    
    DEBUG_PRINTLN("LoRa initialization successful!");
    printLoRaParameters();
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        receiveMessage(packetSize);
    }
    
    if (NODE_ID != GATEWAY_ID) {
        static unsigned long lastSendTime = 0;
        if (millis() - lastSendTime > 30000) {  
            LoRaMessage msg;
            msg.messageType = MSG_TYPE_DATA;
            msg.sourceId = NODE_ID;
            msg.destinationId = GATEWAY_ID;
            msg.hopCount = 0;
            msg.messageId = messageCounter++;
            
            generateRandomData(msg.payload, sizeof(msg.payload) - 1);
            
            DEBUG_PRINT("Sending message: ");
            DEBUG_PRINTLN(msg.payload);
            
            if (sendMessage(msg)) {
                DEBUG_PRINTLN("Message sent successfully");
                metrics.messagesSent++;
            } else {
                DEBUG_PRINTLN("Failed to send message");
                metrics.messagesFailed++;
            }
            
            lastSendTime = millis();
        }
    }
    
    if (millis() - lastDebugTime > DEBUG_INTERVAL) {
        printDebugInfo();
        lastDebugTime = millis();
    }
    
    updateMetrics();
}

bool sendMessage(LoRaMessage& msg) {
    if (!hasValidRoute && msg.messageType == MSG_TYPE_DATA) {
        if (millis() - lastRouteDiscovery > ROUTE_DISCOVERY_INTERVAL) {
            initiateRouteDiscovery();
            return false;
        }
    }

    if (msg.hopCount >= MAX_HOPS) {
        DEBUG_PRINTLN("Maximum hop count exceeded");
        return false;
    }
    
    for (int i = 0; i < RETRY_COUNT; i++) {
        DEBUG_PRINTF("Transmission attempt %d/%d\n", i + 1, RETRY_COUNT);
        
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&msg, sizeof(LoRaMessage));
        
        if (LoRa.endPacket()) {
            if (msg.messageType == MSG_TYPE_DATA) {
                if (waitForAck(msg.messageId)) {
                    DEBUG_PRINTLN("ACK received");
                    return true;
                }
            } else {
                return true;
            }
        }
        
        delay(random(500, 1500));
    }
    
    return false;
}

void receiveMessage(int packetSize) {
    if (packetSize != sizeof(LoRaMessage)) {
        DEBUG_PRINTF("Invalid packet size: %d bytes\n", packetSize);
        return;
    }
    
    LoRaMessage msg;
    LoRa.readBytes((uint8_t*)&msg, sizeof(LoRaMessage));
    
    metrics.lastRSSI = LoRa.packetRssi();
    metrics.lastSNR = LoRa.packetSnr();
    
    updateRoutingTable(msg.sourceId, msg.hopCount, metrics.lastRSSI, metrics.lastSNR);
    
    switch (msg.messageType) {
        case MSG_TYPE_DATA:
            if (msg.destinationId == NODE_ID) {
                sendAck(msg.messageId, msg.sourceId);
                metrics.messagesReceived++;
                DEBUG_PRINTF("Data received: %s\n", msg.payload);
            } else if (msg.hopCount < MAX_HOPS) {
                msg.hopCount++;
                sendMessage(msg);
                metrics.messagesForwarded++;
            }
            break;
            
        case MSG_TYPE_ROUTE_REQUEST:
            if (NODE_ID == GATEWAY_ID) {
                sendRouteResponse(msg.sourceId);
            } else if (msg.hopCount < MAX_HOPS) {
                msg.hopCount++;
                LoRa.beginPacket();
                LoRa.write((uint8_t*)&msg, sizeof(LoRaMessage));
                LoRa.endPacket();
            }
            break;
            
        case MSG_TYPE_ROUTE_RESPONSE:
            if (msg.destinationId == NODE_ID) {
                hasValidRoute = true;
            }
            break;
    }
}

void initiateRouteDiscovery() {
    DEBUG_PRINTLN("Initiating route discovery");
    
    LoRaMessage routeReq;
    routeReq.messageType = MSG_TYPE_ROUTE_REQUEST;
    routeReq.sourceId = NODE_ID;
    routeReq.destinationId = GATEWAY_ID;
    routeReq.hopCount = 0;
    routeReq.messageId = messageCounter++;
    
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&routeReq, sizeof(LoRaMessage));
    LoRa.endPacket();
    
    lastRouteDiscovery = millis();
}

bool waitForAck(uint8_t messageId) {
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
        int packetSize = LoRa.parsePacket();
        if (packetSize == sizeof(LoRaMessage)) {
            LoRaMessage ack;
            LoRa.readBytes((uint8_t*)&ack, sizeof(LoRaMessage));
            if (ack.messageType == MSG_TYPE_ACK && ack.messageId == messageId) {
                return true;
            }
        }
        delay(10);
    }
    return false;
}

void sendAck(uint8_t messageId, uint8_t destinationId) {
    LoRaMessage ack;
    ack.messageType = MSG_TYPE_ACK;
    ack.sourceId = NODE_ID;
    ack.destinationId = destinationId;
    ack.messageId = messageId;
    ack.hopCount = 0;
    
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&ack, sizeof(LoRaMessage));
    LoRa.endPacket();
}

void sendRouteResponse(uint8_t destinationId) {
    LoRaMessage routeResp;
    routeResp.messageType = MSG_TYPE_ROUTE_RESPONSE;
    routeResp.sourceId = NODE_ID;
    routeResp.destinationId = destinationId;
    routeResp.hopCount = 0;
    routeResp.messageId = messageCounter++;
    
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&routeResp, sizeof(LoRaMessage));
    LoRa.endPacket();
}

void updateRoutingTable(uint8_t sourceId, uint8_t hopCount, int8_t rssi, float snr) {
    bool routeFound = false;
    
    for (int i = 0; i < routingTableSize; i++) {
        if (routingTable[i].destinationId == sourceId) {
            if (hopCount < routingTable[i].hopCount || 
                (hopCount == routingTable[i].hopCount && rssi > routingTable[i].lastRSSI)) {
                routingTable[i].hopCount = hopCount;
                routingTable[i].lastUpdate = millis();
                routingTable[i].lastRSSI = rssi;
                routingTable[i].lastSNR = snr;
                metrics.routingTableUpdates++;
            }
            routeFound = true;
            break;
        }
    }
    
    if (!routeFound && routingTableSize < 10) {
        routingTable[routingTableSize].destinationId = sourceId;
        routingTable[routingTableSize].hopCount = hopCount;
        routingTable[routingTableSize].lastUpdate = millis();
        routingTable[routingTableSize].lastRSSI = rssi;
        routingTable[routingTableSize].lastSNR = snr;
        routingTableSize++;
        metrics.routingTableUpdates++;
    }
}

void printRoutingTable() {
    DEBUG_PRINTLN("\n=== Routing Table ===");
    DEBUG_PRINTLN("Dest\tHops\tRSSI\tSNR\tAge(s)");
    
    for (int i = 0; i < routingTableSize; i++) {
        unsigned long age = (millis() - routingTable[i].lastUpdate) / 1000;
        DEBUG_PRINTF("%d\t%d\t%d\t%.1f\t%lu\n",
            routingTable[i].destinationId,
            routingTable[i].hopCount,
            routingTable[i].lastRSSI,
            routingTable[i].lastSNR,
            age);
    }
    DEBUG_PRINTLN("==================");
}

void printDebugInfo() {
    DEBUG_PRINTLN("\n=== Node Status ===");
    DEBUG_PRINTF("Uptime: %lu seconds\n", metrics.uptimeSeconds);
    DEBUG_PRINTF("Messages Sent: %lu\n", metrics.messagesSent);
    DEBUG_PRINTF("Messages Received: %lu\n", metrics.messagesReceived);
    DEBUG_PRINTF("Messages Forwarded: %lu\n", metrics.messagesForwarded);
    DEBUG_PRINTF("Messages Failed: %lu\n", metrics.messagesFailed);
    DEBUG_PRINTF("Last RSSI: %ld\n", metrics.lastRSSI);
    DEBUG_PRINTF("Last SNR: %.2f\n", metrics.lastSNR);
    DEBUG_PRINTF("Routing Table Updates: %lu\n", metrics.routingTableUpdates);
    
    printRoutingTable();
}

void updateMetrics() {
    metrics.uptimeSeconds = millis() / 1000;
}

void generateRandomData(char* payload, int length) {
    const char charset[] = "0123456789"
                          "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                          "abcdefghijklmnopqrstuvwxyz";
    
    for (int i = 0; i < length - 1; i++) {
        int index = random(0, sizeof(charset) - 1);
        payload[i] = charset[index];
    }
    payload[length - 1] = '\0';
}

bool findRoute(uint8_t destinationId, RoutingEntry& route) {
    for (int i = 0; i < routingTableSize; i++) {
        if (routingTable[i].destinationId == destinationId) {
            if (millis() - routingTable[i].lastUpdate < 300000) {
                route = routingTable[i];
                return true;
            }
        }
    }
    return false;
}