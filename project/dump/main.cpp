// LoRa Multi-Hop Implementation for ESP32 with Serial Debug
// Libraries
#include <SPI.h>
#include <LoRa.h>
#include <Arduino.h>

// Debug configuration
#define DEBUG_MODE true           // Set to false to disable debug output

// Debug macros with variable arguments
#define DEBUG_PRINT(x)     if(DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x)   if(DEBUG_MODE) { Serial.println(x); }
#define DEBUG_PRINTF(...)  if(DEBUG_MODE) { Serial.printf(__VA_ARGS__); }

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
#define FREQUENCY 922E6    // Frequency in Hz (915MHz)
#define BANDWIDTH 125E3    // Bandwidth in Hz (125kHz)
#define SPREADING_FACTOR 7
#define TX_POWER 20        // Transmission power in dBm
#define SYNC_WORD 0x12     // Sync word for network isolation

// Node configuration
#define NODE_ID 2          // Unique identifier for this node
#define GATEWAY_ID 2       // Gateway node ID
#define MAX_HOPS 3         // Maximum number of hops allowed
#define RETRY_COUNT 3      // Number of retries for failed transmissions

// Debug timing
#define DEBUG_INTERVAL 10000  // Debug print interval in ms
unsigned long lastDebugTime = 0;

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
    int8_t lastRSSI;     // Added for signal strength tracking
    float lastSNR;       // Added for signal quality tracking
};

// Global variables
RoutingEntry routingTable[10];  // Stores up to 10 routes
uint8_t routingTableSize = 0;
uint8_t messageCounter = 0;

// Forward declarations of all functions
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
    
    // Initialize SPI
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    
    // Initialize LoRa
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    
    if (!LoRa.begin(FREQUENCY)) {
        DEBUG_PRINTLN("Starting LoRa failed!");
        return false;
    }
    
    // Configure LoRa parameters
    LoRa.setSpreadingFactor(SPREADING_FACTOR);
    LoRa.setSignalBandwidth(BANDWIDTH);
    LoRa.setTxPower(TX_POWER);
    LoRa.setSyncWord(SYNC_WORD);
    
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);  // Wait for Serial but timeout after 5s
    
    DEBUG_PRINTLN("\n=== LoRa Multi-Hop Node Starting ===");
    DEBUG_PRINTF("Node ID: %d\n", NODE_ID);
    
    if (!initLoRa()) {
        DEBUG_PRINTLN("LoRa initialization failed!");
        while (1) {
            delay(1000); // Wait in error state
        }
    }
    
    DEBUG_PRINTLN("LoRa initialization successful!");
    printLoRaParameters();
}

// Main loop function
void loop() {
    // Check for incoming messages
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        receiveMessage(packetSize);
    }
    
    // For end nodes: generate and send data periodically
    if (NODE_ID != GATEWAY_ID) {
        static unsigned long lastSendTime = 0;
        if (millis() - lastSendTime > 30000) {  // Send every 30 seconds
            LoRaMessage msg;
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
    
    // Print debug information periodically
    if (millis() - lastDebugTime > DEBUG_INTERVAL) {
        printDebugInfo();
        lastDebugTime = millis();
    }
    
    // Update metrics
    updateMetrics();
}

// Function to send a message
bool sendMessage(LoRaMessage& msg) {
    if (msg.hopCount >= MAX_HOPS) {
        DEBUG_PRINTLN("Maximum hop count exceeded");
        return false;
    }
    
    RoutingEntry route;
    if (!findRoute(msg.destinationId, route)) {
        DEBUG_PRINTLN("No route to destination");
        return false;
    }
    
    // Attempt to send with retries
    for (int i = 0; i < RETRY_COUNT; i++) {
        DEBUG_PRINTF("Transmission attempt %d/%d\n", i + 1, RETRY_COUNT);
        
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&msg, sizeof(LoRaMessage));
        
        if (LoRa.endPacket()) {
            DEBUG_PRINTF("Message sent successfully on attempt %d\n", i + 1);
            return true;
        }
        
        DEBUG_PRINTLN("Transmission failed, retrying...");
        delay(random(500, 1500));  // Random delay before retry
    }
    
    return false;
}

// Function to receive and process messages
void receiveMessage(int packetSize) {
    if (packetSize != sizeof(LoRaMessage)) {
        DEBUG_PRINTF("Invalid packet size: %d bytes\n", packetSize);
        return;
    }
    
    LoRaMessage msg;
    LoRa.readBytes((uint8_t*)&msg, sizeof(LoRaMessage));
    
    // Store signal quality metrics
    metrics.lastRSSI = LoRa.packetRssi();
    metrics.lastSNR = LoRa.packetSnr();
    
    DEBUG_PRINTLN("\n=== Message Received ===");
    DEBUG_PRINTF("Source: %d\n", msg.sourceId);
    DEBUG_PRINTF("Destination: %d\n", msg.destinationId);
    DEBUG_PRINTF("Hop Count: %d\n", msg.hopCount);
    DEBUG_PRINTF("Message ID: %d\n", msg.messageId);
    DEBUG_PRINTF("RSSI: %d\n", metrics.lastRSSI);
    DEBUG_PRINTF("SNR: %.2f\n", metrics.lastSNR);
    
    // Update routing table with signal quality information
    updateRoutingTable(msg.sourceId, msg.hopCount, metrics.lastRSSI, metrics.lastSNR);
    
    // Process message based on destination
    if (msg.destinationId == NODE_ID) {
        DEBUG_PRINTLN("Message is for this node");
        DEBUG_PRINTF("Payload: %s\n", msg.payload);
        metrics.messagesReceived++;
    } else if (msg.hopCount < MAX_HOPS) {
        DEBUG_PRINTLN("Forwarding message");
        msg.hopCount++;
        if (sendMessage(msg)) {
            metrics.messagesForwarded++;
        } else {
            metrics.messagesFailed++;
        }
    }
}

// Function to update routing table
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
                DEBUG_PRINTLN("Route updated");
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
        DEBUG_PRINTLN("New route added");
    }
}

// Function to print routing table
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

// Function to print debug information
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

// Function to update metrics
void updateMetrics() {
    metrics.uptimeSeconds = millis() / 1000;
}

// Function to generate random data
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

// Function to find a route to a destination
bool findRoute(uint8_t destinationId, RoutingEntry& route) {
    for (int i = 0; i < routingTableSize; i++) {
        if (routingTable[i].destinationId == destinationId) {
            // Check if route is still valid (not older than 5 minutes)
            if (millis() - routingTable[i].lastUpdate < 300000) {
                route = routingTable[i];
                return true;
            }
        }
    }
    return false;
}