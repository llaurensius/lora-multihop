// LoRa Multi-Hop Implementation for ESP32 with Serial Debug
// Libraries
#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <FS.h>
#include <SD.h>
#include <Wire.h>
#include <DS3231-RTC.h>
#include <FastLED.h>
#include <Arduino.h>

// Debug configuration
#define DEBUG_MODE true           
#define DEBUG_PRINT(x)     if(DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x)   if(DEBUG_MODE) { Serial.println(x); }
#define DEBUG_PRINTF(...)  if(DEBUG_MODE) { Serial.printf(__VA_ARGS__); }

// Setup existing
// ===================================================

// LED
#define enTxPin     4
#define LED_PIN     2
#define NUM_LEDS    2
CRGB leds[NUM_LEDS];
int hue = 0;

// RTC
RTClib myRTC;

// SD Card
const int CS = 5;

// Software Serial
SoftwareSerial mySerial(16, 17);
SoftwareSerial mySerial1(35, 34);

// Prototypes declaration functions
uint16_t calculateCRC(uint8_t *data, uint8_t len);
unsigned char data[4] = {};
float rain_val;
float distance_val;

uint16_t calculateCRC(uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
      crc ^= data[i];
      for (uint8_t j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc;
  }

// ===================================================
// End of existing setup

// Pin definitions LoRa
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
/*// Pin definitions for ESP32 TTGO
#define LORA_SS 18
#define LORA_RST 24
#define LORA_DIO0 26
//*/

// LoRa parameters
#define FREQUENCY 922E6    
#define BANDWIDTH 125E3    
#define SPREADING_FACTOR 7
#define TX_POWER 20        
#define SYNC_WORD 0x12     

// Node configuration
#define NODE_ID 4           // Set for end node
#define GATEWAY_ID 0      // Set for gateway       
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
    int lastRSSI;
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
} __attribute__((packed));

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
void blinkLED0(CRGB color, int count, int delayMs);
void initSDCard();
void bacaRain();
void bacaDistance();
void Datalog();
void DatalogError();
void DatalogRoutingTables();
void DatalogNodeStatus();
void ledFunction();
void resetLoRa();


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
    
    DEBUG_PRINTLN("LoRa.begin() successful.");
    DEBUG_PRINTLN("LoRa settings applied.");
    DEBUG_PRINTLN("LoRa initialization completed.");
    return true;
}

void setup() {
    Serial.begin(115200);
    mySerial.begin(9600);
    mySerial1.begin(9600);
    Wire.begin();  

    // Fungsi LED
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    leds[0] = CRGB(0, 255, 0);      // LED 0 warna hijau dengan kecerahan penuh
    leds[0].nscale8(255);           // Set kecerahan LED 0
    FastLED.show();
    
    // Inisialisasi SD Card
    initSDCard();

    while (!Serial && millis() < 5000);
    
    DEBUG_PRINTLN("\n=== LoRa Multi-Hop Node Starting ===");
    DEBUG_PRINTF("Node ID: %d\n", NODE_ID);
    
    if (!initLoRa()) {
        DEBUG_PRINTLN("LoRa initialization failed!");
        blinkLED0(CRGB::Red, 2, 500);
        while (1) {
            delay(1000);
        }
    } else {
        DEBUG_PRINTLN("LoRa initialized successfully!");
    }

    pinMode(enTxPin, OUTPUT);
    digitalWrite(enTxPin, HIGH);

    printLoRaParameters();
}

void loop() {

    ledFunction();
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
            
            // Uncomment to using actual data
            generateRandomData(msg.payload, sizeof(msg.payload) - 1);
            DEBUG_PRINT("Sending message: ");
            DEBUG_PRINTLN(msg.payload);
            
            if (sendMessage(msg)) {
                DEBUG_PRINTLN("Message sent successfully");
                blinkLED0(CRGB::Blue, 2, 100);
                metrics.messagesSent++;
                Datalog();
            } else {
                DEBUG_PRINTLN("Failed to send message");
                metrics.messagesFailed++;
                DatalogError();
            }
            
            lastSendTime = millis();
        }
    }
    
    if (millis() - lastDebugTime > DEBUG_INTERVAL) {
        printDebugInfo();
        DatalogRoutingTables();
        DatalogNodeStatus();
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
        DEBUG_PRINTF("Sending message - Type: %d, ID: %d, Dest: %d\n",
            msg.messageType, msg.messageId, msg.destinationId);
        
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&msg, sizeof(LoRaMessage));
        
        if (LoRa.endPacket()) {
            DEBUG_PRINTLN("Packet sent. Waiting for ACK...");
            DEBUG_PRINTF("RSSI: %d, SNR: %.2f\n", 
                        LoRa.packetRssi(),
                        LoRa.packetSnr());
            if (msg.messageType == MSG_TYPE_DATA) {
                if (waitForAck(msg.messageId)) {
                    DEBUG_PRINTLN("ACK received");
                    return true;
                } else {
                    DEBUG_PRINTLN("ACK not received.");
                }
            } else {
                return true;
            }
        } else {
            DEBUG_PRINTLN("LoRa.endPacket() failed!");
            DEBUG_PRINTF("Last RSSI: %d, Last SNR: %.2f\n", 
                        LoRa.packetRssi(),
                        LoRa.packetSnr());
            // Uncomment jika ingin mencoba reset LoRa pada kegagalan
            // resetLoRa();
        }
        
        delay(random(500, 1500));
    }
    
    return false;
}

void parsePayload(const char* payload, float* rain_val, float* distance_val) {
    char* ptr = strstr(payload, "R:");
    if (ptr) {
        *rain_val = atof(ptr + 2);
    }
    
    ptr = strstr(payload, "D:");
    if (ptr) {
        *distance_val = atof(ptr + 2);
    }
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
                parsePayload(msg.payload, &rain_val, &distance_val);
                DEBUG_PRINTLN("=== Data received ===");
                DEBUG_PRINTF("From Node: %d\n", msg.sourceId);
                DEBUG_PRINTF("Raw payload: %s\n", msg.payload);
                DEBUG_PRINTF("Rain Value: %.2f%%\n", rain_val);
                DEBUG_PRINTF("Distance: %.2f cm\n", distance_val);
                DEBUG_PRINTLN("==================");
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
    if (LoRa.endPacket()) {
        DEBUG_PRINTLN("Route request sent.");
        DEBUG_PRINTF("RSSI: %d, SNR: %.2f\n", 
                    LoRa.packetRssi(),
                    LoRa.packetSnr());
    } else {
        DEBUG_PRINTLN("Failed to send route request.");
        DEBUG_PRINTF("Last RSSI: %d, Last SNR: %.2f\n", 
                    LoRa.packetRssi(),
                    LoRa.packetSnr());
    }
    
    lastRouteDiscovery = millis();
}

bool waitForAck(uint8_t messageId) {
    unsigned long startTime = millis();
    DEBUG_PRINTF("Waiting for ACK with ID: %d\n", messageId);
    
    while (millis() - startTime < 5000) {
        int packetSize = LoRa.parsePacket();
        if (packetSize == sizeof(LoRaMessage)) {
            LoRaMessage ack;
            memset(&ack, 0, sizeof(LoRaMessage)); // Clear struktur
            LoRa.readBytes((uint8_t*)&ack, sizeof(LoRaMessage));
            
            DEBUG_PRINTF("Received packet - Type: %d, ID: %d, Source: %d, Dest: %d\n",
                        ack.messageType, ack.messageId, ack.sourceId, ack.destinationId);
            
            if (ack.messageType == MSG_TYPE_ACK && 
                ack.messageId == messageId && 
                ack.destinationId == NODE_ID) {
                DEBUG_PRINTLN("Valid ACK received!");
                return true;
            } else {
                DEBUG_PRINTF("Invalid ACK - Expected ID: %d, Got ID: %d, Type: %d\n",
                            messageId, ack.messageId, ack.messageType);
            }
        }
        delay(10);
    }
    DEBUG_PRINTLN("ACK timeout!");
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
    // Generate random sensor values
    rain_val = random(0, 10000) / 100.0;    // 0-100%
    distance_val = random(0, 50000) / 100.0; // 0-500cm
    
    // Format the values into the payload string
    snprintf(payload, length, "R:%.2f,D:%.2f", rain_val, distance_val);
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

void resetLoRa() {
    DEBUG_PRINTLN("Resetting LoRa module...");
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);
    if (initLoRa()) {
        DEBUG_PRINTLN("LoRa reset successful");
    } else {
        DEBUG_PRINTLN("LoRa reset failed");
    }
}

// Existing Function Code
// ===================================================
void blinkLED0(CRGB color, int times, int delayTime) {
    for (int i = 0; i < times; i++) {
      leds[0] = color;
      FastLED.show();
      delay(delayTime);
      leds[0] = CRGB::Black;  // Matikan LED
      FastLED.show();
      delay(delayTime);
    }
  }

void initSDCard() {
  while (true) { // Loop tak terbatas untuk mencoba inisialisasi
    Serial.print("t0.txt=\"Initializing SD card...\"");

    if (SD.begin(CS)) {
      Serial.print("t0.txt=\"SD card initialization successful!\""); // Jika berhasil
      break; // Keluar dari loop jika inisialisasi berhasil
    } else {
      Serial.print("t0.txt=\"Initialization failed! Retrying in 5 seconds...\"");

      // Jika SD card gagal, LED0 berkedip merah 1 kali
      blinkLED0(CRGB::Red, 1, 500);

      // Tunggu 5 detik sebelum mencoba lagi
      delay(5000);
    }
  }
}

void bacaRain() {
//   Serial.println("Baca Rain jalan");
  uint8_t id = 1; // Ubah tipe data menjadi uint8_t

  // Kirim permintaan baca data ke RS485
  uint8_t buffer[] = { id, 0x03, 0x01, 0x05, 0x00, 0x01, 0x00, 0x00 };
  uint16_t crc = calculateCRC(buffer, sizeof(buffer) - 2);
  buffer[sizeof(buffer) - 2] = crc & 0xFF;  // LSB
  buffer[sizeof(buffer) - 1] = crc >> 8;    // MSB

  // Kirim permintaan ke RS485
  mySerial.write(buffer, sizeof(buffer));

  digitalWrite(enTxPin, LOW);  // Set ke RX mode untuk menerima balasan

  // Menunggu respons dari RS485
  unsigned long timeLimit = millis();
  while (millis() - timeLimit < 500) {  // Tunggu hingga 500ms
    if (mySerial.available()) {
      byte responseHeader[8];

      // Membaca 8 byte dari RS485
      mySerial.readBytes(responseHeader, 8);

      // Tampilkan balasan byte per byte di Serial Monitor
    //   Serial.println("Balasan dari RS485:");
      for (int i = 0; i < 8; i++) {
        // Serial.print("Byte ");
        // Serial.print(i);
        // Serial.print(": 0x");
        // Serial.println(responseHeader[i], HEX);  // Tampilkan dalam format heksadesimal
      }

      // Jika balasan sesuai (misal, id dan kode fungsi benar)
      if (responseHeader[0] == id && responseHeader[1] == 0x03 && responseHeader[2] == 0x02) {
        // Gabungkan byte 3 dan byte 4 menjadi satu nilai 16-bit
        uint16_t rawValue = (responseHeader[3] << 8) | responseHeader[4];

        // Simpan nilai rawValue sebagai int (tidak perlu pembagian karena ingin hasil integer)
        rain_val = rawValue;

        // Tampilkan nilai Rain di Serial Monitor
        // Serial.print("Nilai Rain (int): ");
        Serial.println(rain_val);
      }
    }
  }

  digitalWrite(enTxPin, HIGH);  // Set ke TX mode

//   Serial.println("Baca Rain Selesai");
}

void bacaDistance() {
    // Serial.println("Baca Distance jalan");
  
    unsigned long startTime = millis(); // Catat waktu saat mulai membaca data
  
    // Tunggu sampai data tersedia, dengan batas waktu 2 detik
    while (mySerial1.available() < 4) {
      if (millis() - startTime >= 2000) { // Jika waktu sudah lebih dari 2 detik
        Serial.println("ERROR: Timeout, no data received");
        return; // Keluar dari fungsi
      }
    }
  
    // Baca data sebanyak 4 byte
    for (int i = 0; i < 4; i++) {
      data[i] = mySerial1.read();
    }
  
    // Cek jika byte pertama adalah 0xff
    if (data[0] == 0xff) {
      int sum = (data[0] + data[1] + data[2]) & 0x00FF;  // Hitung checksum
      if (sum == data[3]) {  // Jika checksum sesuai
        float distance;
        distance = (data[1] << 8) + data[2];  // Hitung jarak
        distance_val = distance/10;
        if (distance > 280) {  // Jika jarak di atas ambang batas 28 cm
            
        //   Serial.print("distance = ");
        //   Serial.print(distance / 10);  // Konversi ke cm
        //   Serial.println(" cm");
        } else {
        //   Serial.println("Below the lower limit");
        }
      } else {
        // Serial.println("ERROR: Checksum mismatch");
      }
    } else {
    //   Serial.println("ERROR: Invalid start byte");
    }
  
    // Flush untuk memastikan buffer bersih
    while (mySerial1.available()) {
      mySerial1.read();
    }
  
    // Serial.println("Baca Distance selesai");
}

void DatalogNodeStatus() {
    DateTime now = myRTC.now();

    // Mendapatkan nilai bulan dengan format dua digit
    String getMonthStr = now.getMonth() < 10 ? "0" + String(now.getMonth()) : String(now.getMonth());

    // Mendapatkan nilai hari dengan format dua digit
    String getDayStr = now.getDay() < 10 ? "0" + String(now.getDay()) : String(now.getDay());

    // Menggabungkan semua komponen menjadi string namaFile dengan format "DDMMYYYY"
    String namaFile = getDayStr + getMonthStr + String(now.getYear(), DEC);

    File myFile10 = SD.open("/datalog/" + namaFile + "_status.txt", FILE_APPEND);
    if (myFile10) {
        // Write timestamp
        myFile10.print(now.getYear(), DEC);
        myFile10.print("-");
        myFile10.print(now.getMonth(), DEC);
        myFile10.print("-");
        myFile10.print(now.getDay(), DEC);
        myFile10.print(" ");
        myFile10.print(now.getHour(), DEC);
        myFile10.print(":");
        myFile10.print(now.getMinute(), DEC);
        myFile10.print(":");
        myFile10.print(now.getSecond(), DEC);
        myFile10.println();

        // Write node status
        myFile10.println("=== Node Status ===");
        myFile10.printf("Uptime: %lu seconds\n", metrics.uptimeSeconds);
        myFile10.printf("Messages Sent: %lu\n", metrics.messagesSent);
        myFile10.printf("Messages Received: %lu\n", metrics.messagesReceived);
        myFile10.printf("Messages Forwarded: %lu\n", metrics.messagesForwarded);
        myFile10.printf("Messages Failed: %lu\n", metrics.messagesFailed);
        myFile10.printf("Last RSSI: %d\n", metrics.lastRSSI);
        myFile10.printf("Last SNR: %.2f\n", metrics.lastSNR);
        myFile10.printf("Routing Table Updates: %lu\n", metrics.routingTableUpdates);
        myFile10.println("==================");

        myFile10.close();
    } else {
        Serial.println("error opening file for writing");
    }
}

void DatalogRoutingTables() {
    DateTime now = myRTC.now();

    // Mendapatkan nilai bulan dengan format dua digit
    String getMonthStr = now.getMonth() < 10 ? "0" + String(now.getMonth()) : String(now.getMonth());

    // Mendapatkan nilai hari dengan format dua digit
    String getDayStr = now.getDay() < 10 ? "0" + String(now.getDay()) : String(now.getDay());

    // Menggabungkan semua komponen menjadi string namaFile dengan format "DDMMYYYY"
    String namaFile = getDayStr + getMonthStr + String(now.getYear(), DEC);

    File myFile10 = SD.open("/datalog/" + namaFile + "_routing.txt", FILE_APPEND);
    if (myFile10) {
        // Write timestamp
        myFile10.print(now.getYear(), DEC);
        myFile10.print("-");
        myFile10.print(now.getMonth(), DEC);
        myFile10.print("-");
        myFile10.print(now.getDay(), DEC);
        myFile10.print(" ");
        myFile10.print(now.getHour(), DEC);
        myFile10.print(":");
        myFile10.print(now.getMinute(), DEC);
        myFile10.print(":");
        myFile10.print(now.getSecond(), DEC);
        myFile10.println();

        // Write routing table header
        myFile10.println("=== Routing Table ===");
        myFile10.println("Dest\tHops\tRSSI\tSNR\tAge(s)");

        // Write routing table entries
        for (int i = 0; i < routingTableSize; i++) {
            unsigned long age = (millis() - routingTable[i].lastUpdate) / 1000;
            myFile10.printf("%d\t%d\t%d\t%.1f\t%lu\n",
                routingTable[i].destinationId,
                routingTable[i].hopCount,
                routingTable[i].lastRSSI,
                routingTable[i].lastSNR,
                age);
        }
        myFile10.println("==================");
        myFile10.println();

        myFile10.close();
    } else {
        Serial.println("error opening file for writing");
    }
}

void DatalogError() {
    DateTime now = myRTC.now();

    // Mendapatkan nilai bulan dengan format dua digit
    String getMonthStr = now.getMonth() < 10 ? "0" + String(now.getMonth()) : String(now.getMonth());

    // Mendapatkan nilai hari dengan format dua digit
    String getDayStr = now.getDay() < 10 ? "0" + String(now.getDay()) : String(now.getDay());

    // Menggabungkan semua komponen menjadi string namaFile dengan format "DDMMYYYY"
    String namaFile = getDayStr + getMonthStr + String(now.getYear(), DEC);

    File myFile10 = SD.open("/datalog/" + namaFile + "_error.txt", FILE_APPEND);
    if (myFile10) {
        // Write timestamp
        myFile10.print(now.getYear(), DEC);
        myFile10.print("-");
        myFile10.print(now.getMonth(), DEC);
        myFile10.print("-");
        myFile10.print(now.getDay(), DEC);
        myFile10.print(" ");
        myFile10.print(now.getHour(), DEC);
        myFile10.print(":");
        myFile10.print(now.getMinute(), DEC);
        myFile10.print(":");
        myFile10.print(now.getSecond(), DEC);
        myFile10.print(", ");
        myFile10.print("Failed to send message: ");
        myFile10.print("Rain: ");
        myFile10.print(rain_val);
        myFile10.print(" mm, ");
        myFile10.print("Distance: ");
        myFile10.print(distance_val);
        myFile10.print(" cm, ");
        myFile10.print("RSSI: ");
        myFile10.print(metrics.lastRSSI);
        myFile10.print(", SNR: "); 
        myFile10.print(metrics.lastSNR);
        myFile10.print(", ACK ID: ");
        myFile10.print(messageCounter);
        myFile10.print(", Routing Table Updates: ");
        myFile10.print(metrics.routingTableUpdates);

        myFile10.println("");
        myFile10.close();

    } else {
        Serial.println("error");
    }
}

void Datalog() {
    DateTime now = myRTC.now();

    // Mendapatkan nilai bulan dengan format dua digit
    String getMonthStr = now.getMonth() < 10 ? "0" + String(now.getMonth()) : String(now.getMonth());

    // Mendapatkan nilai hari dengan format dua digit
    String getDayStr = now.getDay() < 10 ? "0" + String(now.getDay()) : String(now.getDay());

    // Menggabungkan semua komponen menjadi string namaFile dengan format "DDMMYYYY"
    String namaFile = getDayStr + getMonthStr + String(now.getYear(), DEC);

    File myFile10 = SD.open("/datalog/" + namaFile + ".txt", FILE_APPEND);
    if (myFile10) {
        // Write timestamp
        myFile10.print(now.getYear(), DEC);
        myFile10.print("-");
        myFile10.print(now.getMonth(), DEC);
        myFile10.print("-");
        myFile10.print(now.getDay(), DEC);
        myFile10.print(" ");
        myFile10.print(now.getHour(), DEC);
        myFile10.print(":");
        myFile10.print(now.getMinute(), DEC);
        myFile10.print(":");
        myFile10.print(now.getSecond(), DEC);
        myFile10.print(", ");
        myFile10.print("Rain: ");
        myFile10.print(rain_val);
        myFile10.print(" mm, ");
        myFile10.print("Distance: ");
        myFile10.print(distance_val);
        myFile10.print(" cm, ");
        myFile10.print("RSSI: ");
        myFile10.print(metrics.lastRSSI);
        myFile10.print(", SNR: "); 
        myFile10.print(metrics.lastSNR);
        myFile10.print(", ACK ID: ");
        myFile10.print(messageCounter);
        myFile10.print(", Routing Table Updates: ");
        myFile10.print(metrics.routingTableUpdates);

        myFile10.println("");
        myFile10.close();

    } else {
        Serial.println("error");
    }
}

void ledFunction() {
    leds[0] = CRGB(0, 255, 0);      // LED 0 warna hijau
    leds[0].nscale8(255);           // Set kecerahan LED 0
    FastLED.show();
  
    leds[1] = CRGB(255, 255, 255);  // LED 1 warna putih
    leds[1].nscale8(100);           // Set kecerahan LED 1
    FastLED.show();
}
// ===================================================
// End of existing function code