// LoRa Multi-hop Gateway
// Libraries
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <DS3231-RTC.h>
#include <SD.h>
#include <Wire.h>
#include <FastLED.h>
#include <Arduino.h>

// Debug configuration
#define DEBUG_MODE true           
#define DEBUG_PRINT(x)     if(DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x)   if(DEBUG_MODE) { Serial.println(x); }
#define DEBUG_PRINTF(...)  if(DEBUG_MODE) { Serial.printf(__VA_ARGS__); }

// LED Configuration
#define LED_PIN     2
#define NUM_LEDS    2
CRGB leds[NUM_LEDS];

// RTC
RTClib myRTC;

// SD Card
const int CS = 5;

// LoRa Pin Configuration
/// Pin default ESP32
#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23
//*/

/*// Pin default TTGO
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
//*/

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

// LoRa Parameters
#define FREQUENCY       922E6    
#define BANDWIDTH       125E3    
#define SPREADING_FACTOR 7
#define TX_POWER       20        
#define SYNC_WORD      0x12     

// Node Configuration
#define NODE_ID        0           
#define GATEWAY_ID     0      

// Message Types
#define MSG_TYPE_DATA 1
#define MSG_TYPE_ROUTE_REQUEST 2
#define MSG_TYPE_ROUTE_RESPONSE 3
#define MSG_TYPE_ACK 4

// WiFi and Web Configuration
// Konfigurasi WiFi melalui file SD Card
char ssid[32];
char password[64]; 

// Konfigurasi WiFi langsung
const char* WIFI_SSID = "mnl-12";           // Ganti dengan SSID WiFi Anda
const char* WIFI_PASSWORD = "h4los3mu4";    // Ganti dengan password WiFi Anda
// URL API untuk POST
const char* serverName = "https://misred-iot.com/api/projects/6/values";
const char* authToken = "01J8D7YTYJ3FQ8RG7MVBSFMHW5";
unsigned long previousMillisWeb = 0;
const int webInterval = 25000;


// State Variables
bool inisiasi = false;
bool koneksi = false;
bool sd_isi = false;

// Data Variables untuk 4 node
float node1_value1, node1_value2;
float node2_value1, node2_value2;
float node3_value1, node3_value2;
float node4_value1, node4_value2;
uint8_t messageCounter = 0;

// Message Structure
struct LoRaMessage {
    uint8_t messageType;   
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t hopCount;      
    uint8_t messageId;
    char payload[32];
} __attribute__((packed));

// Function Declarations
// Function Declarations
void parseCredentialsAndConnect(const String &input);
void connectToWiFi();
void sendDataToWeb();
bool initLoRa();
void blinkLED0(CRGB color, int count, int delayMs);
void ledFunction();
void Datalog();
void parsePayload(const char* payload, float* rain_val, float* distance_val);
void receiveMessage(int packetSize);
void sendAck(uint8_t messageId, uint8_t destinationId);  // Tambahkan ini
void connectToWiFiDirect();
void printLoRaParameters();
void resetLoRa();

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // LED Setup
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    leds[0] = CRGB(0, 255, 0);
    leds[0].nscale8(255);
    FastLED.show();
    
    ///
    // Initialize SD Card
    if (!SD.begin(CS)) {
        DEBUG_PRINTLN("SD Card initialization failed!");
        blinkLED0(CRGB::Red, 2, 500);
        return;
    }
    //*/

    // Initialize LoRa
    if (!initLoRa()) {
        DEBUG_PRINTLN("LoRa initialization failed!");
        blinkLED0(CRGB::Red, 2, 500);
        return;
    } else {
        DEBUG_PRINTLN("LoRa initialized successfully!");
    }
    
    printLoRaParameters();

    ///
    // Check WiFi credentials from SD card
    File file10 = SD.open("/WIFI.txt");
    if (file10) {
        if (file10.size() > 0) {
            String line = file10.readStringUntil('\n');
            sd_isi = true;
            parseCredentialsAndConnect(line);
        } else {
            sd_isi = false;
            inisiasi = true;
        }
        file10.close();
    }
        //*/

    // Connect to WiFi
    /*//
    connectToWiFiDirect();
    //*/
    DEBUG_PRINTLN("Setup completed!");
}

void loop() {
    ledFunction();
    
    // Check for LoRa packets
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        receiveMessage(packetSize);
    }
    
    // Check if it's time to send data to web server
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisWeb >= webInterval) {
        DEBUG_PRINTLN("Attempting to send data to web server...");
        blinkLED0(CRGB::Blue, 3, 50);
        sendDataToWeb();
        Datalog();
        previousMillisWeb = currentMillis;
    }
}

void connectToWiFiDirect() {
    DEBUG_PRINTLN("\nConnecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attemptCount = 0;
    while (WiFi.status() != WL_CONNECTED && attemptCount < 40) {
        delay(500);
        DEBUG_PRINT(".");
        attemptCount++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        DEBUG_PRINTLN("\nWiFi connected");
        DEBUG_PRINTLN("IP address: ");
        DEBUG_PRINTLN(WiFi.localIP());
        koneksi = true;
        inisiasi = true;
    } else {
        DEBUG_PRINTLN("\nFailed to connect to WiFi");
        blinkLED0(CRGB::Red, 3, 200); // Indikasi koneksi gagal
    }
}

void receiveMessage(int packetSize) {
    if (packetSize > 0) {
        LoRaMessage msg;
        int bytesRead = LoRa.readBytes((uint8_t*)&msg, sizeof(LoRaMessage));
        
        DEBUG_PRINTF("Received packet size: %d bytes, Read: %d bytes\n", packetSize, bytesRead);
        DEBUG_PRINTF("Message Type: %d, Source: %d, Destination: %d, Message ID: %d\n",
                    msg.messageType, msg.sourceId, msg.destinationId, msg.messageId);
        DEBUG_PRINTF("RSSI: %d, SNR: %.2f\n", LoRa.packetRssi(), LoRa.packetSnr());
        
        if (msg.messageType == MSG_TYPE_DATA && msg.destinationId == GATEWAY_ID) {
            // Send ACK immediately before any other processing
            DEBUG_PRINTLN("Valid data message received, sending ACK...");
            sendAck(msg.messageId, msg.sourceId);
            
            // Parse data berdasarkan source ID
            float val1, val2;
            parsePayload(msg.payload, &val1, &val2);
            
            // Simpan data sesuai dengan node pengirim
            switch(msg.sourceId) {
                case 1:
                    node1_value1 = val1;
                    node1_value2 = val2;
                    break;
                case 2:
                    node2_value1 = val1;
                    node2_value2 = val2;
                    break;
                case 3:
                    node3_value1 = val1;
                    node3_value2 = val2;
                    break;
                case 4:
                    node4_value1 = val1;
                    node4_value2 = val2;
                    break;
            }
            
            DEBUG_PRINTLN("=== Data received ===");
            DEBUG_PRINTF("From Node: %d\n", msg.sourceId);
            DEBUG_PRINTF("Raw payload: %s\n", msg.payload);
            DEBUG_PRINTF("Value 1: %.2f\n", val1);
            DEBUG_PRINTF("Value 2: %.2f\n", val2);
            DEBUG_PRINTLN("==================");
        }
    }
}

void sendAck(uint8_t messageId, uint8_t destinationId) {
    DEBUG_PRINTF("Preparing ACK for messageId: %d to destination: %d\n", messageId, destinationId);
    
    LoRaMessage ack;
    memset(&ack, 0, sizeof(LoRaMessage)); // Clear struktur
    ack.messageType = MSG_TYPE_ACK;       // Pastikan ini 4
    ack.sourceId = NODE_ID;
    ack.destinationId = destinationId;
    ack.messageId = messageId;
    ack.hopCount = 0;                     // Tambahkan ini
    memset(ack.payload, 0, sizeof(ack.payload)); // Clear payload
    
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&ack, sizeof(LoRaMessage));
    
    if (LoRa.endPacket()) {
        DEBUG_PRINTLN("ACK sent successfully");
        DEBUG_PRINTF("ACK details - Type: %d, ID: %d, Dest: %d\n", 
                    ack.messageType, ack.messageId, ack.destinationId);
        DEBUG_PRINTF("ACK RSSI: %d, SNR: %.2f\n", LoRa.packetRssi(), LoRa.packetSnr());
    } else {
        DEBUG_PRINTLN("Failed to send ACK!");
    }
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

void parseCredentialsAndConnect(const String &input) {
    int firstDelimiter = input.indexOf('n');
    int secondDelimiter = input.indexOf("xx");  // Gunakan string instead of char
    int thirdDelimiter = input.indexOf('z');
    
    if (firstDelimiter != -1 && secondDelimiter != -1 && thirdDelimiter != -1) {
        String ssidStr = input.substring(firstDelimiter + 1, secondDelimiter);
        String passStr = input.substring(secondDelimiter + 2, thirdDelimiter);
        ssidStr.toCharArray(ssid, sizeof(ssid));
        passStr.toCharArray(password, sizeof(password));
        sd_isi = true;
        connectToWiFi();
    }
}

void connectToWiFi() {
    DEBUG_PRINTLN("\nConnecting to WiFi...");
    WiFi.begin(ssid, password);
    
    int attemptCount = 0;
    while (WiFi.status() != WL_CONNECTED && attemptCount < 40) {
        delay(500);
        DEBUG_PRINT(".");
        attemptCount++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        DEBUG_PRINTLN("\nWiFi connected");
        koneksi = true;
        inisiasi = true;
    }
}

void sendDataToWeb() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(serverName);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-Auth-Token", authToken);
        
        // Create JSON payload for 8 data streams (2 data per node × 4 nodes)
        String jsonPayload = "{";
        jsonPayload += "\"datastreams\": [11, 12, 13, 14, 15, 16, 17, 18],";
        jsonPayload += "\"values\": [";
        jsonPayload += String(node1_value1) + ", " + String(node1_value2) + ", ";
        jsonPayload += String(node2_value1) + ", " + String(node2_value2) + ", ";
        jsonPayload += String(node3_value1) + ", " + String(node3_value2) + ", ";
        jsonPayload += String(node4_value1) + ", " + String(node4_value2);
        jsonPayload += "]}";
        
        DEBUG_PRINTLN("Sending payload: " + jsonPayload);
        
        int httpResponseCode = http.POST(jsonPayload);
        
        if (httpResponseCode > 0) {
            String response = http.getString();
            DEBUG_PRINTLN("HTTP Response code: " + String(httpResponseCode));
            DEBUG_PRINTLN("Response: " + response);
            blinkLED0(CRGB::Green, 1, 200);
        } else {
            DEBUG_PRINTLN("Error on sending POST: " + String(httpResponseCode));
            blinkLED0(CRGB::Red, 1, 200);
        }
        
        http.end();
    } else {
        DEBUG_PRINTLN("WiFi Disconnected");
        blinkLED0(CRGB::Red, 3, 200);
        connectToWiFiDirect();
    }
}

void Datalog() {
    DateTime now = myRTC.now();
    String getMonthStr = now.getMonth() < 10 ? "0" + String(now.getMonth()) : String(now.getMonth());
    String getDayStr = now.getDay() < 10 ? "0" + String(now.getDay()) : String(now.getDay());
    String namaFile = getDayStr + getMonthStr + String(now.getYear(), DEC);
    
    File myFile10 = SD.open("/datalog/" + namaFile + ".txt", FILE_APPEND);
    if (myFile10) {
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
        
        // Log data from all nodes
        myFile10.print("Node1: ");
        myFile10.print(node1_value1);
        myFile10.print(", ");
        myFile10.print(node1_value2);
        myFile10.print(" | Node2: ");
        myFile10.print(node2_value1);
        myFile10.print(", ");
        myFile10.print(node2_value2);
        myFile10.print(" | Node3: ");
        myFile10.print(node3_value1);
        myFile10.print(", ");
        myFile10.print(node3_value2);
        myFile10.print(" | Node4: ");
        myFile10.print(node4_value1);
        myFile10.print(", ");
        myFile10.println(node4_value2);
        
        myFile10.close();
    }
}

void blinkLED0(CRGB color, int times, int delayTime) {
    for (int i = 0; i < times; i++) {
        leds[0] = color;
        FastLED.show();
        delay(delayTime);
        leds[0] = CRGB::Black;
        FastLED.show();
        delay(delayTime);
    }
}

void ledFunction() {
    leds[0] = CRGB(0, 255, 0);
    leds[0].nscale8(255);
    FastLED.show();
    
    leds[1] = CRGB(255, 255, 255);
    leds[1].nscale8(100);
    FastLED.show();
}

bool initLoRa() {
    DEBUG_PRINTLN("Initializing LoRa Gateway...");
    
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

void printLoRaParameters() {
    DEBUG_PRINTLN("\n=== LoRa Parameters (Gateway) ===");
    DEBUG_PRINTF("Frequency: %.2f MHz\n", FREQUENCY/1E6);
    DEBUG_PRINTF("Bandwidth: %.2f kHz\n", BANDWIDTH/1E3);
    DEBUG_PRINTF("Spreading Factor: %d\n", SPREADING_FACTOR);
    DEBUG_PRINTF("TX Power: %d dBm\n", TX_POWER);
    DEBUG_PRINTF("Sync Word: 0x%02X\n", SYNC_WORD);
    DEBUG_PRINTLN("=====================");
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