// LoRa Multi-hop Gateway
// Libraries
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <DS3231-RTC.h>
#include <SD.h>
#include <Wire.h>
#include <FastLED.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Debug configuration
#define DEBUG_MODE true           
#define DEBUG_PRINT(x)     if(DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x)   if(DEBUG_MODE) { Serial.println(x); }
#define DEBUG_PRINTF(...)  if(DEBUG_MODE) { Serial.printf(__VA_ARGS__); }

// Measurement Current
// ===================================================
SoftwareSerial mySerial2(14, 0); // RX, TX
String dataMeasurement = "";
void dataLogMeasurement();
// ===================================================
// End of Measurement Current

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

// RTOS Task Handles dan Semaphores
TaskHandle_t loraTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t powerTaskHandle = NULL;
SemaphoreHandle_t loraMutex = NULL;
SemaphoreHandle_t wifiMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
SemaphoreHandle_t sdMutex = NULL;

// Function Declarations
void parseCredentialsAndConnect(const String &input);
void connectToWiFi();
void sendDataToWeb();
bool initLoRa();
void blinkLED0(CRGB color, int count, int delayMs);
void ledFunction();
void Datalog();
void DatalogMeasurement();
void parsePayload(const char* payload, float* rain_val, float* distance_val);
void receiveMessage(int packetSize);
void sendAck(uint8_t messageId, uint8_t destinationId);
void connectToWiFiDirect();
void printLoRaParameters();
void resetLoRa();

//======================= RTOS TASKS =======================//
void loraTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for(;;) {
        // Bagian penerimaan
        if(xSemaphoreTake(loraMutex, portMAX_DELAY)) {
            int packetSize = LoRa.parsePacket();
            if(packetSize) receiveMessage(packetSize);
            xSemaphoreGive(loraMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void webTask(void *pvParameters) {
    for(;;) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillisWeb >= webInterval) {
            DEBUG_PRINTLN("Attempting to send data to web server...");
            blinkLED0(CRGB::Blue, 3, 50);
            sendDataToWeb();
            Datalog();
            previousMillisWeb = currentMillis;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Yield ke task lain
    }
}

void ledTask(void *pvParameters) {
    for(;;) {
        ledFunction();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void powerMeasurementTask(void *pvParameters) {
    for(;;) {
        if(mySerial2.available() > 0) {
            dataMeasurement = mySerial2.readStringUntil('\n');
            if(xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100))) {
                DatalogMeasurement();
                xSemaphoreGive(sdMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//======================= MODIFIED DEBUG FUNCTIONS =======================//
void DEBUG_PRINT_IMPL(String text) {
    if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.print(text);
        xSemaphoreGive(serialMutex);
    }
}

void DEBUG_PRINTF_IMPL(const char* format, ...) {
    if(xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        xSemaphoreGive(serialMutex);
    }
}

//======================= EXISTING FUNCTIONS =======================//
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
        vTaskDelay(pdMS_TO_TICKS(500));
        DEBUG_PRINT(".");
        attemptCount++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        DEBUG_PRINTLN("\nWiFi connected");
        char ip[16];
        WiFi.localIP().toString().toCharArray(ip, sizeof(ip));
        DEBUG_PRINTLN(ip);
        koneksi = true;
        inisiasi = true;
    } else {
        DEBUG_PRINTLN("\nFailed to connect to WiFi");
        blinkLED0(CRGB::Red, 3, 200); // Indikasi koneksi gagal
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
    if(xSemaphoreTake(sdMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        DateTime now = myRTC.now();
        String getMonthStr = now.getMonth() < 10 ? "0" + String(now.getMonth()) : String(now.getMonth());
        String getDayStr = now.getDay() < 10 ? "0" + String(now.getDay()) : String(now.getDay());
        String namaFile = getDayStr + getMonthStr + String(now.getYear(), DEC);
        
        File myFile10 = SD.open("/datalog/" + namaFile + ".txt", FILE_APPEND);
        if(myFile10) {
            myFile10.printf("%04d-%02d-%02d %02d:%02d:%02d, Node1: %.2f, %.2f | Node2: %.2f, %.2f | Node3: %.2f, %.2f | Node4: %.2f, %.2f\n",
                          now.getYear(), now.getMonth(), now.getDay(),
                          now.getHour(), now.getMinute(), now.getSecond(),
                          node1_value1, node1_value2,
                          node2_value1, node2_value2,
                          node3_value1, node3_value2,
                          node4_value1, node4_value2);
            myFile10.close();
        } else {
            DEBUG_PRINTLN("SD Error: Failed to open file");
        }
        xSemaphoreGive(sdMutex);
    } else {
        DEBUG_PRINTLN("Warning: SD card busy, data not logged");
    }
}

void DatalogMeasurement() {
    if(xSemaphoreTake(sdMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        DateTime now = myRTC.now();
        String getMonthStr = now.getMonth() < 10 ? "0" + String(now.getMonth()) : String(now.getMonth());
        String getDayStr = now.getDay() < 10 ? "0" + String(now.getDay()) : String(now.getDay());
        
        // Ganti karakter yang tidak valid dengan garis bawah (_)
        String frequencyStr = String(FREQUENCY/1E6);
        frequencyStr.replace(".", "_"); // Ganti titik dengan garis bawah
        String bandwidthStr = String(BANDWIDTH/1E3);
        bandwidthStr.replace(".", "_"); // Ganti titik dengan garis bawah
        
        String namaFile = getDayStr + getMonthStr + String(now.getYear(), DEC) + 
                          "_Frequency_" + frequencyStr + "MHz" + 
                          "_Bandwidth_" + bandwidthStr + 
                          "_TxPower_" + String(TX_POWER) + "dBm" + 
                          "_SpreadingFactor_" + String(SPREADING_FACTOR);
        
        File myFile10 = SD.open("/datalog/" + namaFile + ".txt", FILE_APPEND);
        if (myFile10) {
            myFile10.printf("%04d-%02d-%02d %02d:%02d:%02d, Measurement: %s\n",
                          now.getYear(), now.getMonth(), now.getDay(),
                          now.getHour(), now.getMinute(), now.getSecond(),
                          dataMeasurement.c_str());
            myFile10.close();
        } else {
            DEBUG_PRINTLN("Data log measurement error");
        }
        xSemaphoreGive(sdMutex);
    } else {
        DEBUG_PRINTLN("Warning: SD card busy, measurement data not logged");
    }
}

void blinkLED0(CRGB color, int times, int delayTime) {
    for (int i = 0; i < times; i++) {
        leds[0] = color;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(delayTime));
        leds[0] = CRGB::Black;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(delayTime));
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
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(LORA_RST, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (initLoRa()) {
        DEBUG_PRINTLN("LoRa reset successful");
    } else {
        DEBUG_PRINTLN("LoRa reset failed");
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

void connectToWiFiDirect() {
    DEBUG_PRINTLN("\nConnecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attemptCount = 0;
    while (WiFi.status() != WL_CONNECTED && attemptCount < 40) {
        vTaskDelay(pdMS_TO_TICKS(500));
        DEBUG_PRINT(".");
        attemptCount++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        DEBUG_PRINTLN("\nWiFi connected");
        char ip[16];
        WiFi.localIP().toString().toCharArray(ip, sizeof(ip));
        DEBUG_PRINTLN(ip);
        koneksi = true;
        inisiasi = true;
    } else {
        DEBUG_PRINTLN("\nFailed to connect to WiFi");
        blinkLED0(CRGB::Red, 3, 200); // Indikasi koneksi gagal
    }
}

void setup() {
    Serial.begin(115200);
    mySerial2.begin(115200);
    Wire.begin();
    
    // LED Setup
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    leds[0] = CRGB(0, 255, 0);
    leds[0].nscale8(255);
    FastLED.show();
    
    // Initialize SD Card
    if (!SD.begin(CS)) {
        DEBUG_PRINTLN("SD Card initialization failed!");
        blinkLED0(CRGB::Red, 2, 500);
        return;
    }
    
    // Initialize LoRa
    if (!initLoRa()) {
        DEBUG_PRINTLN("LoRa initialization failed!");
        blinkLED0(CRGB::Red, 2, 500);
        return;
    } else {
        DEBUG_PRINTLN("LoRa initialized successfully!");
    }
    
    printLoRaParameters();
    
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
    
    // Connect to WiFi
    if (!sd_isi) {
        connectToWiFiDirect();
    }
    
    //============== Bagian RTOS Tambahan ==============//
    // Buat mutex untuk resource sharing
    loraMutex = xSemaphoreCreateMutex();
    wifiMutex = xSemaphoreCreateMutex();
    serialMutex = xSemaphoreCreateMutex();
    sdMutex = xSemaphoreCreateMutex();
    
    // Konfigurasi task priorities
    UBaseType_t loraPriority = 3;    // Prioritas tertinggi
    UBaseType_t webPriority = 2;     // Prioritas medium
    UBaseType_t ledPriority = 1;     // Prioritas terendah
    UBaseType_t powerPriority = 2;   // Prioritas medium
    
    // Buat tasks
    xTaskCreatePinnedToCore(
        loraTask,          // Task function
        "LoRa",            // Nama task
        8192,              // Stack size (bytes)
        NULL,              // Parameters
        loraPriority,      // Priority
        &loraTaskHandle,   // Task handle
        0                  // Core 0
    );
    
    xTaskCreatePinnedToCore(
        webTask,
        "Web",
        4096,
        NULL,
        webPriority,
        &webTaskHandle,
        1                  // Core 1
    );
    
    xTaskCreatePinnedToCore(
        ledTask,
        "LED",
        1024,
        NULL,
        ledPriority,
        &ledTaskHandle,
        1
    );
    
    xTaskCreatePinnedToCore(
        powerMeasurementTask,
        "PowerMonitor",
        4096,
        NULL,
        powerPriority,
        &powerTaskHandle,
        1
    );
    
    // Hapus task setup karena sudah tidak diperlukan
    vTaskDelete(NULL);
}

void loop() {
    // Kosongkan loop utama karena semua operasi sudah di-handle task
    // Tetap diperlukan untuk kompatibilitas Arduino
    vTaskDelete(NULL);
}