#include <RTClib.h>
#include <WiFi.h>
#include <time.h>
#include <HTTPClient.h>

// WiFi credentials
const char* ssid = "Wifi Magister Terapan";

// NTP Server settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;     // GMT+7 = 7*3600
const int daylightOffset_sec = 0;     

// Create RTC object
RTC_DS3231 rtc;

// Forward declarations
bool checkInternetConnection();
void displayDateTime();

void connectToWiFi() {
  Serial.printf("Connecting to %s ", ssid);
  
  WiFi.begin(ssid);  // Open network, no password needed
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    if (checkInternetConnection()) {
      Serial.println("Internet connection confirmed");
    } else {
      Serial.println("No internet access - might need portal login");
    }
  } else {
    Serial.println("\nWiFi Connection Failed!");
  }
}

bool checkInternetConnection() {
  HTTPClient http;
  http.begin("http://www.google.com");
  int httpCode = http.GET();
  http.end();
  
  return httpCode > 0;
}

void updateRTCfromNTP() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

  rtc.adjust(DateTime(timeinfo.tm_year + 1900, 
                     timeinfo.tm_mon + 1, 
                     timeinfo.tm_mday, 
                     timeinfo.tm_hour,
                     timeinfo.tm_min, 
                     timeinfo.tm_sec));
  
  Serial.println("RTC time updated from NTP!");
}

void displayDateTime() {
  DateTime now = rtc.now();

  // Format: "YYYY-MM-DD HH:MM:SS"
  Serial.printf("%04d-%02d-%02d %02d:%02d:%02d\n",
    now.year(),
    now.month(),
    now.day(),
    now.hour(),
    now.minute(),
    now.second()
  );

  // Display temperature
  Serial.printf("Temperature: %.2f°C\n", rtc.getTemperature());
}

void setup() {
    Serial.begin(115200);
    
    // Initialize RTC
    if (!rtc.begin()) {
      Serial.println("Couldn't find RTC");
      while (1);
    }
  
    // Connect to WiFi
    connectToWiFi();
  
    // Get time from NTP and update RTC
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    updateRTCfromNTP();
  
    // Disconnect WiFi
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

void loop() {
    displayDateTime();
    delay(1000);
 }