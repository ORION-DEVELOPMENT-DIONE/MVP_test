#include <Arduino.h>
#include <EmonLib.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Preferences.h>
#include <time.h>

// Communication Protocol Enum
enum CommunicationProtocol {
  PROTOCOL_WIFI = 0,
  PROTOCOL_BLE = 1
};
#define WIFI_RECONNECT_INTERVAL 10000     // 10 seconds between WiFi reconnect attempts
#define WEBSOCKET_RECONNECT_INTERVAL 5000 // 5 seconds between WebSocket reconnect attempts
#define BLE_RECONNECT_INTERVAL 5000       // 5 seconds between BLE reconnect attempts
#define MAX_RECONNECT_ATTEMPTS 5          // Maximum number of reconnection attempts

// Configuration Constants
#define VOLTAGE_PIN 34
#define CURRENT_PIN 35
//NTP Server config

const char* ntpServer= "pool.ntp.org";
const long gmtOffset_sec=3600;
const int daylightOffset_sec=3600;

// Global Objects
EnergyMonitor emon1;
BluetoothSerial SerialBT;
Preferences preferences;
WebSocketsClient webSocket;

// Global Variables
CommunicationProtocol currentProtocol = PROTOCOL_WIFI;
double amps, volts;
float power = 0, wattH = 0, kWh = 0;
float maxCurrent = 0, maxPower = 0;
unsigned long totalRunTime = 0;
unsigned long lastWiFiReconnectAttempt = 0;
unsigned long lastWebSocketReconnectAttempt = 0;
unsigned long lastBLEReconnectAttempt = 0;
int wifiReconnectAttempts = 0;
int websocketReconnectAttempts = 0;
int bleReconnectAttempts = 0;

// WiFi & WebSocket Configuration
const char* WIFI_SSID = "TT_0950";
const char* WIFI_PASSWORD = "hgy2jriavq";
const char* WS_HOST = "192.168.1.18";  // Raspberry Pi IP
const uint16_t WS_PORT = 8266;

// Calibration and Filtering
float voltageCal = 155.75;
float currentCal = 5.75;
const float alpha = 0.5;
float filteredAmps = 0;
float filteredVolts = 0;
struct EnergyMetrics {
  float voltage;
  float current;
  float power;
  float energy;
  float maxCurrent;
  float maxPower;
  unsigned long timestamp;
  unsigned long runTime;
  
  // Convert metrics to JSON-like string for transmission
  String toTransmissionFormat() const {  // Add 'const' here
    char buffer[300];
    snprintf(buffer, sizeof(buffer), 
      "{"
      "\"voltage\":%.2f,"
      "\"current\":%.2f,"
      "\"power\":%.2f,"
      "\"energy\":%.3f,"
      "\"max_current\":%.2f,"
      "\"max_power\":%.2f,"
      "\"timestamp\":%lu,"
      "\"run_time\":%lu"
      "}",
      voltage, current, power, energy, 
      maxCurrent, maxPower, timestamp, runTime);
    return String(buffer);
  }
};

// Function Prototypes
void setupCommunicationProtocol();
void sendDataToRaspberryPi(const EnergyMetrics& metrics);
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
bool getUserProtocolChoice();
void configureNTP();
String getFormattedTimestamp();

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastWiFiReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
      lastWiFiReconnectAttempt = currentMillis;
      
      Serial.println("WiFi Disconnected. Attempting to reconnect...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      
      wifiReconnectAttempts++;
      
      if (WiFi.status() == WL_CONNECTED) {
        wifiReconnectAttempts = 0;
        Serial.println("WiFi Reconnected Successfully");
        // Reconfigure NTP after reconnection
        configureNTP();
      } else if (wifiReconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
        Serial.println("Max WiFi Reconnect Attempts Reached. Restarting Device...");
        ESP.restart();
      }
    }
  }
}

void checkWebSocketConnection() {
  if (WiFi.status() == WL_CONNECTED && !webSocket.isConnected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastWebSocketReconnectAttempt >= WEBSOCKET_RECONNECT_INTERVAL) {
      lastWebSocketReconnectAttempt = currentMillis;
      
      Serial.println("WebSocket Disconnected. Attempting to reconnect...");
      webSocket.begin(WS_HOST, WS_PORT);
      
      websocketReconnectAttempts++;
      
      if (!webSocket.isConnected() && websocketReconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
        Serial.println("Max WebSocket Reconnect Attempts Reached. Restarting Device...");
        ESP.restart();
      }
    }
  }
}

void checkBLEConnection() {
  static bool wasConnected = false;
  bool isConnected = SerialBT.hasClient();
  
  if (!isConnected && wasConnected) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBLEReconnectAttempt >= BLE_RECONNECT_INTERVAL) {
      lastBLEReconnectAttempt = currentMillis;
      
      Serial.println("Bluetooth Disconnected. Attempting to restart...");
      SerialBT.end();
      delay(500);
      SerialBT.begin("ESP32_EnergyMonitor");
      
      bleReconnectAttempts++;
      
      if (bleReconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
        Serial.println("Max Bluetooth Reconnect Attempts Reached. Restarting Device...");
        ESP.restart();
      }
    }
  } else if (isConnected) {
    bleReconnectAttempts = 0;
    wasConnected = true;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Sensor initialization
  analogReadResolution(12);
  emon1.current(CURRENT_PIN, currentCal);
  emon1.voltage(VOLTAGE_PIN, voltageCal, 1.732);
  
  // Protocol selection via Serial
  Serial.println("Select Communication Protocol:");
  Serial.println("0 - WiFi");
  Serial.println("1 - Bluetooth");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  // Read protocol selection
  currentProtocol = (CommunicationProtocol)(Serial.parseInt());
  
  // Setup selected protocol
  setupCommunicationProtocol();
}

void setupCommunicationProtocol() {
  switch(currentProtocol) {
    case PROTOCOL_WIFI:
      // Setup WiFi and WebSocket
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      Serial.print("Connecting to WiFi");
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("\nWiFi Connected");
      
      // Configure NTP after WiFi connection
      configureNTP();
      
      // Setup WebSocket
      webSocket.begin(WS_HOST, WS_PORT);
      webSocket.onEvent(webSocketEvent);
      
      // Disable Bluetooth
      SerialBT.end();
      break;
    
    case PROTOCOL_BLE:
      // Setup Bluetooth
      SerialBT.begin("ESP32_EnergyMonitor");
      Serial.println("Bluetooth Started");
      
      // Disable WiFi
      WiFi.mode(WIFI_MODE_NULL);
      break;
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      websocketReconnectAttempts = 0; // Reset attempts on clean disconnect
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      websocketReconnectAttempts = 0;
      break;
    case WStype_TEXT:
      // Handle any incoming messages if needed
      break;
    case WStype_ERROR:
      Serial.println("WebSocket error");
      break;
  }
}
void configureNTP() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  Serial.println("Waiting for time sync...");
  time_t now = time(nullptr);
  int attempts = 0;
  while (now < 1000000000 && attempts < 10) {
    delay(500);
    now = time(nullptr);
    attempts++;
    Serial.print(".");
  }
  
  if (now > 1000000000) {
    Serial.println("\nTime synchronized successfully");
  } else {
    Serial.println("\nFailed to get time from NTP server");
  }
}

String getFormattedTimestamp() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return "Time Not Available";
  }
  
  char buffer[80];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}
void sendDataToRaspberryPi(String data) {
  switch(currentProtocol) {
    case PROTOCOL_WIFI:
      webSocket.sendTXT(data);
      break;
    case PROTOCOL_BLE:
      SerialBT.println(data);
      break;
  }
}

void loop() {
   if (currentProtocol == PROTOCOL_WIFI) {
    checkWiFiConnection();
    checkWebSocketConnection();
  } else if (currentProtocol == PROTOCOL_BLE) {
    checkBLEConnection();
  }
  
  // Rest of your existing loop code remains the same
  static unsigned long startTime = millis();
  totalRunTime = (millis() - startTime) / 1000;
  
  if(currentProtocol == PROTOCOL_WIFI) {
    webSocket.loop();
  }
  
  // Handle WebSocket if WiFi is active
  totalRunTime = (millis() - startTime) / 1000; // in seconds
  
  if(currentProtocol == PROTOCOL_WIFI) {
    webSocket.loop();
  }
  
  // Energy monitoring logic
  emon1.calcVI(120, 2000);
  volts = emon1.Vrms;
  amps = emon1.calcIrms(1480);
  
  if (amps < 0.1) amps = 0;
  
  // Low-pass filtering
  filteredAmps = alpha * amps + (1 - alpha) * filteredAmps;
  filteredVolts = alpha * volts + (1 - alpha) * filteredVolts;
  
  float instantPower = filteredVolts * filteredAmps;
  power += instantPower;
  
  // Track max values
  maxCurrent = max(maxCurrent, filteredAmps);
  maxPower = max(maxPower, instantPower);
  
  // Periodic data update and transmission
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime >= 5000) {
    lastSendTime = millis();
    
    EnergyMetrics metrics;
    metrics.voltage = filteredVolts;
    metrics.current = filteredAmps;
    metrics.power = instantPower;
    metrics.energy = kWh;
    metrics.maxCurrent = maxCurrent;
    metrics.maxPower = maxPower;
    metrics.timestamp = time(nullptr);
    metrics.runTime = totalRunTime;
    
    // Send data via selected protocol
    sendDataToRaspberryPi(metrics);
    
    // Energy calculation
    wattH += power / 3600.0;
    power = 0;
    
    if (wattH >= 1000.0) {
      float kW = wattH / 1000.0;
      wattH = wattH - (kW * 1000.0);
      kWh += kW;
    }
    
    // Enhanced debug print
    Serial.println("\n===================================");
    Serial.println("   ENERGY MONITORING DATA");
    Serial.println("===================================");
    Serial.printf("  Timestamp   : %s\n", getFormattedTimestamp().c_str());
    Serial.printf("  Protocol    : %s\n", 
                  currentProtocol == PROTOCOL_WIFI ? "WiFi" : "Bluetooth");
    Serial.printf("  Voltage     : %.2f V\n", filteredVolts);
    Serial.printf("  Current     : %.2f A (Max: %.2f A)\n", filteredAmps, maxCurrent);
    Serial.printf("  Power       : %.2f W (Max: %.2f W)\n", instantPower, maxPower);
    Serial.printf("  Total Energy: %.3f kWh\n", kWh);
    Serial.printf("  Run Time    : %lu seconds\n", totalRunTime);
    Serial.println("===================================");
  }

}
void sendDataToRaspberryPi(const EnergyMetrics& metrics) {
  String energyData = metrics.toTransmissionFormat();
  bool dataSent = false;
  
  switch(currentProtocol) {
    case PROTOCOL_WIFI:
      if (WiFi.status() == WL_CONNECTED && webSocket.isConnected()) {
        webSocket.send0TXT(energyData);
        dataSent = true;
      }
      break;
    case PROTOCOL_BLE:
      if (SerialBT.hasClient()) {
        SerialBT.println(energyData);
        dataSent = true;
      }
      break;
  }
  
  if (!dataSent) {
    Serial.println("Failed to send data - connection issues");
  }
}
