/*
 * Ground Transceiver 1
 * 
 * Connects to TP-Link router or falls back to SoftAP mode
 * Displays status on OLED
 * Transmits messages via LoRa to Drone 1
 * Waits for ACK from Drone 1
 */

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

// OLED Display settings for TTGO LoRa32 v1.6.1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define OLED_SDA 21
#define OLED_SCL 22

// LoRa settings
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_FREQ 868E6  // Europe frequency

// WiFi Station settings
const char* ssid = "Emergency_KU"; // TP-Link router SSID
const char* password = ""; // Empty for open network, or set password if configured

// Transmission settings and thresholds
const int transmitInterval = 3000;  // Interval between retries (3 seconds)
const int maxRetries = 20;          // Number of retries before giving up

// Message queue settings
#define MAX_QUEUE_SIZE 10           // Maximum number of messages in the queue
#define MESSAGE_PROCESS_INTERVAL 100 // Reduce from 500ms to 100ms for more responsive queue handling
#define MAX_WAIT_FINAL_ACK_TIME 120000 // 2 minutes to wait for final ACK after D1 ACK
#define MAX_JSON_RESPONSE_SIZE 1024 // Limit JSON response size

// Add variable to control processing time per loop
const unsigned long MAX_PROCESSING_TIME_PER_LOOP = 50; // Max 50ms for queue processing per loop

// Message structure
struct EmergencyMessage {
  char text[128];
  float latitude;
  float longitude;
  int emergencyCode;
  uint32_t messageId;
};

// Message queue item structure
struct MessageQueueItem {
  EmergencyMessage message;
  unsigned long receiveTime;
  unsigned long lastTransmitTime;
  int retryCount;
  bool active;                   // Whether this queue item is in use
  bool finalAckReceived;         // Whether final ACK was received (end-to-end)
};

// Timing structure for message journey
struct MessageTiming {
  uint32_t messageId;
  unsigned long receiveTimeFromApp;
  unsigned long firstTransmitTime;
  unsigned long ackReceiveTime;
  unsigned long finalAckReceiveTime;
  int retryCount;
  bool deliveryComplete;
  int rssi;               // Signal strength in dBm
  float snr;              // Signal-to-noise ratio
  // Signal quality data from the entire chain
  int16_t drone1Rssi;
  float drone1Snr;
  int16_t drone2Rssi;
  float drone2Snr;
  int16_t ground2Rssi;
  float ground2Snr;
};

// System states
enum SystemState {
  IDLE,
  PROCESSING_QUEUE,
  MESSAGE_DELIVERED
};

// Global variables
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);
SystemState currentState = IDLE;
String serialBuffer = ""; // Buffer for incoming serial data
unsigned long lastQueueProcessTime = 0; // Last time we processed the queue

// Message queue
MessageQueueItem messageQueue[MAX_QUEUE_SIZE];
int queueCount = 0;

// Message timing info (latest 10 messages)
const int MAX_TIMING_RECORDS = 10;
MessageTiming timingRecords[MAX_TIMING_RECORDS];
int currentTimingIndex = 0;

// Network status info
struct DeviceStatus {
  unsigned long lastSeen;
  int rssi;
  float snr;
  bool isActive;
};

DeviceStatus deviceStatus[4];  // Status for all 4 devices (indexed 0-3, corresponding to IDs 1-4)

// Function declarations
void handleRoot();
void handleSendMessage();
void handleStatusPage();
void handleDataRequest();
void handleNetworkPage();
void handleNetworkData();
void handleMessageStatusRequest();
void displayStatus(String message);
void displayNetworkInfo();
void sendAckStatusToPython(uint32_t messageId, const char* status, int lastHopRssi, float lastHopSnr, MessageTiming* timingRecord);
void processSerialCommand(String command);
bool addToMessageQueue(EmergencyMessage* message, unsigned long receiveTime);
void processMessageQueue();
void processMessageQueueNonBlocking();
void sendLoRaMessage(MessageQueueItem* queueItem);
int findQueueItemById(uint32_t messageId);
void removeFromQueue(int index);
void updateQueueDisplay();

// Setup function
void setup() {
  Serial.begin(115200);
  serialBuffer.reserve(512); // Reserve space for serial buffer, increased for JSON
  
  // Initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Display startup info
  displayStatus("Starting system...");
  
  // Initialize LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    displayStatus("LoRa init failed!");
    while (true);
  }
  
  // Set up LoRa parameters
  LoRa.setSpreadingFactor(7);      // Changed from 11 to 7 for better reliability at shorter range
  LoRa.setSignalBandwidth(250E3);  // Changed from 125kHz to 250kHz for faster data rate at short range
  LoRa.setCodingRate4(5);          // Changed from 8 (4/8) to 5 (4/5) for better data rate at short range
  LoRa.setTxPower(20);             // Maximum power
  LoRa.enableCrc();                // Enable CRC checking
  LoRa.setSyncWord(0xF3);          // Added sync word for compatibility with Drone1
  
  // Connect to WiFi router
  WiFi.mode(WIFI_STA);
  
  // Optional: Configure static IP if needed
  IPAddress staticIP(192, 168, 0, 2);  // Changed from 192.168.1.2
  IPAddress gateway(192, 168, 0, 1);   // Changed from 192.168.1.1
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(staticIP, gateway, subnet);
  
  WiFi.begin(ssid, password);

  int attempts = 0;
  displayStatus("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected to WiFi. IP: ");
    Serial.println(WiFi.localIP());
    displayStatus("WiFi connected: " + WiFi.localIP().toString());
  } else {
    // Fallback to SoftAP mode if router connection fails
    Serial.println("Failed to connect to router. Switching to SoftAP mode.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    displayStatus("SoftAP mode: " + IP.toString());
  }
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/send", HTTP_POST, handleSendMessage);
  server.on("/status", handleStatusPage);
  server.on("/data", handleDataRequest);
  server.on("/network", handleNetworkPage);
  server.on("/network-data", handleNetworkData);
  server.on("/message-status", handleMessageStatusRequest);
  
  // Initialize timing records and device status
  for (int i = 0; i < MAX_TIMING_RECORDS; i++) {
    timingRecords[i].messageId = 0;
    timingRecords[i].deliveryComplete = false;
    timingRecords[i].rssi = 0;
    timingRecords[i].snr = 0;
    timingRecords[i].drone1Rssi = 0;
    timingRecords[i].drone1Snr = 0;
    timingRecords[i].drone2Rssi = 0;
    timingRecords[i].drone2Snr = 0;
    timingRecords[i].ground2Rssi = 0;
    timingRecords[i].ground2Snr = 0;
    timingRecords[i].retryCount = 0;
    timingRecords[i].receiveTimeFromApp = 0;
    timingRecords[i].firstTransmitTime = 0;
    timingRecords[i].ackReceiveTime = 0;
    timingRecords[i].finalAckReceiveTime = 0;
  }
  
  // Initialize message queue
  for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
    messageQueue[i].active = false;
    messageQueue[i].retryCount = 0;
    messageQueue[i].finalAckReceived = false;
  }
  queueCount = 0;
  
  for (int i = 0; i < 4; i++) {
    deviceStatus[i].lastSeen = 0;
    deviceStatus[i].rssi = 0;
    deviceStatus[i].snr = 0;
    deviceStatus[i].isActive = false;
  }
  deviceStatus[0].isActive = true;  // This device (Ground1) is active
  
  server.begin();
  
  displayStatus("System ready!");
  displayNetworkInfo();
  
  Serial.println("==============================================");
  Serial.println("  Ground Transceiver 1 - Timing Monitor");
  Serial.println("==============================================");
  Serial.println("  Access status page at: http://" + WiFi.localIP().toString() + "/status");
  Serial.println("  Access network status at: http://" + WiFi.localIP().toString() + "/network");
  Serial.println("==============================================");
}

// Loop function
void loop() {
  // Always handle web server requests first
  server.handleClient();
  
  // Process serial commands (keep this fast)
  while (Serial.available() > 0) { // Process all available serial data
    char incomingChar = Serial.read();
    serialBuffer += incomingChar;
    
    if (incomingChar == '\n') {
      if (serialBuffer.length() > 1) {
        processSerialCommand(serialBuffer);
      }
      serialBuffer = "";
    }
    
    if (serialBuffer.length() >= 511) {
      Serial.println("Serial buffer overflow, clearing.");
      serialBuffer = "";
    }
  }
  
  // Check for LoRa packets (high priority)
  parsePacket();
  
  // Process message queue with time limit to avoid blocking other operations
  unsigned long queueProcessStartTime = millis();
      if (queueCount > 0) { // Process queue immediately if items exist
      if (currentState != MESSAGE_DELIVERED) { // Avoid processing if a message just completed this cycle
          currentState = PROCESSING_QUEUE;
          // Process queue with time limits to prevent blocking
          processMessageQueueNonBlocking(); 
      } else {
          currentState = IDLE; // Reset state if message was just delivered this cycle
      }
  } else if (queueCount == 0 && currentState == PROCESSING_QUEUE) {
      currentState = IDLE;
      updateQueueDisplay();
  }
  
  // Additional debug: Log if we're waiting for final ACKs
  static unsigned long lastWaitingLog = 0;
  if (millis() - lastWaitingLog > 10000) { // Log every 10 seconds
    int waitingCount = 0;
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
      if (messageQueue[i].active && !messageQueue[i].finalAckReceived) {
        waitingCount++;
      }
    }
    if (waitingCount > 0) {
      Serial.println("G1 DEBUG: " + String(waitingCount) + " messages waiting for Final ACK");
    }
    lastWaitingLog = millis();
  }
}

// Handle incoming LoRa packets
void parsePacket() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Read packet header
    byte destination = LoRa.read();
    byte source = LoRa.read();
    byte msgType = LoRa.read();
    uint32_t msgId = 0;
    msgId |= ((uint32_t)LoRa.read()) << 24;
    msgId |= ((uint32_t)LoRa.read()) << 16;
    msgId |= ((uint32_t)LoRa.read()) << 8;
    msgId |= LoRa.read();
    
    // Get RSSI and SNR
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    
    // Update sender device status
    if (source >= 1 && source <= 4) {
      deviceStatus[source - 1].lastSeen = millis();
      deviceStatus[source - 1].rssi = rssi;
      deviceStatus[source - 1].snr = snr;
      deviceStatus[source - 1].isActive = true;
    }
    
    // Print raw packet data
    Serial.print("RECV [");
    Serial.print(millis());
    Serial.print("ms] From: ");
    Serial.print(source);
    Serial.print(", To: ");
    Serial.print(destination);
    Serial.print(", Type: 0x");
    Serial.print(msgType, HEX);
    Serial.print(", MsgID: ");
    Serial.print(msgId);
    Serial.print(", RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm, SNR: ");
    Serial.print(snr);
    Serial.println(" dB");
    
    // Check if the message is for this device (ID 1)
    if (destination == 1) {
      if (msgType == 0x01) {  // ACK message -> NO LONGER EXPECTED FROM D1
        // Find this message in our queue
        int queueIndex = findQueueItemById(msgId);
        // THIS BLOCK IS NOW OBSOLETE FOR D1 ACKS, but might be needed if other devices send simple ACKs?
        // For now, just log if we receive one unexpectedly.
        if (queueIndex >= 0) {
            // MessageQueueItem* qItem = &messageQueue[queueIndex];
            // qItem->ackReceived = true; // We no longer use this flag for D1
            Serial.println("WARNING: Received unexpected ACK (Type 0x01) from device " + String(source) + " for message " + String(msgId));
            // Update timing record? Maybe log the time?
             for (int i = 0; i < MAX_TIMING_RECORDS; i++) {
                 if (timingRecords[i].messageId == msgId && timingRecords[i].ackReceiveTime == 0) { // Only log first unexpected ACK time
                     timingRecords[i].ackReceiveTime = millis(); 
                     timingRecords[i].rssi = rssi; // Log signal of this unexpected ACK
                     timingRecords[i].snr = snr;   
                     break;
                 }
             }
        } else {
          Serial.println("Received ACK (Type 0x01) for unknown message ID: " + String(msgId));
        }
      } else if (msgType == 0x03) {  // Final ACK message, relayed by Drone1, originated from Ground2 via Drone2
        int queueIndex = findQueueItemById(msgId);
        if (queueIndex >= 0) {
          MessageQueueItem* qItem = &messageQueue[queueIndex];
          
          // Mark as Final ACK received
          qItem->finalAckReceived = true;
          
          displayStatus("Final ACK for msg " + String(msgId));
          Serial.println("FINAL ACK received for message " + String(msgId));
          
          // Check for enhanced ACK with signal data (larger packet size)
          int16_t ground2Rssi = 0;
          float ground2Snr = 0.0f;
          int16_t drone2Rssi = 0;
          float drone2Snr = 0.0f;
          int16_t drone1Rssi = 0;
          float drone1Snr = 0.0f;
          
          // Enhanced packet should be larger than basic packet
          if (packetSize > 12) { // Basic header + msgId is 7 bytes
            // Read Ground 2 signal data
            ground2Rssi = ((int16_t)LoRa.read() << 8);
            ground2Rssi |= LoRa.read();
            
            uint8_t snrBytes[4];
            // Read Ground 2 SNR (4 bytes)
            for (int i = 0; i < 4; i++) {
              snrBytes[i] = LoRa.read();
            }
            memcpy(&ground2Snr, snrBytes, 4);
            
            // Read Drone 2 signal data if available
            if (packetSize > 18) {
              drone2Rssi = ((int16_t)LoRa.read() << 8);
              drone2Rssi |= LoRa.read();
              
              // Read Drone 2 SNR
              for (int i = 0; i < 4; i++) {
                snrBytes[i] = LoRa.read();
              }
              memcpy(&drone2Snr, snrBytes, 4);
              
              // Read Drone 1 signal data if available
              if (packetSize > 24) {
                drone1Rssi = ((int16_t)LoRa.read() << 8);
                drone1Rssi |= LoRa.read();
                
                // Read Drone 1 SNR
                for (int i = 0; i < 4; i++) {
                  snrBytes[i] = LoRa.read();
                }
                memcpy(&drone1Snr, snrBytes, 4);
              }
            }
            
            Serial.println("Received complete signal chain data:");
            Serial.println("  Ground 2: RSSI " + String(ground2Rssi) + " dBm, SNR " + String(ground2Snr) + " dB");
            Serial.println("  Drone 2: RSSI " + String(drone2Rssi) + " dBm, SNR " + String(drone2Snr) + " dB");
            Serial.println("  Drone 1: RSSI " + String(drone1Rssi) + " dBm, SNR " + String(drone1Snr) + " dB");
          }
          
          // Update timing record for complete journey
          MessageTiming* completedRecord = NULL;
          for (int i = 0; i < MAX_TIMING_RECORDS; i++) {
            if (timingRecords[i].messageId == msgId) {
              timingRecords[i].finalAckReceiveTime = millis();
              timingRecords[i].deliveryComplete = true;
              
              // Store signal chain data
              timingRecords[i].ground2Rssi = ground2Rssi;
              timingRecords[i].ground2Snr = ground2Snr;
              timingRecords[i].drone2Rssi = drone2Rssi;
              timingRecords[i].drone2Snr = drone2Snr;
              timingRecords[i].drone1Rssi = drone1Rssi; // RSSI of Drone1 receiving from Ground2 via Drone2 (from packet)
              timingRecords[i].drone1Snr = drone1Snr;   // SNR of Drone1 receiving from Ground2 via Drone2 (from packet)
                                                        // timingRecords[i].rssi/snr is still D1's ACK to us.
              
              completedRecord = &timingRecords[i];
              break;
            }
          }
          
          // Send success status back to Python via Serial, including all signal data
          if (completedRecord != NULL) {
            sendAckStatusToPython(msgId, "delivered", rssi, snr, completedRecord);
            
            // Calculate and print total transmission time
            unsigned long totalTime = completedRecord->finalAckReceiveTime - completedRecord->receiveTimeFromApp;
            Serial.println("==============================================");
            Serial.println("COMPLETE JOURNEY for message " + String(msgId) + ":");
            Serial.println("Total time: " + String(totalTime) + "ms");
            Serial.println("Retry count (G1->D1): " + String(completedRecord->retryCount)); // Retries before FINAL ACK received
            Serial.println("Signal strength chain (received by each node):");
            Serial.println("  Drone 1 (from G1): RSSI " + String(completedRecord->drone1Rssi) + " dBm, SNR " + String(completedRecord->drone1Snr) + " dB"); // Note: This data comes from the ACK packet
            Serial.println("  Drone 2 (from D1): RSSI " + String(completedRecord->drone2Rssi) + " dBm, SNR " + String(completedRecord->drone2Snr) + " dB");
            Serial.println("  Ground 2 (from D2): RSSI " + String(completedRecord->ground2Rssi) + " dBm, SNR " + String(completedRecord->ground2Snr) + " dB");
            Serial.println("  Ground 1 (Final ACK from D1): RSSI " + String(rssi) + " dBm, SNR " + String(snr) + " dB"); // Use current packet's signal
            Serial.println("==============================================");
          } else {
            Serial.println("WARNING: Could not find timing record for message ID " + String(msgId));
            sendAckStatusToPython(msgId, "delivered", rssi, snr, NULL);
          }
          
          // Display message about successful delivery
          displayStatus("Msg " + String(msgId) + " Delivered!");
          
          // Message will be removed from queue by processMessageQueue
          updateQueueDisplay();
        } else {
          Serial.println("Received FINAL ACK for unknown message ID: " + String(msgId));
        }
      }
    }
  }
  
  // Check for device timeouts (assume devices are offline if not seen in 1 minute)
  unsigned long currentTime = millis();
  for (int i = 0; i < 4; i++) {
    if (deviceStatus[i].isActive && i != 0) { // Skip checking ourselves (Ground1)
      if (currentTime - deviceStatus[i].lastSeen > 60000) { // 1 minute timeout
        deviceStatus[i].isActive = false;
      }
    }
  }
}

// Send message via LoRa
void sendLoRaMessage(MessageQueueItem* queueItem) {
  unsigned long startTime = millis();
  queueItem->lastTransmitTime = startTime;
  queueItem->retryCount++;
  
  // If this is the first transmission, record it in timing records
  if (queueItem->retryCount == 1) {
    for (int i = 0; i < MAX_TIMING_RECORDS; i++) {
      if (timingRecords[i].messageId == queueItem->message.messageId) {
        timingRecords[i].firstTransmitTime = startTime;
        break;
      }
    }
  }
  
  // Update timing record for retry count
  for (int i = 0; i < MAX_TIMING_RECORDS; i++) {
    if (timingRecords[i].messageId == queueItem->message.messageId) {
      timingRecords[i].retryCount = queueItem->retryCount;
      break;
    }
  }
  
  // Switch to idle mode before transmitting
  LoRa.idle(); 
  delay(20); // Short stabilization delay
  
  LoRa.beginPacket();
  LoRa.write(2);  // Destination: Drone 1
  LoRa.write(1);  // Source: Ground 1
  LoRa.write(0x00);  // Msg Type: Data
  
  // Write message ID
  LoRa.write((queueItem->message.messageId >> 24) & 0xFF);
  LoRa.write((queueItem->message.messageId >> 16) & 0xFF);
  LoRa.write((queueItem->message.messageId >> 8) & 0xFF);
  LoRa.write(queueItem->message.messageId & 0xFF);
  
  // Write message content
  LoRa.write((uint8_t*)&queueItem->message, sizeof(EmergencyMessage));
  
  bool sent = LoRa.endPacket();
  
  // Immediately put the radio back into receive mode
  LoRa.receive();
  
  // Log transmission
  if (sent) {
    Serial.print("SEND [");
    Serial.print(startTime);
    Serial.print("ms] To: 2, MsgID: ");
    Serial.print(queueItem->message.messageId);
    Serial.print(", Attempt: ");
    Serial.println(queueItem->retryCount);
    Serial.println("Packet transmission successful, switched back to receive mode");
  } else {
    Serial.print("SEND FAILED [");
    Serial.print(startTime);
    Serial.print("ms] To: 2, MsgID: ");
    Serial.print(queueItem->message.messageId);
    Serial.print(", Attempt: ");
    Serial.println(queueItem->retryCount);
  }
}

// Display status on OLED
void displayStatus(String message) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ground Station 1");
  display.println("----------------");
  display.println(message);
  display.display();
  
  Serial.println(message);
}

// Display network information
void displayNetworkInfo() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ground Station 1");
  display.println("----------------");
  display.println("WiFi: " + String(ssid));
  display.println("IP: " + String(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString()));
  display.println("Status: Ready");
  display.display();
}

// Web server root handler
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Emergency SOS</title>";
  html += "<style>";
  html += "body { font-family: Arial; margin: 0; padding: 20px; background: #f8f9fa; }";
  html += ".container { max-width: 500px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
  html += "h1 { color: #dc3545; }";
  html += "label { display: block; margin: 10px 0 5px; }";
  html += "input, textarea, select { width: 100%; padding: 10px; margin-bottom: 15px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }";
  html += "button { background: #dc3545; color: white; border: none; padding: 12px 20px; border-radius: 4px; cursor: pointer; font-size: 16px; width: 100%; }";
  html += "button:hover { background: #c82333; }";
  html += ".links { margin-top: 20px; text-align: center; }";
  html += ".links a { color: #007bff; text-decoration: none; margin: 0 10px; }";
  html += ".network-status { display: flex; align-items: center; justify-content: space-between; margin-bottom: 20px; padding: 10px; background: #e9ecef; border-radius: 5px; }";
  html += ".status-indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 6px; }";
  html += ".online { background-color: #28a745; }";
  html += ".offline { background-color: #dc3545; }";
  html += "</style>";
  html += "<script>";
  html += "function updateDeviceStatus() {";
  html += "  fetch('/network-data')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      for (let i = 0; i < data.length; i++) {";
  html += "        const indicator = document.getElementById('status-indicator-' + (i+1));";
  html += "        const statusText = document.getElementById('status-text-' + (i+1));";
  html += "        if (data[i].isActive) {";
  html += "          indicator.className = 'status-indicator online';";
  html += "          statusText.textContent = 'Online';";
  html += "        } else {";
  html += "          indicator.className = 'status-indicator offline';";
  html += "          statusText.textContent = 'Offline';";
  html += "        }";
  html += "      }";
  html += "    });";
  html += "}";
  html += "document.addEventListener('DOMContentLoaded', function() {";
  html += "  updateDeviceStatus();";
  html += "  setInterval(updateDeviceStatus, 5000);";
  html += "  if (navigator.geolocation) {";
  html += "    navigator.geolocation.getCurrentPosition(function(position) {";
  html += "      document.getElementById('latitude').value = position.coords.latitude;";
  html += "      document.getElementById('longitude').value = position.coords.longitude;";
  html += "    });";
  html += "  }";
  html += "});";
  html += "</script>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>Emergency SOS Message</h1>";
  
  html += "<div class='network-status'>";
  html += "<div><span id='status-indicator-1' class='status-indicator online'></span> Ground 1: <span id='status-text-1'>Online</span></div>";
  html += "<div><span id='status-indicator-2' class='status-indicator offline'></span> Drone 1: <span id='status-text-2'>Offline</span></div>";
  html += "<div><span id='status-indicator-3' class='status-indicator offline'></span> Drone 2: <span id='status-text-3'>Offline</span></div>";
  html += "<div><span id='status-indicator-4' class='status-indicator offline'></span> Ground 2: <span id='status-text-4'>Offline</span></div>";
  html += "</div>";
  
  html += "<form action='/send' method='post'>";
  html += "<label for='name'>Your Name:</label>";
  html += "<input type='text' id='name' name='name' required>";
  html += "<label for='location'>Location Description:</label>";
  html += "<input type='text' id='location' name='location' required>";
  html += "<label for='latitude'>Latitude:</label>";
  html += "<input type='number' id='latitude' name='latitude' step='0.000001' required>";
  html += "<label for='longitude'>Longitude:</label>";
  html += "<input type='number' id='longitude' name='longitude' step='0.000001' required>";
  html += "<label for='emergency'>Emergency Type:</label>";
  html += "<select id='emergency' name='emergency' required>";
  html += "<option value='1'>Medical Emergency</option>";
  html += "<option value='2'>Fire Emergency</option>";
  html += "<option value='3'>Security Emergency</option>";
  html += "<option value='4'>Structural Damage</option>";
  html += "<option value='5'>Other</option>";
  html += "</select>";
  html += "<label for='message'>Emergency Message:</label>";
  html += "<textarea id='message' name='message' rows='4' required></textarea>";
  html += "<button type='submit'>SEND SOS</button>";
  html += "</form>";
  html += "<div class='links'><a href='/status'>View Message Status</a> | <a href='/network'>Network Status</a></div>";
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

// Status page handler - shows timing statistics
void handleStatusPage() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Emergency Network Status</title>";
  html += "<style>";
  html += "body { font-family: Arial; margin: 0; padding: 20px; background: #f8f9fa; }";
  html += ".container { max-width: 1000px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
  html += "h1, h2 { color: #343a40; }";
  html += "table { width: 100%; border-collapse: collapse; margin: 20px 0; }";
  html += "th, td { padding: 12px 15px; text-align: left; border-bottom: 1px solid #ddd; }";
  html += "th { background-color: #f8f9fa; color: #495057; }";
  html += "tr:hover { background-color: #f1f1f1; }";
  html += ".success { color: #28a745; }";
  html += ".failure { color: #dc3545; }";
  html += ".pending { color: #ffc107; }";
  html += ".links { margin-top: 20px; text-align: center; }";
  html += ".links a { color: #007bff; text-decoration: none; margin: 0 10px; }";
  html += ".refresh { text-align: right; margin-bottom: 20px; }";
  html += ".signal-details { background: #f8f9fa; padding: 10px; border-radius: 5px; margin-top: 8px; display: none; }";
  html += ".show-details { cursor: pointer; color: #007bff; text-decoration: underline; }";
  html += ".signal-bar { height: 10px; background: #e9ecef; border-radius: 5px; overflow: hidden; margin-bottom: 5px; }";
  html += ".signal-fill { height: 100%; transition: width 0.5s; }";
  html += ".weak { background: #dc3545; }";
  html += ".medium { background: #ffc107; }";
  html += ".strong { background: #28a745; }";
  html += ".signal-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; }";
  html += ".signal-node { padding: 8px; border-radius: 5px; border: 1px solid #ddd; }";
  html += ".signal-node h4 { margin-top: 0; margin-bottom: 8px; }";
  html += ".queue-panel { background: #e8f4f8; padding: 15px; border-radius: 8px; margin-bottom: 20px; }";
  html += ".message-content { max-width: 200px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }";
  html += ".queue-items { display: grid; grid-template-columns: repeat(auto-fill, minmax(250px, 1fr)); gap: 10px; margin-top: 10px; }";
  html += ".queue-item { background: #fff; border: 1px solid #ddd; border-radius: 5px; padding: 10px; }";
  html += ".queue-item.completed { border-left: 4px solid #28a745; }";
  html += ".queue-item.in-progress { border-left: 4px solid #ffc107; }";
  html += ".queue-item.pending { border-left: 4px solid #17a2b8; }";
  html += ".badge { display: inline-block; padding: 3px 7px; border-radius: 4px; font-size: 12px; font-weight: bold; }";
  html += ".badge-success { background: #d4edda; color: #155724; }";
  html += ".badge-warning { background: #fff3cd; color: #856404; }";
  html += ".badge-info { background: #d1ecf1; color: #0c5460; }";
  html += ".badge-danger { background: #f8d7da; color: #721c24; }";
  html += ".no-items { text-align: center; color: #6c757d; padding: 20px; }";
  html += ".in-queue { background-color: #e0f7fa; }"; // Light blue for rows active in queue
  html += "</style>";
  html += "<script>";
  html += "function loadData() {";
  html += "  fetch('/data')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      const queueDiv = document.getElementById('queue-container');";
  html += "      queueDiv.innerHTML = '';";
  html += "      const queueTitle = document.createElement('h3');";
  html += "      queueTitle.textContent = 'Message Queue: ' + data.queueCount + ' / ' + data.maxQueueSize;";
  html += "      queueDiv.appendChild(queueTitle);";
  html += "      if (data.queue && data.queue.length > 0) {";
  html += "        const queueGrid = document.createElement('div');";
  html += "        queueGrid.className = 'queue-items';";
  html += "        data.queue.forEach(item => {";
  html += "          const queueItemDiv = document.createElement('div');";
  html += "          let statusClass = 'pending';";
  html += "          let statusBadge = '';";
  html += "          if (item.finalAckReceived) {";
  html += "            statusClass = 'completed';";
  html += "            statusBadge = '<span class=\"badge badge-success\">Completed</span>';";
  html += "          } else if (item.ackReceived) {";
  html += "            statusClass = 'in-progress';";
  html += "            statusBadge = '<span class=\"badge badge-warning\">ACK Received</span>';";
  html += "          } else {";
  html += "            statusBadge = '<span class=\"badge badge-info\">Attempt ' + item.retryCount + '</span>';";
  html += "          }";
  html += "          queueItemDiv.className = 'queue-item ' + statusClass;";
  html += "          queueItemDiv.innerHTML = `<div><strong>ID:</strong> ${item.messageId}</div><div><strong>Status:</strong> ${statusBadge}</div>`;";
  html += "          if (item.message && item.message.text) {";
  html += "            queueItemDiv.innerHTML += `<div class='message-content'><strong>Message:</strong> ${item.message.text}</div>`;";
  html += "          }";
  html += "          queueGrid.appendChild(queueItemDiv);";
  html += "        });";
  html += "        queueDiv.appendChild(queueGrid);";
  html += "      } else {";
  html += "        const noItems = document.createElement('div');";
  html += "        noItems.className = 'no-items';";
  html += "        noItems.textContent = 'No active messages in queue';";
  html += "        queueDiv.appendChild(noItems);";
  html += "      }";
  html += "      const tableBody = document.getElementById('dataTable').getElementsByTagName('tbody')[0];";
  html += "      tableBody.innerHTML = '';";
  html += "      if (data.messages) { data.messages.forEach(record => {"; // Check if messages array exists
  html += "        if (record.messageId === 0) return;";
  html += "        const row = tableBody.insertRow();";
  html += "        if (record.isInQueue) { row.className = 'in-queue'; }"; // Highlight if active in queue
  html += "        row.insertCell(0).textContent = record.messageId;";
  html += "        row.insertCell(1).textContent = record.receiveTimeFromApp;";
  html += "        row.insertCell(2).textContent = record.firstTransmitTime;";
  html += "        row.insertCell(3).textContent = record.ackReceiveTime || 'Waiting...';";
  html += "        row.insertCell(4).textContent = record.finalAckReceiveTime || 'Waiting...';";
  html += "        row.insertCell(5).textContent = record.retryCount;";
  html += "        const signalCell = row.insertCell(6);";
  html += "        if (record.deliveryComplete) {";
  html += "          let signalHtml = `${record.rssi} dBm / ${record.snr.toFixed(1)} dB`;";
  html += "          signalHtml += '<br><span class=\"show-details\" onclick=\"toggleDetails(' + record.messageId + ')\">Show signal chain</span>';";
  html += "          signalHtml += `<div id='details-${record.messageId}' class='signal-details'>`;";
  html += "          signalHtml += `<div class='signal-grid'>`;";
  html += "          signalHtml += `<div class='signal-node'><h4>Ground 1</h4>${createSignalBar(record.rssi)}RSSI: ${record.rssi} dBm<br>SNR: ${record.snr.toFixed(1)} dB</div>`;";
  html += "          signalHtml += `<div class='signal-node'><h4>Drone 1</h4>${createSignalBar(record.drone1Rssi)}RSSI: ${record.drone1Rssi} dBm<br>SNR: ${record.drone1Snr.toFixed(1)} dB</div>`;";
  html += "          signalHtml += `<div class='signal-node'><h4>Drone 2</h4>${createSignalBar(record.drone2Rssi)}RSSI: ${record.drone2Rssi} dBm<br>SNR: ${record.drone2Snr.toFixed(1)} dB</div>`;";
  html += "          signalHtml += `<div class='signal-node'><h4>Ground 2</h4>${createSignalBar(record.ground2Rssi)}RSSI: ${record.ground2Rssi} dBm<br>SNR: ${record.ground2Snr.toFixed(1)} dB</div>`;";
  html += "          signalHtml += `</div></div>`;";
  html += "          signalCell.innerHTML = signalHtml;";
  html += "        } else {";
  html += "          signalCell.textContent = record.rssi ? record.rssi + ' dBm / ' + record.snr.toFixed(1) + ' dB' : '-';";
  html += "        }";
  html += "        const statusCell = row.insertCell(7);";
  html += "        const timeCell = row.insertCell(8);";
  html += "        if (record.deliveryComplete) {";
  html += "          statusCell.textContent = 'Complete';";
  html += "          statusCell.className = 'success';";
  html += "          const totalTime = record.finalAckReceiveTime - record.receiveTimeFromApp;";
  html += "          timeCell.textContent = totalTime + ' ms';";
  html += "        } else if (record.retryCount > " + String(maxRetries) + ") {"; // Specifically check for retryCount > maxRetries for Final ACK Timeout
  html += "          statusCell.textContent = 'Failed (Final ACK Timeout)';";
  html += "          statusCell.className = 'failure';";
  html += "          timeCell.textContent = 'Timeout after D1 ACK';";
  html += "        } else if (record.retryCount >= " + String(maxRetries) + ") {"; // Check for retryCount == maxRetries for G1 ACK Failure
  html += "          statusCell.textContent = 'Failed (G1 ACK)';";
  html += "          statusCell.className = 'failure';";
  html += "          timeCell.textContent = 'Failed after ' + record.retryCount + ' G1 retries';";
  html += "        } else if (record.ackReceiveTime) {";
  html += "          statusCell.textContent = 'In Progress';";
  html += "          statusCell.className = 'pending';";
  html += "          timeCell.textContent = '-';";
  html += "        } else {";
  html += "          statusCell.textContent = 'Pending';";
  html += "          statusCell.className = 'pending';";
  html += "          timeCell.textContent = '-';";
  html += "        }";
  html += "      });}"; // End of data.messages.forEach and if
  html += "      document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();";
  html += "    }).catch(error => console.error('Error fetching or processing data:', error));"; // Add error catching
  html += "}";
  html += "function toggleDetails(msgId) {";
  html += "  const detailsDiv = document.getElementById('details-' + msgId);";
  html += "  if (detailsDiv) { if (detailsDiv.style.display === 'block') { detailsDiv.style.display = 'none'; } else { detailsDiv.style.display = 'block'; } }";
  html += "}";
  html += "function createSignalBar(rssi) {";
  html += "  const signalPercent = Math.min(100, Math.max(0, (rssi + 120) / 90 * 100));";
  html += "  let signalClass = 'weak';";
  html += "  if (signalPercent >= 66) signalClass = 'strong';";
  html += "  else if (signalPercent >= 33) signalClass = 'medium';";
  html += "  return `<div class='signal-bar'><div class='signal-fill ${signalClass}' style='width:${signalPercent}%'></div></div>`;";
  html += "}";
  html += "window.onload = function() {";
  html += "  loadData();";
  html += "  setInterval(loadData, 2000);"; // Corrected this line
  html += "};";
  html += "</script>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>Emergency Network - Message Timing</h1>";
  html += "<div class='refresh'>Last updated: <span id='lastUpdate'></span></div>";
  html += "<div class='queue-panel'>";
  html += "<h2>Message Queue</h2>";
  html += "<div id='queue-container'></div>";
  html += "</div>";
  html += "<h2>Message History</h2>";
  html += "<table id='dataTable'>";
  html += "<thead><tr>";
  html += "<th>Message ID</th><th>Received from App</th><th>First Transmit</th><th>ACK Received</th>";
  html += "<th>Final ACK</th><th>Retries</th><th>Signal Quality</th><th>Status</th><th>Total Time</th>";
  html += "</tr></thead><tbody></tbody></table>";
  html += "<div class='links'>";
  html += "<a href='/'>Send Emergency Message</a> | ";
  html += "<a href='/network'>View Network Status</a>";
  html += "</div></div></body></html>";
  server.send(200, "text/html", html);
}

// Network status page - shows status of all devices
void handleNetworkPage() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Emergency Network - Device Status</title>";
  html += "<style>";
  html += "body { font-family: Arial; margin: 0; padding: 20px; background: #f8f9fa; }";
  html += ".container { max-width: 1000px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
  html += "h1, h2 { color: #343a40; }";
  html += ".device-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 20px; margin: 20px 0; }";
  html += ".device { padding: 15px; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1); transition: all 0.3s; }";
  html += ".device:hover { transform: translateY(-5px); box-shadow: 0 5px 15px rgba(0,0,0,0.2); }";
  html += ".device h2 { margin-top: 0; color: #495057; }";
  html += ".device p { margin: 8px 0; }";
  html += ".device.online { background: #d4edda; border-left: 5px solid #28a745; }";
  html += ".device.offline { background: #f8d7da; border-left: 5px solid #dc3545; }";
  html += ".signal-strength { margin-top: 10px; }";
  html += ".signal-bar { height: 10px; background: #e9ecef; border-radius: 5px; overflow: hidden; }";
  html += ".signal-fill { height: 100%; background: #28a745; transition: width 0.5s; }";
  html += ".weak { background: #dc3545; }";
  html += ".medium { background: #ffc107; }";
  html += ".strong { background: #28a745; }";
  html += ".refresh { text-align: right; margin-bottom: 20px; }";
  html += ".links { margin-top: 20px; text-align: center; }";
  html += ".links a { color: #007bff; text-decoration: none; margin: 0 10px; }";
  html += ".stats-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px; margin-top: 30px; }";
  html += ".stats-card { background: #f8f9fa; padding: 15px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }";
  html += ".signal-history { height: 150px; background: #fff; border: 1px solid #ddd; margin-top: 10px; position: relative; }";
  html += ".history-line { position: absolute; width: 100%; height: 1px; background: #e9ecef; }";
  html += ".history-label { position: absolute; right: 0; font-size: 12px; color: #6c757d; }";
  html += ".history-point { position: absolute; width: 6px; height: 6px; border-radius: 50%; background: #007bff; transform: translate(-50%, -50%); }";
  html += ".latest-message { background: #e8f4f8; padding: 15px; border-radius: 8px; margin-top: 15px; }";
  html += ".latest-message h3 { margin-top: 0; color: #0056b3; }";
  html += ".message-details { margin-top: 10px; }";
  html += ".message-details p { margin: 5px 0; }";
  html += "</style>";
  html += "<script>";
  html += "let signalHistory = {";
  html += "  drone1: [],";
  html += "  drone2: [],";
  html += "  ground2: []";
  html += "};";
  html += "let latestMessage = null;";
  html += "function loadNetworkData() {";
  html += "  fetch('/network-data')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      data.forEach((device, index) => {";
  html += "        const deviceEl = document.getElementById('device-' + (index+1));";
  html += "        if (device.isActive) {";
  html += "          deviceEl.className = 'device online';";
  html += "          document.getElementById('status-' + (index+1)).textContent = 'Online';";
  html += "          document.getElementById('last-seen-' + (index+1)).textContent = 'Last seen: ' + new Date(device.lastSeen).toLocaleTimeString();";
  html += "          document.getElementById('rssi-' + (index+1)).textContent = 'Signal Strength: ' + device.rssi + ' dBm / SNR: ' + device.snr.toFixed(1) + ' dB';";
  html += "          // Calculate signal strength percentage (RSSI typically ranges from -120 to -30)";
  html += "          const signalPercent = Math.min(100, Math.max(0, (device.rssi + 120) / 90 * 100));";
  html += "          const signalFill = document.getElementById('signal-fill-' + (index+1));";
  html += "          signalFill.style.width = signalPercent + '%';";
  html += "          if (signalPercent < 33) {";
  html += "            signalFill.className = 'signal-fill weak';";
  html += "          } else if (signalPercent < 66) {";
  html += "            signalFill.className = 'signal-fill medium';";
  html += "          } else {";
  html += "            signalFill.className = 'signal-fill strong';";
  html += "          }";
  html += "          // Store signal history (except for Ground 1)";
  html += "          if (index > 0) {";
  html += "            const deviceType = ['', 'drone1', 'drone2', 'ground2'][index];";
  html += "            if (deviceType && device.rssi !== 0) {";
  html += "              signalHistory[deviceType].push({";
  html += "                time: Date.now(),";
  html += "                rssi: device.rssi,";
  html += "                snr: device.snr";
  html += "              });";
  html += "              // Keep only last 20 readings";
  html += "              if (signalHistory[deviceType].length > 20) {";
  html += "                signalHistory[deviceType].shift();";
  html += "              }";
  html += "            }";
  html += "          }";
  html += "        } else {";
  html += "          deviceEl.className = 'device offline';";
  html += "          document.getElementById('status-' + (index+1)).textContent = 'Offline';";
  html += "          if (device.lastSeen > 0) {";
  html += "            document.getElementById('last-seen-' + (index+1)).textContent = 'Last seen: ' + new Date(device.lastSeen).toLocaleTimeString();";
  html += "          } else {";
  html += "            document.getElementById('last-seen-' + (index+1)).textContent = 'Never connected';";
  html += "          }";
  html += "          document.getElementById('signal-fill-' + (index+1)).style.width = '0%';";
  html += "        }";
  html += "      });";
  html += "      document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();";
  html += "      updateSignalHistory();";
  html += "    });";
  html += "}";
  html += "function updateSignalHistory() {";
  html += "  const deviceTypes = ['drone1', 'drone2', 'ground2'];";
  html += "  deviceTypes.forEach(type => {";
  html += "    const canvas = document.getElementById(`history-${type}`);";
  html += "    const history = signalHistory[type];";
  html += "    canvas.innerHTML = '';";
  html += "    if (history.length < 2) return;";
  html += "    // Add background lines";
  html += "    for (let i = 0; i <= 4; i++) {";
  html += "      const y = i * 25 + 25;";
  html += "      const rssi = -30 - i * 20;";
  html += "      canvas.innerHTML += `<div class='history-line' style='top: ${y}px;'></div>`;";
  html += "      canvas.innerHTML += `<div class='history-label' style='top: ${y}px;'>${rssi} dBm</div>`;";
  html += "    }";
  html += "    // Plot points";
  html += "    const maxTime = Date.now();";
  html += "    const minTime = maxTime - 60000; // 1 minute history";
  html += "    history.forEach((point, i) => {";
  html += "      const x = ((point.time - minTime) / 60000) * canvas.offsetWidth;";
  html += "      // RSSI range from -30 to -120 dBm mapped to 0-100px";
  html += "      const y = ((point.rssi + 30) / -90) * 100 + 25;";
  html += "      canvas.innerHTML += `<div class='history-point' style='left: ${x}px; top: ${y}px;' title='RSSI: ${point.rssi} dBm, SNR: ${point.snr.toFixed(1)} dB'></div>`;";
  html += "    });";
  html += "  });";
  html += "}";
  html += "window.onload = function() {";
  html += "  loadNetworkData();";
  html += "  setInterval(loadNetworkData, 2000);"; // Auto refresh every 2 seconds
  html += "  window.addEventListener('resize', updateSignalHistory);";
  html += "}";
  html += "</script>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>Emergency Network - Device Status</h1>";
  html += "<div class='refresh'>Last updated: <span id='lastUpdate'></span></div>";
  html += "<div class='device-grid'>";
  
  // Ground 1
  html += "<div id='device-1' class='device online'>";
  html += "<h2>Ground Transceiver 1</h2>";
  html += "<p>Role: Access Point & Message Origin</p>";
  html += "<p id='status-1'>Online</p>";
  html += "<p id='last-seen-1'>Current device</p>";
  html += "<p id='rssi-1'>-</p>";
  html += "<div class='signal-strength'>";
  html += "<div class='signal-bar'><div id='signal-fill-1' class='signal-fill' style='width:100%'></div></div>";
  html += "</div></div>";
  
  // Drone 1
  html += "<div id='device-2' class='device offline'>";
  html += "<h2>Drone 1</h2>";
  html += "<p>Role: Campus Coverage</p>";
  html += "<p id='status-2'>Waiting...</p>";
  html += "<p id='last-seen-2'>Not yet connected</p>";
  html += "<p id='rssi-2'>-</p>";
  html += "<div class='signal-strength'>";
  html += "<div class='signal-bar'><div id='signal-fill-2' class='signal-fill'></div></div>";
  html += "</div></div>";
  
  // Drone 2
  html += "<div id='device-3' class='device offline'>";
  html += "<h2>Drone 2</h2>";
  html += "<p>Role: Relay</p>";
  html += "<p id='status-3'>Waiting...</p>";
  html += "<p id='last-seen-3'>Not yet connected</p>";
  html += "<p id='rssi-3'>-</p>";
  html += "<div class='signal-strength'>";
  html += "<div class='signal-bar'><div id='signal-fill-3' class='signal-fill'></div></div>";
  html += "</div></div>";
  
  // Ground 2
  html += "<div id='device-4' class='device offline'>";
  html += "<h2>Ground Transceiver 2</h2>";
  html += "<p>Role: Final Destination</p>";
  html += "<p id='status-4'>Waiting...</p>";
  html += "<p id='last-seen-4'>Not yet connected</p>";
  html += "<p id='rssi-4'>-</p>";
  html += "<div class='signal-strength'>";
  html += "<div class='signal-bar'><div id='signal-fill-4' class='signal-fill'></div></div>";
  html += "</div></div>";
  
  html += "</div>";
  
  // Signal History
  html += "<h2>Signal Strength History</h2>";
  html += "<div class='stats-grid'>";
  html += "<div class='stats-card'>";
  html += "<h3>Drone 1 Signal</h3>";
  html += "<div id='history-drone1' class='signal-history'></div>";
  html += "</div>";
  html += "<div class='stats-card'>";
  html += "<h3>Drone 2 Signal</h3>";
  html += "<div id='history-drone2' class='signal-history'></div>";
  html += "</div>";
  html += "<div class='stats-card'>";
  html += "<h3>Ground 2 Signal</h3>";
  html += "<div id='history-ground2' class='signal-history'></div>";
  html += "</div>";
  html += "</div>";
  
  // Latest Message Details
  html += "<div id='latest-message' class='latest-message' style='display:none;'>";
  html += "<h3>Latest Completed Message</h3>";
  html += "<div id='latest-msg-details' class='message-details'></div>";
  html += "</div>";
  
  html += "<div class='links'>";
  html += "<a href='/'>Send Emergency Message</a> | ";
  html += "<a href='/status'>View Message Status</a>";
  html += "</div></div></body></html>";
  
  server.send(200, "text/html", html);
}

// Network data handler for the network status page
void handleNetworkData() {
  StaticJsonDocument<512> doc; // Use smaller static document
  JsonArray array = doc.to<JsonArray>();
  
  for (int i = 0; i < 4; i++) {
    JsonObject obj = array.createNestedObject();
    obj["deviceId"] = i + 1;
    obj["lastSeen"] = deviceStatus[i].lastSeen;
    obj["rssi"] = deviceStatus[i].rssi;
    obj["snr"] = deviceStatus[i].snr;
    obj["isActive"] = deviceStatus[i].isActive;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// JSON data handler for the status page
void handleDataRequest() {
  // Start with a smaller JSON document
  DynamicJsonDocument doc(MAX_JSON_RESPONSE_SIZE);
  
  // Add queue information
  doc["queueCount"] = queueCount;
  doc["maxQueueSize"] = MAX_QUEUE_SIZE;
  
  // Add queue items - only include essential data
  JsonArray queueArray = doc.createNestedArray("queue");
  int activeCount = 0;
  for (int i = 0; i < MAX_QUEUE_SIZE && activeCount < 3; i++) { // Limit to 3 active queue items
    if (messageQueue[i].active) {
      JsonObject qItem = queueArray.createNestedObject();
      qItem["messageId"] = messageQueue[i].message.messageId;
      qItem["retryCount"] = messageQueue[i].retryCount;
      qItem["finalAckReceived"] = messageQueue[i].finalAckReceived;
      
      // Include minimal message info - just emergency code
      qItem["emergencyCode"] = messageQueue[i].message.emergencyCode;
      
      activeCount++;
    }
  }
  
  // Add message history - only include the 5 most recent messages
  JsonArray array = doc.createNestedArray("messages");
  
  // Find the 5 most recent messages
  uint32_t recentMsgIds[5] = {0};
  unsigned long recentMsgTimes[5] = {0};
  
  // First pass - find the 5 most recent messages
  for (int i = 0; i < MAX_TIMING_RECORDS; i++) {
    if (timingRecords[i].messageId != 0) {
      // Check if this message is more recent than any in our list
      for (int j = 0; j < 5; j++) {
        if (timingRecords[i].receiveTimeFromApp > recentMsgTimes[j]) {
          // Shift down all older messages
          for (int k = 4; k > j; k--) {
            recentMsgIds[k] = recentMsgIds[k-1];
            recentMsgTimes[k] = recentMsgTimes[k-1];
          }
          // Insert this message
          recentMsgIds[j] = timingRecords[i].messageId;
          recentMsgTimes[j] = timingRecords[i].receiveTimeFromApp;
          break;
        }
      }
    }
  }
  
  // Second pass - add only the recent messages
  for (int i = 0; i < MAX_TIMING_RECORDS; i++) {
    // Only include the message if it's in our recent list
    bool isRecent = false;
    for (int j = 0; j < 5; j++) {
      if (timingRecords[i].messageId == recentMsgIds[j]) {
        isRecent = true;
        break;
      }
    }
    
    if (timingRecords[i].messageId != 0 && isRecent) {
      JsonObject obj = array.createNestedObject();
      obj["messageId"] = timingRecords[i].messageId;
      obj["receiveTimeFromApp"] = timingRecords[i].receiveTimeFromApp;
      obj["firstTransmitTime"] = timingRecords[i].firstTransmitTime;
      obj["ackReceiveTime"] = timingRecords[i].ackReceiveTime;
      obj["finalAckReceiveTime"] = timingRecords[i].finalAckReceiveTime;
      obj["retryCount"] = timingRecords[i].retryCount;
      obj["deliveryComplete"] = timingRecords[i].deliveryComplete;
      obj["rssi"] = timingRecords[i].rssi;
      obj["snr"] = timingRecords[i].snr;
      
      // Check if message is in queue (active) or only in history
      bool isInQueue = false;
      for (int j = 0; j < MAX_QUEUE_SIZE; j++) {
        if (messageQueue[j].active && messageQueue[j].message.messageId == timingRecords[i].messageId) {
          isInQueue = true;
          break;
        }
      }
      obj["isInQueue"] = isInQueue;
    }
  }
  
  // Include detailed signal chain data if available and delivered
  for (int i = 0; i < MAX_TIMING_RECORDS; i++) {
    if (timingRecords[i].messageId != 0 && timingRecords[i].deliveryComplete) {
       bool found_in_recent = false;
       for(int j=0; j<5; ++j) {
          if (timingRecords[i].messageId == recentMsgIds[j]) {
             found_in_recent = true;
             break;
          }
       }
       if (found_in_recent) {
          // Find the corresponding JSON object to add signal details
          for (JsonVariant message : array) {
             if (message["messageId"] == timingRecords[i].messageId) {
                JsonObject obj = message.as<JsonObject>();
                obj["ground2Rssi"] = timingRecords[i].ground2Rssi;
                obj["ground2Snr"] = timingRecords[i].ground2Snr;
                obj["drone2Rssi"] = timingRecords[i].drone2Rssi;
                obj["drone2Snr"] = timingRecords[i].drone2Snr;
                obj["drone1Rssi"] = timingRecords[i].drone1Rssi;
                obj["drone1Snr"] = timingRecords[i].drone1Snr;
                break;
             }
          }
       }
    }
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// Handle emergency message submission from web interface
void handleSendMessage() {
  if (server.hasArg("message") && server.hasArg("latitude") && server.hasArg("longitude") && server.hasArg("emergency")) {
    // Record receive time immediately
    unsigned long receiveTime = millis();
    
    String message = server.arg("name") + ": " + server.arg("message") + " (at " + server.arg("location") + ")";
    
    // Create new message 
    EmergencyMessage newMessage;
    strncpy(newMessage.text, message.c_str(), sizeof(newMessage.text) - 1);
    newMessage.text[sizeof(newMessage.text) - 1] = '\0';  // Ensure null termination
    
    newMessage.latitude = server.arg("latitude").toFloat();
    newMessage.longitude = server.arg("longitude").toFloat();
    newMessage.emergencyCode = server.arg("emergency").toInt();
    newMessage.messageId = random(1, 1000000);  // Generate random message ID
    
    // Log the new message
    Serial.println("==============================================");
    Serial.print("NEW MESSAGE RECEIVED [");
    Serial.print(receiveTime);
    Serial.println("ms]");
    Serial.println("Content: " + String(newMessage.text));
    Serial.println("Location: " + String(newMessage.latitude, 6) + ", Lon: " + String(newMessage.longitude, 6));
    Serial.println("Emergency Type: " + String(newMessage.emergencyCode));
    Serial.println("Message ID: " + String(newMessage.messageId));
    Serial.println("==============================================");
    
    // Add to message queue
    if (addToMessageQueue(&newMessage, receiveTime)) {
      String response = "Message received. ID:" + String(newMessage.messageId);
      server.send(200, "text/plain", response);
    } else {
      server.send(503, "text/plain", "Message queue is full. Please try again later.");
    }
  } else {
    server.send(400, "text/plain", "Missing required parameters");
  }
}

// Function to process commands received over Serial from Python
void processSerialCommand(String command) {
  Serial.print("Serial command received: ");
  Serial.println(command);

  DynamicJsonDocument doc(512); // Match buffer reservation
  DeserializationError error = deserializeJson(doc, command);

  if (error) {
    Serial.print(F("deserializeJson() for serial command failed: "));
    Serial.println(error.f_str());
    // Send error back to Python?
    DynamicJsonDocument errDoc(128);
    errDoc["type"] = "ack_status";
    errDoc["messageId"] = 0; // Or try to extract if possible
    errDoc["status"] = "error_parsing_command";
    errDoc["error_details"] = error.f_str();
    String errJson;
    serializeJson(errDoc, errJson);
    Serial.println(errJson);
    return;
  }

  // Check for required fields from Python message
  if (doc.containsKey("text") && doc.containsKey("emergencyCode") && doc.containsKey("messageId")) {
    unsigned long receiveTime = millis(); // Record receive time from Python

    // Create new message
    EmergencyMessage newMessage;
    strncpy(newMessage.text, doc["text"], sizeof(newMessage.text) - 1);
    newMessage.text[sizeof(newMessage.text) - 1] = '\0';
    newMessage.latitude = doc["latitude"] | 0.0; // Default to 0.0 if not present
    newMessage.longitude = doc["longitude"] | 0.0;
    newMessage.emergencyCode = doc["emergencyCode"];
    newMessage.messageId = doc["messageId"]; // IMPORTANT: Use messageId from Python

    Serial.println("==============================================");
    Serial.print("NEW MESSAGE from PYTHON [");
    Serial.print(receiveTime);
    Serial.println("ms]");
    Serial.println("Text: " + String(newMessage.text));
    Serial.println("Lat: " + String(newMessage.latitude, 6) + ", Lon: " + String(newMessage.longitude, 6));
    Serial.println("Code: " + String(newMessage.emergencyCode));
    Serial.println("MsgID: " + String(newMessage.messageId));
    Serial.println("==============================================");
    
    // Add to message queue
    if (addToMessageQueue(&newMessage, receiveTime)) {
      // Send an acknowledgment back to Python
      DynamicJsonDocument ackDoc(128);
      ackDoc["type"] = "message_queued";
      ackDoc["messageId"] = newMessage.messageId;
      ackDoc["status"] = "queued";
      String ackJson;
      serializeJson(ackDoc, ackJson);
      Serial.println(ackJson);
    } else {
      // Send error back to Python
      DynamicJsonDocument errDoc(128);
      errDoc["type"] = "ack_status";
      errDoc["messageId"] = newMessage.messageId;
      errDoc["status"] = "error_queue_full";
      String errJson;
      serializeJson(errDoc, errJson);
      Serial.println(errJson);
    }
  } else {
    Serial.println("Serial JSON command missing required fields (text, emergencyCode, messageId)");
    DynamicJsonDocument errDoc(128);
    errDoc["type"] = "ack_status";
    errDoc["messageId"] = doc.containsKey("messageId") ? doc["messageId"].as<uint32_t>() : 0;
    errDoc["status"] = "error_missing_fields";
    String errJson;
    serializeJson(errDoc, errJson);
    Serial.println(errJson);
  }
}

// Add a message to the queue
bool addToMessageQueue(EmergencyMessage* message, unsigned long receiveTime) {
  // Check if we already have this message in the queue
  for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
    if (messageQueue[i].active && messageQueue[i].message.messageId == message->messageId) {
      Serial.println("Message ID " + String(message->messageId) + " already in queue, not adding again");
      return false;
    }
  }
  
  // Find an empty slot in the queue
  int emptySlot = -1;
  for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
    if (!messageQueue[i].active) {
      emptySlot = i;
      break;
    }
  }
  
  if (emptySlot == -1) {
    Serial.println("ERROR: Message queue full, cannot add new message");
    return false;
  }
  
  // Add message to queue
  messageQueue[emptySlot].message = *message;
  messageQueue[emptySlot].receiveTime = receiveTime;
  messageQueue[emptySlot].lastTransmitTime = 0; // Will be set when first transmitted
  messageQueue[emptySlot].retryCount = 0;
  messageQueue[emptySlot].active = true;
  messageQueue[emptySlot].finalAckReceived = false;
  
  queueCount++;
  
  // Add to timing records - FULLY RESET the record being reused
  currentTimingIndex = (currentTimingIndex + 1) % MAX_TIMING_RECORDS;
  MessageTiming* tr = &timingRecords[currentTimingIndex];
  
  Serial.println("Reusing timingRecord index: " + String(currentTimingIndex) + " for new msgID: " + String(message->messageId));

  tr->messageId = message->messageId;
  tr->receiveTimeFromApp = receiveTime;
  tr->firstTransmitTime = 0;
  tr->ackReceiveTime = 0;
  tr->finalAckReceiveTime = 0;
  tr->retryCount = 0;
  tr->deliveryComplete = false;
  tr->rssi = 0;
  tr->snr = 0;
  tr->drone1Rssi = 0;
  tr->drone1Snr = 0.0f;
  tr->drone2Rssi = 0;
  tr->drone2Snr = 0.0f;
  tr->ground2Rssi = 0;
  tr->ground2Snr = 0.0f;
  
  Serial.println("Added message ID " + String(message->messageId) + " to queue position " + String(emptySlot));
  updateQueueDisplay();
  
  return true;
}

// Find a queue item by message ID
int findQueueItemById(uint32_t messageId) {
  for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
    if (messageQueue[i].active && messageQueue[i].message.messageId == messageId) {
      return i;
    }
  }
  return -1; // Not found
}

// Remove a message from the queue
void removeFromQueue(int index) {
  if (index < 0 || index >= MAX_QUEUE_SIZE || !messageQueue[index].active) {
    return;
  }
  
  messageQueue[index].active = false;
  queueCount--;
  
  // Display updated queue status
  updateQueueDisplay();
}

// Update the OLED display with queue information
void updateQueueDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ground Station 1");
  display.println("----------------");
  
  if (queueCount == 0) {
    display.println("Queue Empty");
    display.println("Ready for Messages");
  } else {
    display.println("Queue: " + String(queueCount) + " messages");
    
    // Show the first few messages in the queue
    int shown = 0;
    for (int i = 0; i < MAX_QUEUE_SIZE && shown < 3; i++) {
      if (messageQueue[i].active) {
        String status;
        if (messageQueue[i].finalAckReceived) {
          status = "Done";
        } else if (messageQueue[i].retryCount > 0) { // Check retryCount instead of ackReceived
          status = "R:" + String(messageQueue[i].retryCount);
        } else {
          status = "Wait"; // Initial state before any retries
        }
        
        display.print("ID:");
        display.print(messageQueue[i].message.messageId);
        display.print(" ");
        display.println(status);
        
        shown++;
      }
    }
  }
  
  display.display();
}

// New endpoint for iOS app to check message status
void handleMessageStatusRequest() {
  if (!server.hasArg("id")) {
    server.send(400, "application/json", "{\"error\":\"Missing message ID\"}");
    return;
  }
  
  uint32_t messageId = server.arg("id").toInt();
  bool found = false;
  bool delivered = false;
  String status = "unknown";
  
  // Check if message is in the active queue
  int queueIndex = findQueueItemById(messageId);
  if (queueIndex >= 0) {
    found = true;
    if (messageQueue[queueIndex].finalAckReceived) {
      status = "delivered";
      delivered = true;
    } else if (messageQueue[queueIndex].retryCount > 0) { // Check retryCount instead of ackReceived
      status = "in_progress";
    } else {
      status = "pending";
    }
  } else {
    // If not in the active queue, check the timing records for completed/failed messages
    for (int i = 0; i < MAX_TIMING_RECORDS; i++) {
      if (timingRecords[i].messageId == messageId) {
        found = true;
        delivered = timingRecords[i].deliveryComplete;
        
        if (delivered) {
          status = "delivered";
        } else if (timingRecords[i].ackReceiveTime > 0) {
          status = "in_progress";
        } else if (timingRecords[i].retryCount >= maxRetries) {
          status = "failed";
        } else {
          status = "pending";
        }
        
        break;
      }
    }
  }
  
  // Create JSON response
  DynamicJsonDocument doc(256);
  doc["messageId"] = messageId;
  doc["found"] = found;
  doc["delivered"] = delivered;
  doc["status"] = status;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// Send ACK status to Python via Serial
void sendAckStatusToPython(uint32_t messageId, const char* status, int lastHopRssi, float lastHopSnr, MessageTiming* timingRecord) {
  DynamicJsonDocument jsonDoc(512); // Increased size for more data
  jsonDoc["type"] = "ack_status";
  jsonDoc["messageId"] = messageId;
  jsonDoc["status"] = status;
  
  // Basic signal info (e.g., from the ACK packet received by Ground1)
  if (strcmp(status, "delivered") == 0 || strcmp(status, "ack_from_d1") == 0) { // "ack_from_d1" is hypothetical
    jsonDoc["rssi_to_g1"] = lastHopRssi; // RSSI of the packet G1 received (e.g. final ACK)
    jsonDoc["snr_to_g1"] = lastHopSnr;   // SNR of the packet G1 received
  }

  if (timingRecord != NULL && strcmp(status, "delivered") == 0) {
    // Include full signal chain if available and delivered
    JsonObject signalChain = jsonDoc.createNestedObject("signalChain");
    signalChain["g1_initial_ack_rssi"] = timingRecord->rssi; // D1's ACK to G1
    signalChain["g1_initial_ack_snr"] = timingRecord->snr;
    signalChain["d1_final_ack_rssi"] = timingRecord->drone1Rssi; // D1 receiving final ACK
    signalChain["d1_final_ack_snr"] = timingRecord->drone1Snr;
    signalChain["d2_final_ack_rssi"] = timingRecord->drone2Rssi; // D2 receiving final ACK
    signalChain["d2_final_ack_snr"] = timingRecord->drone2Snr;
    signalChain["g2_final_ack_rssi"] = timingRecord->ground2Rssi; // G2 receiving data
    signalChain["g2_final_ack_snr"] = timingRecord->ground2Snr;
    jsonDoc["totalTimeMs"] = timingRecord->finalAckReceiveTime - timingRecord->receiveTimeFromApp;
  }

  String outputJson;
  serializeJson(jsonDoc, outputJson);
  Serial.println(outputJson); // Send JSON string over serial
  Serial.flush(); // Ensure it's sent immediately
}

// Completely replacing the processMessageQueueNonBlocking function
void processMessageQueueNonBlocking() {
  static int lastProcessedIndex = 0;
  unsigned long startTime = millis();
  bool timeExceeded = false;
  
  // Process one item per call, cycling through the queue
  for (int attempt = 0; attempt < MAX_QUEUE_SIZE && !timeExceeded; attempt++) {
    int i = (lastProcessedIndex + attempt) % MAX_QUEUE_SIZE;
    
    if (!messageQueue[i].active) continue;
    
    MessageQueueItem* qItem = &messageQueue[i];
    
    // 1. Check if Final ACK has been received for this message
    if (qItem->finalAckReceived) {
      Serial.println("Message ID " + String(qItem->message.messageId) + " already has final ACK. Removing from active queue.");
      removeFromQueue(i);
      lastProcessedIndex = i; // Update to process next one in next call
      updateQueueDisplay(); // Ensure display reflects removal
      currentState = MESSAGE_DELIVERED; // Set state to reflect completion
      break; // Processed one item
    }
    
    // 2. Check for Final ACK timeout
    // Find the corresponding timing record to get the first transmit time
    unsigned long firstTxTime = 0;
    MessageTiming* currentTimingRecord = NULL;
    for (int tr_idx = 0; tr_idx < MAX_TIMING_RECORDS; tr_idx++) {
      if (timingRecords[tr_idx].messageId == qItem->message.messageId) {
        firstTxTime = timingRecords[tr_idx].firstTransmitTime;
        currentTimingRecord = &timingRecords[tr_idx];
        break;
      }
    }

    if (firstTxTime > 0 && (millis() - firstTxTime > MAX_WAIT_FINAL_ACK_TIME)) {
      Serial.println("Message ID " + String(qItem->message.messageId) + " FAILED: Timed out waiting for final ACK.");
      Serial.println("  First transmit time: " + String(firstTxTime) + ", Current time: " + String(millis()));
      displayStatus("Msg " + String(qItem->message.messageId) + " Timeout");
      if (currentTimingRecord != NULL) {
        sendAckStatusToPython(qItem->message.messageId, "failed_timeout_final_ack", 0, 0, currentTimingRecord);
      } else {
        sendAckStatusToPython(qItem->message.messageId, "failed_timeout_final_ack_no_record", 0, 0, NULL);
      }
      removeFromQueue(i);
      lastProcessedIndex = i;
      updateQueueDisplay();
      currentState = IDLE; // Or some error state
      break; // Processed one item
    }
    
    // 3. Check if it's time to transmit/retransmit MSG_DATA
    if (!qItem->finalAckReceived && 
        (qItem->lastTransmitTime == 0 || (millis() - qItem->lastTransmitTime > transmitInterval))) {
      
      if (qItem->retryCount >= maxRetries) {
        Serial.println("Message ID " + String(qItem->message.messageId) + " FAILED: Max retries (" + String(maxRetries) + ") reached waiting for Final ACK.");
        displayStatus("Msg " + String(qItem->message.messageId) + " Failed");
        if (currentTimingRecord != NULL) {
          sendAckStatusToPython(qItem->message.messageId, "failed_max_retries", 0, 0, currentTimingRecord);
        } else {
          sendAckStatusToPython(qItem->message.messageId, "failed_max_retries_no_record", 0, 0, NULL);
        }
        removeFromQueue(i);
        lastProcessedIndex = i;
        updateQueueDisplay();
        currentState = IDLE; // Or some error state
        break; // Processed one item
      }
      
      // Transmit message
      sendLoRaMessage(qItem); // This function increments retryCount and sets lastTransmitTime
      
      // Update display
      if (qItem->retryCount == 1) {
        displayStatus("Sending msg " + String(qItem->message.messageId));
      } else {
        displayStatus("Retry " + String(qItem->retryCount) + " for " + String(qItem->message.messageId));
      }
      updateQueueDisplay();
      
      // NO LONGER LISTEN FOR INTERMEDIATE ACK HERE
      // listenForAck(qItem); -- Removed
      
      lastProcessedIndex = i; // Update to process next one in next call
      currentState = PROCESSING_QUEUE; // Still processing
      break; // Processed one item, will check again in next loop or next call to this function
    }
    
    // If we iterated through the whole queue and didn't break, it means no active items needed processing right now
    if (attempt == MAX_QUEUE_SIZE -1) {
        // This case might mean all active items are waiting for transmitInterval or final ACK.
        // No action needed here, just let it cycle.
    }

    // Check if processing time is getting too long for this single call
    if (millis() - startTime > MAX_PROCESSING_TIME_PER_LOOP) {
      timeExceeded = true;
      Serial.println("Queue processing time limit reached for this cycle");
      // lastProcessedIndex is already updated if an item was processed.
      // If no item was processed but time limit hit (e.g. due to many inactive slots),
      // lastProcessedIndex will ensure we start from a different spot next time.
    }
  }
  
  // If queue becomes empty after processing, update state
  if (queueCount == 0 && currentState == PROCESSING_QUEUE) {
      currentState = IDLE;
      updateQueueDisplay(); // Show "Queue Empty"
  }
}

// Removing the listenForAck function completely
// void listenForAck(MessageQueueItem* qItem) {
//   unsigned long listenStartTime = millis();
//   int listenTimeMs = 1000; // Reduced from 1500ms to 1000ms for faster processing
//   
//   Serial.println("Listening for ACK for " + String(listenTimeMs) + "ms...");
//   
//   bool ackReceived = false;
//   while (millis() - listenStartTime < listenTimeMs && !ackReceived) {
//     // Handle web server during listening to prevent timeouts
//     server.handleClient();
//     
//     // Check for LoRa packets
//     int ackPacketSize = LoRa.parsePacket();
//     if (ackPacketSize) {
//       Serial.println("Packet detected during ACK wait period!");
//       
//       // Read packet header
//       byte destination = LoRa.read();
//       byte source = LoRa.read();
//       byte msgType = LoRa.read();
//       
//       uint32_t ackMsgId = 0;
//       ackMsgId |= ((uint32_t)LoRa.read()) << 24;
//       ackMsgId |= ((uint32_t)LoRa.read()) << 16;
//       ackMsgId |= ((uint32_t)LoRa.read()) << 8;
//       ackMsgId |= LoRa.read();
//       
//       // If this is an ACK for our message
//       if (destination == 1 && source == 2 && msgType == 0x01 && ackMsgId == qItem->message.messageId) {
//         Serial.println("ACK RECEIVED during wait period!");
//         ackReceived = true;
//         
//         // Get signal quality data
//         int rssi = LoRa.packetRssi();
//         float snr = LoRa.packetSnr();
//         
//         // Update device status
//         deviceStatus[source-1].lastSeen = millis();
//         deviceStatus[source-1].rssi = rssi;
//         deviceStatus[source-1].snr = snr;
//         deviceStatus[source-1].isActive = true;
//         
//         // Mark as ACK received
//         qItem->ackReceived = true;
//         
//         displayStatus("ACK from D1 for msg " + String(qItem->message.messageId));
//         Serial.println("ACK received from Drone 1 for message " + String(qItem->message.messageId));
//         
//         // Update timing record
//         for (int j = 0; j < MAX_TIMING_RECORDS; j++) {
//           if (timingRecords[j].messageId == qItem->message.messageId) {
//             timingRecords[j].ackReceiveTime = millis();
//             timingRecords[j].rssi = rssi;
//             timingRecords[j].snr = snr;
//             break;
//           }
//         }
//         
//         // Update display with queue status
//         updateQueueDisplay();
//       }
//     }
//     
//     // Short delay but keep web server responsive
//     yield();
//     delay(5); // Reduced from 10ms to 5ms to check more frequently
//   }
//   
//   if (ackReceived) {
//     Serial.println("ACK successfully processed, continuing with queue");
//   } else {
//     Serial.println("No ACK received during listen period");
//   }
// }
  