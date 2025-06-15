/*
 * Basic LoRa Test - Receiver (for Drone 1)
 * 
 * Simple test to verify LoRa communication between ESP32 boards
 * Displays status information on OLED display
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display settings for TTGO LoRa32
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define OLED_SDA 21
#define OLED_SCL 22

// LoRa settings for TTGO LoRa32
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_FREQ 868E6  // Europe frequency

// Test message structure - must match sender
struct TestMessage {
  char text[32];
  uint32_t counter;
  uint32_t timestamp;
};

// Global variables
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TestMessage receivedMessage;
unsigned long lastReceivedTime = 0;
uint32_t lastCounter = 0;
bool messageReceived = false;

void setup() {
  Serial.begin(115200);
  Serial.println("LoRa Receiver Test");
  
  // Initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  displayStatus("Starting...");
  
  // Initialize LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(LORA_FREQ)) {
    displayStatus("LoRa init failed!");
    Serial.println("LoRa init failed!");
    while (true);
  }
  
  // Set up LoRa parameters - same as sender
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(20);
  LoRa.enableCrc();
  
  displayStatus("LoRa initialized!\nWaiting for message...");
  Serial.println("LoRa initialized successfully!");
}

void loop() {
  // Check for LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    processPacket(packetSize);
  }
  
  // Update display status periodically
  if (millis() - lastReceivedTime > 10000 && messageReceived) {
    displayStatus("Waiting for message...\nLast: #" + String(lastCounter));
  }
}

void processPacket(int packetSize) {
  // Read packet header
  byte destination = LoRa.read();
  byte source = LoRa.read();
  byte msgType = LoRa.read();
  
  uint32_t msgId = 0;
  msgId |= ((uint32_t)LoRa.read()) << 24;
  msgId |= ((uint32_t)LoRa.read()) << 16;
  msgId |= ((uint32_t)LoRa.read()) << 8;
  msgId |= LoRa.read();
  
  // Get RSSI and SNR for signal quality
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
  // Check if this message is for us (Drone 1 = ID 2)
  if (destination == 2) {
    // Read the message data
    LoRa.readBytes((uint8_t*)&receivedMessage, sizeof(TestMessage));
    
    // Update status info
    lastReceivedTime = millis();
    lastCounter = receivedMessage.counter;
    messageReceived = true;
    
    // Display and log the message
    Serial.println("==============================================");
    Serial.println("Message received from Ground 1");
    Serial.println("Message ID: " + String(msgId));
    Serial.println("Counter: " + String(receivedMessage.counter));
    Serial.println("Text: " + String(receivedMessage.text));
    Serial.println("RSSI: " + String(rssi) + " dBm, SNR: " + String(snr) + " dB");
    Serial.println("==============================================");
    
    // Update display with received message
    String statusMsg = "Message received!\n";
    statusMsg += "From: Ground 1\n";
    statusMsg += "Count: " + String(receivedMessage.counter) + "\n";
    statusMsg += "RSSI: " + String(rssi) + " dBm";
    displayStatus(statusMsg);
    
    // Send acknowledgment
    delay(500);  // Small delay before sending ACK
    sendAck(source, msgId);
  }
}

void sendAck(byte destination, uint32_t msgId) {
  Serial.println("Sending ACK to Ground 1");
  
  LoRa.beginPacket();
  LoRa.write(destination);  // To: Ground 1
  LoRa.write(2);           // From: Drone 1
  LoRa.write(0x01);        // Msg Type: ACK
  
  // Message ID
  LoRa.write((msgId >> 24) & 0xFF);
  LoRa.write((msgId >> 16) & 0xFF);
  LoRa.write((msgId >> 8) & 0xFF);
  LoRa.write(msgId & 0xFF);
  
  LoRa.endPacket();
}

void displayStatus(String message) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LoRa Test - Receiver");
  display.println("----------------");
  display.println(message);
  display.display();
} 