/*
 * Basic LoRa Test - Sender (for Ground 1)
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

// Test message structure
struct TestMessage {
  char text[32];
  uint32_t counter;
  uint32_t timestamp;
};

// Global variables
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TestMessage message;
uint32_t counter = 0;
unsigned long lastSendTime = 0;
const int sendInterval = 5000;  // 5 seconds between transmissions

void setup() {
  Serial.begin(115200);
  Serial.println("LoRa Sender Test");
  
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
  
  // Set up LoRa parameters
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(20);
  LoRa.enableCrc();
  
  displayStatus("LoRa initialized!\nReady to send");
  Serial.println("LoRa initialized successfully!");
}

void loop() {
  if (millis() - lastSendTime > sendInterval) {
    sendTestMessage();
    lastSendTime = millis();
  }
}

void sendTestMessage() {
  // Prepare message
  snprintf(message.text, sizeof(message.text), "Hello from Ground 1");
  message.counter = counter++;
  message.timestamp = millis();
  
  // Send message
  displayStatus("Sending message...\nCount: " + String(message.counter));
  Serial.print("Sending message: ");
  Serial.println(message.counter);
  
  LoRa.beginPacket();
  LoRa.write(2);  // Destination: Drone 1
  LoRa.write(1);  // Source: Ground 1
  LoRa.write(0x00);  // Msg Type: Test Data
  
  // Write a 4-byte message ID (using counter)
  LoRa.write((message.counter >> 24) & 0xFF);
  LoRa.write((message.counter >> 16) & 0xFF);
  LoRa.write((message.counter >> 8) & 0xFF);
  LoRa.write(message.counter & 0xFF);
  
  // Write message content
  LoRa.write((uint8_t*)&message, sizeof(TestMessage));
  
  LoRa.endPacket();
  
  displayStatus("Message sent!\nCount: " + String(message.counter));
  Serial.println("Message sent!");
  
  // Brief display of success before showing ready state
  delay(1000);
  displayStatus("Ready to send\nLast count: " + String(message.counter));
}

void displayStatus(String message) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LoRa Test - Sender");
  display.println("----------------");
  display.println(message);
  display.display();
} 