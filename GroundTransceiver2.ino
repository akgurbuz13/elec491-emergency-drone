/*
 * Ground Transceiver 2 - LilyGo LoRa32 T3
 *
 * Final destination for emergency messages.
 * Receives messages from Drone2 (ID 3).
 * Sends Final ACK back to Drone2 (ID 3) immediately upon message receipt.
 * Displays message summary on OLED.
 * (Placeholder for HTML dashboard integration)
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>

// OLED Display settings for LilyGo LoRa32 T3
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST -1 // Reset pin for OLED, or -1 if not used/tied to MCU reset
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

// LoRa settings for LilyGo LoRa32 T3
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14  // LoRa RST pin
#define LORA_DIO0 26 // LoRa DIO0 pin
#define LORA_FREQ 868E6 // Europe frequency

// LoRa parameters (MUST match Drone2, Drone1, Ground1)
#define LORA_SF 7
#define LORA_BANDWIDTH 250E3
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0xF3
#define LORA_TX_POWER 20 // Max TX power

// Device IDs
#define GROUND1_ID 1
#define DRONE1_ID 2
#define DRONE2_ID 3
#define GROUND2_ID 4 // This device

// Message types
#define MSG_DATA 0x00
#define MSG_FINAL_ACK 0x03

// Message structure (MUST match all other devices)
struct EmergencyMessage {
  char text[128];
  float latitude;
  float longitude;
  int emergencyCode;
  uint32_t messageId; // Ensure this matches sender's structure
};

// WiFi access point settings
const char* ssid = "Emergency_G2"; // Ground 2 access point name
const char* password = ""; // Empty for open network

// Message storage for dashboard
#define MAX_SAVED_MESSAGES 20
struct StoredMessage {
  uint32_t messageId;
  char text[128];
  char location[64]; // Added location field for user-provided location descriptions
  float latitude;
  float longitude;
  int emergencyCode;
  unsigned long receiveTime;
  int rssi;
  float snr;
  bool used;
};

// Global variables
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
StoredMessage savedMessages[MAX_SAVED_MESSAGES];
int currentMessageIndex = 0;
WebServer server(80);
unsigned long lastPacketReceivedTime = 0; // Tracks when the last packet was received
unsigned long scanCount = 0; // Counts how many times we've checked for packets

// Function Declarations
void initOLED();
void initLoRa();
void displayStatus(String line1 = "", String line2 = "", String line3 = "", String line4 = "", bool wrap = false);
void processPacket(int packetSize);
void sendFinalAckToDrone2(uint32_t msgId, int rssiAtG2, float snrAtG2);
void safeLoRaTransmit(bool enableTransmit);
void storeMessage(EmergencyMessage* msg, unsigned long receiveTime, int rssi, float snr);
void handleRoot();
void handleMessages();
void handleData();

void setup() {
  Serial.begin(115200);
  // Wait a bit longer for serial to initialize fully, just in case
  for (int i=0; i<10; i++) { delay(50); } // 500ms total

  Serial.println("--- Stage 1: Serial Initialized ---");
  Serial.println("Ground Transceiver 2 Booting Up...");
  Serial.flush(); // Ensure this message is sent

  Serial.println("--- Stage 2: Calling initOLED() ---");
  Serial.flush();
  initOLED();
  Serial.println("--- Stage 3: initOLED() COMPLETED ---");
  Serial.flush();

  Serial.println("--- Stage 4: Calling initLoRa() ---");
  Serial.flush();
  initLoRa();
  Serial.println("--- Stage 5: initLoRa() COMPLETED ---");
  Serial.flush();

  // Initialize WiFi in AP mode
  Serial.println("--- Stage 6: Setting up WiFi Access Point ---");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Initialize web server routes
  Serial.println("--- Stage 7: Setting up Web Server ---");
  server.on("/", handleRoot);
  server.on("/messages", handleMessages);
  server.on("/data", handleData);
  server.begin();
  Serial.println("HTTP server started");
  
  // Initialize message storage
  for (int i = 0; i < MAX_SAVED_MESSAGES; i++) {
    savedMessages[i].used = false;
  }

  Serial.println("--- Stage 8: Calling displayStatus() after init ---");
  Serial.flush();
  displayStatus("Ground Trans. 2", "System Initialized", "WiFi: " + String(ssid), "IP: " + IP.toString());
  Serial.println("--- Stage 9: displayStatus() COMPLETED ---");
  Serial.flush();
  
  Serial.println("Ground Transceiver 2 Setup Fully Initialized. Waiting for messages...");
  Serial.println("Access dashboard at http://" + IP.toString());
  Serial.println("=============================================");
  Serial.flush();
}

void loop() {
    // Handle web server client requests
    server.handleClient();
    
    // Check for incoming LoRa packets
    int packetSize = LoRa.parsePacket();
    scanCount++; // Increment scan count on each loop iteration
    
    if (packetSize > 0) {
      processPacket(packetSize);
    } else {
      // Check for RF activity even if no valid packet (helps diagnose if there's RF but corrupted packets)
      if (LoRa.available()) {
        Serial.println("RF activity detected but no valid packet structure");
        // Discard any bytes
        while (LoRa.available()) {
          LoRa.read();
        }
      }
    }
    
    // Yield for ESP32 background tasks
    yield();
}

void initOLED() {
    Serial.println("  initOLED: Entered function."); Serial.flush();

    if (OLED_RST != -1) {
        Serial.println("  initOLED: Resetting OLED via RST pin."); Serial.flush();
        pinMode(OLED_RST, OUTPUT);
        digitalWrite(OLED_RST, LOW);
        delay(10);
        digitalWrite(OLED_RST, HIGH);
        delay(10);
        Serial.println("  initOLED: OLED Reset complete."); Serial.flush();
    } else {
        Serial.println("  initOLED: OLED_RST is -1, skipping hardware reset."); Serial.flush();
    }

    Serial.println("  initOLED: Calling Wire.begin()."); Serial.flush();
    Wire.begin(OLED_SDA, OLED_SCL); // Initialize I2C
    Serial.println("  initOLED: Wire.begin() complete."); Serial.flush();

    Serial.println("  initOLED: Delaying before display.begin()."); Serial.flush();
    delay(100);
    Serial.println("  initOLED: Delay complete. Calling display.begin()."); Serial.flush();

    // For LilyGo T3 boards, LED_BUILTIN is typically GPIO 25 (the blue LED)
    // If display.begin() fails, it often hangs. We'll try to make it blink an LED.
    #ifndef LED_BUILTIN 
    #define LED_BUILTIN 25
    #endif

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("  initOLED: FATAL - display.begin() FAILED. SSD1306 Allocation Failed."); Serial.flush();
        Serial.println("  Check OLED wiring (SDA, SCL), I2C address (0x3C), and power."); Serial.flush();
        Serial.println("  Board will now attempt to blink LED_BUILTIN (GPIO 25) and halt."); Serial.flush();
        
        pinMode(LED_BUILTIN, OUTPUT); 
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH); delay(200);
            digitalWrite(LED_BUILTIN, LOW); delay(200);
            Serial.print("X"); // Keep serial active to show it's in this loop
        }
    }

    Serial.println("  initOLED: display.begin() OK. Proceeding with display setup."); Serial.flush();
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("GROUND 2");

    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println("OLED Initialized");
    display.setCursor(0, 30);
    display.println("Booting system...");
    display.display();
    Serial.println("  initOLED: Initial OLED display updated. Exiting function."); Serial.flush();
    delay(1500);
}

void initLoRa() {
    Serial.println("Initializing LoRa for Ground Transceiver 2...");
    Serial.println("- Frequency: " + String(LORA_FREQ / 1E6) + " MHz");
    Serial.println("- SF: " + String(LORA_SF));
    Serial.println("- BW: " + String(LORA_BANDWIDTH / 1E3) + " kHz");
    Serial.println("- CR: 4/" + String(LORA_CODING_RATE));
    Serial.println("- SyncWord: 0x" + String(LORA_SYNC_WORD, HEX));
    
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("FATAL: Starting LoRa failed!");
        displayStatus("FATAL ERROR", "LoRa Init Failed!");
        while (true);
    }

    // Set LoRa parameters
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN); // Use PA_BOOST for consistent power output
    LoRa.enableCrc();

    // Start listening for packets
    safeLoRaTransmit(false); // Ensure we are in receive mode
    Serial.println("LoRa Initialized OK.");
}

void displayStatus(String line1, String line2, String line3, String line4, bool wrap) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    
    if (wrap) {
        display.setTextWrap(true);
        display.println(line1); // Line 1 can be long and wrap
        // Subsequent lines start after wrapped content if any
    } else {
        display.setTextWrap(false);
        display.println(line1);
        display.setCursor(0, 10);
        display.println(line2);
        display.setCursor(0, 20);
        display.println(line3);
        display.setCursor(0, 30);
        display.println(line4);
    }
    display.display();
}

// Safely switch LoRa mode and ensure it's back in receive after transmit
void safeLoRaTransmit(bool enableTransmit) {
    if (enableTransmit) {
        LoRa.idle(); // Go to standby mode before transmitting
        delay(10); // Longer delay for stability
    } else {
        delay(10); // Longer delay for stability
        LoRa.receive(); // Put back into continuous receive mode
    }
}

void processPacket(int packetSize) {
  unsigned long receiveTime = millis();
  lastPacketReceivedTime = receiveTime; // Update the last packet received time
  int rssiAtG2 = LoRa.packetRssi();
  float snrAtG2 = LoRa.packetSnr();

  // --- Read Header ---
  if (packetSize < 7) { // Dest(1) + Src(1) + Type(1) + MsgID(4) = 7 bytes minimum
      Serial.println("ERROR: Packet too small (" + String(packetSize) + " bytes), ignoring.");
      safeLoRaTransmit(false); // Ensure back in RX mode
      return;
  }

  byte destination = LoRa.read();
  byte source = LoRa.read();
  byte msgType = LoRa.read();
  uint32_t msgId = 0;
  msgId |= ((uint32_t)LoRa.read()) << 24;
  msgId |= ((uint32_t)LoRa.read()) << 16;
  msgId |= ((uint32_t)LoRa.read()) << 8;
  msgId |= LoRa.read();
  
    Serial.print("G2 RECV [" + String(receiveTime) + "ms] ");
    Serial.print("From: " + String(source) + " ");
    Serial.print("To: " + String(destination) + " ");
    Serial.print("Type: 0x" + String(msgType, HEX) + " ");
    Serial.print("ID: " + String(msgId) + " ");
    Serial.print("RSSI: " + String(rssiAtG2) + " SNR: " + String(snrAtG2, 1));
    Serial.println(" Size: " + String(packetSize));
    Serial.flush();

    // --- Process based on Destination ---
    if (destination != GROUND2_ID) {
        Serial.println("  -> Packet not for me. Ignoring.");
        safeLoRaTransmit(false); // Ensure back in RX mode
        return;
      }
      
    // --- Process based on Source and Type ---
    if (source == DRONE2_ID && msgType == MSG_DATA) {
        Serial.println("  -> DATA received from Drone 2 for MsgID: " + String(msgId));
        
        if (packetSize < (7 + sizeof(EmergencyMessage))) {
             Serial.println("ERROR: DATA packet too small for EmergencyMessage payload. Size: " + String(packetSize));
             safeLoRaTransmit(false);
        return;
      }
      
        EmergencyMessage receivedMsg;
        int payloadAvailable = LoRa.available();
        
        Serial.println("  -> Payload bytes available: " + String(payloadAvailable) + " vs expected: " + String(sizeof(EmergencyMessage)));
        Serial.flush();
        
        if (payloadAvailable != sizeof(EmergencyMessage)) {
            Serial.println("ERROR: Expected payload size " + String(sizeof(EmergencyMessage)) + " but got " + String(payloadAvailable) + ". Ignoring.");
            while(LoRa.available()) { LoRa.read(); }
            safeLoRaTransmit(false);
            return;
        }
        
        LoRa.readBytes((uint8_t *)&receivedMsg, sizeof(EmergencyMessage));

        if (receivedMsg.messageId != msgId) {
            Serial.println("ERROR: Header MsgID (" + String(msgId) + ") != Payload MsgID (" + String(receivedMsg.messageId) + "). Ignoring.");
            safeLoRaTransmit(false);
            return;
        }
        
        Serial.println("  --- EMERGENCY MESSAGE RECEIVED ---");
        Serial.println("  MsgID: " + String(receivedMsg.messageId));
        Serial.println("  Text: " + String(receivedMsg.text));
        Serial.println("  Lat: " + String(receivedMsg.latitude, 6) + ", Lon: " + String(receivedMsg.longitude, 6));
        Serial.println("  Code: " + String(receivedMsg.emergencyCode));
        Serial.println("  Received by G2 with RSSI: " + String(rssiAtG2) + " dBm, SNR: " + String(snrAtG2, 1) + " dB");
        Serial.println("  ----------------------------------");
        Serial.flush();

        // Store message for dashboard
        storeMessage(&receivedMsg, receiveTime, rssiAtG2, snrAtG2);

        // Display message on OLED
        String oledLine1 = "MSG RECVD! ID:" + String(receivedMsg.messageId);
        String oledLine2 = "From D2, Code:" + String(receivedMsg.emergencyCode);
        String oledLine3 = String(receivedMsg.text).substring(0, 20); // Show first 20 chars
         if (String(receivedMsg.text).length() > 20) oledLine3 += "...";
        String oledLine4 = "RSSI:" + String(rssiAtG2) + " SNR:" + String(snrAtG2,1);
        displayStatus(oledLine1, oledLine2, oledLine3, oledLine4);

        // Send Final ACK back to Drone 2 after a longer brief delay to ensure Drone2 is ready to receive
        delay(150); // Increased from 50ms to 150ms
        sendFinalAckToDrone2(receivedMsg.messageId, rssiAtG2, snrAtG2);

    } else {
        Serial.println("  -> Unknown Source/Type combination for Ground2 or not MSG_DATA. Ignoring.");
        while(LoRa.available()) { LoRa.read(); }
        safeLoRaTransmit(false); // Ensure back in RX mode
    }
}

void sendFinalAckToDrone2(uint32_t msgId, int rssiAtG2, float snrAtG2) {
    unsigned long sendTime = millis();
    Serial.println("G2 SEND_ACK_START [" + String(sendTime) + "ms] To: " + String(DRONE2_ID) + " ID: " + String(msgId));

    int16_t g2_rssi_val = (int16_t)rssiAtG2;
    float g2_snr_val = snrAtG2;
    uint8_t snrBytes[4];
    memcpy(snrBytes, &g2_snr_val, 4);
    int payloadSize = 6; // RSSI (2 bytes) + SNR (4 bytes)

    bool sent = false;
    int attempts = 0;
    const int MAX_ACK_ATTEMPTS = 4; 
    const int ACK_RETRY_DELAY_MS = 200; // Increased slightly

    while (!sent && attempts < MAX_ACK_ATTEMPTS) {
        attempts++;
        // Update display FOR EACH ATTEMPT
        displayStatus("Sending ACK Att." + String(attempts), "To: Drone 2", "ID: " + String(msgId));
        Serial.println("  -> G2_SEND_ACK: Attempt " + String(attempts) + " for MsgID: " + String(msgId));
        
        safeLoRaTransmit(true); 
        delay(20); 

        LoRa.beginPacket();
        LoRa.write(DRONE2_ID); LoRa.write(GROUND2_ID); LoRa.write(MSG_FINAL_ACK);
        LoRa.write((msgId >> 24) & 0xFF); LoRa.write((msgId >> 16) & 0xFF);
        LoRa.write((msgId >> 8) & 0xFF); LoRa.write(msgId & 0xFF);
        LoRa.write((g2_rssi_val >> 8) & 0xFF); LoRa.write(g2_rssi_val & 0xFF);
        LoRa.write(snrBytes, 4);
        
        // Serial.println("    ACK Payload: G2_RSSI=" + String(g2_rssi_val) + ", G2_SNR=" + String(g2_snr_val, 1));
        // Serial.println("    Total packet size: " + String(7 + payloadSize) + " bytes");
        // Serial.flush(); // Reduce serial flush for speed

        delay(10); 
        sent = LoRa.endPacket();

        if (sent) {
            Serial.println("  -> G2_SEND_ACK: Success on attempt " + String(attempts));
        } else {
            Serial.println("  -> G2_SEND_ACK: Failed on attempt " + String(attempts));
            if (attempts < MAX_ACK_ATTEMPTS) {
                delay(ACK_RETRY_DELAY_MS);
            }
        }
        delay(20); 
        safeLoRaTransmit(false); // Go back to RX mode after each attempt
    }

    delay(50); // Final delay after loop
    
    if (sent) {
        displayStatus("ACK Sent to D2", "MsgID: " + String(msgId), "Attempts: "+String(attempts), "RSSI:" + String(rssiAtG2));
        delay(1500); 
    } else {
        Serial.println("ERROR: G2_SEND_ACK: Definitive Fail after " + String(attempts) + " attempts!");
        displayStatus("ERROR Sending ACK", "To: D2, ID: " + String(msgId), "Failed: "+String(attempts)+" tries");
        delay(1500); 
    }
    
    // Final revert to idle display
    displayStatus("Ground Trans. 2", "LoRa Initialized", "Waiting for Data");
    safeLoRaTransmit(false); // Ensure in RX mode
}

// Store received message for the dashboard
void storeMessage(EmergencyMessage* msg, unsigned long receiveTime, int rssi, float snr) {
  // Use the next available slot, overwriting the oldest if full
  savedMessages[currentMessageIndex].messageId = msg->messageId;
  strncpy(savedMessages[currentMessageIndex].text, msg->text, 127);
  savedMessages[currentMessageIndex].text[127] = '\0'; // Ensure null termination
  savedMessages[currentMessageIndex].latitude = msg->latitude;
  savedMessages[currentMessageIndex].longitude = msg->longitude;
  savedMessages[currentMessageIndex].emergencyCode = msg->emergencyCode;
  savedMessages[currentMessageIndex].receiveTime = receiveTime;
  savedMessages[currentMessageIndex].rssi = rssi;
  savedMessages[currentMessageIndex].snr = snr;
  savedMessages[currentMessageIndex].used = true;
  
  // Extract location from message text if available
  // Format is typically "Name: Message (at Location)" for web interface
  String messageText = String(msg->text);
  int atIndex = messageText.indexOf(" (at ");
  if (atIndex > 0 && messageText.endsWith(")")) {
    // Extract location between "(at " and ")"
    String location = messageText.substring(atIndex + 5, messageText.length() - 1);
    strncpy(savedMessages[currentMessageIndex].location, location.c_str(), 63);
    savedMessages[currentMessageIndex].location[63] = '\0';
  } else {
    // No location found in text, leave empty
    savedMessages[currentMessageIndex].location[0] = '\0';
  }
  
  // Move to next slot
  currentMessageIndex = (currentMessageIndex + 1) % MAX_SAVED_MESSAGES;
  
  Serial.println("  -> Message stored in dashboard at index " + String((currentMessageIndex - 1 + MAX_SAVED_MESSAGES) % MAX_SAVED_MESSAGES));
}

// Web server route handlers
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Ground Transceiver 2 - Dashboard</title>";
  html += "<style>";
  html += "body { font-family: Arial; margin: 0; padding: 20px; background: #f8f9fa; }";
  html += ".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
  html += "h1 { color: #dc3545; }";
  html += "h2 { color: #343a40; margin-top: 30px; }";
  html += ".status-panel { background: #e9ecef; padding: 15px; border-radius: 5px; margin-bottom: 20px; }";
  html += ".status-item { margin-bottom: 5px; }";
  html += ".btn { display: inline-block; background: #007bff; color: white; padding: 10px 15px; border-radius: 5px; text-decoration: none; margin-top: 20px; }";
  html += ".btn:hover { background: #0069d9; }";
  html += "</style>";
  html += "<script>";
  html += "function refreshStatus() {";
  html += "  fetch('/data')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      document.getElementById('lastPacket').textContent = data.lastPacketTime;";
  html += "      document.getElementById('rssi').textContent = data.rssi + ' dBm';";
  html += "      document.getElementById('snr').textContent = data.snr + ' dB';";
  html += "      document.getElementById('scanCount').textContent = data.scanCount;";
  html += "      document.getElementById('uptime').textContent = data.uptime;";
  html += "    });";
  html += "  setTimeout(refreshStatus, 2000);";
  html += "}";
  html += "window.onload = refreshStatus;";
  html += "</script>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>Ground Transceiver 2 Dashboard</h1>";
  
  html += "<div class='status-panel'>";
  html += "<h2>System Status</h2>";
  html += "<div class='status-item'><strong>Last Packet Received:</strong> <span id='lastPacket'>-</span></div>";
  html += "<div class='status-item'><strong>Signal Strength:</strong> <span id='rssi'>-</span></div>";
  html += "<div class='status-item'><strong>Signal Quality:</strong> <span id='snr'>-</span></div>";
  html += "<div class='status-item'><strong>Scan Count:</strong> <span id='scanCount'>-</span></div>";
  html += "<div class='status-item'><strong>Uptime:</strong> <span id='uptime'>-</span></div>";
  html += "</div>";
  
  html += "<a href='/messages' class='btn'>View Emergency Messages</a>";
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

void handleMessages() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Emergency Messages - Ground 2</title>";
  html += "<style>";
  html += "body { font-family: Arial; margin: 0; padding: 20px; background: #f8f9fa; }";
  html += ".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
  html += "h1 { color: #dc3545; }";
  html += "table { width: 100%; border-collapse: collapse; margin-top: 20px; }";
  html += "th, td { padding: 12px; text-align: left; border-bottom: 1px solid #ddd; }";
  html += "th { background-color: #f8f9fa; }";
  html += "tr:hover { background-color: #f1f1f1; }";
  html += ".message-text { max-width: 300px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }";
  html += ".code1 { background-color: #f8d7da; }"; // Medical
  html += ".code2 { background-color: #fff3cd; }"; // Fire
  html += ".code3 { background-color: #d1ecf1; }"; // Security
  html += ".code4 { background-color: #d4edda; }"; // Structural
  html += ".code5 { background-color: #e2e3e5; }"; // Other
  html += ".btn { display: inline-block; background: #6c757d; color: white; padding: 10px 15px; border-radius: 5px; text-decoration: none; margin-top: 20px; }";
  html += ".btn:hover { background: #5a6268; }";
  html += ".btn-primary { background: #007bff; }";
  html += ".btn-primary:hover { background: #0069d9; }";
  html += ".refresh { margin-top: 20px; text-align: right; }";
  html += ".no-messages { text-align: center; padding: 30px; color: #6c757d; }";
  html += "</style>";
  html += "<script>";
  
  html += "function getEmergencyTypeName(code) {";
  html += "  switch(parseInt(code)) {";
  html += "    case 1: return 'Medical';";
  html += "    case 2: return 'Fire';";
  html += "    case 3: return 'Security';";
  html += "    case 4: return 'Structural';";
  html += "    case 5: return 'Other';";
  html += "    default: return 'Unknown';";
  html += "  }";
  html += "}";
  
  html += "window.onload = function() {";
  // Setup emergency type names
  html += "  document.querySelectorAll('.emergency-code').forEach(el => {";
  html += "    const code = el.textContent;";
  html += "    el.textContent = getEmergencyTypeName(code);";
  html += "    el.parentElement.classList.add('code' + code);";
  html += "  });";
  // Always auto-refresh every 7 seconds
  html += "  setInterval(function() { window.location.reload(); }, 5000);"; 
  html += "};";
  html += "</script>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>Emergency Messages Received</h1>";
  
  // Count active messages
  int activeMessageCount = 0;
  for (int i = 0; i < MAX_SAVED_MESSAGES; i++) {
    if (savedMessages[i].used) {
      activeMessageCount++;
    }
  }
  
  if (activeMessageCount > 0) {
    html += "<table>";
    html += "<tr><th>ID</th><th>Time</th><th>Message</th><th>Emergency</th><th>Location</th><th>GPS</th><th>Signal</th></tr>";
    
    // Display messages in reverse order (newest first)
    for (int i = 0; i < MAX_SAVED_MESSAGES; i++) {
      int idx = (currentMessageIndex - 1 - i + MAX_SAVED_MESSAGES) % MAX_SAVED_MESSAGES;
      if (savedMessages[idx].used) {
        unsigned long ageMs = millis() - savedMessages[idx].receiveTime;
        String ageStr;
        if (ageMs < 60000) {
          ageStr = String(ageMs / 1000) + " sec ago";
        } else if (ageMs < 3600000) {
          ageStr = String(ageMs / 60000) + " min ago";
        } else {
          ageStr = String(ageMs / 3600000) + " hr ago";
        }
        
        // Location logic
        String locationDisplay = "-";
        // If fire SOS from Python app (location empty or generic), show SNA Building
        bool isFire = savedMessages[idx].emergencyCode == 2;
        bool isPythonApp = strlen(savedMessages[idx].text) > 0 && (strlen(savedMessages[idx].location) == 0 || String(savedMessages[idx].location) == "" || String(savedMessages[idx].location) == "-" || String(savedMessages[idx].location) == "N/A");
        if (isFire && isPythonApp) {
          locationDisplay = "SNA Building";
        } else if (strlen(savedMessages[idx].location) > 0 && String(savedMessages[idx].location) != "-" && String(savedMessages[idx].location) != "N/A") {
          locationDisplay = String(savedMessages[idx].location);
        }
        
        // GPS logic
        String gpsDisplay = "-";
        if (fabs(savedMessages[idx].latitude) > 0.0001 && fabs(savedMessages[idx].longitude) > 0.0001) {
          gpsDisplay = String(savedMessages[idx].latitude, 6) + ", " + String(savedMessages[idx].longitude, 6);
          gpsDisplay += "<br><a href='https://maps.google.com/?q=" + String(savedMessages[idx].latitude, 6) + "," + String(savedMessages[idx].longitude, 6) + "' target='_blank'>View on Map</a>";
        }
        
        html += "<tr>";
        html += "<td>" + String(savedMessages[idx].messageId) + "</td>";
        html += "<td>" + ageStr + "</td>";
        html += "<td class='message-text'>" + String(savedMessages[idx].text) + "</td>";
        html += "<td class='emergency-code'>" + String(savedMessages[idx].emergencyCode) + "</td>";
        html += "<td>" + locationDisplay + "</td>";
        html += "<td>" + gpsDisplay + "</td>";
        html += "<td>RSSI: " + String(savedMessages[idx].rssi) + " dBm<br>SNR: " + String(savedMessages[idx].snr, 1) + " dB</td>";
        html += "</tr>";
      }
    }
    
    html += "</table>";
  } else {
    html += "<div class='no-messages'>No emergency messages received yet</div>";
  }
  
  html += "<div class='refresh'>";
  html += "  <a href='/messages' class='btn btn-primary'>Refresh Now</a> "; // Kept manual refresh
  html += "  <a href='/' class='btn'>Back to Dashboard</a>";
  html += "</div>";
  
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"lastPacketTime\":\"" + (lastPacketReceivedTime > 0 ? String((millis() - lastPacketReceivedTime) / 1000) + " seconds ago" : "Never") + "\",";
  json += "\"rssi\":" + String(LoRa.packetRssi()) + ",";
  json += "\"snr\":" + String(LoRa.packetSnr(), 1) + ",";
  json += "\"scanCount\":" + String(scanCount) + ",";
  
  // Calculate uptime
  unsigned long uptimeMs = millis();
  unsigned long uptimeSec = uptimeMs / 1000;
  unsigned long uptimeMin = uptimeSec / 60;
  unsigned long uptimeHour = uptimeMin / 60;
  unsigned long uptimeDay = uptimeHour / 24;
  
  uptimeHour %= 24;
  uptimeMin %= 60;
  uptimeSec %= 60;
  
  json += "\"uptime\":\"";
  if (uptimeDay > 0) json += String(uptimeDay) + "d ";
  json += String(uptimeHour) + "h " + String(uptimeMin) + "m " + String(uptimeSec) + "s\"";
  
  json += "}";
  
  server.send(200, "application/json", json);
} 