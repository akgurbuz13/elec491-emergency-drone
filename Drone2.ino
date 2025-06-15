/*
 * Drone 2 Relay - TTGO T-Beam v1.1
 *
 * Receives DATA messages from Drone1 (ID 2) and forwards them to Ground2 (ID 4).
 * Receives Final ACKs from Ground2 (ID 4) and forwards them back to Drone1 (ID 2).
 *
 * NO local ACKs are sent or expected by this device.
 * Assumes LoRa parameters compatible with Drone1 and Ground2.
 * Hardware: TTGO T-Beam v1.1
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <axp20x.h>

// AXP192 Power Management
AXP20X_Class axp;

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define OLED_SDA 21
#define OLED_SCL 22

// LoRa settings for T-Beam v1.1
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 23 // T-Beam specific reset pin
#define LORA_DIO0 26
#define LORA_FREQ 868E6 // Europe frequency

// LoRa parameters (MUST match Drone1 and Ground1, and be compatible with Ground2)
#define LORA_SF 7
#define LORA_BANDWIDTH 250E3
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0xF3
#define LORA_TX_POWER 20

// Battery monitoring
#define BATTERY_PIN 35

// Device IDs
#define GROUND1_ID 1 
#define DRONE1_ID 2
#define DRONE2_ID 3 // This device
#define GROUND2_ID 4

// Message types
#define MSG_DATA 0x00
#define MSG_ACK 0x01       // Defined but NOT used by Drone2
#define MSG_FINAL_ACK 0x03 // Received from Ground2, forwarded to Drone1

// Message structure (MUST match Ground1 and others)
struct EmergencyMessage {
  char text[128];
  float latitude;
  float longitude;
  int emergencyCode;
  uint32_t messageId;
};

// Global variables
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
float batteryVoltage = 0;
unsigned long lastBatteryCheck = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 30000; // 30 seconds

// Buffer for final ACK payload received from Ground2 and forwarded to Drone1
#define MAX_FINAL_ACK_PAYLOAD_FROM_G2 10 // Max expected from G2: G2_RSSI(2)+SNR(4) = 6. Add buffer.
byte finalAckPayloadBufferFromG2[MAX_FINAL_ACK_PAYLOAD_FROM_G2];

// Variables for managing ACK expectation from Ground 2
uint32_t waitingForG2AckMsgId = 0;      // Message ID Drone 2 is currently expecting an ACK for from Ground 2
unsigned long timeForwardedToG2 = 0; // Timestamp when the message was forwarded to Ground 2
const unsigned long G2_ACK_TIMEOUT = 4000; // Reduced to 4 seconds for Ground 2 ACK

// Variables for monitoring radio activity
unsigned long scanCount = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastPacketTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 10000; // 10 seconds

// Function Declarations
void initAXP192();
void initOLED();
void initLoRa();
void updateBatteryStatus();
void displayStatus(String line1 = "", String line2 = "", String line3 = "", String line4 = "");
void processPacket(int packetSize);
void forwardToGround2(EmergencyMessage msg, uint32_t msgId, int rssi, float snr);
void forwardFinalAckToDrone1(uint32_t msgId, byte *payload, int payloadSize, int rssiFromG2, float snrFromG2);
void safeLoRaTransmit(bool enableTransmit);

void setup() {
  Serial.begin(115200);
  // Remove Serial wait - it blocks devices without serial connection
  delay(100); // Just a short delay to stabilize
  Serial.println("Drone 2 Relay Booting Up...");

  // Initialize I2C first
  Wire.begin(OLED_SDA, OLED_SCL);
  delay(10); // Short delay for I2C bus stability

  initAXP192();
  initOLED();
  initLoRa();
  updateBatteryStatus();

  displayStatus("Drone 2 Relay", "LoRa Initialized", "Waiting for Data", "Batt: " + String(batteryVoltage, 2) + "V");
  Serial.println("Drone 2 Relay Initialized. Waiting for messages...");
  Serial.println("==============================================");
}

void loop() {
    int packetSize = LoRa.parsePacket();
    scanCount++;
    
    if (packetSize >= 7) { // MINIMUM_HEADER_SIZE = 7 bytes. Process if potentially valid.
        lastPacketTime = millis();
        processPacket(packetSize); 
    } else if (packetSize > 0 && packetSize < 7) { // Runt packet detected by LoRa.parsePacket()
        Serial.print("RUNT PACKET detected by LoRa.parsePacket(). Size: " + String(packetSize) + ". Attempting to clear...");
        // LoRa.parsePacket() might have already read these bytes if it determined a size.
        // To be safe, ensure the radio is in a good state if we don't fully trust the packet.
        // Reading out remaining bytes based on LoRa.available() might be safer if parsePacket() is unreliable here.
        int clearedBytes = 0;
        unsigned long runtClearStart = millis();
        while(LoRa.available() && clearedBytes < packetSize && (millis() - runtClearStart < 20)) { // Read at most packetSize or timeout
             LoRa.read();
             clearedBytes++;
        }
        Serial.println(" Cleared " + String(clearedBytes) + " byte(s).");
        safeLoRaTransmit(false); // Ensure back in RX mode
    } else { // packetSize is 0 or less (error from parsePacket, or no packet)
        // This is the original "RF activity detected" block
        if (LoRa.available()) { 
            int peekByte = LoRa.peek(); 
            Serial.print("RF activity detected (parsePacket returned <=0). First byte: 0x");
            Serial.println(peekByte, HEX);
            unsigned long flushStart = millis();
            int flushedCount = 0;
            while(LoRa.available() && millis() - flushStart < 50) { 
                LoRa.read(); 
                flushedCount++;
            }
            if(flushedCount > 0) Serial.println("  Flushed " + String(flushedCount) + " bytes from RF activity.");
        }
    }
  
  // Heartbeat output
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime > HEARTBEAT_INTERVAL) {
    Serial.println("D2 HEARTBEAT: Actively listening for packets. Scans: " + String(scanCount));
    Serial.println("  LoRa: SF=" + String(LORA_SF) + ", BW=" + String(LORA_BANDWIDTH/1000) + "kHz, CR=4/" + String(LORA_CODING_RATE));
    Serial.println("  Last packet: " + (lastPacketTime > 0 ? String((currentTime - lastPacketTime)/1000) + "s ago" : "Never"));
    Serial.println("  Waiting for G2 ACK: " + String(waitingForG2AckMsgId > 0 ? "Yes (ID: " + String(waitingForG2AckMsgId) + ")" : "No"));
    
    scanCount = 0;
    lastHeartbeatTime = currentTime;
  }
  
  if (millis() - lastBatteryCheck > BATTERY_CHECK_INTERVAL) {
        updateBatteryStatus();
        lastBatteryCheck = millis();
        // Update battery on line 4, but only if not showing a specific status
        if (waitingForG2AckMsgId == 0) { // Only update if not in a specific waiting state
            display.fillRect(0, 3 * 10, SCREEN_WIDTH, 10, SSD1306_BLACK);
            display.setCursor(0, 3 * 10);
            display.print("Batt: " + String(batteryVoltage, 2) + "V");
            display.display();
        }
    }

    // Check for Ground 2 ACK timeout with improved reporting
    if (waitingForG2AckMsgId != 0) {
      unsigned long waitingTime = millis() - timeForwardedToG2;
      
      // Check for timeout
      if (waitingTime > G2_ACK_TIMEOUT) {
        uint32_t timedOutMsgId = waitingForG2AckMsgId; // Store ID before clearing
        waitingForG2AckMsgId = 0; // Reset waiting state FIRST
        Serial.println("TIMEOUT waiting for ACK from G2 for MsgID: " + String(timedOutMsgId) + 
                      " after " + String(waitingTime / 1000) + " seconds");
        displayStatus("G2 ACK Timeout", "MsgID: " + String(timedOutMsgId), "Resetting...", "Batt: " + String(batteryVoltage, 2) + "V");
        delay(1000); // Show timeout message for 1 second
        displayStatus("Drone 2 Relay", "LoRa Initialized", "Waiting for Data", "Batt: " + String(batteryVoltage, 2) + "V");
      }
    }

    yield();
}

void initAXP192() {
    Serial.println("Starting AXP192 initialization...");
    Serial.println("  I2C pins: SDA=" + String(OLED_SDA) + ", SCL=" + String(OLED_SCL));
    Serial.println("  AXP192 address: 0x" + String(AXP192_SLAVE_ADDRESS, HEX));

    // Reset I2C bus first
    Wire.end();
    delay(50);
    Wire.begin(OLED_SDA, OLED_SCL);
    delay(50);
    
    // Try to initialize the AXP192 with multiple attempts
    bool axpInitSuccess = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.println("  AXP192 initialization attempt " + String(attempt) + "/3");
        if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_PASS) {
            axpInitSuccess = true;
            Serial.println("  AXP192 initialized successfully on attempt " + String(attempt));
            break;
        }
        delay(100); // Wait before retry
    }
    
    if (!axpInitSuccess) {
        Serial.println("WARNING: AXP192 initialization failed after 3 attempts!");
        Serial.println("  - Will attempt to use default power configuration");
        Serial.println("  - Battery monitoring may be unavailable");
        Serial.println("  - Some peripherals may not receive power correctly");
        
        // Try direct I2C communication as a diagnostic
        Wire.beginTransmission(AXP192_SLAVE_ADDRESS);
        byte error = Wire.endTransmission();
        if (error == 0) {
            Serial.println("  - I2C communication with AXP192 address succeeded, but library initialization failed");
        } else {
            Serial.println("  - I2C communication failed with error: " + String(error));
            Serial.println("  - This might indicate a hardware issue or address mismatch");
        }
    } else {
        // Power on essential components
        axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // LORA radio
        Serial.println("  - AXP192: LoRa radio power (LDO2) enabled");
        
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS
        Serial.println("  - AXP192: GPS power (LDO3) enabled");
        
        axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON); // OLED
        Serial.println("  - AXP192: OLED power (DCDC2) enabled");
        
        axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
        Serial.println("  - AXP192: EXTEN power enabled");
        
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // ESP32 Core
        Serial.println("  - AXP192: ESP32 Core power (DCDC1) enabled");

        // Enable battery voltage/current ADC
        axp.adc1Enable(AXP202_BATT_VOL_ADC1 | AXP202_BATT_CUR_ADC1, true);
        Serial.println("  - AXP192: Battery monitoring enabled");
        
        // Get initial battery reading to confirm ADC is working
        if (axp.isBatteryConnect()) {
            float v = axp.getBattVoltage() / 1000.0;
            Serial.println("  - Battery detected: " + String(v, 2) + "V");
        } else {
            Serial.println("  - No battery detected, may be running on USB power");
        }
    }
    
    // Even if AXP init failed, try to power on LoRa directly with GPIO
    // Some T-Beam variants need this regardless of AXP status
    Serial.println("  - Setting up additional power pins as backup");
    // On some T-Beams GPIO21 controls power to LoRa
    pinMode(21, OUTPUT);
    digitalWrite(21, HIGH);
}

void initOLED() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("FATAL: SSD1306 Allocation Failed");
        while (true);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("OLED Initialized");
    display.display();
    delay(500);
}

void initLoRa() {
    Serial.println("Initializing LoRa...");
    Serial.println("- Frequency: " + String(LORA_FREQ / 1E6) + " MHz");
    Serial.println("- SF: " + String(LORA_SF));
    Serial.println("- BW: " + String(LORA_BANDWIDTH / 1E3) + " kHz");
    Serial.println("- CR: 4/" + String(LORA_CODING_RATE));
    Serial.println("- SyncWord: 0x" + String(LORA_SYNC_WORD, HEX));
    Serial.println("- TX Power: " + String(LORA_TX_POWER) + " dBm");
    Serial.println("- LoRa pins - SS:" + String(LORA_SS) + ", RST:" + String(LORA_RST) + ", DIO0:" + String(LORA_DIO0));
    Serial.println("- SPI pins - SCK:" + String(LORA_SCK) + ", MISO:" + String(LORA_MISO) + ", MOSI:" + String(LORA_MOSI));
    
    // Manual reset of LoRa module before initialization
    Serial.println("- Performing manual reset of LoRa module...");
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(50); // Give the module time to reset
    
    // Initialize SPI
    Serial.println("- Initializing SPI bus...");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    delay(10);
    
    // Configure LoRa module
    Serial.println("- Setting LoRa pins...");
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    
    // Try to initialize LoRa with multiple attempts
    bool loraInitialized = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.print("- LoRa initialization attempt " + String(attempt) + "/3... ");
        if (LoRa.begin(LORA_FREQ)) {
            Serial.println("SUCCESS");
            loraInitialized = true;
            break;
        } else {
            Serial.println("FAILED");
            delay(200);
        }
    }

    if (!loraInitialized) {
        Serial.println("FATAL: Starting LoRa failed after 3 attempts!");
        displayStatus("FATAL ERROR", "LoRa Init Failed!");
        
        // Try one last power cycle of the LoRa module using GPIO
        Serial.println("- Attempting power cycle via GPIO21...");
        pinMode(21, OUTPUT);
        digitalWrite(21, LOW);  // Power off
        delay(500);
        digitalWrite(21, HIGH); // Power on
        delay(500);
        
        if (LoRa.begin(LORA_FREQ)) {
            Serial.println("- LoRa initialization succeeded after power cycle!");
            loraInitialized = true;
        } else {
            Serial.println("- LoRa initialization failed even after power cycle!");
            // Continue despite failure to see if anything works
        }
    }

    // Set LoRa parameters
    Serial.println("- Configuring LoRa parameters...");
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.enableCrc();
    
    // Start listening
    safeLoRaTransmit(false);
    if (loraInitialized) {
        Serial.println("LoRa Initialized OK.");
    } else {
        Serial.println("WARNING: LoRa initialization might have failed, but continuing anyway.");
    }
    
    // Test transmission to check if LoRa is working
    Serial.println("- Sending test packet...");
    LoRa.beginPacket();
    LoRa.print("D2 Init");
    bool testPacketSent = LoRa.endPacket();
    if (testPacketSent) {
        Serial.println("- Test packet sent successfully");
    } else {
        Serial.println("- Failed to send test packet");
    }
    
    // Return to receive mode
    safeLoRaTransmit(false);
}

void updateBatteryStatus() {
    static bool axpWarningShown = false;
    static bool lastBatteryState = false;

    // First try via AXP192
    if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) != AXP_FAIL) {
        lastBatteryState = axp.isBatteryConnect();
        if (lastBatteryState) {
            batteryVoltage = axp.getBattVoltage() / 1000.0;
            Serial.println("Battery: " + String(batteryVoltage, 2) + "V (via AXP192)");
        } else {
            batteryVoltage = 0; // No battery
            Serial.println("No battery detected via AXP192, likely on USB power");
        }
        axpWarningShown = false; // Reset warning flag if AXP works now
    } 
    // Fallback to ADC pin if AXP fails
    else {
        // Show warning once per session
        if (!axpWarningShown) {
            Serial.println("WARNING: AXP192 unavailable for battery reading, using ADC fallback");
            axpWarningShown = true;
        }
        
        // Try to use analog pin for battery level
        int rawValue = analogRead(BATTERY_PIN);
        float voltage = rawValue * 2.0 * 3.3 / 4095.0;
        
        // Basic sanity check - LiPo battery should be between 3.0-4.2V
        if (voltage >= 3.0 && voltage <= 4.3) {
            batteryVoltage = voltage;
            Serial.println("Battery: " + String(batteryVoltage, 2) + "V (via ADC pin)");
        } else if (lastBatteryState) {
            batteryVoltage = 3.7; // Use typical LiPo value if reading is invalid but we had battery before
            Serial.println("Battery: using default 3.7V (invalid ADC reading: " + String(voltage, 2) + "V)");
        } else {
            batteryVoltage = 0; // Likely running from USB
            Serial.println("No battery detected via ADC, likely on USB power");
        }
    }
}

void displayStatus(String line1, String line2, String line3, String line4) {
    display.clearDisplay();
    display.setCursor(0, 0); display.println(line1);
    display.setCursor(0, 10); display.println(line2);
    display.setCursor(0, 20); display.println(line3);
    display.setCursor(0, 30); display.println(line4);
    display.display();
}

void safeLoRaTransmit(bool enableTransmit) {
    if (enableTransmit) {
        LoRa.idle(); 
        delay(20); // Changed from 10ms for stability
    } else {
        delay(20); // Changed from 10ms for stability
        LoRa.receive(); 
    }
}

void processPacket(int packetSize) {
  unsigned long receiveTime = millis();
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
    if (packetSize < 7) {
        Serial.println("ERROR: Pkt too small (" + String(packetSize) + "), ignoring.");
        safeLoRaTransmit(false); return;
    }

  byte destination = LoRa.read();
  byte source = LoRa.read();
  byte msgType = LoRa.read();
  uint32_t msgId = 0;
  msgId |= ((uint32_t)LoRa.read()) << 24;
  msgId |= ((uint32_t)LoRa.read()) << 16;
  msgId |= ((uint32_t)LoRa.read()) << 8;
  msgId |= LoRa.read();
  
    Serial.print("RECV [" + String(receiveTime) + "ms] ");
    Serial.print("From:" + String(source) + " To:" + String(destination) + " T:0x" + String(msgType, HEX) + " ID:" + String(msgId));
    Serial.println(" RSSI:" + String(rssi) + " SNR:" + String(snr, 1) + " Sz:" + String(packetSize));
    Serial.flush(); // Ensure log is immediately displayed

    if (destination != DRONE2_ID) {
        Serial.println("  -> Pkt not for me. Ignoring.");
        safeLoRaTransmit(false); return;
    }

    if (source == DRONE1_ID && msgType == MSG_DATA) {
        Serial.println("  -> DATA from Drone 1.");
        displayStatus("DATA Recv D1", "ID: " + String(msgId), "RSSI: " + String(rssi), "Forwarding G2...");

        if (packetSize < (7 + sizeof(EmergencyMessage))) {
             Serial.println("ERROR: DATA pkt too small for EmergMsg. Sz: " + String(packetSize));
             safeLoRaTransmit(false); return;
        }
        EmergencyMessage receivedMsg;
        int payloadAvailable = LoRa.available();
        Serial.println("  -> Payload available: " + String(payloadAvailable) + " vs expected: " + String(sizeof(EmergencyMessage)));
        Serial.flush();
        if (payloadAvailable != sizeof(EmergencyMessage)) {
            Serial.println("ERROR: Expect payload " + String(sizeof(EmergencyMessage)) + " got " + String(payloadAvailable));
            while(LoRa.available()) { LoRa.read(); } safeLoRaTransmit(false); return;
        }
        LoRa.readBytes((uint8_t *)&receivedMsg, sizeof(EmergencyMessage));
        if (receivedMsg.messageId != msgId) {
            Serial.println("ERROR: Hdr MsgID (" + String(msgId) + ") != Pyld MsgID (" + String(receivedMsg.messageId) + ")");
            safeLoRaTransmit(false); return;
        }
        Serial.println("  -> Pyld ID: " + String(receivedMsg.messageId) + " Txt: " + String(receivedMsg.text));
        Serial.flush(); // Flush logs before forwarding

        // Set ACK expectation state BEFORE starting the relay attempts for this messageId
        waitingForG2AckMsgId = msgId; 
        timeForwardedToG2 = millis();

        forwardToGround2(receivedMsg, msgId, rssi, snr);

    } else if (source == GROUND2_ID && msgType == MSG_FINAL_ACK) {
        Serial.println("  -> FINAL ACK from Ground 2.");
        // Check if this is the ACK we were specifically waiting for
        if (msgId == waitingForG2AckMsgId) {
            Serial.println("  -> This is the expected ACK for MsgID: " + String(msgId));
            waitingForG2AckMsgId = 0; // Clear the waiting state for this specific message
        } else if (waitingForG2AckMsgId != 0) {
            Serial.println("  -> Received ACK for MsgID: " + String(msgId) + ", but was expecting for MsgID: " + String(waitingForG2AckMsgId));
        } else {
            Serial.println("  -> Received an unsolicited/late ACK for MsgID: " + String(msgId));
        }
        displayStatus("FINAL ACK G2", "ID: " + String(msgId), "RSSI: " + String(rssi), "Forwarding D1...");
        int payloadSize = LoRa.available();
        if (payloadSize > 0 && payloadSize <= MAX_FINAL_ACK_PAYLOAD_FROM_G2) {
            LoRa.readBytes(finalAckPayloadBufferFromG2, payloadSize);
            Serial.println("  -> Read " + String(payloadSize) + "b signal data.");
            Serial.flush();
            forwardFinalAckToDrone1(msgId, finalAckPayloadBufferFromG2, payloadSize, rssi, snr);
        } else if (payloadSize > MAX_FINAL_ACK_PAYLOAD_FROM_G2) {
             Serial.println("ERROR: FINAL ACK pyld too large (" + String(payloadSize) + "b). Ignoring.");
             while(LoRa.available()) { LoRa.read(); } safeLoRaTransmit(false);
          } else {
            Serial.println("WARN: FINAL ACK from G2 no signal data.");
            Serial.flush();
            forwardFinalAckToDrone1(msgId, NULL, 0, rssi, snr);
        }
    } else {
        Serial.println("  -> Unknown Src/Type. Ignoring.");
        while(LoRa.available()) { LoRa.read(); } safeLoRaTransmit(false);
    }
}

void forwardToGround2(EmergencyMessage msg, uint32_t msgId, int rssi, float snr) {
    Serial.println("FORWARDING to G2: MsgID=" + String(msgId));
    displayStatus("Forwarding to G2", "ID: " + String(msgId), "RSSI D1: " + String(rssi), "Sending...");
    
    // Flush serial before transmission
    Serial.flush();
    
    safeLoRaTransmit(true);
    
    // Add additional delay for stability before transmission
    delay(20);
    
    Serial.println("  -> Preparing packet with " + String(sizeof(EmergencyMessage)) + " byte payload");
    
    // Retry mechanism for reliability
    bool sent = false;
    int attempts = 0;
    const int MAX_ATTEMPTS = 3;
    
    while (!sent && attempts < MAX_ATTEMPTS) {
        attempts++;
        
        LoRa.beginPacket();
        LoRa.write(GROUND2_ID);       // Destination
        LoRa.write(DRONE2_ID);        // Source (This device)
        LoRa.write(MSG_DATA);         // Type
        LoRa.write((msgId >> 24) & 0xFF); // Message ID
        LoRa.write((msgId >> 16) & 0xFF);
        LoRa.write((msgId >> 8) & 0xFF);
        LoRa.write(msgId & 0xFF);
        
        int bytesWritten = LoRa.write((uint8_t *)&msg, sizeof(EmergencyMessage));
        
        // Longer delay before ending packet
        delay(10);
        
        sent = LoRa.endPacket();
        
        if (sent) {
            Serial.println("  -> FWD->G2: SUCCESS. Wrote " + String(bytesWritten) + " bytes on attempt " + String(attempts));
            break;
        } else {
            Serial.println("  -> FWD->G2: FAILED on attempt " + String(attempts));
            if (attempts < MAX_ATTEMPTS) {
                Serial.println("  -> Retrying in 100ms...");
                delay(100); // Wait before retry
            }
        }
    }
    
    // Ensure some delay after transmission
    delay(20);
    
    // Immediately return to listen mode
    safeLoRaTransmit(false);
    
    delay(20); // Allow radio to settle in RX mode 
    
    if (sent) {
        // Update display to show waiting for ACK from G2
        displayStatus("Waiting for ACK", "From Ground 2", "ID: " + String(msgId), "Batt: " + String(batteryVoltage, 2) + "V");
        
        // Update the timestamp AFTER our final transmission attempt and settling
        timeForwardedToG2 = millis();
        waitingForG2AckMsgId = msgId;

        Serial.println("  -> Now waiting for ACK from Ground 2 (timeout: " + String(G2_ACK_TIMEOUT/1000) + "s)");
    } else {
        // Handle the failure case
        Serial.println("ERROR: Failed to forward message to Ground 2 after " + String(MAX_ATTEMPTS) + " attempts");
        displayStatus("ERROR", "Failed to send", "to Ground 2", "Batt: " + String(batteryVoltage, 2) + "V");
        delay(2000); // Show error for 2 seconds
        displayStatus("Drone 2 Relay", "LoRa Initialized", "Waiting for Data", "Batt: " + String(batteryVoltage, 2) + "V");
        
        // Since we couldn't send, clear the waiting state
        waitingForG2AckMsgId = 0;
    }
}

void forwardFinalAckToDrone1(uint32_t msgId, byte *payloadFromG2, int payloadFromG2Size, int rssiFromG2, float snrFromG2) {
  unsigned long sendTime = millis();
    Serial.print("FWD_ACK->D1 [" + String(sendTime) + "ms] To:" + String(DRONE1_ID) + " ID:" + String(msgId));
    displayStatus("FWD ACK D1", "ID: " + String(msgId), "RSSI G2: " + String(rssiFromG2), "Sending...");
    Serial.flush(); // Flush before radio operations

    safeLoRaTransmit(true);
    
    // Add retry mechanism for better reliability
    bool sent = false;
    int attempts = 0;
    const int MAX_ATTEMPTS = 3;
    
    while (!sent && attempts < MAX_ATTEMPTS) {
        attempts++;
        
        LoRa.beginPacket();
        LoRa.write(DRONE1_ID); LoRa.write(DRONE2_ID); LoRa.write(MSG_FINAL_ACK);
        LoRa.write((msgId >> 24) & 0xFF); LoRa.write((msgId >> 16) & 0xFF);
        LoRa.write((msgId >> 8) & 0xFF); LoRa.write(msgId & 0xFF);

        // Payload for Drone1: G2_signal_data (from payloadFromG2) + D2_signal_data (current RSSI/SNR from G2)
        
        // 1. Write payload received from Ground2 (G2's view of D2's transmission to it)
        if (payloadFromG2 != NULL && payloadFromG2Size > 0) {
            LoRa.write(payloadFromG2, payloadFromG2Size);
            Serial.print(" PyldG2:" + String(payloadFromG2Size) + "b");
        } else {
            Serial.print(" NoPyldG2");
        }

        // 2. Add current signal data (Drone2's view of Ground2's transmission to it)
        // This is the RSSI/SNR of the FINAL_ACK packet *as received by Drone2 from Ground2*
        int16_t d2_rssi_from_g2 = (int16_t)rssiFromG2;
        float d2_snr_from_g2 = snrFromG2;
        LoRa.write((d2_rssi_from_g2 >> 8) & 0xFF);
        LoRa.write(d2_rssi_from_g2 & 0xFF);
        uint8_t* snrBytes = (uint8_t*)&d2_snr_from_g2;
        LoRa.write(snrBytes, 4);
        Serial.print(" D2sig(G2):RSSI" + String(d2_rssi_from_g2) + "/SNR" + String(d2_snr_from_g2,1));

        // Brief delay before ending packet
        delay(10);
        
        sent = LoRa.endPacket();
        
        if (sent) {
            Serial.println(" SUCCESS on attempt " + String(attempts));
            break;
        } else {
            Serial.println(" FAILED on attempt " + String(attempts));
            if (attempts < MAX_ATTEMPTS) {
                Serial.println("  -> Retrying in 100ms...");
                delay(100); // Wait before retry
            }
        }
    }
    
    // Ensure some delay after transmission
    delay(20);
    
    safeLoRaTransmit(false);
    
    if (sent) {
        displayStatus("Completed Chain", "Forwarded ACK", "ID: " + String(msgId), "Batt: " + String(batteryVoltage, 2) + "V");
    } else {
        Serial.println("ERROR: Failed to forward Final ACK to Drone 1 after " + String(MAX_ATTEMPTS) + " attempts");
        displayStatus("ERROR FWD ACK", "To: Drone 1", "ID: " + String(msgId), "Send Failed!");
        delay(2000); // Show error for 2 seconds
        displayStatus("Drone 2 Relay", "LoRa Initialized", "Waiting for Data", "Batt: " + String(batteryVoltage, 2) + "V");
    }
} 