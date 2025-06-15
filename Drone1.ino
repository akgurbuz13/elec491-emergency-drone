/*
 * Drone 1 Relay - TTGO T-Beam v1.1
 *
 * Receives messages from Ground1 (ID 1) and forwards them to Drone2 (ID 3).
 * Receives Final ACKs from Drone2 (ID 3, originating from Ground2 ID 4)
 * and forwards them back to Ground1 (ID 1).
 *
 * NO local ACKs are sent or expected.
 * Uses LoRa parameters compatible with GroundTransceiver1.
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

// LoRa parameters (MUST match Ground1)
#define LORA_SF 7
#define LORA_BANDWIDTH 250E3
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0xF3
#define LORA_TX_POWER 20

// Battery monitoring
#define BATTERY_PIN 35

// Device IDs
#define GROUND1_ID 1
#define DRONE1_ID 2 // This device
#define DRONE2_ID 3
#define GROUND2_ID 4 // Originator of Final ACK

// Message types
#define MSG_DATA 0x00
#define MSG_ACK 0x01       // Defined but NOT used by Drone1
#define MSG_FINAL_ACK 0x03 // Relayed by Drone1

// Message structure (MUST match Ground1 and others)
struct EmergencyMessage {
    char text[128];
    float latitude;
    float longitude;
    int emergencyCode;
    uint32_t messageId; // Ensure this matches sender's structure
};

// Global variables
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
float batteryVoltage = 0;
unsigned long lastBatteryCheck = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 30000; // 30 seconds

// Buffer for final ACK payload
#define MAX_FINAL_ACK_PAYLOAD 30 // Max expected size (G2_RSSI(2)+SNR(4) + D2_RSSI(2)+SNR(4) + D1_RSSI(2)+SNR(4)) = 18. Add buffer.
byte finalAckPayloadBuffer[MAX_FINAL_ACK_PAYLOAD];

// Variables for managing ACK expectation from Drone 2
uint32_t waitingForD2AckMsgId = 0;      // Message ID Drone 1 is currently expecting an ACK for from Drone 2
unsigned long timeForwardedToD2 = 0; // Timestamp when the message was forwarded to Drone 2
const unsigned long D2_ACK_TIMEOUT = 4000; // Reduced to 4 seconds for Drone 2 ACK

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
void forwardToDrone2(EmergencyMessage msg, uint32_t msgId, int rssi, float snr);
void forwardFinalAckToGround1(uint32_t msgId, byte *payload, int payloadSize, int rssi, float snr);
void safeLoRaTransmit(bool enableTransmit);

void setup() {
    Serial.begin(115200);
    // Remove Serial wait - it can block devices without serial connection
    delay(100); // Short delay only
    Serial.println("Drone 1 Relay Booting Up...");

    // Initialize I2C first
    Wire.begin(OLED_SDA, OLED_SCL);
    delay(10); // Short delay for I2C bus stability

    Serial.println("Attempting AXP192 Init FIRST...");
    initAXP192(); // Call this before OLED

    Serial.println("Attempting OLED Init...");
    initOLED();

    initLoRa();
    updateBatteryStatus(); // Get initial battery voltage

    displayStatus("Drone 1 Relay", "LoRa Initialized", "Waiting for Data", "Batt: " + String(batteryVoltage, 2) + "V");
    Serial.println("Drone 1 Relay Initialized. Waiting for messages...");
    Serial.println("=============================================");
}

void loop() {
    int packetSize = LoRa.parsePacket();
    scanCount++;
    
    if (packetSize >= 7) { // MINIMUM_HEADER_SIZE = 7 bytes. Process if potentially valid.
        lastPacketTime = millis();
        processPacket(packetSize); 
    } else if (packetSize > 0 && packetSize < 7) { // Runt packet detected by LoRa.parsePacket()
        Serial.print("RUNT PACKET detected by LoRa.parsePacket(). Size: " + String(packetSize) + ". Attempting to clear...");
        int clearedBytes = 0;
        unsigned long runtClearStart = millis();
        while(LoRa.available() && clearedBytes < packetSize && (millis() - runtClearStart < 20)) { // Read at most packetSize or timeout
             LoRa.read();
             clearedBytes++;
        }
        Serial.println(" Cleared " + String(clearedBytes) + " byte(s).");
        safeLoRaTransmit(false); // Ensure back in RX mode
    } else { // packetSize is 0 or less (error from parsePacket, or no packet)
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

    // Update battery status periodically
    if (millis() - lastBatteryCheck > BATTERY_CHECK_INTERVAL) {
        updateBatteryStatus();
        lastBatteryCheck = millis();
        // Update display only if idle or not specifically waiting for D2 ACK
        if (waitingForD2AckMsgId == 0) { 
            display.fillRect(0, 3 * 10, SCREEN_WIDTH, 10, SSD1306_BLACK); // Clear line 4 area
            display.setCursor(0, 3 * 10); 
            display.print("Batt: " + String(batteryVoltage, 2) + "V");
            display.display();
        }
    }
    
    // Heartbeat output
    if (millis() - lastHeartbeatTime > HEARTBEAT_INTERVAL) {
        Serial.println("D1 HEARTBEAT: Actively listening. Scans: " + String(scanCount));
        Serial.println("  LoRa: SF=" + String(LORA_SF) + ", BW=" + String(LORA_BANDWIDTH/1000) + "kHz, CR=4/" + String(LORA_CODING_RATE));
        Serial.println("  Last packet: " + (lastPacketTime > 0 ? String((millis() - lastPacketTime)/1000) + "s ago" : "Never"));
        Serial.println("  Waiting for D2 ACK: " + String(waitingForD2AckMsgId > 0 ? "Yes (ID: " + String(waitingForD2AckMsgId) + ")" : "No"));
        
        scanCount = 0;
        lastHeartbeatTime = millis();
    }
    
    // Check for Drone 2 ACK timeout with improved reporting
    if (waitingForD2AckMsgId != 0) {
        unsigned long waitingTime = millis() - timeForwardedToD2;
        
        // Check for timeout
        if (waitingTime > D2_ACK_TIMEOUT) {
            uint32_t timedOutMsgId = waitingForD2AckMsgId; // Store ID before clearing
            waitingForD2AckMsgId = 0; // Reset waiting state FIRST
            Serial.println("TIMEOUT waiting for ACK from D2 for MsgID: " + String(timedOutMsgId) + 
                        " after " + String(waitingTime / 1000) + " seconds");
            displayStatus("D2 ACK Timeout", "MsgID: " + String(timedOutMsgId), "Resetting...", "Batt: " + String(batteryVoltage, 2) + "V");
            delay(1000); // Show timeout message for 1 second
            displayStatus("Drone 1 Relay", "LoRa Initialized", "Waiting for Data", "Batt: " + String(batteryVoltage, 2) + "V");
            safeLoRaTransmit(false); // ADDED: Ensure LoRa is in receive mode after timeout
        }
    }

    // Yield for ESP32 background tasks
    yield();
}

void initAXP192() {
    // Wire.begin(OLED_SDA, OLED_SCL); // REMOVED - Moved to setup()
    if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        Serial.println("WARNING: AXP192 Init Failed!");
        Serial.println("  - Will attempt to use default power configuration");
        Serial.println("  - Battery monitoring may be unavailable");
    } else {
        Serial.println("AXP192 Initialized Successfully!");
        // Power on essential components
        axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // LORA radio
        Serial.println("  - AXP192: LoRa radio power (LDO2) enabled");
        
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS (keep on for potential future use)
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
}

void initOLED() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("FATAL: SSD1306 Allocation Failed");
        while (true); // Stop execution
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
    LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN); // Use PA_BOOST for T-Beam
    LoRa.enableCrc();

    // Start listening for packets
    safeLoRaTransmit(false); // Ensure we are in receive mode
    Serial.println("LoRa Initialized OK.");
}

void updateBatteryStatus() {
    if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) != AXP_FAIL) {
        if (axp.isBatteryConnect()) {
            batteryVoltage = axp.getBattVoltage() / 1000.0;
        } else {
            batteryVoltage = 3.7; // Default value when no battery detected
        }
    } else {
        // Fallback to ADC pin if AXP fails - less reliable on T-Beam v1.1
        batteryVoltage = analogRead(BATTERY_PIN) * 2.0 * 3.3 / 4095.0;
        // Basic sanity check for ADC fallback
        if (batteryVoltage < 2.0) batteryVoltage = 3.7; // Default to typical LiPo voltage
    }
    Serial.println("Battery Voltage: " + String(batteryVoltage, 2) + "V");
}

void displayStatus(String line1, String line2, String line3, String line4) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(line1);
    display.setCursor(0, 10);
    display.println(line2);
    display.setCursor(0, 20);
    display.println(line3);
    display.setCursor(0, 30);
    display.println(line4);
    display.display();
}

// Safely switch LoRa mode and ensure it's back in receive after transmit
void safeLoRaTransmit(bool enableTransmit) {
    if (enableTransmit) {
        LoRa.idle(); // Go to standby mode before transmitting
        delay(20); // Changed from 10ms for stability
    } else {
        delay(20); // Changed from 10ms for stability
        LoRa.receive(); // Put back into continuous receive mode
    }
}

void processPacket(int packetSize) {
    unsigned long receiveTime = millis();
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

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

    Serial.print("RECV [" + String(receiveTime) + "ms] ");
    Serial.print("From: " + String(source) + " ");
    Serial.print("To: " + String(destination) + " ");
    Serial.print("Type: 0x" + String(msgType, HEX) + " ");
    Serial.print("ID: " + String(msgId) + " ");
    Serial.print("RSSI: " + String(rssi) + " SNR: " + String(snr, 1));
    Serial.println(" Size: " + String(packetSize));
    
    // Flush serial output to ensure logs are visible immediately
    Serial.flush();

    // --- Process based on Destination ---
    if (destination != DRONE1_ID) {
        Serial.println("  -> Packet not for me. Ignoring.");
        safeLoRaTransmit(false); // Ensure back in RX mode even if ignored
        return;
    }

    // --- Process based on Source and Type ---
    if (source == GROUND1_ID && msgType == MSG_DATA) {
        // Data from Ground1 -> Forward to Drone2
        Serial.println("  -> DATA received from Ground 1.");
        
        // Check if we're currently waiting for an ACK from Drone 2 for a previous message
        if (waitingForD2AckMsgId != 0) {
            Serial.println("  -> NOTE: Still waiting for ACK from D2 for previous MsgID: " + String(waitingForD2AckMsgId));
            Serial.println("  -> Will now handle new message and abandon previous wait state");
            // Reset the waiting state since we're handling a new message
            waitingForD2AckMsgId = 0;
        }
        
        displayStatus("DATA Received", "From: Ground 1", "ID: " + String(msgId), "RSSI: " + String(rssi));

        if (packetSize < (7 + sizeof(EmergencyMessage))) {
             Serial.println("ERROR: DATA packet too small for EmergencyMessage payload. Size: " + String(packetSize));
             safeLoRaTransmit(false);
             return;
        }

        EmergencyMessage receivedMsg;
        // Check how many bytes are actually available for the payload
        int payloadAvailable = LoRa.available();
        
        // Added more detailed logging for payload size
        Serial.println("  -> Payload bytes available: " + String(payloadAvailable) + " vs expected: " + String(sizeof(EmergencyMessage)));
        
        if (payloadAvailable != sizeof(EmergencyMessage)) {
            Serial.println("ERROR: Expected payload size " + String(sizeof(EmergencyMessage)) + " but got " + String(payloadAvailable) + ". Ignoring.");
             // Read remaining bytes to clear buffer
            while(LoRa.available()) { LoRa.read(); }
            safeLoRaTransmit(false);
            return;
        }
        
        LoRa.readBytes((uint8_t *)&receivedMsg, sizeof(EmergencyMessage));

        // Basic validation
        if (receivedMsg.messageId != msgId) {
            Serial.println("ERROR: Header MsgID (" + String(msgId) + ") != Payload MsgID (" + String(receivedMsg.messageId) + "). Ignoring.");
            safeLoRaTransmit(false);
            return;
        }
        
        Serial.println("  -> Payload MsgID: " + String(receivedMsg.messageId));
        Serial.println("  -> Text: " + String(receivedMsg.text)); // Print first part for verification

        // Flush serial before forwarding
        Serial.flush(); 

        // Set ACK expectation state BEFORE starting the relay attempts for this messageId
        waitingForD2AckMsgId = msgId; 
        timeForwardedToD2 = millis();

        // Forward the message immediately, with redundancy
        forwardToDrone2(receivedMsg, msgId, rssi, snr);

    } else if (source == DRONE2_ID && msgType == MSG_FINAL_ACK) {
        // Final ACK from Drone2 -> Forward to Ground1
        Serial.println("  -> FINAL ACK received from Drone 2.");

        // Check if this is the ACK we were specifically waiting for
        if (msgId == waitingForD2AckMsgId) {
            Serial.println("  -> This is the expected FINAL ACK from D2 for MsgID: " + String(msgId));
            waitingForD2AckMsgId = 0; // Clear the waiting state for this specific message
        } else if (waitingForD2AckMsgId != 0) {
            Serial.println("  -> Received FINAL ACK from D2 for MsgID: " + String(msgId) + ", but was expecting for MsgID: " + String(waitingForD2AckMsgId));
            // For reliability, clear the waiting state anyway to avoid getting stuck
            Serial.println("  -> Clearing wait state anyway to avoid getting stuck");
            waitingForD2AckMsgId = 0;
        } else {
            Serial.println("  -> Received an unsolicited/late FINAL ACK from D2 for MsgID: " + String(msgId));
        }

        displayStatus("FINAL ACK Recv D2", "From: Drone 2", "ID: " + String(msgId), "RSSI: " + String(rssi));

        // Brief delay to ensure we're ready to process
        delay(10);
        
        // Read the remaining payload (signal strength data)
        int payloadSize = LoRa.available();
        if (payloadSize > 0 && payloadSize <= MAX_FINAL_ACK_PAYLOAD) {
            LoRa.readBytes(finalAckPayloadBuffer, payloadSize);
            Serial.println("  -> Read " + String(payloadSize) + " bytes of signal data payload.");
            // Forward the final ACK + payload
            forwardFinalAckToGround1(msgId, finalAckPayloadBuffer, payloadSize, rssi, snr);
        } else if (payloadSize > MAX_FINAL_ACK_PAYLOAD) {
             Serial.println("ERROR: FINAL ACK payload too large (" + String(payloadSize) + " bytes). Ignoring.");
              // Read remaining bytes to clear buffer
             while(LoRa.available()) { LoRa.read(); }
             safeLoRaTransmit(false);
        } else {
            Serial.println("WARNING: FINAL ACK received with no signal data payload.");
            // Forward anyway, but without payload
            forwardFinalAckToGround1(msgId, NULL, 0, rssi, snr);
        }

    } else {
        Serial.println("  -> Unknown Source/Type combination. Ignoring.");
         // Read remaining bytes to clear buffer
        while(LoRa.available()) { LoRa.read(); }
        safeLoRaTransmit(false); // Ensure back in RX mode
    }
}

void forwardToDrone2(EmergencyMessage msg, uint32_t msgId, int rssi, float snr) {
    unsigned long sendTime = millis(); // Timestamp for the start of this relay operation
    Serial.println("FORWARDING TO D2: MsgID " + String(msgId));
    displayStatus("Forwarding to D2", "ID: " + String(msgId), "RSSI G1: " + String(rssi));

    // Flush serial before transmitting
    Serial.flush();

    safeLoRaTransmit(true); // Prepare for transmission
    
    // Add additional delay for stability
    delay(20);
    
    // Retry mechanism for better reliability
    bool packetSent = false;
    int attempts = 0;
    const int MAX_ATTEMPTS = 3;
    
    while (!packetSent && attempts < MAX_ATTEMPTS) {
        attempts++;
        
        LoRa.beginPacket();
        // Header
        LoRa.write(DRONE2_ID);       // Destination
        LoRa.write(DRONE1_ID);       // Source (This device)
        LoRa.write(MSG_DATA);        // Type
        LoRa.write((msgId >> 24) & 0xFF); // Message ID
        LoRa.write((msgId >> 16) & 0xFF);
        LoRa.write((msgId >> 8) & 0xFF);
        LoRa.write(msgId & 0xFF);
        // Payload
        int bytesWritten = LoRa.write((uint8_t *)&msg, sizeof(EmergencyMessage));
        Serial.print("    -> Payload bytes written: " + String(bytesWritten) + " of " + String(sizeof(EmergencyMessage)));
    
        // Brief delay before ending packet to ensure radio is ready
        delay(10);
    
        packetSent = LoRa.endPacket();
        
        if (packetSent) {
            Serial.println(" | FWD->D2: SUCCESS on attempt " + String(attempts));
            break;
        } else {
            Serial.println(" | ATTEMPT " + String(attempts) + " FAILED");
            if (attempts < MAX_ATTEMPTS) {
                Serial.println("    -> Retrying in 100ms...");
                delay(100); // Wait before retry
            }
        }
    }
    
    // Ensure some delay after transmission
    delay(20);
    
    // Immediately return to listen mode
    safeLoRaTransmit(false);
    
    if (packetSent) {
        // Update display to show waiting for ACK from D2
        displayStatus("Awaiting ACK D2", "From: Drone 2", "ID: " + String(msgId), "Batt: " + String(batteryVoltage, 2) + "V");
        Serial.println("  -> Now waiting for ACK from Drone 2 (timeout: " + String(D2_ACK_TIMEOUT/1000) + "s)");
    } else {
        // Handle the failure case
        Serial.println("ERROR: Failed to forward message to Drone 2 after " + String(MAX_ATTEMPTS) + " attempts");
        displayStatus("ERROR", "Failed to send", "to Drone 2", "Batt: " + String(batteryVoltage, 2) + "V");
        delay(2000); // Show error for 2 seconds
        displayStatus("Drone 1 Relay", "LoRa Initialized", "Waiting for Data", "Batt: " + String(batteryVoltage, 2) + "V");
        
        // Since we couldn't send, clear the waiting state
        waitingForD2AckMsgId = 0;
    }
}

void forwardFinalAckToGround1(uint32_t msgId, byte *payload, int payloadSize, int rssi, float snr) {
    unsigned long sendTime = millis();
    Serial.print("FWD_ACK->G1 [" + String(sendTime) + "ms] ");
    Serial.print("To: " + String(GROUND1_ID) + " ");
    Serial.println("ID: " + String(msgId));
    displayStatus("Forwarding ACK", "To: Ground 1", "ID: " + String(msgId), "Prev RSSI: " + String(rssi));

    safeLoRaTransmit(true); // Prepare for transmission

    LoRa.beginPacket();
    // Header
    LoRa.write(GROUND1_ID);      // Destination
    LoRa.write(DRONE1_ID);       // Source (This device)
    LoRa.write(MSG_FINAL_ACK);   // Type
    LoRa.write((msgId >> 24) & 0xFF); // Message ID
    LoRa.write((msgId >> 16) & 0xFF);
    LoRa.write((msgId >> 8) & 0xFF);
    LoRa.write(msgId & 0xFF);

    // Signal Payload (if provided)
    if (payload != NULL && payloadSize > 0) {
        LoRa.write(payload, payloadSize);
        Serial.println("  -> Including " + String(payloadSize) + " bytes of signal payload.");
    }

    // Append *this* drone's signal data (RSSI/SNR from Drone 2)
    // This is the signal quality of the FINAL_ACK packet as received by Drone 1 from Drone 2.
    int16_t d1_rssi = (int16_t)rssi;
    float d1_snr = snr;
    LoRa.write((d1_rssi >> 8) & 0xFF);
    LoRa.write(d1_rssi & 0xFF);
    uint8_t* snrBytes = (uint8_t*)&d1_snr;
    LoRa.write(snrBytes, 4);
    Serial.println("  -> Adding D1 signal: RSSI " + String(d1_rssi) + ", SNR " + String(d1_snr, 1));
    // Note: This increases packet size by 6 bytes. Ground1 must parse this.

    // Add brief delay before ending packet to ensure radio is ready
    delayMicroseconds(200);

    bool sent = LoRa.endPacket();
    
    if (sent) {
        Serial.println("  -> FWD_ACK->G1: Success.");
        displayStatus("Completed Chain", "ID: " + String(msgId), "RSSI: " + String(rssi), "Batt: " + String(batteryVoltage, 2) + "V");
    } else {
        Serial.println("ERROR: FWD_ACK->G1: Send Failed!");
        displayStatus("ERROR FWD ACK", "To: Ground 1", "ID: " + String(msgId), "Send Failed!");
        
        // Try once more with a delay
        Serial.println("  -> Retrying Final ACK transmission to Ground1...");
        delay(100);
        LoRa.beginPacket();
        // Header
        LoRa.write(GROUND1_ID);      // Destination
        LoRa.write(DRONE1_ID);       // Source (This device)
        LoRa.write(MSG_FINAL_ACK);   // Type
        LoRa.write((msgId >> 24) & 0xFF); // Message ID
        LoRa.write((msgId >> 16) & 0xFF);
        LoRa.write((msgId >> 8) & 0xFF);
        LoRa.write(msgId & 0xFF);
        
        // Signal Payload (if provided)
        if (payload != NULL && payloadSize > 0) {
            LoRa.write(payload, payloadSize);
        }
        
        // Append *this* drone's signal data
        int16_t d1_rssi = (int16_t)rssi;
        float d1_snr = snr;
        LoRa.write((d1_rssi >> 8) & 0xFF);
        LoRa.write(d1_rssi & 0xFF);
        uint8_t* snrBytes = (uint8_t*)&d1_snr;
        LoRa.write(snrBytes, 4);
        
        delay(10);
        sent = LoRa.endPacket();
        
        if (sent) {
            Serial.println("  -> FWD_ACK->G1: Success on retry.");
        } else {
            Serial.println("  -> FWD_ACK->G1: Failed on retry too!");
        }
    }
    
    delayMicroseconds(500);
    safeLoRaTransmit(false); // Go back to receive mode
} 