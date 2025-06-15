# LoraComm

# Drone Emergency Communication Network

## Overview
This project creates a resilient emergency communication network using ESP32 LoRa modules deployed on drones and ground stations. The system enables emergency message transmission when traditional networks are unavailable, with comprehensive signal quality monitoring throughout the transmission chain.

## System Components
1. **Ground Transceiver 1**: Serves as entry point, connecting to a TP-Link router or creating a WiFi access point, providing web interface for users to submit emergency messages
2. **Drone 1**: Flying drone that receives messages from Ground 1 and forwards to Drone 2
3. **Drone 2**: Relay drone that forwards messages to Ground Transceiver 2
4. **Ground Transceiver 2**: Final receiver that confirms message reception and sends signal data back through the chain
5. **iOS App**: Emergency message submission interface with real-time transmission status tracking

## Hardware Requirements
- 4× TTGO LoRa32 ESP32 development boards with 0.96" OLED display (only needed on Ground 1)
- Micro-USB cables for programming
- 4× LiPo batteries (3.7V) with JST connectors for field deployment
- TP-Link WR802N router for extended coverage (optional)
- External antennas for extended range (optional)

## Software Requirements
- Arduino IDE 1.8.x or newer
- ESP32 board support package (by Espressif Systems)
- Required libraries:
  - LoRa (by Sandeep Mistry)
  - Adafruit SSD1306 (for OLED display)
  - Adafruit GFX Library
  - Wire (built-in)
  - SPI (built-in)
  - WiFi (built-in)
  - WebServer (built-in)
  - ArduinoJson (for Ground Transceiver 1)
- Xcode 14.0+ for iOS app development

## Setup Instructions

### 1. Install Required Libraries
In Arduino IDE:
1. Go to Tools → Manage Libraries
2. Search for and install:
   - "LoRa" by Sandeep Mistry
   - "Adafruit SSD1306"
   - "Adafruit GFX Library"
   - "ArduinoJson"

### 2. Install ESP32 Board Support
1. Go to File → Preferences
2. Add this URL to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Go to Tools → Board → Boards Manager
4. Search for "esp32" and install "ESP32 by Espressif Systems"

### 3. Configure Board Settings
For each ESP32 board:
1. Select Tools → Board → ESP32 Arduino → ESP32 Dev Module
2. Set Flash Mode to "QIO" (default)
3. Set Upload Speed to "115200" (more reliable)
4. Select the correct port when the device is connected

### 4. Upload Sketches
Upload each sketch to its corresponding board:
1. GroundTransceiver1.ino → First ground station (with OLED)
2. Drone1.ino → First drone
3. Drone2.ino → Second drone (relay)
4. GroundTransceiver2.ino → Second ground station

## Network Configuration

### Option 1: Direct SoftAP Mode
Ground Transceiver 1 can operate in standalone mode creating its own WiFi network:
- SSID: "Emergency_KU"
- IP Address: 192.168.4.1
- No password (open network for emergency access)

### Option 2: TP-Link Router Integration (Recommended)
For better coverage and device compatibility:
1. Configure the TP-Link WR802N router:
   - Set operation mode to "Access Point"
   - SSID: "Emergency_KU"
   - Set LAN IP to 192.168.1.1
   - Reserve 192.168.1.2 for Ground Transceiver 1 via MAC address
2. Ground Transceiver 1 will connect to this router and get the static IP 192.168.1.2

## iOS App Setup

### 1. Open the Project in Xcode
1. Navigate to the EmergencyApp directory
2. Open EmergencyMessenger.xcodeproj in Xcode

### 2. Configure App for Your Device
1. Select your team (Apple ID) under Signing & Capabilities
2. Connect your iPhone via USB
3. Select your device in the deployment target dropdown

### 3. Build and Run
1. Press Cmd+R or click the build and run button
2. When prompted on your device, trust the developer certificate

### 4. Using the App
1. Connect your iPhone to "Emergency_KU" WiFi
2. Open the Emergency Messenger app
3. Fill in all required details
4. Tap "SEND SOS" and monitor transmission status
   - Orange indicator: Transmitting/Awaiting confirmation
   - Green indicator: Message delivered successfully
   - Red indicator: Transmission failed (can retry)

For detailed app instructions, see [EmergencyApp/README.md](EmergencyApp/README.md)

## Usage Instructions

### Ground Transceiver 1
1. Power on the device
2. Connect to the WiFi network "Emergency_KU"
3. Open a web browser and navigate to:
   - 192.168.4.1 (if using SoftAP mode)
   - 192.168.1.2 (if using TP-Link router)
4. Fill out the emergency form with:
   - Your name
   - Location description
   - Location coordinates (automatically filled if browser allows)
   - Emergency type
   - Message details
5. Submit the form and monitor transmission status

### Monitoring and Status
Ground Transceiver 1 provides three web interfaces:
- http://[ip]/ - Emergency message submission form
- http://[ip]/status - Message status dashboard with timing and signal quality
- http://[ip]/network - Network status dashboard with signal strength monitoring
- http://[ip]/message-status?id=[msgId] - JSON API for app status checking

### Drone 1 & Drone 2
1. Power on both drones and place them within range of their respective communication partners
2. The system will automatically relay messages when received
3. Signal quality data will be gathered and passed through the chain

### Ground Transceiver 2
1. Power on the device
2. It will automatically receive messages from Drone 2, send acknowledgments, and include signal data
3. When a message is received, details will be output to the serial monitor

## Transmission Process & Status Tracking

### Complete Message Flow
1. User submits message via iOS app or web form
2. Ground Transceiver 1 sends to Drone 1 via LoRa
3. Drone 1 acknowledges receipt and forwards to Drone 2
4. Drone 2 acknowledges receipt and forwards to Ground Transceiver 2
5. Ground Transceiver 2 sends final acknowledgment with signal data
6. Acknowledgment flows back through the chain to Ground Transceiver 1
7. Ground Transceiver 1 notifies iOS app of delivery completion

### Status Tracking Features
- Web dashboard shows real-time message journey tracking
- Signal quality visualization at each hop
- iOS app polls for status every 2 seconds
- Full transmission timeout after 30 seconds
- Automatic retry mechanism for failed transmissions
- Signal strength history graphs for network optimization

## Troubleshooting

### Common Issues
1. **LoRa initialization failure**: Ensure SPI pins are correctly defined for your board
2. **Transmission not working**: Check that all devices are within range (~1-2km line of sight)
3. **Cannot connect to WiFi**: Verify the router is working or restart Ground Transceiver 1
4. **OLED not displaying**: Verify I2C pins are correct (SDA=21, SCL=22 for TTGO LoRa32)
5. **iOS app cannot connect**: Ensure your iPhone is connected to the "Emergency_KU" WiFi
6. **No status updates**: Check that all devices in the chain are powered and functioning

### Serial Monitor
All devices output detailed status information to the Serial Monitor (115200 baud). Connect to monitor transmission status, signal quality, and debug information.

## Message Format
Each emergency message contains:
- Text content (user message)
- Location coordinates (latitude/longitude)
- Emergency code (type of emergency)
- Message ID (unique identifier)

## Signal Monitoring
The system tracks signal quality at every hop:
- RSSI (Received Signal Strength Indicator) in dBm
- SNR (Signal-to-Noise Ratio) in dB
- Complete signal chain visualization in the web dashboard
- Historical signal strength trends

## Testing
See the `integration_guide.md` file for detailed testing procedures and our `LoRaTest` sketches for simple communication tests.

## GitHub Repository

This project is maintained on GitHub at: https://github.com/akgurbuz13/Emergency-Drone-Network.git

To push code changes:
1. Open Terminal
2. Navigate to your LoraComm directory
3. Run these commands:
   ```
   git add .
   git commit -m "Updated code with transmission status tracking"
   git push origin main
   ```

If push doesn't complete, try:
1. Verify your GitHub credentials
2. Check your internet connection
3. Try using a personal access token if 2FA is enabled:
   ```
   git remote set-url origin https://[USERNAME]:[TOKEN]@github.com/akgurbuz13/Emergency-Drone-Network.git
   ```

## Future Enhancements
- GPS integration for automatic drone location tracking
- Battery voltage monitoring
- Multiple message queue support
- Encryption for secure communication
- Support for message priorities
- Image and audio transmission capabilities
