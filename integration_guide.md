# Drone Emergency Communication Network - Integration Guide

This guide provides detailed instructions for setting up, integrating, and testing the complete Drone Emergency Communication Network system.

## System Overview

The system consists of:
1. **Ground Transceiver 1**: Connected to TP-Link WR802N router, provides web interface and iOS app connectivity
2. **Drone 1**: Flying drone that scans for emergencies
3. **Drone 2**: Relay drone
4. **Ground Transceiver 2**: Final receiver of emergency messages
5. **iOS App**: User interface for submitting emergency messages

## Key Features
- Complete end-to-end message delivery with acknowledgments
- Signal quality (RSSI/SNR) monitoring at every transmission hop
- Web-based monitoring dashboard with real-time status
- Signal strength visualization and history tracking
- iOS app integration for emergency message submission
- Automatic retry mechanism for reliable delivery

## Prerequisites

- 4× TTGO ESP32 LoRa development boards with OLED display (at least for Ground 1)
- 4× LiPo batteries (3.7V) with JST connectors
- TP-Link WR802N router
- Mac with Xcode 14+ for iOS app development
- iPhone XS Max or newer running iOS 17+
- USB cables for programming ESP32 boards
- Arduino IDE with ESP32 support and required libraries

## Setup Steps

### Step 1: Configure the TP-Link WR802N Router

1. Connect to the router's default admin interface (typically 192.168.0.1)
2. Set operation mode to "Access Point"
3. Configure the wireless settings:
   - SSID: "Emergency-Network"
   - Security: None (for easy emergency access) or WPA2 with simple password
   - Channel: Choose a clear channel (run a WiFi scanner to find the least congested)
4. Set LAN IP address to 192.168.1.1
5. Configure DHCP server:
   - IP Range: 192.168.1.100 - 192.168.1.200
   - Reserve 192.168.1.2 for Ground Transceiver 1 (based on its MAC address)

### Step 2: Program the ESP32 Boards

1. **Install Required Libraries in Arduino IDE**:
   - LoRa by Sandeep Mistry
   - Adafruit SSD1306
   - Adafruit GFX Library
   - ArduinoJson

2. **Upload Code to Boards**:
   - Connect each ESP32 board via USB
   - Select correct board (ESP32 Dev Module) and port
   - Configure upload speed to 115200 bps for reliability
   - Upload the appropriate sketch to each board:
     - `GroundTransceiver1.ino` → First ground station
     - `Drone1.ino` → First drone
     - `Drone2.ino` → Second drone (relay)
     - `GroundTransceiver2.ino` → Second ground station

3. **Verify LoRa Communication First**:
   - Before full integration, use the `LoRaTest` sketches to verify basic communication
   - This ensures hardware is working correctly before proceeding

### Step 3: Set Up iOS App

1. **Open Project in Xcode**:
   - Navigate to the `EmergencyApp` directory
   - Open `EmergencyMessenger.xcodeproj`

2. **Configure Signing**:
   - Select project in Navigator
   - Go to Signing & Capabilities tab
   - Select your Apple ID team
   - Update Bundle Identifier if needed

3. **Configure Network Settings**:
   - Open `NetworkManager.swift`
   - Verify the base URL points to Ground Transceiver 1's IP address:
     - `192.168.4.1` if using SoftAP mode
     - `192.168.1.2` if using TP-Link router

4. **Build and Install**:
   - Connect iPhone via USB
   - Select your device in the device menu
   - Click Run (▶) to build and install
   - Trust the developer profile on the iPhone if prompted

### Step 4: Physical Setup

1. **Position Equipment**:
   - Set up the TP-Link router and Ground Transceiver 1 at base location
   - Position Drone 1 where it can survey the target area
   - Position Drone 2 as a relay between Drone 1 and Ground Transceiver 2
   - Set up Ground Transceiver 2 at the final destination

2. **Power Setup**:
   - Connect LiPo batteries to all ESP32 boards
   - Power on the TP-Link router (via USB power bank if in field)

## Testing Procedure

### Phase 1: Basic LoRa Testing

1. Use the provided `LoRaTest` sketches to verify basic LoRa communication works:
   - Upload `LoRaTestSender.ino` to one ESP32
   - Upload `LoRaTestReceiver.ino` to another ESP32
   - Verify messages are transmitted and received correctly
   - Check OLED displays for confirmation

### Phase 2: WiFi Connectivity Test

1. Connect your phone to the "Emergency-Network" WiFi
2. Verify Ground Transceiver 1 is connected to WiFi (check OLED display)
3. Open a web browser and navigate to the appropriate IP:
   - http://192.168.4.1 (SoftAP mode)
   - http://192.168.1.2 (TP-Link router mode)
4. Verify the emergency form loads correctly

### Phase 3: iOS App Test

1. Launch the Emergency SOS app on your iPhone
2. Verify the app can detect your location
3. Send a test message with dummy information
4. Verify the confirmation screen appears

### Phase 4: LoRa Chain Test

1. Monitor the OLED display on Ground Transceiver 1 to confirm:
   - Message received from app
   - "Transmitting..." status
   - "ACK received from Drone 1" status

2. Check Serial Monitor output on all devices to confirm:
   - Drone 1 receives message and forwards to Drone 2
   - Drone 2 receives message and forwards to Ground Transceiver 2
   - Ground Transceiver 2 receives the message
   - Final ACK with signal data propagates back through the chain

3. Verify that the OLED on Ground Transceiver 1 shows "Message delivered!" when the full chain completes

### Phase 5: Signal Quality Monitoring

1. Access the monitoring dashboard:
   - Navigate to http://[ground1-ip]/status for message timing information
   - Navigate to http://[ground1-ip]/network for network status visualization

2. Send a test message through the system

3. Verify on the status page:
   - Message shows complete delivery status
   - Signal quality data from all hops is displayed
   - Timing information is accurately recorded

4. Verify on the network page:
   - All devices show as online
   - Signal strength indicators are active
   - Signal history is being recorded

### Phase 6: Range Testing

1. Gradually increase the distance between devices while repeating the messaging test
2. Identify maximum reliable range between each pair of devices
3. Monitor signal quality metrics as distance increases
4. Adjust positions or LoRa parameters as needed

## Troubleshooting

### WiFi Connectivity Issues
- Verify router power and configuration
- Check that Ground Transceiver 1 connects to WiFi or falls back to SoftAP mode
- Confirm IP addresses are correctly configured

### LoRa Transmission Issues
- Check that all devices are powered and initialized
- Verify antennas are properly connected
- Reduce distance between devices for initial testing
- Check signal quality metrics for poor connections
- Increase spreading factor (SF) for better range at the cost of speed

### iOS App Connection Problems
- Confirm iPhone is connected to the correct WiFi network
- Verify Ground Transceiver 1 IP address in NetworkManager class
- Check Local Network permission is granted to the app

## Signal Quality Interpretation

### RSSI (Received Signal Strength Indicator)
- Typical range: -30 dBm (excellent) to -120 dBm (very poor)
- Better than -70 dBm: Excellent connection
- -70 to -85 dBm: Good connection
- -86 to -100 dBm: Moderate connection
- Below -100 dBm: Poor connection, may be unreliable

### SNR (Signal-to-Noise Ratio)
- Typical range: 20 dB (excellent) to -20 dB (very poor)
- Above 5 dB: Excellent signal quality
- 0 to 5 dB: Good signal quality
- -10 to 0 dB: Marginal signal quality
- Below -10 dB: Poor signal quality, may be unreliable

## System Expansion

### Adding Automatic Emergency Detection
- Connect sensors (fire, gas, etc.) to Drone 1
- Modify Drone1.ino to periodically read sensors and auto-generate messages
- Update message struct to indicate detection source (human vs. automatic)

### Implementing Encrypted Communication
- Add AES-128 encryption to message payloads
- Distribute pre-shared keys to all devices
- Add encryption/decryption functions to the communication chain

## Maintenance

### Battery Management
- Monitor battery voltage periodically
- Set up low-voltage alerts or auto-shutdown
- Create a battery replacement schedule

### Software Updates
- Document current firmware versions for all devices
- Use OTA updates when possible (may require custom implementation)
- Keep backups of all code before making updates 