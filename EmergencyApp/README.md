# Emergency SOS iOS App

This iOS app is part of the Drone Emergency Communication Network project. It enables users to send emergency messages through a local WiFi network to the Ground Transceiver 1, which then forwards the message through a chain of LoRa-equipped drones to reach emergency responders when traditional communication networks are unavailable.

## Features

- Send emergency SOS messages with location information
- Works on iOS 17+ (iPhone XS Max and newer devices)
- Connects to local WiFi network (no internet required)
- Includes user information, location data, emergency type, and custom message
- Real-time location updates
- User-friendly interface with status indicators
- **NEW**: Real-time transmission status tracking with visual feedback
- **NEW**: Automatic retry functionality for failed transmissions
- **NEW**: Complete message delivery confirmation through the entire drone network

## Integration with LoRa Network

The app communicates with the Ground Transceiver 1 ESP32 over local WiFi. It does not communicate directly with any LoRa device.

1. **Communication Flow**:
   - App sends HTTP POST request to Ground Transceiver 1
   - Ground Transceiver 1 forwards the message via LoRa to Drone 1
   - Drone 1 forwards to Drone 2, which forwards to Ground Transceiver 2
   - Acknowledgments flow back through the same chain
   - App continuously polls Ground Transceiver 1 for delivery status
   - When full delivery is confirmed, app shows success message

2. **Network Configuration**:
   - The app must be connected to the same WiFi network as Ground Transceiver 1
   - By default, the app is configured to connect to IP address 192.168.1.2
   - The network SSID should be "Emergency_KU" (open network with no password)
   - This assumes Ground Transceiver 1 is connected to a TP-Link router
   - For direct SoftAP mode, change the IP in NetworkManager to 192.168.4.1

## Transmission Status Tracking

The app provides real-time feedback on message delivery status:

1. **Visual Indicators**:
   - **Orange button with spinner**: Message is being transmitted or awaiting confirmation
   - **Green button with checkmark**: Message successfully delivered to final destination
   - **Red button with warning icon**: Transmission failed or timed out

2. **Status Flow**:
   - When the user taps "SEND SOS", the app shows "SENDING SOS..."
   - After initial confirmation from Ground Transceiver 1, changes to "WAITING FOR CONFIRMATION..."
   - App polls the status endpoint every 2 seconds to check delivery progress
   - Upon successful delivery through entire chain, shows green "MESSAGE DELIVERED"
   - If no confirmation after 30 seconds, times out and allows retry

3. **Timeout Handling**:
   - If no delivery confirmation is received within 30 seconds, user can retry
   - "Cancel and try again" option appears after timeout
   - No duplicate messages are sent unless user explicitly retries

## Setup Instructions for Development

### Prerequisites

- Mac with macOS Monterey (12.0) or later
- Xcode 14.0 or later
- iOS device running iOS 17 or later (for testing)
- Apple ID (free developer account is sufficient for development/testing)
- Ground Transceiver 1 ESP32 configured and running

### Installation Steps

1. **Clone the Repository**
   - Clone this repository to your local machine
   - Navigate to the `EmergencyApp` directory

2. **Open in Xcode**
   - Double-click the `EmergencyMessenger.xcodeproj` file, or open it from within Xcode
   - Wait for Xcode to index the project

3. **Configure Network Settings**
   - Open `EmergencyMessenger.swift` file
   - Locate the `NetworkManager` class (around line 45)
   - Update the `serverAddress` variable to match your Ground Transceiver 1's IP address:
     ```swift
     // For TP-Link router mode:
     private let serverAddress = "192.168.1.2"
     
     // For SoftAP mode (uncomment if needed):
     // private let serverAddress = "192.168.4.1"
     ```

4. **Configure Signing**
   - Select the project in the Project Navigator
   - Go to the "Signing & Capabilities" tab
   - Choose your Team (Apple ID)
   - Xcode will automatically manage the provisioning profile for development

5. **Set Bundle Identifier**
   - In the same "Signing & Capabilities" section, set a unique Bundle Identifier
   - Example: `com.yourname.emergencymessenger`

6. **Configure Permissions**
   - The app already includes the necessary permission requests in Info.plist:
     - `NSLocationWhenInUseUsageDescription` for GPS access
     - `NSLocalNetworkUsageDescription` for local network access
     - `NSAppTransportSecurity` with `NSAllowsLocalNetworking` for HTTP connections

7. **Build and Run**
   - Connect your iOS device via USB
   - Select your device from the device menu
   - Click the Run button (▶️) or press Cmd+R
   - When prompted, allow location and local network access permissions

### Testing Without Paid Developer Account

When using a free Apple Developer account:

1. **Trust Developer on Device**
   - The first time you run the app, go to Settings > General > Device Management on your iPhone
   - Find your Apple ID and tap "Trust"

2. **7-Day Limit**
   - The app will expire after 7 days with a free account
   - You'll need to reconnect to Xcode and rebuild to refresh the app

## Usage with the Emergency Network

1. **Connect to the Emergency Network**:
   - Connect your iPhone to the "Emergency_KU" WiFi network
   - This is either created by Ground Transceiver 1 (SoftAP mode) or by the TP-Link router
   - No internet connection is required

2. **Launch the Emergency SOS app**

3. **Fill in your emergency details**:
   - Your full name
   - Current location description (required even if GPS is available)
   - Emergency type (medical, fire, security, structural, or other)
   - Detailed message about the emergency

4. **Check Location Status**:
   - The app will automatically capture your GPS coordinates if available
   - GPS is helpful but not required - you can send with just a text location description

5. **Send the Message**:
   - Tap the "SEND SOS" button to transmit your emergency message
   - Watch the button change color to indicate transmission status
   - Wait for the green "MESSAGE DELIVERED" confirmation
   - If timeout occurs, you can tap "RETRY SEND" to attempt again

## Testing and Verification

1. **Local Network Testing**:
   - Connect to the "Emergency_KU" WiFi
   - Open a web browser and navigate to Ground Transceiver 1's IP
   - Verify the web interface loads correctly

2. **App Communication Test**:
   - Send a test message from the app
   - Check the Ground Transceiver 1 OLED display to confirm message reception
   - Check Ground Transceiver 1's web interface at `/status` to track message progress

3. **Full Chain Test**:
   - With all components powered, send a test message
   - Monitor the complete journey through the Ground Transceiver 1 status dashboard
   - Verify the app updates status correctly as the message moves through the system

## Troubleshooting

- **Cannot Connect to Network**: Ensure you're connected to the "Emergency_KU" WiFi
- **Location Not Available**: Go to Settings > Privacy > Location Services and ensure the app has permission
- **Cannot Send Messages**: Verify the ESP32 is powered on and connected to the same network
- **App Not Installing**: Ensure your Apple ID is trusted in device settings
- **Wrong IP Address**: If the app can't connect, verify the IP address in NetworkManager.swift matches your Ground Transceiver 1 configuration
- **Status Not Updating**: If status seems stuck, check that all devices in the chain are powered on and functioning

## Future Improvements

- Offline maps for better location awareness
- Message queue for when connectivity is intermittent
- Battery optimization for emergency scenarios
- Dark mode for power conservation
- Support for non-text media (photos, audio) in emergency messages 