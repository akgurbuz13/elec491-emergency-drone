# LoRa Test Sketches

These minimal test sketches allow you to quickly verify LoRa communication between two ESP32 LoRa boards before implementing the full Drone Emergency Communication Network.

## Files

- `LoRaTestSender.ino` - Upload to Ground Transceiver 1 ESP32
- `LoRaTestReceiver.ino` - Upload to Drone 1 ESP32

## Hardware Requirements

- 2× TTGO LoRa32 ESP32 development boards with 0.96" OLED display
- USB cables for programming
- Power source (USB or batteries)

## Setup Instructions

### 1. Install Required Libraries
In Arduino IDE:
1. Go to Tools → Manage Libraries
2. Search for and install:
   - "LoRa" by Sandeep Mistry
   - "Adafruit SSD1306"
   - "Adafruit GFX Library"

### 2. Upload Sketches
1. Connect the first ESP32 board via USB
2. Select the correct board and port
3. Upload `LoRaTestSender.ino`
4. Connect the second ESP32 board
5. Upload `LoRaTestReceiver.ino`

## Test Procedure

1. Power both boards (can be via USB)
2. The sender board will:
   - Display "Ready to send" on the OLED
   - Automatically send a test message every 5 seconds
   - The OLED will briefly show "Message sent!" after each transmission

3. The receiver board will:
   - Display "Waiting for message..." on the OLED
   - When a message is received, display the message details with signal strength
   - Automatically send back an acknowledgment to the sender

## Expected Results

- The sender will send regular test messages
- The receiver will display received messages with RSSI and SNR values
- Both boards will show status information on their OLED displays
- Details will also be output to the Serial Monitor (115200 baud)

## Troubleshooting

- **No messages received**: Ensure both boards are correctly programmed and powered
- **LoRa initialization error**: Check SPI pin definitions match your board model
- **Display not working**: Verify I2C pins (SDA=21, SCL=22 by default)
- **Poor signal quality**: Try moving the boards closer together initially

## Next Steps

Once basic LoRa communication is verified, you can proceed with setting up the complete Drone Emergency Communication Network as described in the main project instructions.

## About the Test Message Format

- Standard header (destination, source, message type, message ID)
- Simple payload structure:
  - Text message (32 bytes max)
  - Counter (incrementing number for each message)
  - Timestamp (milliseconds since boot)

This approach matches the message header format used in the full system to ensure compatibility. 