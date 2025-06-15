#!/usr/bin/env python3
import os
import sys
import time
import json
import argparse
import serial
import threading
import queue
import datetime
import logging
from typing import Dict, Any, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lora_bridge.log"))
    ]
)

logger = logging.getLogger('LoRa Bridge')

# Message structure matching the GroundTransceiver1 EmergencyMessage structure
class EmergencyMessage:
    def __init__(self, text: str, emergency_code: int, latitude: float = 0.0, longitude: float = 0.0):
        """
        Create an emergency message.
        
        Args:
            text: The emergency message text
            emergency_code: Emergency type code (1=fire, 2=damage, 3=other)
            latitude: Optional GPS latitude
            longitude: Optional GPS longitude
        """
        self.text = text[:127]  # Limit to 127 chars plus null terminator
        self.emergency_code = emergency_code
        self.latitude = latitude
        self.longitude = longitude
        self.message_id = int(time.time())  # Use timestamp as message ID
        
    def to_json(self) -> Dict[str, Any]:
        """Convert message to JSON format for LoRa transmission"""
        return {
            "text": self.text,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "emergencyCode": self.emergency_code,
            "messageId": self.message_id
        }
        
    def __str__(self) -> str:
        return f"EmergencyMessage(id={self.message_id}, code={self.emergency_code}, text='{self.text}')"


class LoraBridge:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0, 
                 msg_queue_size: int = 10):
        """
        Initialize the LoRa bridge.
        
        Args:
            port: Serial port connected to GroundTransceiver1
            baudrate: Serial port baudrate
            timeout: Serial timeout
            msg_queue_size: Size of the message queue
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.connected = False
        self.running = False
        
        # Message queue for sending
        self.msg_queue = queue.Queue(maxsize=msg_queue_size)
        
        # Thread for handling serial communication
        self.serial_thread = None
        
        # Message history
        self.message_history = []
        self.max_history = 50

        # ACK status tracking
        self.ack_statuses: Dict[int, Dict[str, Any]] = {} # message_id -> {status, timestamp, details}
        
    def connect(self) -> bool:
        """Connect to the LoRa module via serial port"""
        if self.connected:
            logger.info("Already connected to LoRa module")
            return True
            
        try:
            logger.info(f"Connecting to LoRa module on {self.port}...")
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.connected = True
            logger.info(f"Connected to LoRa module on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to LoRa module: {e}")
            return False
            
    def disconnect(self) -> None:
        """Disconnect from the LoRa module"""
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception as e:
                logger.error(f"Error closing serial connection: {e}")
            finally:
                self.serial_conn = None
                self.connected = False
                logger.info("Disconnected from LoRa module")
    
    def send_message(self, message: EmergencyMessage) -> bool:
        """
        Queue a message for sending to the LoRa module.
        
        Args:
            message: The emergency message to send
            
        Returns:
            bool: True if the message was queued successfully, False otherwise
        """
        try:
            # Add message to the queue with a timestamp
            self.msg_queue.put((message, time.time()), block=False)
            logger.info(f"Queued message: {message}")
            return True
        except queue.Full:
            logger.error("Message queue is full. Message not sent.")
            return False
    
    def _process_queue(self) -> None:
        """Process the message queue and send messages to LoRa module"""
        while self.running:
            try:
                # Try to get a message from the queue
                message, timestamp = self.msg_queue.get(block=True, timeout=1.0)
                
                # If connection is lost, try to reconnect
                if not self.connected:
                    if not self.connect():
                        logger.error("Failed to reconnect to LoRa module. Message not sent.")
                        self.msg_queue.put((message, timestamp))  # Put message back in queue
                        time.sleep(5)  # Wait before retrying
                        continue
                
                # Send the message
                try:
                    message_json = json.dumps(message.to_json()) + "\n"
                    self.serial_conn.write(message_json.encode())
                    self.serial_conn.flush()
                    
                    # Add to history and initialize ACK status
                    msg_id = message.message_id
                    self.message_history.append({
                        "message": message.to_json(),
                        "timestamp": datetime.datetime.now().isoformat(),
                        "status": "sent"
                    })
                    self.ack_statuses[msg_id] = {"status": "sent", "timestamp": time.time(), "details": {}}
                    
                    # Trim history if needed
                    if len(self.message_history) > self.max_history:
                        self.message_history = self.message_history[-self.max_history:]
                    
                    logger.info(f"Message sent: {message}")
                    
                except Exception as e:
                    logger.error(f"Error sending message: {e}")
                    # If there was an error, put the message back in the queue
                    if time.time() - timestamp < 60:  # Only retry messages less than 60 seconds old
                        self.msg_queue.put((message, timestamp))
                        self.connected = False  # Mark as disconnected to trigger reconnect
                
                # Mark the task as done
                self.msg_queue.task_done()
                
            except queue.Empty:
                # No messages in the queue
                pass
            except Exception as e:
                logger.error(f"Error in message processing thread: {e}")
                time.sleep(1)
                
    def _read_response(self) -> None:
        """Read and process responses from the LoRa module"""
        try:
            if self.connected and self.serial_conn and self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode('utf-8').strip()
                if response:
                    logger.info(f"Received from LoRa: {response}")
                    try:
                        # Try to parse as JSON
                        data = json.loads(response)
                        # Process the data if needed
                        if isinstance(data, dict) and data.get("type") == "ack_status":
                            msg_id = data.get("messageId")
                            status = data.get("status")
                            if msg_id is not None and status is not None:
                                self.ack_statuses[msg_id] = {
                                    "status": status,
                                    "timestamp": time.time(),
                                    "details": data # Store all details like rssi, snr
                                }
                                logger.info(f"ACK status updated for MsgID {msg_id}: {status}")
                            else:
                                logger.warning(f"Received malformed ACK status: {data}")
                        # You can add more parsers for other types of messages from ESP32 here
                    except json.JSONDecodeError:
                        # Not JSON, just log as plain text or handle as a simple string response
                        logger.debug(f"Received non-JSON serial data: {response}")
                    except Exception as e:
                        logger.error(f"Error processing received data: {e}")

        except Exception as e:
            logger.error(f"Error reading response: {e}")
            self.connected = False  # Mark as disconnected to trigger reconnect
    
    def start(self) -> bool:
        """Start the LoRa bridge"""
        if not self.connect():
            return False
            
        self.running = True
        self.serial_thread = threading.Thread(target=self._run, daemon=True)
        self.serial_thread.start()
        logger.info("LoRa Bridge started")
        return True
    
    def _run(self) -> None:
        """Main loop for the serial communication thread"""
        while self.running:
            try:
                # Process outgoing messages
                self._process_queue()
                
                # Read incoming responses
                self._read_response()
                
                # Small delay to prevent CPU hogging
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Error in run loop: {e}")
                time.sleep(1)
    
    def stop(self) -> None:
        """Stop the LoRa bridge"""
        self.running = False
        if self.serial_thread:
            self.serial_thread.join(timeout=2.0)
        self.disconnect()
        logger.info("LoRa Bridge stopped")
        
    def get_queue_status(self) -> Dict[str, Any]:
        """Get the status of the message queue"""
        return {
            "queue_size": self.msg_queue.qsize(),
            "queue_full": self.msg_queue.full(),
            "connected": self.connected,
            "running": self.running
        }
        
    def get_message_history(self) -> list:
        """Get the message history"""
        return self.message_history

    def get_ack_status(self, message_id: int) -> Optional[Dict[str, Any]]:
        """
        Get the ACK status for a given message ID.
        
        Args:
            message_id: The ID of the message to check.
            
        Returns:
            A dictionary with status info if found, else None.
            Example: {"status": "delivered", "timestamp": 1678886400, "details": {...}}
        """
        return self.ack_statuses.get(message_id)


# Main function to use the bridge directly
def main():
    parser = argparse.ArgumentParser(description="LoRa Bridge for Emergency Messages")
    parser.add_argument("--port", "-p", type=str, required=True, help="Serial port for LoRa module")
    parser.add_argument("--baudrate", "-b", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--message", "-m", type=str, help="Send a test message")
    parser.add_argument("--type", "-t", type=int, default=3, 
                        help="Emergency code (1=fire, 2=damage, 3=other)")
    args = parser.parse_args()
    
    bridge = LoraBridge(port=args.port, baudrate=args.baudrate)
    
    if args.message:
        # Send a test message
        bridge.connect()
        msg = EmergencyMessage(text=args.message, emergency_code=args.type)
        if bridge.send_message(msg):
            logger.info(f"Test message queued: {msg}")
            time.sleep(2)  # Wait for message to be sent
        bridge.disconnect()
        return
    
    # Start the bridge in continuous mode
    if bridge.start():
        try:
            # Interactive console for sending test messages
            print("\n============================================")
            print("LoRa Bridge is running")
            print("Enter message text or 'exit' to quit")
            print("============================================")
            
            while True:
                try:
                    text = input("Message (or 'exit'): ")
                    
                    if text.lower() == 'exit':
                        break
                    
                    if text.strip():
                        code = int(input("Emergency code (1=fire, 2=damage, 3=other): ") or "3")
                        msg = EmergencyMessage(text=text, emergency_code=code)
                        bridge.send_message(msg)
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"Error: {e}")
        
        finally:
            bridge.stop()
            print("Bridge stopped")


if __name__ == "__main__":
    main() 