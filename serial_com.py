#!/usr/bin/env python3

"""
Direct MAVLink Serial Communication

This script implements direct serial communication with a Pixhawk,
sending and receiving raw MAVLink messages without using PyMAVLink.
It follows the logic of the MAVROS/MAVConn code.

Dependencies:
    - pyserial
"""

import argparse
import sys
import time
import threading
import struct
import logging
import serial
import binascii
import queue
from datetime import datetime

# MAVLink protocol constants
MAVLINK_STX = 0xFD  # Start of MAVLink v2 packet
MAVLINK_STX_V1 = 0xFE  # Start of MAVLink v1 packet
MAVLINK_MAX_PAYLOAD_LEN = 255

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('MAVLinkSerial')

class MAVLinkMessage:
    """Simple MAVLink message container"""
    def __init__(self, msgid=0, payload=b'', sysid=0, compid=0, seq=0):
        self.msgid = msgid
        self.payload = payload
        self.sysid = sysid
        self.compid = compid
        self.seq = seq
        self.len = len(payload)
        self.magic = MAVLINK_STX  # Default to MAVLink v2

class MAVLinkSerialComm:
    def __init__(self, device, baudrate, system_id=0, component_id=0):
        """
        Initialize MAVLink serial communication
        
        Args:
            device (str): Serial device path
            baudrate (int): Serial baudrate
            system_id (int): Our system ID
            component_id (int): Our component ID
        """
        self.device = device
        self.baudrate = baudrate
        self.system_id = system_id
        self.component_id = component_id
        
        # Serial connection
        self.serial_dev = None
        
        # Queues for message sending
        self.tx_queue = queue.Queue(maxsize=100)  # Limit queue size
        
        # Threading control
        self.is_running = False
        self.tx_in_progress = False
        self.lock = threading.RLock()
        
        # Statistics
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.rx_messages = 0
        self.tx_messages = 0
        self.dropped_messages = 0
        self.parse_errors = 0
        
        # Remote tracking
        self.remote_addrs = set()  # Set of (sysid, compid) tuples
        
        # Threads
        self.rx_thread = None
        self.tx_thread = None
        self.heartbeat_thread = None
        self.current_rx_seq = 0
        self.current_tx_seq = 0
        
    def open(self):
        """Open the serial connection"""
        logger.info(f"Opening device: {self.device} @ {self.baudrate} bps")
        
        try:
            self.serial_dev = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,  # Non-blocking read with short timeout
                write_timeout=1.0
            )
            
            # Set RTS/CTS hardware flow control if available on this platform
            try:
                self.serial_dev.rtscts = False  # Disable hardware flow control by default
            except AttributeError:
                logger.warning("Hardware flow control not available on this platform")
            
            if not self.serial_dev.is_open:
                self.serial_dev.open()
                
            # Flush buffers
            self.serial_dev.reset_input_buffer()
            self.serial_dev.reset_output_buffer()
            
            logger.info("Serial device opened successfully")
            return True
            
        except serial.SerialException as e:
            logger.error(f"Failed to open serial device: {e}")
            return False
    
    def start(self):
        """Start the communication threads"""
        if not self.serial_dev or not self.serial_dev.is_open:
            logger.error("Cannot start: Serial device not open")
            return False
        
        self.is_running = True
        
        # Start the receive thread
        self.rx_thread = threading.Thread(target=self._rx_thread_func, name="MAVLinkRX")
        self.rx_thread.daemon = True
        self.rx_thread.start()
        
        # Start the transmit thread
        self.tx_thread = threading.Thread(target=self._tx_thread_func, name="MAVLinkTX")
        self.tx_thread.daemon = True
        self.tx_thread.start()
        
        # Start the heartbeat thread
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_thread_func, name="MAVLinkHB")
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        
        logger.info("Communication threads started")
        return True
    
    def stop(self):
        """Stop all threads and close the connection"""
        logger.info("Stopping MAVLink communication")
        self.is_running = False
        
        # Wait for threads to finish
        for thread in [self.rx_thread, self.tx_thread, self.heartbeat_thread]:
            if thread and thread.is_alive():
                thread.join(timeout=1.0)
        
        # Close serial connection
        if self.serial_dev and self.serial_dev.is_open:
            self.serial_dev.close()
            logger.info("Serial device closed")
    
    def is_open(self):
        """Check if serial connection is open"""
        return self.serial_dev is not None and self.serial_dev.is_open
    
    def send_message(self, msg):
        """
        Send a MAVLink message
        
        Args:
            msg: A MAVLinkMessage object
        """
        if not self.is_open():
            logger.error("Cannot send: Serial device not open")
            return False
        
        with self.lock:
            try:
                # Log message details (similar to MAVConnInterface::log_send)
                proto_version_str = "v2.0" if msg.magic == MAVLINK_STX else "v1.0"
                logger.debug(
                    f"send: {proto_version_str} Message-Id: {msg.msgid} [{msg.len} bytes] "
                    f"IDs: {msg.sysid}.{msg.compid} Seq: {msg.seq}"
                )
                
                # Add message to send queue
                if self.tx_queue.full():
                    logger.error(f"DROPPED Message-Id {msg.msgid}: TX queue overflow")
                    self.dropped_messages += 1
                    return False
                
                self.tx_queue.put(msg)
                return True
                
            except Exception as e:
                logger.error(f"Failed to queue message: {e}")
                return False
    
    def send_raw_data(self, data):
        """
        Send raw bytes over the serial connection
        
        Args:
            data (bytes): Raw data to send
        """
        if not self.is_open():
            logger.error("Cannot send: Serial device not open")
            return False
        
        with self.lock:
            try:
                # Add raw data to send queue with dummy message
                msg = MAVLinkMessage()
                msg.payload = data
                msg.len = len(data)
                msg.msgid = 0  # Special marker for raw data
                
                if self.tx_queue.full():
                    logger.error("DROPPED raw data: TX queue overflow")
                    self.dropped_messages += 1
                    return False
                
                self.tx_queue.put(msg)
                return True
                
            except Exception as e:
                logger.error(f"Failed to queue raw data: {e}")
                return False
    
    def _tx_thread_func(self):
        """Transmit thread function"""
        logger.info("TX thread started")
        
        while self.is_running:
            try:
                # Get a message from the queue
                try:
                    msg = self.tx_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                
                if not self.is_open():
                    logger.error("Serial device closed, TX thread exiting")
                    break
                
                # Special case for raw data
                if msg.msgid == 0 and msg.payload:
                    bytes_written = self.serial_dev.write(msg.payload)
                    self.tx_bytes += bytes_written
                    self.tx_queue.task_done()
                    continue
                
                # Regular MAVLink message - encode and send
                # This is a simplified MAVLink packet encoding as an example
                # For a complete implementation, you would need to implement the full MAVLink serialization
                
                # Pack message into MAVLink frame
                header_len = 10  # MAVLink v2 header size
                buffer = bytearray(header_len + msg.len + 2)  # +2 for checksum
                
                # MAVLink v2 header
                buffer[0] = MAVLINK_STX
                buffer[1] = msg.len
                buffer[2] = msg.seq & 0xFF
                buffer[3] = self.system_id & 0xFF
                buffer[4] = self.component_id & 0xFF
                
                # Message ID (3 bytes in MAVLink v2)
                buffer[5] = msg.msgid & 0xFF
                buffer[6] = (msg.msgid >> 8) & 0xFF
                buffer[7] = (msg.msgid >> 16) & 0xFF
                
                # Payload
                for i, b in enumerate(msg.payload):
                    buffer[header_len + i] = b
                
                # Calculate CRC (simplified - just a placeholder)
                # Real implementation would compute proper X25 CRC
                crc = sum(buffer[1:header_len + msg.len]) & 0xFFFF
                buffer[header_len + msg.len] = crc & 0xFF
                buffer[header_len + msg.len + 1] = (crc >> 8) & 0xFF
                
                # Send the bytes
                bytes_written = self.serial_dev.write(buffer)
                
                # Update stats
                with self.lock:
                    self.tx_bytes += bytes_written
                    self.tx_messages += 1
                    self.current_tx_seq = (self.current_tx_seq + 1) & 0xFF
                
                self.tx_queue.task_done()
                
            except serial.SerialException as e:
                logger.error(f"Serial error in TX thread: {e}")
                break
                
            except Exception as e:
                logger.error(f"Error in TX thread: {e}")
                continue
        
        logger.info("TX thread exiting")
    
    def _rx_thread_func(self):
        """Receive thread function"""
        logger.info("RX thread started")
        
        # Simplified MAVLink parser state
        state = 0  # 0=looking for STX, 1=reading header, 2=reading payload
        payload_len = 0
        header_len = 0
        msgid = 0
        sysid = 0
        compid = 0
        seq = 0
        buffer = bytearray(MAVLINK_MAX_PAYLOAD_LEN + 20)  # Max payload + headers
        
        while self.is_running:
            try:
                if not self.is_open():
                    logger.error("Serial device closed, RX thread exiting")
                    break
                
                # Read available bytes (non-blocking)
                bytes_available = self.serial_dev.in_waiting
                if bytes_available == 0:
                    time.sleep(0.001)  # Small sleep to prevent CPU overuse
                    continue
                
                # Read data
                data = self.serial_dev.read(bytes_available)
                if not data:
                    continue
                
                # Update received bytes count
                with self.lock:
                    self.rx_bytes += len(data)
                
                # Simplified MAVLink packet parsing
                # This is a very basic implementation - real code would use the full MAVLink parser
                for byte in data:
                    if state == 0:  # Looking for packet start
                        if byte == MAVLINK_STX:  # MAVLink v2
                            state = 1
                            buffer[0] = byte
                            header_len = 10  # MAVLink v2 header len
                            buffer_index = 1
                        elif byte == MAVLINK_STX_V1:  # MAVLink v1
                            state = 1
                            buffer[0] = byte
                            header_len = 6  # MAVLink v1 header len
                            buffer_index = 1
                    
                    elif state == 1:  # Reading header
                        buffer[buffer_index] = byte
                        buffer_index += 1
                        
                        if buffer_index == header_len:
                            # Header complete, extract info
                            if buffer[0] == MAVLINK_STX:  # MAVLink v2
                                payload_len = buffer[1]
                                seq = buffer[2]
                                sysid = buffer[3]
                                compid = buffer[4]
                                msgid = buffer[5] | (buffer[6] << 8) | (buffer[7] << 16)
                            else:  # MAVLink v1
                                payload_len = buffer[1]
                                seq = buffer[2]
                                sysid = buffer[3]
                                compid = buffer[4]
                                msgid = buffer[5]
                            
                            if payload_len > MAVLINK_MAX_PAYLOAD_LEN:
                                # Invalid length, reset parser
                                logger.warning(f"Invalid payload length: {payload_len}")
                                state = 0
                                self.parse_errors += 1
                            else:
                                state = 2  # Move to payload reading
                                payload_index = 0
                    
                    elif state == 2:  # Reading payload and checksum
                        buffer[buffer_index] = byte
                        buffer_index += 1
                        payload_index += 1
                        
                        if payload_index >= payload_len + 2:  # +2 for checksum
                            # Complete message received
                            
                            # Real implementation would validate CRC here
                            
                            # Create message object
                            msg = MAVLinkMessage()
                            msg.msgid = msgid
                            msg.sysid = sysid
                            msg.compid = compid
                            msg.seq = seq
                            msg.len = payload_len
                            msg.magic = buffer[0]
                            
                            # Extract payload
                            payload_start = header_len
                            payload_end = payload_start + payload_len
                            msg.payload = bytes(buffer[payload_start:payload_end])
                            
                            # Process the message
                            self._process_message(msg)
                            
                            # Reset parser state
                            state = 0
                            self.current_rx_seq = seq
                
            except serial.SerialException as e:
                logger.error(f"Serial error in RX thread: {e}")
                break
                
            except Exception as e:
                logger.error(f"Error in RX thread: {e}")
                state = 0  # Reset parser state on error
                self.parse_errors += 1
        
        logger.info("RX thread exiting")
    
    def _process_message(self, msg):
        """Process a received MAVLink message"""
        with self.lock:
            self.rx_messages += 1
            
            # Track remote systems
            remote_addr = (msg.sysid, msg.compid)
            if remote_addr not in self.remote_addrs and msg.sysid != 0:
                self.remote_addrs.add(remote_addr)
                logger.info(f"Detected remote system: {msg.sysid}.{msg.compid}")
        
        # Log message details
        proto_version = "v2.0" if msg.magic == MAVLINK_STX else "v1.0"
        payload_hex = binascii.hexlify(msg.payload).decode('ascii')
        
        logger.debug(
            f"Received: {proto_version} Message-Id: {msg.msgid} [{msg.len} bytes] "
            f"IDs: {msg.sysid}.{msg.compid} Seq: {msg.seq}"
        )
        
        # Check for HEARTBEAT (msgid 0)
        if msg.msgid == 0 and msg.len >= 9:
            # Extract type and autopilot from heartbeat
            # MAVLink HEARTBEAT format: type(1), autopilot(1), base_mode(1), custom_mode(4), system_status(1)
            if len(msg.payload) >= 9:
                mav_type = msg.payload[0]
                mav_autopilot = msg.payload[1]
                logger.info(
                    f"HEARTBEAT from {msg.sysid}.{msg.compid} - "
                    f"Type: {mav_type}, Autopilot: {mav_autopilot}"
                )
        
        # Check for STATUSTEXT messages (msgid 253)
        if msg.msgid == 253:
            try:
                # Extract severity and text from STATUSTEXT message
                severity = msg.payload[0]
                # Text is 50 bytes, null-terminated ASCII
                text_bytes = msg.payload[1:51]
                # Find the null terminator
                null_pos = text_bytes.find(0)
                if null_pos != -1:
                    text_bytes = text_bytes[:null_pos]
                text = text_bytes.decode('ascii', errors='replace')
                
                # Print received STATUSTEXT
                logger.info(f"RECEIVED STATUSTEXT from {msg.sysid}.{msg.compid}: [{severity}] {text}")
            except Exception as e:
                logger.error(f"Failed to parse STATUSTEXT: {e}")
                logger.error(f"Raw payload: {binascii.hexlify(msg.payload).decode()}")
    
    def _heartbeat_thread_func(self):
        """Thread that sends periodic heartbeats"""
        logger.info("Heartbeat thread started")
        
        # Constants for heartbeat message
        MAVLINK_MSG_ID_HEARTBEAT = 0
        MAV_TYPE_GCS = 6
        MAV_AUTOPILOT_INVALID = 8
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
        MAV_STATE_ACTIVE = 4
        
        while self.is_running:
            try:
                if self.is_open():
                    # Create heartbeat message
                    heartbeat = MAVLinkMessage()
                    heartbeat.msgid = MAVLINK_MSG_ID_HEARTBEAT
                    heartbeat.sysid = self.system_id
                    heartbeat.compid = self.component_id
                    heartbeat.seq = self.current_tx_seq
                    
                    # Pack heartbeat payload:
                    # type(1), autopilot(1), base_mode(1), custom_mode(4), system_status(1)
                    # Custom mode is 4 bytes (uint32) but needs to be packed as a single value
                    custom_mode = 0  # combined 4-byte value
                    
                    heartbeat.payload = struct.pack(
                        '<BBBIB',  # Format: type, autopilot, base_mode, custom_mode, system_status
                        MAV_TYPE_GCS,  # type
                        MAV_AUTOPILOT_INVALID,  # autopilot
                        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base_mode
                        custom_mode,  # custom_mode (4 bytes as a single uint32)
                        MAV_STATE_ACTIVE  # system_status
                    )
                    heartbeat.len = len(heartbeat.payload)
                    
                    # Send the heartbeat
                    self.send_message(heartbeat)
                    logger.debug("Sent heartbeat")
                
                # Wait for next heartbeat interval
                time.sleep(1.0)  # 1 Hz heartbeat
                
            except Exception as e:
                logger.error(f"Error in heartbeat thread: {e}")
                time.sleep(1.0)
        
        logger.info("Heartbeat thread exiting")
    
    def print_stats(self):
        """Print communication statistics"""
        with self.lock:
            print("\nMAVLink Communication Statistics:")
            print(f"  Received bytes: {self.rx_bytes}")
            print(f"  Transmitted bytes: {self.tx_bytes}")
            print(f"  Received messages: {self.rx_messages}")
            print(f"  Transmitted messages: {self.tx_messages}")
            print(f"  Dropped messages: {self.dropped_messages}")
            print(f"  Parse errors: {self.parse_errors}")
            print(f"  Current RX sequence: {self.current_rx_seq}")
            print(f"  Current TX sequence: {self.current_tx_seq}")
            print(f"  Remote systems: {len(self.remote_addrs)}")
            
            for i, (sysid, compid) in enumerate(sorted(self.remote_addrs)):
                print(f"    Remote [{i}]: {sysid}.{compid}")
    
    def send_command_long(self, command, param1=0, param2=0, param3=0, 
                         param4=0, param5=0, param6=0, param7=0,
                         target_sysid=1, target_compid=1):
        """
        Send a COMMAND_LONG message
        
        Args:
            command: MAVLink command number
            param1-7: Command parameters
            target_sysid: Target system ID
            target_compid: Target component ID
        """
        MAVLINK_MSG_ID_COMMAND_LONG = 76
        
        msg = MAVLinkMessage()
        msg.msgid = MAVLINK_MSG_ID_COMMAND_LONG
        msg.sysid = self.system_id
        msg.compid = self.component_id
        msg.seq = self.current_tx_seq
        
        # Pack COMMAND_LONG payload
        # Format: target_system(1), target_component(1), command(2), confirmation(1), param1-7(4 each)
        msg.payload = struct.pack(
            '<BBHBfffffff',
            target_sysid,
            target_compid,
            command,
            0,  # confirmation
            float(param1),
            float(param2),
            float(param3),
            float(param4),
            float(param5),
            float(param6),
            float(param7)
        )
        msg.len = len(msg.payload)
        
        return self.send_message(msg)
    
    def arm_disarm(self, arm=True, target_sysid=1, target_compid=1):
        """
        Send arm/disarm command
        
        Args:
            arm: True to arm, False to disarm
            target_sysid: Target system ID
            target_compid: Target component ID
        """
        # MAV_CMD_COMPONENT_ARM_DISARM (400)
        return self.send_command_long(
            400,  # MAV_CMD_COMPONENT_ARM_DISARM
            1 if arm else 0,  # param1: 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0,
            target_sysid,
            target_compid
        )
    
    def send_statustext(self, text, severity=0, target_sysid=0, target_compid=0):
        """
        Send a STATUSTEXT message
        
        Args:
            text (str): The status text message (max 50 chars)
            severity (int): Severity value 0-7 (0=emergency, 7=debug)
            target_sysid (int): Target system ID (0 for broadcast)
            target_compid (int): Target component ID (0 for broadcast)
        
        Returns:
            bool: True if message was queued successfully
        """
        MAVLINK_MSG_ID_STATUSTEXT = 253
        
        # Truncate text to 50 characters (MAVLink 2.0 spec)
        text = str(text)[:50]
        
        # Pad the text to exactly 50 characters with nulls
        padded_text = text.ljust(50, '\0')
        
        # For debugging
        print(f"Sending STATUSTEXT with content: '{text}'")
        print(f"Text length: {len(text)}, Padded length: {len(padded_text)}")
        
        msg = MAVLinkMessage()
        msg.msgid = MAVLINK_MSG_ID_STATUSTEXT
        
        # Try to match MAVROS sender IDs - target ground station (14.1) instead of default
        msg.sysid = 1  # Use system ID 1 (typically the vehicle)
        msg.compid = 1  # Use component ID 1 (typically the autopilot)
        msg.seq = self.current_tx_seq
        
        # Convert string to bytes
        text_bytes = padded_text.encode('ascii')
        
        # Structure for MAVLink 2.0: severity(1), text(50), id(1), chunk_seq(1)
        msg.payload = struct.pack(
            '<B50sBB',
            severity & 0xFF,
            text_bytes,
            0,  # id - used for multi-part messages
            0   # chunk_seq - used for multi-part messages
        )
        msg.len = len(msg.payload)
        
        # Print the raw bytes for debugging
        print(f"Raw payload (hex): {binascii.hexlify(msg.payload).decode()}")
        
        return self.send_message(msg)


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Direct MAVLink Serial Communication')
    parser.add_argument('--device', default='/dev/ttyACM0', help='Serial device')
    parser.add_argument('--baudrate', type=int, default=57600, help='Serial baudrate')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    args = parser.parse_args()
    
    # Set log level
    if args.debug:
        logger.setLevel(logging.DEBUG)
    
    # Create and start serial communication
    mavlink_serial = MAVLinkSerialComm(args.device, args.baudrate)
    
    if not mavlink_serial.open():
        logger.error("Failed to open serial device")
        sys.exit(1)
    
    if not mavlink_serial.start():
        logger.error("Failed to start communication")
        mavlink_serial.stop()
        sys.exit(1)
    
    logger.info("MAVLink serial communication started")
    logger.info(f"Press Ctrl+C to exit, or type commands (arm, disarm, stats, help, quit)")
    
    try:
        while True:
            cmd = input("\nCommand> ").strip().lower()
            
            if cmd == 'quit' or cmd == 'exit':
                break
                
            elif cmd == 'help':
                print("\nAvailable commands:")
                print("  arm                     - Arm the vehicle")
                print("  disarm                  - Disarm the vehicle")
                print("  stats                   - Show communication statistics")
                print("  status <text> [severity] - Send a status text message (severity 0-7)")
                print("  help                    - Show this help")
                print("  quit                    - Exit program")
            
            elif cmd == 'arm':
                if mavlink_serial.arm_disarm(True):
                    print("Arm command sent")
                else:
                    print("Failed to send arm command")
            
            elif cmd == 'disarm':
                if mavlink_serial.arm_disarm(False):
                    print("Disarm command sent")
                else:
                    print("Failed to send disarm command")
            
            elif cmd == 'stats':
                mavlink_serial.print_stats()

            elif cmd.startswith('status '):
                # Parse the status command
                rest = cmd[7:].strip()  # Remove 'status ' prefix and trim
                
                if not rest:
                    print("Usage: status <text> [severity]")
                    continue
                
                # Split the message into words
                words = rest.split()
                
                # Check if the last word is a number (severity)
                try:
                    severity = int(words[-1])
                    if 0 <= severity <= 7:
                        # Last word is a valid severity
                        text = ' '.join(words[:-1])
                    else:
                        # Number out of range, treat as part of the message
                        text = rest
                        severity = 6  # Default to INFO
                except ValueError:
                    # Last word is not a number, use all words as text
                    text = rest
                    severity = 6  # Default to INFO
                
                # Send the status text
                if mavlink_serial.send_statustext(text, severity):
                    print(f"Status text sent: [{severity}] {text}")
                else:
                    print("Failed to send status text")
                        
            elif cmd:
                print(f"Unknown command: {cmd}")
                print("Type 'help' for available commands")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    
    finally:
        mavlink_serial.stop()
        logger.info("Program terminated")


if __name__ == '__main__':
    main()