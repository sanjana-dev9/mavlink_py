from pymavlink import mavutil
import time

# Connect to Pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Wait for heartbeat to confirm connection
master.wait_heartbeat()
print(f"Connected to system: {master.target_system}, component: {master.target_component}")

master.mav.srcSystem = 0
master.mav.srcComponent = 0

# Target the broadcast address (0) so all systems receive the message
# This is critical for message forwarding through the system
master.target_system = 0  # Broadcast to all systems
master.target_component = 0  # Broadcast to all components

# Severity levels
# EMERGENCY = 0
# ALERT = 1
# CRITICAL = 2
# ERROR = 3
# WARNING = 4
# NOTICE = 5
# INFO = 6
# DEBUG = 7
def send_status_message(text, severity=mavutil.mavlink.MAV_SEVERITY_WARNING):
    # Ensure text is properly formatted (not too long)
    text = text[:50]  # Limit length to be safe
    
    # Explicitly set message fields for maximum compatibility
    master.mav.statustext_send(
        severity,  # Severity level
        text.encode()  # Text must be encoded as bytes
    )
    
    print(f"Status message sent: '{text}' with severity {severity}")
    time.sleep(0.1)  # Ensure message has time to process

send_status_message("Connected succesfully", severity=mavutil.mavlink.MAV_SEVERITY_INFO)