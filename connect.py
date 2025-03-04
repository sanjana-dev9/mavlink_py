from pymavlink import mavutil
import time

# Connect with proper system ID and force MAVLink 2
connection = mavutil.mavlink_connection(
    '/dev/ttyAMA0', 
    baud=115200,
    source_system=27,  # Unique system ID for Raspberry Pi
    source_component=1,
    force_mavlink2=True
)

print("Waiting for heartbeat...")
connection.wait_heartbeat()
print(f"Connected to system {connection.target_system} component {connection.target_component}")

# Main loop
while True:
    msg = connection.recv_match(blocking=True)
    
    if not msg or msg.get_type() == "HEARTBEAT":
        continue
    
    print(f"Message received: {msg.get_type()}")
    
    if msg.get_type() == "STATUSTEXT":
        msg_dict = msg.to_dict()
        message_text = msg_dict["text"]
        send_text = ""
        
        if "RC7: Relay1 LOW" in message_text:
            print("Relay 1 is LOW - sending response")
            send_text = f"PI DETECTED RELAY LOW {int(time.time())}"
        else:
            print("Relay 1 is HIGH - sending response")
            send_text = f"PI DETECTED RELAY HIGH {int(time.time())}"
        
            
        # Send a high-priority status message
        connection.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_WARNING,  # Higher priority
            send_text.encode()  # Short, clear message
        )
        
        print("Response sent")