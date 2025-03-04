from pymavlink import mavutil
import time

# Connect to your MAVLink network
connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
print("Listening for MAVLink components...")

# Dictionary to store discovered systems
systems = {}

# Listen for 60 seconds to discover components
end_time = time.time() + 60
while time.time() < end_time:
    msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg is not None:
        # Extract system and component IDs
        system_id = msg.get_srcSystem()
        component_id = msg.get_srcComponent()
        
        # Get type information
        system_type = mavutil.mavlink.enums['MAV_TYPE'][msg.type].name
        autopilot_type = mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].name if hasattr(msg, 'autopilot') else "Unknown"
        
        # Create a unique key for this system/component pair
        key = f"{system_id}_{component_id}"
        
        # Store in our dictionary if new
        if key not in systems:
            systems[key] = {
                'system_id': system_id,
                'component_id': component_id,
                'type': system_type,
                'autopilot': autopilot_type,
                'last_seen': time.time()
            }
            print(f"Discovered: System ID: {system_id}, Component ID: {component_id}, Type: {system_type}")
        else:
            # Update last seen timestamp
            systems[key]['last_seen'] = time.time()

print("\n--- MAVLink Network Scan Results ---")
print("System ID | Component ID | Type | Autopilot")
print("-" * 60)
for key, info in systems.items():
    print(f"{info['system_id']:9} | {info['component_id']:12} | {info['type']:20} | {info['autopilot']}")