from pymavlink import mavutil
import time

connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Wait for a heartbeat to confirm connection
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print(f"Connected to system {connection.target_system} component {connection.target_component}")

# Main loop to receive messages
while True:
    # Wait for a message
    msg = connection.recv_match(type='STATUSTEXT',blocking=True)
    
    # Convert message to dictionary for easier access
    msg_dict = msg.to_dict()
    print(msg_dict)
    # Check if it's a STATUSTEXT message
    if msg.get_type() == "STATUSTEXT":
        # Get the text from the message
        message_text = msg_dict["text"]
        print(message_text)
        # Check if it contains our relay info
        if "RC7: Relay1 HIGH" in message_text:
            print("Message received: Relay 1 is HIGH")
            # Define the severity level of the message
            # 0=EMERGENCY, 1=ALERT, 2=CRITICAL, 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
            severity = mavutil.mavlink.MAV_SEVERITY_INFO

            # The text message to send
            text_message = "Received relay status: HIGH. Taking action."

            # Create the MAVLink message
            # Ensure the string is encoded to bytes
            message = mavutil.mavlink.MAVLink_statustext_encode(severity, text_message.encode())

            # Send the message
            connection.mav.send(message)
            
            print("Status message sent back to MAVLink")
            time.sleep(0.5)
        elif "RC7: Relay1 LOW" in message_text:
            print("Message received: Relay 1 is LOW")
            
            # Define the severity level of the message
            # 0=EMERGENCY, 1=ALERT, 2=CRITICAL, 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
            severity = mavutil.mavlink.MAV_SEVERITY_INFO

            # The text message to send
            text_message = "Received relay status: LOW. Taking action."

            # Create the MAVLink message
            # Ensure the string is encoded to bytes
            message = mavutil.mavlink.MAVLink_statustext_encode(severity, text_message.encode())

            # Send the message
            connection.mav.send(message)
            
            print("Status message sent back to MAVLink")
            time.sleep(0.001)
    else:
        continue