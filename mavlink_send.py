from pymavlink import mavutil
import time

# connect to FMU over USB
the_connection = mavutil.mavlink_connection('/dev/ttyACM0')

# wait until FMU sends a heartbeat so we know the link is up
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system,
      the_connection.target_component))

# alternate sending 0 and 1 every second to blink the LED
value = 0
while 1:
    message = mavutil.mavlink.MAVLink_debug_message(0, value, 0.0)
    the_connection.mav.send(message)
    time.sleep(1)
    value = (value + 1) % 2
    print("Message sent")
