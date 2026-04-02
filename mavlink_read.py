from pymavlink import mavutil

# connect to FMU over USB
the_connection = mavutil.mavlink_connection('/dev/ttyACM0')

# wait until FMU sends a heartbeat so we know the link is up
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system,
      the_connection.target_component))

# read and print ATTITUDE messages (roll, pitch, yaw)
while 1:
    msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
    print(msg)
