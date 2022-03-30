"""
Example of how to filter for specific mavlink messages coming from the
autopilot using pymavlink.

Can also filter within recv_match command - see "Read all parameters" example
"""
# Import mavutil
from pymavlink import mavutil

# Create the connection
# From topside computer
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

while True:
    msg = master.recv_match()
    if not msg:
        continue
    msg_type = msg.get_type()
    print("\n\n*****Got message: %s*****" % msg.get_type())
    print("Message: %s" % msg)
    if msg_type == 'GPS_RAW_INT':
        pass
    # if msg_type == 'VFR_HUD':
        # print("\n\n*****Got message: %s*****" % msg.get_type())
        # print("Message: %s" % msg)
    #     # print("\nAs dictionary: %s" % msg.to_dict())
    #     # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
    #     # print("\nSystem status: %s" % msg.system_status)
    elif msg_type[1] == 'U':
        print("\n\n*****Got message: %s*****" % msg.get_type())
    # else:
        # print("\n\n*****Got message: %s*****" % msg_type[0])