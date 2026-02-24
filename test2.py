from pymavlink import mavutil
m = mavutil.mavlink_connection("udp:127.0.0.1:14540")
m.wait_heartbeat()
m.mav.command_long_send(m.target_system,m.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,int(1e6/20),0,0,0,0,0)
print("requested LOCAL_POSITION_NED")
for _ in range(50):
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
    if msg:
        print("got", msg)
        break