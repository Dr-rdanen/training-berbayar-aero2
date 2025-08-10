from pymavlink import mavutil
import time, math

# Connect ke drone
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

start_time = time.time()

# Ubah mode drone menjadi GUIDED
def set_guided():
    while True:
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        master.set_mode('GUIDED')
        if heartbeat and heartbeat.custom_mode == 4:
            print("Mode set to GUIDED!")
            break
        time.sleep(0.5)

# Cek kondisi pre-arm dan arming motor
def arm():
    print("waiting for pre-arm and arming drone")
    while True:
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        master.arducopter_arm()
        if heartbeat and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone is armed!")
            break
        time.sleep(0.5)

# Takeoff ke ketinggian yang ditentukan
def takeoff(altitude):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude
    )
    print("Taking off...")

# Bergerak ke posisi yang ditentukan berdasarkan orientasi drone
def send_position(x, y, z, duration, yaw=0):
    end_time = time.time() + duration
    v_x = x/duration
    v_y = y/duration
    v_z = z/duration
    while time.time() < end_time:
        master.mav.set_position_target_local_ned_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            x, y, z,
            v_x, v_y, v_z,
            0, 0, 0,
            yaw, 0
        )

# Yaw drone sebesar nilai yang ditentukan
def turn_yaw(angle, speed, direction, relative = True):
    print("Turning 90 degrees")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        angle,   # target angle
        speed,   # speed deg/s
        direction,    # direction: 1 = clockwise; 0 = counter-clockwise
        1 if relative else 0,    # relative offset: 0 = absolute
        0, 0, 0
    )

# Mengubah mode menjadi LAND dan meendaratkan drone
def land():
    print("Landing")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

# Membaca posisi drone dengan GPS
def read_position():
    print("Reading current GPS position...")
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1000
    print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}m")


set_guided()
time.sleep(2)

arm()
master.motors_armed_wait()

# Naik ke ketinggian 5m
takeoff(5)
time.sleep(5)

# Bergerak maju 5m dalam waktu 5s
print("Moving 5m")
send_position(5, 0, 0, 5)
time.sleep(6)

# Yaw drone ke kanan sebesar 90 derajat
print("turning right")
turn_yaw(90, 30, 0)
time.sleep(5)

# Membaca koordinat drone saat ini
read_position()

# Bergerak maju 4m dalam waktu 2s 
print("Moving 4m")
send_position(4, 0, 0, 2)
time.sleep(6)

# Mendaratkan drone
land()

# Menunggu drone disarm
master.motors_disarmed_wait()
print("Landed and disarmed")