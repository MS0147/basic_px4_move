#!/usr/bin/env python

from pymavlink import mavutil
from pymavlink import mavwp
import time
import sys
from pyproj import Proj,transform
import numpy as np
import pandas as pd

'''lat = float(sys.argv[1])
lon = float(sys.argv[2])
alt = float(sys.argv[3])
uav_status = int(sys.argv[4]) #start=1, end=3, else=2'''
tmp=open('drone1.txt','r')
tmp_arg=[]
j=0
for i in tmp:
    tmp_arg.append(float(i))
lat=tmp_arg[0]
lon=tmp_arg[1]
tmp.close()
alt=15
uav_status=2
#---------- GPS -> lat,lon -----------#
if(int(abs(lat))>90 or int(abs(lon)>180)):
    lat, lon = transform(proj_UTMK,proj_WGS84,x1,y1)
#-------------------------------------#


def cmd_set_home(home_location, altitude):
    print 'mav_target_system', mav.target_system, 'mav_target_component', mav.target_component
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, home_location[0], home_location[1], altitude)

def handle_mission_current(msg, nextwaypoint):
    if msg.seq > nextwaypoint:
        print "Moving to waypoint %s" % msg.seq
        nextwaypoint = msg.seq + 1
        print "Next Waypoint %s" % nextwaypoint
	print('\n\n')
    return nextwaypoint


print('Program start!')
# python /usr/local/bin/mavproxy.py --mav=/dev/tty.usbserial-DN01WM7R --baudrate 57600 --out udp:127.0.0.1:14540 --out udp:127.0.0.1:14550
connection_string = '0.0.0.0:14540'
mav = mavutil.mavlink_connection('udp:'+connection_string)
#mav = mavutil.mavlink_connection('/dev/ttyUSB0')

print('Connection finish & wait heartbeat')
mav.wait_heartbeat()
print("HEARTBEAT OK\n")

# Send Home location
home_location = (lat,lon)
cmd_set_home(home_location, 0)
msg = mav.recv_match(type=['COMMAND_ACK'],blocking=True)
print 'Set home location: {0} {1}\n\n'.format(home_location[0], home_location[1])

time.sleep(1)

wp = mavwp.MAVWPLoader()
seq = 0
altitude = alt
autocontinue = 1
current = 0
param1 = 15.0 # minimum pitch
frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

tmp_lat=0
tmp_lon=0

while(True):
    tmp=open('drone1.txt','r')
    tmp_arg=[]
    j=0
    for i in tmp:
        tmp_arg.append(float(i))
    lat=tmp_arg[0]
    lon=tmp_arg[1]
    tmp.close()


    if tmp_lat==lat and tmp_lon==lon:
        time.sleep(1)
        continue
    else:
        tmp_lat=lat
        tmp_lon=lon
    
    if uav_status == 1:
        p = mavutil.mavlink.MAVLink_mission_item_message(mav.target_system, mav.target_component, seq, frame,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            current, autocontinue, param1, 0, 0, 0, lat, lon, altitude)
    elif uav_status == 3:
        p = mavutil.mavlink.MAVLink_mission_item_message(mav.target_system, mav.target_component, seq, frame,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            current, autocontinue, 0, 0, 0, 0, lat, lon, altitude)
    else:
        p = mavutil.mavlink.MAVLink_mission_item_message(mav.target_system, mav.target_component, seq, frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current, autocontinue, 0, 0, 0, 0, lat, lon, altitude)
    wp.clear()
    wp.add(p)


    # Send Waypoint to airframe
    mav.waypoint_clear_all_send()
    mav.waypoint_count_send(wp.count())


    # Send request message
    for i in range(wp.count()):
        msg = mav.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print msg
        mav.mav.send(wp.wp(msg.seq))
        print 'Sending waypoint {0}\n\n'.format(msg.seq)


    # Receive OK message
    msg = mav.recv_match(type=['MISSION_ACK'],blocking=True)
    print('msg, msg.type')
    print msg, msg.type


    # Read Waypoint from airframe
    mav.waypoint_request_list_send()
    waypoint_count = 0

    mav.mav.mission_ack_send(mav.target_system, mav.target_component, 0) # OKAY

    # Change Mission Mode
    PX4_CUSTOM_MAIN_MODE_AUTO = 4.0
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4.0
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5.0

    auto_mode_flags =  mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED 

    PX4_MAV_MODE = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags

    mav.mav.set_mode_send(1, PX4_MAV_MODE, PX4_CUSTOM_MAIN_MODE_AUTO)


    # Send Heartbeat
    mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
    mav.wait_heartbeat()
    print("HEARTBEAT OK\n")


    # MISSION START
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = mav.recv_match(type=['COMMAND_ACK'],blocking=True)
    print msg

    print lat,lon
    time.sleep(1)
