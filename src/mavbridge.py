#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# Main program and a small set of standard pubs/subs
#
# Originally developed by Mike Clement, 2014
#
# This software may be freely used, modified, and distributed.
# This software comes with no warranty.
#
# Credit: This software is inspired by the following:
#   https://github.com/cberzan/roscopter

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
from argparse import ArgumentParser
import os
import sys

# Import "standard" ROS message types
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import std_msgs.msg as stdmsg
import autopilot_bridge.msg as apmsg

# Other needed ROS imports
from tf.transformations import quaternion_from_euler

# Import the bridge
from mavbridge_core import MAVLinkBridge

#-----------------------------------------------------------------------
# Standard ROS subscribers

# Purpose: (Dis)arm throttle
# Fields: 
# .data - True arms, False disarms
def sub_arm_throttle(message, bridge):
    if message.data:
        bridge.master.arducopter_arm()
    else:
        bridge.master.arducopter_disarm()

# Purpose: Changes autopilot mode
# (must be in mode_mapping dictionary)
# Fields: 
# .data - string representing mode
def sub_change_mode(message, bridge):
    mav_map = bridge.master.mode_mapping()
    if message.data in mav_map:
        bridge.master.set_mode(mav_map[message.data])
    else:
        # TODO expose rospy.logwarn from bridge
        print "sub_change_mode: invalid mode "  + message.data

# Purpose: Go to a lat/lon/alt in GUIDED mode
# Fields: 
# .lat - Decimal degrees
# .lon - Decimal degrees
# .alt - Decimal meters **AGL wrt home**
def sub_guided_goto(message, bridge):
    bridge.master.mav.mission_item_send(
        bridge.master.target_system,
        bridge.master.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 0, 0, 0, 0,
        message.lat, message.lon, message.alt)

# Purpose: Go to a specified waypoint by index/sequence number
# Fields: 
# .data - Must be an index in current mission set
def sub_waypoint_goto(message, bridge):
    bridge.master.waypoint_set_current_send(message.data)

#-----------------------------------------------------------------------
# Standard ROS publishers

# Publish an Imu message
def pub_imu(msg_type, msg, bridge):
    pub = bridge.get_ros_pub("imu", Imu)
    imu = Imu()
    imu.header.stamp = bridge.project_ap_time()
    imu.header.frame_id = 'base_footprint'
    quat = quaternion_from_euler(msg.roll, msg.pitch, msg.yaw, 'sxyz')
    imu.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    pub.publish(imu)

# Publish GPS data in NavSatFix form
def pub_gps(msg_type, msg, bridge):
    pub = bridge.get_ros_pub("gps", NavSatFix)
    fix = NavSatFix()
    fix.header.stamp = bridge.project_ap_time()
    fix.header.frame_id = 'base_footprint'
    fix.latitude = msg.lat/1e07
    fix.longitude = msg.lon/1e07
    fix.altitude = msg.alt/1e03
    fix.status.status = NavSatStatus.STATUS_FIX
    fix.status.service = NavSatStatus.SERVICE_GPS
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    pub.publish(fix)
                
# Publish GPS data in Odometry form
def pub_gps_odom(msg_type, msg, bridge):
    pub = bridge.get_ros_pub("gps_odom", Odometry)
    odom = Odometry()
    odom.header.stamp = bridge.project_ap_time()
    odom.header.frame_id = 'base_footprint'
    odom.pose.pose.position.x = msg.lat/1e07
    odom.pose.pose.position.y = msg.lon/1e07
    odom.pose.pose.position.z = msg.alt/1e03
    odom.pose.pose.orientation.x = 1
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = 0
    odom.pose.pose.orientation.w = 0
    odom.pose.covariance = ( 0.1, 0, 0, 0, 0, 0,
                             0, 0.1, 0, 0, 0, 0,
                             0, 0, 0.1, 0, 0, 0,
                             0, 0, 0, 99999, 0, 0,
                             0, 0, 0, 0, 99999, 0,
                             0, 0, 0, 0, 0, 99999 )
    pub.publish(odom)

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # Grok args
    # TODO: Make these ROS params
    parser = ArgumentParser("rosrun autopilot_bridge mavbridge.py")
    parser.add_argument('-d', "--device", dest="device", 
                        help="serial device or network socket", default="auto-detect")
    parser.add_argument('-b', "--baudrate", dest="baudrate", type=int,
                        help="serial baud rate", default=57600)
    parser.add_argument('-m', "--module", dest="module", action='append', 
                        help="load module mavbridge_MODULE.pyc")
    parser.add_argument("--ros-basename", dest="basename", 
                        help="ROS namespace basename", default="autopilot")
    parser.add_argument("--gps-time-hack", dest="gps_time_hack", 
                        action="store_true", default=False,
                        help="try to set system clock from autopilot time")
    parser.add_argument("--track-time-delta", dest="track_time_delta", 
                        action="store_true", default=False,
                        help="track AP time, use to project current time")
    parser.add_argument("--spam-mavlink", dest="spam_mavlink", 
                        action="store_true", default=False,
                        help="print every received mavlink message")
    args = parser.parse_args()
    
    # If device isn't explicitly specified, try to detect it
    # TODO: Might make better use of /dev/serial/by-id/*
    #  for some devices, like the SiK telemetry radio
    if args.device == "auto-detect":
        if os.path.exists("/dev/ttyACM0"):  # Indicates USB
            args.device = "/dev/ttyACM0"
            args.baudrate = 115200
        elif os.path.exists("/dev/ttyUSB0"):  # Indicates radio
            args.device = "/dev/ttyUSB0"
        elif os.path.exists("/dev/ttyAMA0"):  # Indicates serial
            args.device = "/dev/ttyAMA0"
        else:
            print "Error: could not find a suitable device.\n"
            sys.exit(-1)
    elif args.device.find(':') == -1 and not os.path.exists(args.device):
        # ':' check allows for specifying network sockets
        print "Specified device doesn't exist.\n"
        sys.exit(-1)

    # User-friendly hello message
    print "Starting MAVLink <-> ROS interface over the following link:\n" + \
          ("  device:\t\t%s\n" % args.device) + \
          ("  baudrate:\t\t%s\n" % str(args.baudrate))

    # Initialize the bridge
    try:
        bridge = MAVLinkBridge(device=args.device,
                               baudrate=args.baudrate,
                               basename=args.basename,
                               sync_local_clock=args.gps_time_hack,
                               track_time_delta=args.track_time_delta,
                               spam_mavlink=args.spam_mavlink)
    except Exception as ex:
        print ex.args[0]
        sys.exit(-1)

    # Register Standard MAVLink events
    bridge.add_mavlink_event("ATTITUDE", pub_imu)
    bridge.add_mavlink_event("GLOBAL_POSITION_INT", pub_gps)
    bridge.add_mavlink_event("GLOBAL_POSITION_INT", pub_gps_odom)

    # Register Standard ROS Subcriber events
    bridge.add_ros_sub_event("arm", stdmsg.Bool, sub_arm_throttle) 
    bridge.add_ros_sub_event("mode", stdmsg.String, sub_change_mode) 
    bridge.add_ros_sub_event("guided_goto", apmsg.LLA, sub_guided_goto) 
    bridge.add_ros_sub_event("waypoint_goto", stdmsg.UInt16, sub_waypoint_goto) 

    # Register modules
    loaded_modules = {}
    if args.module is not None:
        for m in args.module:
            m_name = "mavbridge_%s" % m
            if m_name in loaded_modules:
                print "Module '%s' already loaded" % m_name
                continue
            try:
                # Blatantly copied from MAVProxy and a Stack Overflow answer;
                # necessary to deal with subtleties of Python imports
                m_obj = __import__(m_name)
                components = m_name.split('.')
                for comp in components[1:]:
                    m_obj = getattr(m_obj, comp)
                reload(m_obj)
                loaded_modules[m_name] = m_obj.init(bridge)
                print "Module '%s' loaded successfully" % m_name
            except Exception as ex:
                print "Failed to load module '%s': %s" % (m_name, ex.args[0])

    # Start loop (this won't return until we terminate or ROS shuts down)
    print "Starting autopilot bridge loop...\n"
    while True:
        try:
            bridge.run_loop()
            break  # If loop legitimately returned, we'll exit
        except Exception as ex:
            print "MAVLinkBridge Loop Error: " + ex.args[0]
            print "...restarting loop..."

