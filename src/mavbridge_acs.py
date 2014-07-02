#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# ACS-specific handlers
#
# Originally developed by Mike Clement, 2014
#
# This software may be freely used, modified, and distributed.
# This software comes with no warranty.
#
# Credit: This software was inspired by the following:
#   https://github.com/cberzan/roscopter

#-----------------------------------------------------------------------
# Import libraries

from pymavlink import mavutil
import std_msgs.msg as stdmsg
import autopilot_bridge.msg as apmsg

#-----------------------------------------------------------------------
# Ugly globals

# Mappings between MAVLink mode strings and our own enumeration
# TODO replace this
mode_mav_to_enum = { 'RTL' : apmsg.Status.MODE_RALLY,
                     'MANUAL' : apmsg.Status.MODE_MANUAL,
                     'FBWA' : apmsg.Status.MODE_FBW,
                     'GUIDED' : apmsg.Status.MODE_GUIDED,
                     'AUTO' : apmsg.Status.MODE_AUTO }
mode_enum_to_mav = { v:k for (k,v) in mode_mav_to_enum.items() }

#-----------------------------------------------------------------------
# MAVLink message handlers

# Publish a status message with many fields
def pub_status(msg_type, msg, bridge):
    pub = bridge.get_ros_pub("status", apmsg.Status)
    sta = apmsg.Status()
    sta.header.stamp = bridge.project_ap_time()
    if bridge.master.flightmode in mode_mav_to_enum:
        sta.mode = mode_mav_to_enum[bridge.master.flightmode]
    else:
        sta.mode = apmsg.Status.MODE_UNKNOWN
    sta.armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    sta.ahrs_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_AHRS)
    sta.alt_rel = bridge.master.field('GLOBAL_POSITION_INT', 'relative_alt', 0)
    sta.as_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)
    sta.as_read = bridge.master.field('VFR_HUD', 'airspeed', 0)
    sta.gps_ok = (bridge.master.field('GPS_RAW_INT', 'fix_type', 0) == 3)
    sta.gps_sats = bridge.master.field('GPS_RAW_INT', 'satellites_visible', 0)
    sta.gps_eph = bridge.master.field('GPS_RAW_INT', 'eph', 0)
    sta.ins_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | \
                                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO)
    sta.mag_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG)
    sta.mis_cur = bridge.master.field('MISSION_CURRENT', 'seq', 0)
    sta.pwr_ok = not (bridge.master.field('POWER_STATUS', 'flags', 0) \
                    & mavutil.mavlink.MAV_POWER_STATUS_CHANGED)
    sta.pwr_batt_rem = bridge.master.field('SYS_STATUS', 'battery_remaining', -1)
    sta.pwr_batt_vcc = bridge.master.field('SYS_STATUS', 'voltage_battery', -1)
    sta.pwr_batt_cur = bridge.master.field('SYS_STATUS', 'current_battery', -1)
    pub.publish(sta)

#-----------------------------------------------------------------------
# ROS subscriber handlers

# Purpose: Send heartbeat to AP when heartbeat arrives
# NOTE: Not yet supported in MAVLink master branch
# Fields: None
def sub_heartbeat(message, bridge):
    bridge.master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_TYPE_GENERIC,
        0,
        0, # TODO make sure it doesn't need to be: ap_last_custom_mode,
        mavutil.mavlink.MAV_STATE_ACTIVE)

# Purpose: Initiate barometer calibration
# Fields: None
def sub_calpress(message, bridge):
    bridge.master.calibrate_pressure()

# Purpose: Changes autopilot mode
# (must be in mode_mapping dictionary)
# Fields:
# .data - index from mode_mav_to_enum
def sub_change_mode(message, bridge):
    mav_map = bridge.master.mode_mapping()
    if message.data in mode_enum_to_mav and \
        mode_enum_to_mav[message.data] in mav_map:
        bridge.master.set_mode(mav_map[mode_enum_to_mav[message.data]])
    else:
        raise Exception("invalid mode %u" % message.data)

# Purpose: Initiates landing
# NOTE: Not yet supported in MAVLink master branch
# Fields: None
def sub_landing(message, bridge):
    bridge.master.mav.command_long_send(
        bridge.master.target_system,
        bridge.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_RALLY_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)

# Purpose: Aborts landing
# NOTE: Not yet supported in MAVLink master branch
# Fields: 
# .data - Altitude to return to (meters, integer)
def sub_landing_abort(message, bridge):
    # TODO: Do we need to repeat until we see RTL again??
    bridge.master.mav.command_long_send(
        bridge.master.target_system,
        bridge.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_GO_AROUND,
        0,
        message.data, # TODO better to use parameter
        0, 0, 0, 0, 0, 0)

#-----------------------------------------------------------------------
# init()

def init(bridge):
    bridge.add_mavlink_event("HEARTBEAT", pub_status)
    bridge.add_ros_sub_event("heartbeat", stdmsg.Empty, sub_heartbeat)
    bridge.add_ros_sub_event("calpress", stdmsg.Empty, sub_calpress)
    bridge.add_ros_sub_event("mode_num", stdmsg.UInt8, sub_change_mode)
    bridge.add_ros_sub_event("land", stdmsg.Empty, sub_landing)
    bridge.add_ros_sub_event("land_abort", stdmsg.UInt16, sub_landing_abort)
    return True
