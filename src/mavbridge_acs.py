#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# ACS-specific handlers
#
# Originally developed by Mike Clement, 2014
#
# This software may be freely used, modified, and distributed.
# This software comes with no warranty.

#-----------------------------------------------------------------------
# Import libraries

from pymavlink import mavutil
import std_msgs.msg as stdmsg
import autopilot_bridge.msg as apmsg
import autopilot_bridge.srv as apsrv
import time

#-----------------------------------------------------------------------
# Ugly globals

# Mappings between MAVLink mode strings and our own enumeration
# TODO replace this
mode_mav_to_enum = { 'RTL' : apmsg.Status.MODE_RALLY,
                     'MANUAL' : apmsg.Status.MODE_MANUAL,
                     'FBWA' : apmsg.Status.MODE_FBWA,
                     'FBWB' : apmsg.Status.MODE_FBWB,
                     'CIRCLE' : apmsg.Status.MODE_CIRCLE,
                     'GUIDED' : apmsg.Status.MODE_GUIDED,
                     'AUTO' : apmsg.Status.MODE_AUTO }
mode_enum_to_mav = { v:k for (k,v) in mode_mav_to_enum.items() }

#-----------------------------------------------------------------------
# MAVLink message handlers

def mav_statustext(msg_type, msg, bridge):
    # Sets the flag for calibration service (see below)
    if str(msg.text) == "Calibrating barometer":
        srv_calpress.started = True
    if str(msg.text) == "zero airspeed calibrated":
        srv_calpress.done = True

def mav_autopilot_version(msg_type, msg, bridge):
    srv_version.msg = msg

def pub_pose_att_vel(msg_type, msg, bridge):
    pub = bridge.get_ros_pub("acs_pose", apmsg.Geodometry, queue_size=1)
    odom = apmsg.Geodometry()
    odom.header.stamp = bridge.project_ap_time(msg)
    odom.header.frame_id = 'base_footprint'
    # Position (lat/lon/alt, with two alts)
    odom.pose.pose.position.lat = msg.lat/1e07
    odom.pose.pose.position.lon = msg.lon/1e07
    odom.pose.pose.position.alt = msg.alt/1e02
    odom.pose.pose.position.rel_alt = msg.relative_alt/1e02
    odom.pose.pose.position.using_alt = True
    odom.pose.pose.position.using_rel_alt = True
    # Orientation (quaternion)
    odom.pose.pose.orientation.x = msg.quat[0]
    odom.pose.pose.orientation.y = msg.quat[1]
    odom.pose.pose.orientation.z = msg.quat[2]
    odom.pose.pose.orientation.w = msg.quat[3]
    # The covariance matrix is not yet usable
    odom.pose.covariance = ( 0.1, 0, 0, 0, 0, 0,
                             0, 0.1, 0, 0, 0, 0,
                             0, 0, 0.1, 0, 0, 0,
                             0, 0, 0, 99999, 0, 0,
                             0, 0, 0, 0, 99999, 0,
                             0, 0, 0, 0, 0, 99999 )
    # Linear and angular velocities, with unknown covariance (in m/s)
    odom.twist.twist.linear.x = msg.vx / 1e02  # north-positive
    odom.twist.twist.linear.y = msg.vy / 1e02  # east-positive
    odom.twist.twist.linear.z = msg.vz / 1e02  # down-positive
    # Angular velocities in rad/s
    odom.twist.twist.angular.x = msg.rollspeed
    odom.twist.twist.angular.y = msg.pitchspeed
    odom.twist.twist.angular.z = msg.yawspeed
    # The covariance matrix is not yet usable
    odom.twist.covariance = ( 0.1, 0, 0, 0, 0, 0,
                             0, 0.1, 0, 0, 0, 0,
                             0, 0, 0.1, 0, 0, 0,
                             0, 0, 0, 99999, 0, 0,
                             0, 0, 0, 0, 99999, 0,
                             0, 0, 0, 0, 0, 99999 )
    pub.publish(odom)

# Publish a status message with many fields
def pub_status(msg_type, msg, bridge):
    pub = bridge.get_ros_pub("status", apmsg.Status, queue_size=1)
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
# ROS service handlers

def srv_calpress(req, bridge):
    # User must supply a positive timeout in seconds
    if req.timeout <= 0:
        return { 'ok' : False }

    # Reset attributes
    srv_calpress.started = False
    srv_calpress.done = False

    # Cycle, waiting for flag to be raised (see mavlink handler above)
    end_time = time.time() + req.timeout
    while time.time() < end_time:
        if not srv_calpress.started:
            # Retry as needed
            bridge.master.calibrate_pressure()
        time.sleep(0.5)  # NOTE: may round to next increment
        if srv_calpress.done:
            return { 'ok' : True }
    return { 'ok' : False }

# Initialize attributes
srv_calpress.started = False
srv_calpress.done = False

def srv_version(req, bridge):
    # User must supply a positive timeout in seconds
    if req.timeout <= 0:
        return { 'ok' : False }

    # Init/reset variables
    srv_version.msg = None
    rsp = apsrv.VersionResponse()
    rsp.ok = False

    # Cycle, waiting for flag to be raised (see mavlink handler above)
    end_time = time.time() + req.timeout
    while time.time() < end_time:
        # (re)send request each cycle
        bridge.master.mav.autopilot_version_request_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_TYPE_GENERIC)
        time.sleep(0.2)  # NOTE: may round to next 0.2s increment
        if srv_version.msg is not None:
            print str(srv_version.msg)
            # Shorter variable names for less typing
            msg = srv_version.msg
            mav = mavutil.mavlink

            rsp.ok = True

            rsp.cap_mission_float = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT
            rsp.cap_param_float = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
            rsp.cap_mission_int = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_MISSION_INT
            rsp.cap_command_int = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_COMMAND_INT
            rsp.cap_param_union = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_PARAM_UNION
            rsp.cap_ftp = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_FTP
            rsp.cap_set_attitude = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
            rsp.cap_set_position_local = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED
            rsp.cap_set_position_global = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
            rsp.cap_terrain = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_TERRAIN
            rsp.cap_set_actuator = msg.capabilities & \
                mav.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET

            rsp.flight_sw = msg.flight_sw_version
            rsp.middleware = msg.middleware_sw_version
            rsp.os_sw = msg.os_sw_version
            rsp.board = msg.board_version
            rsp.flight_custom = ''.join(map(lambda x: chr(x),
                                            msg.flight_custom_version))
            rsp.middleware_custom = ''.join(map(lambda x: chr(x),
                                                msg.middleware_custom_version))
            rsp.os_custom = ''.join(map(lambda x: chr(x),
                                        msg.os_custom_version))
            rsp.vendor_id = msg.vendor_id
            rsp.product_id = msg.product_id
            rsp.uid = msg.uid

            break
    return rsp

#-----------------------------------------------------------------------
# ROS subscriber handlers

# Purpose: Payload-to-autopilot heartbeast, indicates payload is healthy
# NOTE: Not yet supported in MAVLink master branch
# Fields: None
def sub_heartbeat_onboard(message, bridge):
    bridge.master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_TYPE_GENERIC,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE)

# Purpose: Ground-to-air heartbeat, indicates link from GCS
def sub_heartbeat_ground(message, bridge):
    # TODO check for timeliness
    bridge.master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_TYPE_GENERIC,
        0, 0, 0)

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

# Purpose: Go to a lat/lon/alt in AUTO mode ordered by payload
# Fields:
# .lat - Decimal degrees
# .lon - Decimal degrees
# .alt - Decimal meters **AGL wrt home**
def sub_payload_waypoint(message, bridge):
    bridge.master.mav.command_long_send(
        bridge.master.target_system,
        bridge.master.target_component,
        mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO,
        0, 0, 0, 0, 0,
        message.lat, message.lon, message.alt)

# Purpose: reboot the autopilot
# NOTE: core should realize the autopilot rebooted and fix timing
def sub_reboot(message, bridge):
    bridge.master.reboot_autopilot()

#-----------------------------------------------------------------------
# init()

def init(bridge):
    bridge.add_mavlink_event("STATUSTEXT", mav_statustext)
    bridge.add_mavlink_event("AUTOPILOT_VERSION", mav_autopilot_version)
    bridge.add_mavlink_event("HEARTBEAT", pub_status)
    bridge.add_mavlink_event("GLOBAL_POS_ATT_NED", pub_pose_att_vel)
    bridge.add_ros_srv_event("calpress", apsrv.TimedAction, srv_calpress)
    bridge.add_ros_srv_event("version", apsrv.Version, srv_version)
    bridge.add_ros_sub_event("heartbeat_onboard", apmsg.Heartbeat, sub_heartbeat_onboard, log=False)
    bridge.add_ros_sub_event("heartbeat_ground", apmsg.Heartbeat, sub_heartbeat_ground, log=False)
    bridge.add_ros_sub_event("mode_num", stdmsg.UInt8, sub_change_mode)
    bridge.add_ros_sub_event("land", stdmsg.Empty, sub_landing)
    bridge.add_ros_sub_event("land_abort", stdmsg.UInt16, sub_landing_abort)
    bridge.add_ros_sub_event("payload_waypoint", apmsg.LLA, sub_payload_waypoint, log=False) 
    bridge.add_ros_sub_event("reboot", stdmsg.Empty, sub_reboot)
    return True
