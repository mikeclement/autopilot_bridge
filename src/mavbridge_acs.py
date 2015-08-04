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

import rospy
from pymavlink import mavutil
import std_msgs.msg as stdmsg
import std_srvs.srv as stdsrv
import autopilot_bridge.msg as apmsg
import autopilot_bridge.srv as apsrv
import time
from threading import RLock
import math

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
                     'AUTO' : apmsg.Status.MODE_AUTO,
                     'LOITER' : apmsg.Status.MODE_LOITER,
                     'INITIALIZING' : apmsg.Status.MODE_INITIALIZING}
mode_enum_to_mav = { v:k for (k,v) in mode_mav_to_enum.items() }

#-----------------------------------------------------------------------
# MAVLink message handlers

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
    master = bridge.get_master()
    pub = bridge.get_ros_pub("status", apmsg.Status, queue_size=1)
    sta = apmsg.Status()
    sta.header.stamp = bridge.project_ap_time()
    if master.flightmode in mode_mav_to_enum:
        sta.mode = mode_mav_to_enum[master.flightmode]
    else:
        sta.mode = apmsg.Status.MODE_UNKNOWN
    sta.armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    sta.ahrs_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_AHRS)
    sta.alt_rel = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0)
    sta.as_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)
    sta.as_read = master.field('VFR_HUD', 'airspeed', 0)
    sta.gps_ok = (master.field('GPS_RAW_INT', 'fix_type', 0) == 3)
    sta.gps_sats = master.field('GPS_RAW_INT', 'satellites_visible', 0)
    sta.gps_eph = master.field('GPS_RAW_INT', 'eph', 0)
    sta.ins_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | \
                                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO)
    sta.mag_ok = bridge.check_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG)
    sta.mis_cur = master.field('MISSION_CURRENT', 'seq', 0)
    sta.pwr_ok = not (master.field('POWER_STATUS', 'flags', 0) \
                    & mavutil.mavlink.MAV_POWER_STATUS_CHANGED)
    sta.pwr_batt_rem = master.field('SYS_STATUS', 'battery_remaining', -1)
    sta.pwr_batt_vcc = master.field('SYS_STATUS', 'voltage_battery', -1)
    sta.pwr_batt_cur = master.field('SYS_STATUS', 'current_battery', -1)
    pub.publish(sta)

#-----------------------------------------------------------------------
# ROS service handling classes

# Autopilot calibration services
class Calibrations(object):

    def __init__(self, bridge):
        self._bridge = bridge
        self._lock = RLock()
        self._running = False
        self._started = False
        self._finished = False

        # Register events
        bridge.add_mavlink_event("STATUSTEXT", self._statustext)
        bridge.add_ros_srv_event("cal_pressure", apsrv.TimedAction,
            lambda r,b: self._calibrate(r.timeout, press=True))
        bridge.add_ros_srv_event("cal_gyros", apsrv.TimedAction,
            lambda r,b: self._calibrate(r.timeout, gyro=True))

    def _statustext(self, msg_type, msg, bridge):
        # STATUSTEXT messages tell us the progress
        if str(msg.text) == "Beginning INS calibration; do not move plane":
            self._started = True
        elif str(msg.text) == "Calibrating barometer":
            self._started = True
        elif str(msg.text) == "zero airspeed calibrated":
            self._finished = True

    def _send(self, gyro, press):
        self._bridge.get_master().mav.command_long_send(
            self._bridge.get_master().target_system,
            self._bridge.get_master().target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, int(gyro), 0, int(press), 0, 0, 0, 0)

    def _calibrate(self, timeout, gyro=False, press=False):
        # User must supply a positive timeout in seconds
        if timeout <= 0:
            return { 'ok' : False }

        # Protect checking if already calibrating
        with self._lock:
            if self._running:
                return { 'ok' : False }
            self._running = True

        try:
            # Reset attributes
            self._started = False
            self._finished = False
            end_time = time.time() + timeout

            # Send once and wait a bit
            self._send(gyro, press)
            time.sleep(2.0)

            # Cycle, trying to initiate and awaiting response
            while time.time() < end_time:
                # Retry if needed
                if not self._started:
                    self._send(gyro, press)

                # Give the system some time
                time.sleep(1.0)

                # See if it's finished
                if self._finished:
                    break
        except Exception as ex:
            rospy.logwarn("Calibration exception: " + str(ex.args[0]))
        finally:
            # Release for other users to re-calibrate
            with self._lock:
                self._running = False
                return { 'ok' : self._finished }

# Demo servos and motor
# NOTE: relies on the 'fpr' module being loaded for param fetches
class Demos(object):

    def __init__(self, bridge):
        self._bridge = bridge
        self._lock = RLock()
        self._running = False
        self._time = 0.0
        self._pwms = []
        self._prm = { 'RC1_MIN'  : None,
                      'RC1_TRIM' : None,
                      'RC1_MAX'  : None,
                      'RC2_MIN'  : None,
                      'RC2_TRIM' : None,
                      'RC2_MAX'  : None,
                      'RC3_MIN'  : None,
                      'RC3_MAX'  : None }

        # Register events
        bridge.add_ros_srv_event("demo_servos", stdsrv.Empty, self._servos)
        bridge.add_ros_srv_event("demo_motor", stdsrv.Empty, self._motor)

    # NOTE: Probably shouldn't use a "private" method in fpr
    def _get_params(self):
        get_param = self._bridge.get_module('fpr')._get_param
        for pn in self._prm:
            ret = get_param(pn, force=False)
            if not isinstance(ret, float):
                raise Exception("Could not fetch " + pn)
            self._prm[pn] = ret

    def _pwm_reset(self):
        self._pwms = [0] * 8

    # NOTE: This is specific to our current platform
    def _pwm_trim(self):
        self._pwm_reset()
        self._pwms[0] = self._prm['RC1_TRIM']
        self._pwms[1] = self._prm['RC2_TRIM']
        self._pwms[2] = self._prm['RC3_MIN']

    # NOTE: This does very little sanity checking
    def _pwm_set(self, chan, val):
        # If we use 'MIN', 'MAX', or 'TRIM' for val, look up param
        if isinstance(val, str):
            pn = 'RC' + str(chan) + '_' + str(val)
            if pn not in self._prm:
                raise Exception("Don't have param " + pn)
            val = self._prm[pn]
        self._pwms[chan-1] = val

    def _pwm_send(self):
        self._bridge.get_master().mav.rc_channels_override_send(
            self._bridge.get_master().target_system,
            self._bridge.get_master().target_component,
            *self._pwms)

    def _time_reset(self):
        self._time = time.time()

    # Variable-time sleep based on how long previous ops have taken
    def _time_sleep(self, max_time):
        self._time += max_time
        t = time.time()
        if self._time > t:
            time.sleep(self._time - t)

    # Runs a sequence described by ((chan, val, wait), ...)
    def _run_seq(self, seq):
        # Protect checking if already demoing
        with self._lock:
            if self._running:
                return {}
            self._running = True

        # Run the demo
        try:
            self._get_params()  # Trim all surfaces first
            self._pwm_trim()
            time.sleep(0.1)  # Let things settle
            self._time_reset()
            for chan, val, wait in seq:
                self._pwm_set(chan, val)
                self._pwm_send()
                self._time_sleep(wait)
            self._pwm_trim()  # Back to trim
            self._pwm_send()
            time.sleep(0.1)  # Let things settle
            self._pwm_reset()  # Undo overrides
            self._pwm_send()
        except Exception as ex:
            rospy.logwarn("Demo exception: " + str(ex.args[0]))
        finally:
            # Release for other users to demo
            with self._lock:
                self._running = False
            return {}

    # UP, DOWN, LEFT, RIGHT
    def _servos(self, *args):
        WAIT = 1.0
        seq = ((2, 'MAX', WAIT), (2, 'MIN', WAIT),
               (2, 'TRIM', 0.0),
               (1, 'MIN', WAIT), (1, 'MAX', WAIT))
        self._run_seq(seq)
        return {}

    def _motor(self, *args):
        SHORT = 1.0
        LONG = 2.0
        self._get_params()
        mid = (self._prm['RC3_MIN'] + self._prm['RC3_MAX']) / 2.0
        seq = ((3, mid, SHORT), (3, 'MAX', LONG), (3, mid, SHORT))
        self._run_seq(seq)
        return {}

class Version(object):

    def __init__(self, bridge):
        self._bridge = bridge
        self._msg = None

        # Register events
        bridge.add_mavlink_event("AUTOPILOT_VERSION", self._mav)
        bridge.add_ros_srv_event("version", apsrv.Version, self._version)

    def _reply(self):
        rsp = apsrv.VersionResponse()

        # Shorter variable names for less typing
        msg = self._msg
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

        return rsp

    def _mav(self, msg_type, msg, bridge):
        self._msg = msg

    def _version(self, req, bridge):
        # If we already have the message, just use it
        if self._msg:
            return self._reply()

        # User must supply a positive timeout in seconds
        if req.timeout <= 0:
            return { 'ok' : False }

        # Cycle, request and awaiting a response
        end_time = time.time() + req.timeout
        while time.time() < end_time:
            # Send the version request
            bridge.get_master().mav.autopilot_version_request_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_TYPE_GENERIC)

            # Wait a moment
            time.sleep(0.2)

            # Check for a response
            if self._msg:
                return self._reply()

        # If no response within timeout, fail
        return { 'ok' : False }

#-----------------------------------------------------------------------
# ROS subscriber handlers

# Purpose: Payload-to-autopilot heartbeast, indicates payload is healthy
# NOTE: Not yet supported in MAVLink master branch
# Fields: None
def sub_heartbeat_onboard(message, bridge):
    bridge.get_master().mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_TYPE_GENERIC,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE)

# Purpose: Ground-to-air heartbeat, indicates link from GCS
def sub_heartbeat_ground(message, bridge):
    # TODO check for timeliness
    bridge.get_master().mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_TYPE_GENERIC,
        0, 0, 0)

# Purpose: Changes autopilot mode
# (must be in mode_mapping dictionary)
# Fields:
# .data - index from mode_mav_to_enum
def sub_change_mode(message, bridge):
    mav_map = bridge.get_master().mode_mapping()
    if message.data in mode_enum_to_mav and \
        mode_enum_to_mav[message.data] in mav_map:
        bridge.get_master().set_mode(mav_map[mode_enum_to_mav[message.data]])
    else:
        raise Exception("invalid mode %u" % message.data)

# Purpose: Initiates landing
# NOTE: Not yet supported in MAVLink master branch
# Fields: None
def sub_landing(message, bridge):
    bridge.get_master().mav.command_long_send(
        bridge.get_master().target_system,
        bridge.get_master().target_component,
        mavutil.mavlink.MAV_CMD_DO_RALLY_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)

# Purpose: Aborts landing
# NOTE: Not yet supported in MAVLink master branch
# Fields: 
# .data - Altitude to return to (meters, integer)
def sub_landing_abort(message, bridge):
    # TODO: Do we need to repeat until we see RTL again??
    bridge.get_master().mav.command_long_send(
        bridge.get_master().target_system,
        bridge.get_master().target_component,
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
    bridge.get_master().mav.command_long_send(
        bridge.get_master().target_system,
        bridge.get_master().target_component,
        mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO,
        0, 0, 0, 0, 0,
        message.lat, message.lon, message.alt)

# Purpose: reboot the autopilot
# NOTE: core should realize the autopilot rebooted and fix timing
def sub_reboot(message, bridge):
    bridge.get_master().reboot_autopilot()

# Purpose: update the autopilot ground weather settings
def sub_weather_update(message, bridge):
    bridge.get_master().mav.raw_pressure_send(
            int(bridge.project_ap_time().to_nsec() / 1000.0),
            int(message.baro_millibars * 10.0), 0, 0, 
            int(message.temp_C * 100.0))
                
#-----------------------------------------------------------------------
# init()

# Initialize Demo object

def init(bridge):
    # MAVLink events
    bridge.add_mavlink_event("HEARTBEAT", pub_status)
    bridge.add_mavlink_event("GLOBAL_POS_ATT_NED", pub_pose_att_vel)

    # (Fairly) idempotent subscribers
    bridge.add_ros_sub_event("heartbeat_onboard", apmsg.Heartbeat,
                             sub_heartbeat_onboard, log=False)
    bridge.add_ros_sub_event("heartbeat_ground", apmsg.Heartbeat,
                             sub_heartbeat_ground, log=False)
    bridge.add_ros_sub_event("mode_num", stdmsg.UInt8, sub_change_mode)
    bridge.add_ros_sub_event("land", stdmsg.Empty, sub_landing)
    bridge.add_ros_sub_event("land_abort", stdmsg.UInt16, sub_landing_abort)
    bridge.add_ros_sub_event("payload_waypoint", apmsg.LLA,
                             sub_payload_waypoint, log=False)
    bridge.add_ros_sub_event("reboot", stdmsg.Empty, sub_reboot)
    bridge.add_ros_sub_event("weather_update", apmsg.WeatherData, sub_weather_update) 

    # Services handled in classes
    acs_cal = Calibrations(bridge)
    acs_demo = Demos(bridge)
    acs_ver = Version(bridge)

    # Return object references just in case objects would otherwise get GC'd
    return (acs_cal, acs_demo, acs_ver)
