#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# Core class
#
# Originally developed by Mike Clement, 2014
#
# This software may be freely used, modified, and distributed.
# This software comes with no warranty.

#-----------------------------------------------------------------------
# Import libraries

# Python imports
from optparse import OptionParser
import os
import time

# ROS imports
import rospy

# mavlink imports
from pymavlink import mavutil


#-----------------------------------------------------------------------
# Class to contain info about a timed event

class MLBTimedEvent(object):
    def __init__(self, interval, callback):
        self._interval = interval
        self._callback = callback
        self._next_time = time.time() + self._interval

    def due(self, t=None):
        if not t:
            t = time.time()
        return bool(self._next_time <= t)

    def run(self, bridge, t=None):
        self._callback(bridge)
        if not t:
            t = time.time()
        self._next_time = t + self._interval

#-----------------------------------------------------------------------
# Class that wraps mavlink connection object

class MAVLinkBridge(object):

    ### Initialization ###

    def __init__(self, device, baudrate,
                 basename='autopilot',
                 mavlink_rate=10.0,
                 loop_rate=50.0,
                 sync_local_clock=False,
                 track_time_delta=False,
                 spam_mavlink=False):

        # Settings that get used later
        self.basename = basename
        self.spam_mavlink = spam_mavlink
        self.loop_rate = loop_rate
        self.track_time_delta=track_time_delta

        # Internal variables
        self.master = None
        self.mav_events = {}
        self.mav_events_all = []
        self.ros_sub_events = {}
        self.ros_srv_events = {}
        self.ros_pubs = {}
        self.timed_events = []
        self.ap_time_delta = rospy.Duration(0, 0)
        self.ap_boot_time = None

        # Initialize ROS node
        try:
            rospy.init_node(basename)
        except Exception as ex:
            raise Exception("ROS init error: " + ex.args[0])

        # Initialize mavlink connection
        try:
            # Open connection
            self.master = mavutil.mavlink_connection(
                device,
                int(baudrate),
                autoreconnect=True)

            # Wait for a heartbeat so we know the target system IDs
            self.master.wait_heartbeat()

            # Set up output stream from master
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                mavlink_rate,
                1)
        except Exception as ex:
            raise Exception("MAVLink init error: " + str(ex.args[0]))

        # If requested, attempt to sync local clock to autopilot
        if (sync_local_clock):
            rospy.loginfo("Waiting for nonzero SYSTEM_TIME message to sync local clock...")
            if self._sync_local_clock():
                rospy.loginfo("Successfully synced local clock.")
            else:
                rospy.logwarn("Failed to sync local clock!")
        else:
            rospy.logwarn("Warning: local clock may differ from autopilot time.")

    ### Useful functions for modular handlers ###

    # Return projected current time of AP (UTC) as rospy.Time object
    # If a MAVLink message object with the 'time_boot_ms' attribute is given,
    #   project the actual AP time using the boot-time offset.
    # Otherwise, use current system time as the basis.
    # If track_time_delta is True, make adjustment based on timing observations
    def project_ap_time(self, msg=None):
        if msg is not None and self.ap_boot_time is not None and hasattr(msg, 'time_boot_ms'):
            t = self.ap_boot_time + rospy.Duration.from_sec(float(msg.time_boot_ms) / 1e03)
        else:
            t = rospy.Time.now()
        return t - self.ap_time_delta

    # Check whether sensor is present and healthy, based on last SYS_STATUS message
    def check_sensor_health(self, sensor_bits):
        present = self.master.field('SYS_STATUS', 'onboard_control_sensors_enabled', 0)
        present = ((present & sensor_bits) == sensor_bits)
        healthy = self.master.field('SYS_STATUS', 'onboard_control_sensors_health', 0)
        healthy = ((healthy & sensor_bits) == sensor_bits)
        return (present and healthy)

    # Get a ROS publisher for a given topic (within basename) and message type
    # If matching publisher exists, it is returned; otherwise it is created.
    # Enforces one message type per topic
    # NOTE: Can pass optional args to rospy.Publisher() in **pub_args;
    #  of course, all users of the publisher will be subject to same options
    def get_ros_pub(self, topic, topic_type, **pub_args):
        if topic in self.ros_pubs:
            (t_type, t_pub) = self.ros_pubs[topic]
            if t_type == topic_type:
                return t_pub
            else:
                raise Exception("topic '%s' already registered with a different type" % topic)
        else:
            try:
                pub = rospy.Publisher("%s/%s"%(self.basename, topic), topic_type, **pub_args)
                self.ros_pubs[topic] = (topic_type, pub)
                return pub
            except Exception as ex:
                raise Exception("failed to create topic %s: %s" % (topic, ex.args[0]))

    ### Add handlers of various sorts ###

    # Add a callback that triggers on receipt of a MAVLink message type
    # callback must be of the form:
    #   foo(message_type, message, bridge_object)
    # NOTE: if message_type == '*', will get called for ALL messages (almost)
    def add_mavlink_event(self, message_type, callback):
        if message_type == '*':
            self.mav_events_all.append(callback)
        elif message_type in self.mav_events:
            self.mav_events[message_type].append(callback)
        else:
            self.mav_events[message_type] = [callback]

    # Add a callback that triggers on receipt of a ROS topic message
    # Callback should be of the form:
    #   foo(message, bridge_object)
    # NOTE: Can pass optional args to rospy.Subscriber() in *sub_args
    def add_ros_sub_event(self, topic, topic_type, callback, log=True, *sub_args):
        if topic in self.ros_sub_events:
            raise Exception("topic '%s' already has a subscriber" % topic)
        else:
            # This creates a closure with 'topic', 'callback', etc inside
            def callback_wrapper(msg):
                try:
                    callback(msg, self)
                    if log:
                        rospy.loginfo("ROS sub recvd: %s" % topic)
                except Exception as ex:
                    rospy.logwarn("ROS sub error (%s): %s" % (topic, ex.args[0]))
                return True
            self.ros_sub_events[topic] = \
                rospy.Subscriber("%s/%s"%(self.basename, topic),
                                 topic_type,
                                 callback_wrapper,
                                 *sub_args)

    # Add a callback that triggers on receipt of a ROS service request
    # Callback should be of the form:
    #   foo(request, bridge_object)
    # NOTE: Can pass optional args to rospy.Service() in *srv_args
    def add_ros_srv_event(self, srv_name, srv_type, callback, log=True, *srv_args):
        if srv_name in self.ros_srv_events:
            raise Exception("service '%s' is already defined" % srv_name)
        else:
            # This creates a closure and adds some queueing logic
            def callback_wrapper(req):
                try:
                    # TODO: Add queueing logic to prevent service calls
                    #  from stepping on each other
                    if log:
                        rospy.loginfo("ROS srv recvd: %s" % srv_name)
                    return callback(req, self)
                except Exception as ex:
                    rospy.logwarn("ROS srv error (%s): %s" % (srv_name, ex.args[0]))
                # TODO: return something the caller can handle
                return True
            self.ros_srv_events[srv_name] = \
                rospy.Service("%s/%s"%(self.basename, srv_name),
                              srv_type,
                              callback_wrapper,
                              *srv_args)

    # Add a callback that triggers periodically
    # Callback should be of the form:
    #   foo(bridge_object)
    def add_timed_event(self, hz, callback):
        interval = 0
        if hz == 0:
            # 0 Hz means run as fast as possible (don't do this unless you mean it!)
            interval = 0
        else:
            interval = 1.0 / float(hz)
        self.timed_events.append(MLBTimedEvent(interval, callback))

    ### Main loop ###

    # Start main loop
    # NOTE: This won't return until ROS is terminated
    def run_loop(self):
        self._mainloop()

    #----------------------------------------------------------------------#
    ### Internals ###

    # Process a SYSTEM_TIME message and return componentized seconds
    def _process_system_time(self, msg):
        # Cannot use if the message has a zero time in it
        if msg.time_unix_usec == 0:
            return None

        # Get local system time for calculations
        local_time = time.time()

        # Calculate adjusted autopilot time, accounting for processing latencies
        # NOTE: would like to determine transmission latency, but lack info
        snd_rcv_time = float(msg.time_unix_usec) / 1e06 - msg._timestamp
        adjusted_time = local_time + snd_rcv_time

        # If tracking time delta, update the delta
        # Using simplified numerical solution, but really local_time - adjusted_time
        if self.track_time_delta:
            self.ap_time_delta = rospy.Time.from_sec(0.0 - snd_rcv_time)

        # Record the time at which the autopilot booted (when time_boot_ms == 0)
        if self.ap_boot_time is None:
            self.ap_boot_time = rospy.Time.from_sec(
                                    float(msg.time_unix_usec) / 1e06 - \
                                    float(msg.time_boot_ms) / 1e03)

        return adjusted_time

    # Set system clock based on SYSTEM_TIME message
    # NOTE: May not return if usable SYSTEM_TIME message not seen
    # NOTE: Requires ability to set date using 'date' command
    def _sync_local_clock(self):
        msg = None
        while True:
            msg = self.master.recv_match(type='SYSTEM_TIME', blocking=True)
            if msg.time_unix_usec:
                break
        t = self._process_system_time(msg)
        if t is None:
            return False
        return not bool(os.system("sudo date -s '@%f'" % t))

    # Handle a received MAVLink message
    def _handle_msg(self, msg):
        msg_type = msg.get_type()

        # Handle special cases
        if msg_type == "BAD_DATA":
            return  # Ignore "bad data"
        elif msg_type == "SYSTEM_TIME":
            self._process_system_time(msg)

        # If you *really* want to see what's coming out of mavlink
        if self.spam_mavlink:
            print msg_type + " @ " + str(msg._timestamp) + ":\n  " \
                + "\n  ".join("%s: %s" % (k, v) for (k, v) \
                              in sorted(vars(msg).items()) \
                              if not k.startswith('_'))

        # Run any all-types events
        for ev in self.mav_events_all:
            try:
                ev(msg_type, msg, self)
            except Exception as ex:
                rospy.logwarn("MAVLink wildcard event error (%s): %s" % \
                              (msg_type, ex.args[0]))

        # Run any type-specific events
        if msg_type not in self.mav_events:
            return
        for ev in self.mav_events[msg_type]:
            try:
                ev(msg_type, msg, self)
            except Exception as ex:
                rospy.logwarn("MAVLink event error (%s): %s" % \
                              (msg_type, ex.args[0]))

    # Handle timed events
    def _handle_timed(self):
        t = time.time()
        for ev in [ev for ev in self.timed_events if ev.due(t)]:
            # NOTE: technically, want to use the time when the event
            # actually ran, but this is slightly more efficient.
            ev.run(self, t)

    # Main loop that receives and handles mavlink messages
    def _mainloop(self):
        # Try to run this loop at >= LOOP_RATE Hz
        r = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            # Check for a MAVLink message
            msg = None
            try:
                msg = self.master.recv_match(blocking=False)
            except Exception as ex:
                rospy.logwarn("MAV Recv error: " + ex.args[0])

            # Process the message
            if msg:
                self._handle_msg(msg)

            # Handle any timed tasks
            self._handle_timed()

            # If there were no new messages, sleep for a bit
            # NOTE: If messages are coming in fast enough, we won't sleep
            if not msg:
                r.sleep()

