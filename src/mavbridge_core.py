#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# Core class
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

# Python imports
from optparse import OptionParser
import os

# ROS imports
import rospy

# mavlink imports
from pymavlink import mavutil


#-----------------------------------------------------------------------
# Class that wraps mavlink connection object

class MAVLinkBridge(object):

    ### Initialization ###

    def __init__(self, device, baudrate,
                 basename='autopilot',
                 mavlink_rate=10.0,
                 loop_rate=10.0,
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
            raise Exception("MAVLink init error: " + ex.args[0])

        # If requested, attempt to sync local clock to autopilot
        if (sync_local_clock):
            rospy.log_info("Waiting for nonzero SYSTEM_TIME message to sync local clock...")
            if self._sync_local_clock():
                rospy.loginfo("Successfully synced local clock.")
            else:
                rospy.logwarn("Failed to sync local clock!")
        else:
            rospy.logwarn("Warning: local clock may differ from autopilot time.")

    ### Useful functions for modular handlers ###

    # Return projected current time of AP (UTC) as rospy.Time object
    # If track_time_delta is False, this should always return rospy.Time.now()
    def project_ap_time(self):
        return rospy.Time.now() - self.ap_time_delta

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
    # NOTE: Can pass optional args to rospy.Publisher() in *pub_args;
    #  of course, all users of the publisher will be subject to same options
    def get_ros_pub(self, topic, topic_type, *pub_args):
        if topic in self.ros_pubs:
            (t_type, t_pub) = self.ros_pubs[topic]
            if t_type == topic_type:
                return t_pub
            else:
                raise Exception("topic '%s' already registered with a different type" % topic)
        else:
            try:
                pub = rospy.Publisher("%s/%s"%(self.basename, topic), topic_type, *pub_args)
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
    def add_ros_sub_event(self, topic, topic_type, callback, *sub_args):
        if topic in self.ros_sub_events:
            raise Exception("topic '%s' already has a subscriber" % topic)
        else:
            # This creates a closure with 'topic', 'callback', etc inside
            def callback_wrapper(msg):
                try:
                    callback(msg, self)
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
    def add_ros_srv_event(self, srv_name, srv_type, callback, *srv_args):
        if srv_name in self.ros_srv_events:
            raise Exception("service '%s' is already defined" % srv_name)
        else:
            # This creates a closure and adds some queueing logic
            def callback_wrapper(req):
                try:
                    # TODO: Add queueing logic to prevent service calls
                    #  from stepping on each other
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
    # NOTE: Cannot currently run a task faster than self.loop_rate Hz,
    #  and as implemented may run a bit slower (depends on _mainloop())
    # Callback should be of the form:
    #   foo(bridge_object)
    def add_timed_event(self, hz, callback):
        period = 0
        if hz == 0:
            # 0 Hz means run at loop_rate, intentionally
            period = 1
        else:
            period = int(self.loop_rate / hz)
        if period <= 0:
            raise Exception("Timed event at %d Hz exceeds maximum of %d Hz" % \
                            (hz, self.loop_rate))
        self.timed_events.append({ 'period' : period,
                                   'next_in' : period,
                                   'cb' : callback } )

    ### Main loop ###

    # Start main loop
    # NOTE: This won't return until ROS is terminated
    def run_loop(self):
        self._mainloop()

    #----------------------------------------------------------------------#
    ### Internals ###

    # Set system clock based on SYSTEM_TIME message
    # NOTE: May not return if usable SYSTEM_TIME message not seen
    # NOTE: Requires ability to set date using 'date' command
    def _sync_local_clock(self):
        msg = None
        while True:
            msg = self.master.recv_match(type='SYSTEM_TIME', blocking=True)
            if msg.time_unix_usec:
                break
        secs = int(msg.time_unix_usec / 1e06)
        nsecs = (msg.time_unix_usec % 1e06) * 1e03
        return not bool(os.system("sudo date -s '@%u.%u'" % (secs, nsecs)))

    # Update delta between local and AP time (provided in usec)
    def _update_ap_time(self, ap_epoch_usec):
        if ap_epoch_usec == 0:  # If no AP time, ignore
            return
        ap_epoch_sec = int(ap_epoch_usec / 1e06)
        ap_nsec = (ap_epoch_usec % 1e06) * 1e03
        ap_time = rospy.Time(ap_epoch_sec, ap_nsec)
        self.ap_time_delta = rospy.Time.now() - ap_time

    # Main loop that receives and handles mavlink messages
    def _mainloop(self):
        # Try to run this loop at LOOP_RATE Hz
        r = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            # Process all new messages from master
            # NOTE: if too many messages come in, we'll never break
            #  out of this loop and subscribers may not run.
            while True:
                # Check for messages, break loop if none available
                msg = None
                try:
                    msg = self.master.recv_match(blocking=False)
                except:
                    # TODO might need to reconnect to autopilot
                    rospy.logwarn("MAV Recv error: " + ex.args[0])
                    break  # NOTE: Unclear if this is the right behavior
                if not msg:
                    # No messages right now, do rest of outer loop
                    break

                msg_type = msg.get_type()

                # Handle any internal tasks and special cases
                if msg_type == "BAD_DATA":
                    # Ignore "bad data"
                    continue
                elif msg_type == "SYSTEM_TIME":
                    # Adjust known time offset from autopilot's
                    # NOTE: This method doesn't account for mavlink latency
                    if self.track_time_delta:
                        self._update_ap_time(msg.time_unix_usec)

                # Run any all-types events
                for ev in self.mav_events_all:
                    try:
                        ev(msg_type, msg, self)
                    except Exception as ex:
                        rospy.logwarn("MAVLink wildcard event error (%s): %s" % \
                                      (msg_type, ex.args[0]))

                # Run any type-specific events
                if msg_type in self.mav_events:
                    for ev in self.mav_events[msg_type]:
                        try:
                            ev(msg_type, msg, self)
                        except Exception as ex:
                            rospy.logwarn("MAVLink event error (%s): %s" % \
                                          (msg_type, ex.args[0]))

                # If you *really* want to see what's coming out of mavlink
                if self.spam_mavlink:
                    print msg_type + " @ " + str(msg._timestamp) + ":\n  " \
                        + "\n  ".join("%s: %s" % (k, v) for (k, v) \
                                      in sorted(vars(msg).items()) \
                                      if not k.startswith('_'))

            # Run any time-based events that are due
            for ev in self.timed_events:
                ev['next_in'] -= 1

                if ev['next_in'] <= 0:
                    try:
                        cb = ev['cb']
                        cb(self)
                    except Exception as ex:
                        rospy.logwarn("Timed event error: %s" % ex.args[0])
                    finally:
                        ev['next_in'] = ev['period']
            

            # Sleep so ROS subscribers (and services) can run
            r.sleep()

