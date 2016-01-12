#!/usr/bin/env python
#
# MAVLinkBridge.py - Core class
#
#Written in 2015 by the Advanced Robotic Systems Engineering Laboratory at the
#U.S. Naval Postgraduate School, Monterey, California, USA.
#
#Pursuant to 17 USC 105, this work is not subject to copyright in the
#United States and is in the public domain.
#
#THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
#REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
#AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
#INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
#LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
#OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
#PERFORMANCE OF THIS SOFTWARE.
#

#-----------------------------------------------------------------------
# Import libraries

# Python imports
import glob
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
        self._next_time += self._interval

#-----------------------------------------------------------------------
# Class that wraps mavlink connection object

class MAVLinkBridge(object):

    ### Initialization and Module Loading ###

    def __init__(self, device, baudrate,
                 basename='autopilot',     # ROS topic basename
                 mavlink_rate=10.0,        # MAVLink message request rate (Hz)
                 loop_rate=50.0,           # Nominal internal loop rate (Hz)
                 sync_local_clock=False,   # Sync system time from SYSTEM_TIME
                 serial_relief=0,          # Limit serial backlog size (bytes)
                 spam_mavlink=False):      # Print ALL MAVLink messages

        # Settings that get used later
        self._device = device
        self._baudrate = baudrate
        self._basename = basename
        self._spam_mavlink = spam_mavlink
        self._loop_rate = loop_rate
        self._serial_relief = serial_relief
        self._mavlink_rate = mavlink_rate

        # Internal variables
        self._master = None
        self._modules = {}
        self._mav_events = {}
        self._mav_events_all = []
        self._ros_sub_events = {}
        self._ros_srv_events = {}
        self._ros_pubs = {}
        self._timed_events = []
        self._ap_boot_time = None

        # Initialize ROS node
        try:
            rospy.init_node(basename)
        except Exception as ex:
            raise Exception("ROS init error: " + str(ex))

        # Initialize mavlink connection
        try:
            self._initialize(sync_local_clock)
        except Exception as ex:
            raise Exception("MAVLink init error: " + str(ex))

    # Add in handlers from a module file
    # NOTE: Must be an object, perhaps loaded using __import__() / reload()
    def add_module(self, mod_name, mod_obj):
        if mod_name in self._modules:
            return
        self._modules[mod_name] = mod_obj.init(self)

    ### Useful functions for modular handlers ###

    # Provide access to underlying MAVLink connection
    def get_master(self):
        return self._master

    # Provide access to loaded module objects
    # NOTE: Most useful if module's init() returns an object instance
    def get_module(self, m_name):
        if m_name in self._modules:
            return self._modules[m_name]
        return None

    # Return projected current time of AP (UTC) as rospy.Time object
    # If a MAVLink message object with the 'time_boot_ms' attribute is given,
    #   project the actual message time using the boot-time offset.
    # Otherwise, use current system time as the basis.
    def project_ap_time(self, msg=None):
        if msg is not None and \
           self._ap_boot_time is not None and \
           hasattr(msg, 'time_boot_ms'):
            return self._ap_boot_time + \
                   rospy.Duration.from_sec(float(msg.time_boot_ms) / 1e03)
        return rospy.Time.now()

    # Check whether sensor is present and healthy, based on last SYS_STATUS message
    def check_sensor_health(self, sensor_bits):
        present = self._master.field('SYS_STATUS', 'onboard_control_sensors_enabled', 0)
        present = ((present & sensor_bits) == sensor_bits)
        healthy = self._master.field('SYS_STATUS', 'onboard_control_sensors_health', 0)
        healthy = ((healthy & sensor_bits) == sensor_bits)
        return (present and healthy)

    # Get a ROS publisher for a given topic (within basename) and message type
    # If matching publisher exists, it is returned; otherwise it is created.
    # Enforces one message type per topic
    # NOTE: Can pass optional args to rospy.Publisher() in **pub_args;
    #  of course, all users of the publisher will be subject to same options
    def get_ros_pub(self, topic, topic_type, **pub_args):
        if topic in self._ros_pubs:
            (t_type, t_pub) = self._ros_pubs[topic]
            if t_type == topic_type:
                return t_pub
            else:
                raise Exception("topic '%s' already registered with a different type" % topic)
        else:
            try:
                pub = rospy.Publisher("%s/%s"%(self._basename, topic), topic_type, **pub_args)
                self._ros_pubs[topic] = (topic_type, pub)
                return pub
            except Exception as ex:
                raise Exception("failed to create topic %s: %s" % (topic, str(ex)))

    ### Add handlers of various sorts ###

    # Add a callback that triggers on receipt of a MAVLink message type
    # callback must be of the form:
    #   foo(message_type, message, bridge_object)
    # NOTE: if message_type == '*', will get called for ALL messages (almost)
    def add_mavlink_event(self, message_type, callback):
        if message_type == '*':
            self._mav_events_all.append(callback)
        elif message_type in self._mav_events:
            self._mav_events[message_type].append(callback)
        else:
            self._mav_events[message_type] = [callback]

    # Add a callback that triggers on receipt of a ROS topic message
    # Callback should be of the form:
    #   foo(message, bridge_object)
    # NOTE: Can pass optional args to rospy.Subscriber() in *sub_args
    def add_ros_sub_event(self, topic, topic_type, callback, log=True, *sub_args):
        if topic in self._ros_sub_events:
            raise Exception("topic '%s' already has a subscriber" % topic)
        else:
            # This creates a closure with 'topic', 'callback', etc inside
            def callback_wrapper(msg):
                try:
                    callback(msg, self)
                    if log:
                        rospy.loginfo("ROS sub recvd: %s" % topic)
                except Exception as ex:
                    rospy.logwarn("ROS sub error (%s): %s" % (topic, str(ex)))
                return True
            self._ros_sub_events[topic] = \
                rospy.Subscriber("%s/%s"%(self._basename, topic),
                                 topic_type,
                                 callback_wrapper,
                                 *sub_args)

    # Add a callback that triggers on receipt of a ROS service request
    # Callback should be of the form:
    #   foo(request, bridge_object)
    # NOTE: Can pass optional args to rospy.Service() in *srv_args
    def add_ros_srv_event(self, srv_name, srv_type, callback, log=True, *srv_args):
        if srv_name in self._ros_srv_events:
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
                    rospy.logwarn("ROS srv error (%s): %s" % (srv_name, str(ex)))
                # TODO: return something the caller can handle
                return True
            self._ros_srv_events[srv_name] = \
                rospy.Service("%s/%s"%(self._basename, srv_name),
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
        self._timed_events.append(MLBTimedEvent(interval, callback))

    ### Main loop ###

    # Start main loop
    # NOTE: This won't return until ROS is terminated
    def run_loop(self):
        while not rospy.is_shutdown():
            try:
                self._mainloop()
            except Exception as ex:
                rospy.logwarn("MAVLinkBridge loop restart: " + str(ex))

    #----------------------------------------------------------------------#
    ### Internals ###

    # Perform initialization of MAVLink stream
    # Necessary on startup and if connection fails
    def _initialize(self, sync=False):
        # Try to close any old connection first
        if self._master is not None:
            try:
                self._master.close()
            except:
                pass

        # Cycle until device becomes available and we can connect to it
        device_not_found_once = False
        while not rospy.is_shutdown():
            device = self._device

            # If not a network connection, check for matching file
            if device is None or ':' not in device:
                device = self._find_device(device)

            # If no device found
            if device is None:
                # Display a warning (first time)
                if not device_not_found_once:
                    rospy.logwarn("MAVLinkBridge: no device found, " + \
                                  "will keep checking ...")
                    device_not_found_once = True

                # Wait a moment, then try again
                time.sleep(1)
                continue

            # Try to create connection
            try:
                rospy.loginfo("MAVLinkBridge: Connecting to %s ..." % device)
                self._master = mavutil.mavlink_connection(
                    device,
                    int(self._baudrate),
                    autoreconnect=True)
            except Exception as ex:
                rospy.logwarn("MAVLinkBridge: " + str(ex))

            # If succeeded, move on to rest of init
            break

            # If failed, wait a moment then try again
            time.sleep(1)

        # Wait for a heartbeat so we know the target system IDs
        rospy.loginfo("MAVLinkBridge: Waiting for autopilot heartbeat ...")
        self._master.wait_heartbeat()

        # Set up output stream from master
        self._master.mav.request_data_stream_send(
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            self._mavlink_rate,
            1)

        # Initialize clock reference
        self._init_time(sync)
        rospy.loginfo("MAVLinkBridge initialized.")

    # Find a matching device by name, or search for serial devices
    def _find_device(self, device):
        # If auto-detecting, find a device to try
        # NOTE: based on pymavlink, with some mods for our typical use
        if device is None:
            # Generic search order
            devlist = glob.glob('/dev/ttyUSB*') + \
                      glob.glob('/dev/ttyACM*') + \
                      glob.glob('/dev/ttyAMA*') + \
                      glob.glob('/dev/serial/by-id/*') + \
                      glob.glob('/dev/ttyS*')
        else:
            # Handles '*' in device name, or exact device
            devlist = glob.glob(device)
        if len(devlist) == 0:
            return None
        return devlist[0]

    # Initialize local clock and time variables
    # Can either sync with AP time or use the local clock; either way,
    # want to know when AP booted so we can project other message times
    # NOTE: We toyed with using locks to prevent time access during
    # time initialization, but it doesn't seem helpful.
    def _init_time(self, sync=False):
        # Wait for a valid SYSTEM_TIME message
        rospy.loginfo("MAVLinkBridge: Waiting for usable SYSTEM_TIME ...")
        msg = None
        while msg is None:
            msg = self._master.recv_match(type='SYSTEM_TIME', blocking=True)
            if not msg.time_boot_ms:  # MUST determine boot time
                msg = None
            if sync and not msg.time_unix_usec:  # If syncing, need GPS time
                msg = None

        # Decide whether to base time off autopilot or payload clock,
        # in either case use a timestamp associated with msg
        if sync:
            ref_time = float(msg.time_unix_usec) / 1e06
        else:
            ref_time = msg._timestamp

        # Record the real time when the AP booted (when time_boot_ms == 0)
        self._ap_boot_time = rospy.Time.from_sec(
                                ref_time - float(msg.time_boot_ms) / 1e03)

        # If not updating the local clock, then we're done
        if not sync:
            rospy.logwarn("Warning: local clock may differ from AP clock.")
            return

        # Sync local clock to AP time, fast-forwarded by processing delay
        # NOTE: If we knew the serial latency, we'd account for that here
        ap_time = float(msg.time_unix_usec) / 1e06 + time.time() - \
                  msg._timestamp
        if bool(os.system("sudo date -s '@%f'" % ap_time)):
            rospy.logwarn("Failed to sync local clock!")
        else:
            rospy.loginfo("Successfully synced local clock.")

    # Handle special STATUSTEXT messages
    def _handle_statustext(self, msg):
        # Autopilot rebooted, need to re-init time
        if 'Ready to FLY.' in msg.text:
            self._init_time()

        # NOTE: Filtering out spam messages with little content
        if msg.text == "command received: ":
            return

        # Log rest of messages
        rospy.loginfo("MAVLink STATUSTEXT: %s" % msg.text)

    # Handle a received MAVLink message
    def _handle_msg(self, msg):
        msg_type = msg.get_type()

        # Handle special cases
        if msg_type == "BAD_DATA":
            return  # Ignore "bad data"
        elif msg_type == "STATUSTEXT":
            self._handle_statustext(msg)

        # If you *really* want to see what's coming out of mavlink
        if self._spam_mavlink:
            print msg_type + " @ " + str(msg._timestamp) + ":\n  " \
                + "\n  ".join("%s: %s" % (k, v) for (k, v) \
                              in sorted(vars(msg).items()) \
                              if not k.startswith('_'))

        # Run any all-types events
        for ev in self._mav_events_all:
            try:
                ev(msg_type, msg, self)
            except Exception as ex:
                rospy.logwarn("MAVLink wildcard event error (%s): %s" % \
                              (msg_type, str(ex)))

        # Run any type-specific events
        if msg_type not in self._mav_events:
            return
        for ev in self._mav_events[msg_type]:
            try:
                ev(msg_type, msg, self)
            except Exception as ex:
                rospy.logwarn("MAVLink event error (%s): %s" % \
                              (msg_type, str(ex)))

    # Handle timed events
    def _handle_timed(self, t=None):
        if not t:
            t = time.time()
        for ev in self._timed_events:
            if ev.due(t):
                # NOTE: technically, want to use the time when the event
                # actually ran, but this is slightly more efficient.
                ev.run(self, t)

    # Cope with system saturation by reading and discarding data
    # when serial connection gets overwhelmed, such as by rosbag
    # NOTE: This is ugly since it relies on the internals of
    # pymavlink; if those change, this will break.
    def _handle_serial_relief(self, maxbytes):
        if not isinstance(self._master, mavutil.mavserial):
            return
        try:
            waiting = self._master.port.inWaiting()
            if waiting > maxbytes:
                self._master.port.read(waiting - maxbytes)
        except Exception as ex:
            rospy.logwarn("Serial relief error: " + str(ex))

    # Main loop that receives and handles mavlink messages
    def _mainloop(self):
        # Try to run this loop at >= LOOP_RATE Hz
        r = rospy.Rate(self._loop_rate)
        while not rospy.is_shutdown():
            # Try to keep serial line from being overwhelmed
            if self._serial_relief:
                self._handle_serial_relief(self._serial_relief)

            # Check for a MAVLink message
            msg = None
            ex_msg = None
            try:
                msg = self._master.recv_match(blocking=False)
            except Exception as ex:
                rospy.logwarn("MAV Recv error: " + str(ex))
                ex_msg = str(ex)

            # Do additional error handling
            if ex_msg and 'device disconnected' in ex_msg:
                # Try to reconnect (device might have moved)
                self._initialize()

            # Process the message
            if msg:
                self._handle_msg(msg)

            # Handle any timed tasks
            self._handle_timed()

            # If there were no new messages, sleep for a bit
            # NOTE: If messages are coming in fast enough, we won't sleep
            if not msg:
                r.sleep()
