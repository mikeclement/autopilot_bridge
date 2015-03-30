#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# MAVLink slaving
#
# Originally developed by Mike Clement, 2014
#
# This software may be freely used, modified, and distributed.
# This software comes with no warranty.

#-----------------------------------------------------------------------
# Import libraries

import rospy
from pymavlink import mavutil
import autopilot_bridge.srv as apsrv
import os

#-----------------------------------------------------------------------
# Class to manage slaved mavlink connections

class mavbridge_slave(object):
    # Types of messages NOT to forward either direction
    EXCEPTED_TYPES = ['BAD_DATA', 'REQUEST_DATA_STREAM']

    def __init__(self, master):
        self.master = master
        self.slaves = {}

    ''' API '''

    # Forward (almost) all incoming MAVLink messages to all slaves
    def mav_message(self, msg_type, msg, bridge):
        # Omit certain problematic message types
        if msg_type in mavbridge_slave.EXCEPTED_TYPES:
            return

        # Currently forward all non-excepted messages to ALL slaves
        for s in self.slaves:
            self.slaves[s].write(msg.get_msgbuf())

    # Periodically check for input from slave connections and forward most
    def timed_message(self, bridge):
        for s in self.slaves:
            # No internal loop, so we don't get stuck handling one
            #  talkative slave
            try:
                # Try to get and parse some messages from this slave
                buf = self.slaves[s].recv()
                msgs = self.slaves[s].mav.parse_buffer(buf)
                if not msgs:
                    continue

                # Might parse multiple messages
                for m in msgs:
                    # Only forward if not an excepted type
                    if m.get_type() not in mavbridge_slave.EXCEPTED_TYPES:
                        bridge.get_master().write(m.get_msgbuf())
            except Exception:
                rospy.logwarn("Error processing message from slave %s: %s" \
                              % (s, ex.args[0]))

    # Handle services requests to open/close slave channels
    # Channels can be specified in a few ways, including:
    #   /path/to/device,BAUDRATE
    #   PROTO:IP:PORT
    def srv_setup(self, req, bridge):
        # Establish a channel
        if req.enable:
            # If it exists already, no problem!
            if req.channel in self.slaves:
                return { 'ok' : True }

            # Parse out baudrate if supplied
            if ',' in req.channel and not os.path.exists(req.channel):
                port, baud = req.channel.split(',')
            else:
                port, baud = req.channel, 57600

            # Try to establish the channel
            try:
                self.slaves[req.channel] = \
                    mavutil.mavlink_connection(port,
                                               baud=int(baud),
                                               input=False)
            except Exception as ex:
                rospy.logwarn("Error creating slave channel '%s': %s" \
                              % (req.channel, ex.args[0]))
                return { 'ok' : False }
            return { 'ok' : True }

        # Close the channel
        else:
            # If it doesn't exist, no problem!
            if req.channel not in self.slaves:
               return { 'ok' : True }

            # Try to tear down channel
            try:
                self.slaves[req.channel].close()
            except Exception:
                pass  # Some connections don't implement close()

            # Remove the channel from the list
            self.slaves.pop(req.channel)
            return { 'ok' : True }

#-----------------------------------------------------------------------
# init()

def init(bridge):
    s_obj = mavbridge_slave(bridge.get_master())
    bridge.add_mavlink_event("*", s_obj.mav_message)
    bridge.add_ros_srv_event("slave_setup", apsrv.SlaveSetup, s_obj.srv_setup)
    bridge.add_timed_event(20, s_obj.timed_message)
    return s_obj
