#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# Parameter handling
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

import rospy
from pymavlink import mavutil
import autopilot_bridge.srv as apsrv
import time
import sys

#-----------------------------------------------------------------------
# Class to access mavlink params from ROS

class mavbridge_param(object):
    def __init__(self, master):
        self.master = master
        self.params = {}
        self.param_highest = -1
        self.initial_fetch_done = False

        # Start a fetch of all params
        self.master.param_fetch_all()

    # Handle incoming PARAM_VALUE messages
    # NOTE: assuming that params[] keeps up to date with any
    #  other changes because of this handler
    def mav_param_value(self, msg_type, msg, bridge):
        # Parse the name first, in case something is awry
        try:
            name = str(msg.param_id).upper()
        except:
            return

        # Store the param and track highest index we've seen (for retries)
        if name in self.params and msg.param_index == 65535:
            # This seems to happen when we set a param; don't want to set highest
            (v,t,i) = self.params[name]
            self.params[name] = (msg.param_value, msg.param_type, i)
        else:
            self.params[name] = (msg.param_value, msg.param_type, msg.param_index)
            self.param_highest = max(self.param_highest, msg.param_index)

        # NOTE: This retry mechanism is rather aggressive and may result in
        #  many redundant retries for a single param. If using this with
        #  a wireless link, consider adding a retry timeout.
        if len(self.params) < self.param_highest + 1:
            indices = [i for (k,(v,t,i)) in self.params.items()]
            highest = range(self.param_highest+1)
            missing = list(set(highest).difference(indices))
            # Ask for *all* missing parameters with lower indices than this one
            for m in missing:
                self.master.param_fetch_one(int(m))

        # Just let the user know that all params are fetched once
        elif not self.initial_fetch_done and \
            msg.param_count == self.param_highest + 1:
            self.initial_fetch_done = True
            rospy.loginfo("Finished fetching all params")

    # Handle incoming ROS service requests to get param values
    def srv_param_get(self, req, bridge):
        tries = 3
        pn = req.name.upper()
        while tries:
            if pn in self.params and self.params[pn][0] is not None:
                # If we've already fetched this value
                return { 'ok' : True, 'value' : float(self.params[pn][0]) }
            else:
                # Otherwise try asking for it (and let mav_param_value()
                #  handle the response for us)
                tries -= 1
                self.master.param_fetch_one(pn)
                time.sleep(0.5)  # TODO: this might be able to be shorter
        return { 'ok' : False, 'value' : float(0.0) }

    # Handle incoming ROS service requests to set param values
    # NOTE: Built on top of srv_param_get(); if you change that,
    #  be sure to change this accordingly
    def srv_param_set(self, req, bridge):
        # Make sure the parameter exists first
        old = self.srv_param_get(req, bridge)
        if old and not old['ok']:
            return { 'ok' : False, 'value' : float(0.0) }

        # Try to set the new param value
        self.master.param_set_send(str(req.name), float(req.value))

        # Invalidate the cached version so it has to be refetched
        (v,t,i) = self.params[req.name.upper()]
        self.params[req.name.upper()] = (None, t, i)

        # Give the autopilot a moment
        time.sleep(0.1)

        # If the newly-fetched parameter matches in value, we're good
        new = self.srv_param_get(req, bridge)
        if new and new['ok'] and new['value'] == req.value:
            return new
        else:
            return { 'ok' : False, 'value' : float(0.0) }


#-----------------------------------------------------------------------
# init()

def init(bridge):
    p_obj = mavbridge_param(bridge.master)
    bridge.add_mavlink_event("PARAM_VALUE", p_obj.mav_param_value)
    bridge.add_ros_srv_event("param_get", apsrv.Param, p_obj.srv_param_get)
    bridge.add_ros_srv_event("param_set", apsrv.Param, p_obj.srv_param_set)
    return p_obj
