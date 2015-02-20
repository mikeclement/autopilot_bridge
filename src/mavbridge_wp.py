#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# Waypoint handling
#
# Originally developed by Mike Clement, 2014
#
# This software may be freely used, modified, and distributed.
# This software comes with no warranty.

#-----------------------------------------------------------------------
# Import libraries

import rospy
from pymavlink import mavutil
import autopilot_bridge.msg as apmsg
import autopilot_bridge.srv as apsrv
import time
import sys

#-----------------------------------------------------------------------
# Class to access mavlink waypoints from ROS

class mavbridge_wp(object):
    def __init__(self):
        self.wp_list = []
        self.fetch_in_progress = False
        self.fetch_last_only = False
        self.fetch_current = -1
        self.fetch_lowest = -1
        self.fetch_highest = -1
        self.push_in_progress = False
        self.push_current = -1
        self.timeout = -1

    # Return val at array index, or None
    def _get(self, idx):
        try:
            return self.wp_list[idx]
        except:
            return None

    # Add or set a waypoint in the array
    def _set(self, idx, val):
        while len(self.wp_list) < idx + 1:
            self.wp_list.append(None)
        self.wp_list[idx] = val

    # Clear our the array
    def _clr(self):
        self.wp_list = []

    # Get count of populated entries in array
    def _cnt(self):
        return len([wp for wp in self.wp_list if wp is not None])

    # Reset timeout time
    def _reset_to(self):
        self.timeout = time.time() + 0.5  # TODO: tune this

    # Check if timed out
    def _check_to(self):
        return self.timeout <= time.time()

    # Send up the "next" waypoint
    def _push(self, bridge, seq):
        if self._get(seq):
            (f, co, cu, a, p1, p2, p3, p4, x, y, z) = self._get(seq)
            wp = mavutil.mavlink.MAVLink_mission_item_message(
                     bridge.master.target_system,
                     bridge.master.target_component,
                     seq,
                     f, co, cu, a, p1, p2, p3, p4, x, y, z)
            bridge.master.mav.send(wp)

    ''' MAVLink waypoint state machine '''

    # Handle incoming MISSION_COUNT messages
    def mav_mission_count(self, msg_type, msg, bridge):
        # Only handle if we're actively seeking the count
        if self.fetch_in_progress and self.fetch_current == -1:
            self._reset_to()
            self._clr()

            # Bound our fetching range
            if self.fetch_last_only:
                self.fetch_current = msg.count - 1
                self.fetch_highest = msg.count - 1
            else:
                self.fetch_current = self.fetch_lowest
                if self.fetch_highest < 0 or self.fetch_highest > msg.count -1:
                    self.fetch_highest = msg.count - 1

            # No waypoints to fetch in our range; we're done
            if msg.count == 0 or msg.count - 1 < self.fetch_lowest or \
               self.fetch_highest < self.fetch_lowest:
                self.fetch_in_progress = False
                return

            # Make the first request
            bridge.master.waypoint_request_send(self.fetch_current)

    # Handle incoming MISSION_ITEM messages
    def mav_mission_item(self, msg_type, msg, bridge):
        # Ignore repeated and spurious messages
        if self.fetch_in_progress and msg.seq == self.fetch_current:
            self._reset_to()
            self._set(msg.seq, (msg.frame, msg.command,
                                msg.current, msg.autocontinue,
                                msg.param1, msg.param2, msg.param3, msg.param4,
                                msg.x, msg.y, msg.z))

            if msg.seq == self.fetch_highest:
                self.fetch_in_progress = False
            else:
                self.fetch_current = msg.seq + 1
                bridge.master.waypoint_request_send(self.fetch_current)

    # Handle incoming MISSION_REQUEST messages
    def mav_mission_request(self, msg_type, msg, bridge):
        if self.push_in_progress and msg.seq < self._cnt():
            self._reset_to()
            self.push_current = msg.seq
            self._push(bridge, msg.seq)

    # Handle incoming MISSION_ACK messages
    def mav_mission_ack(self, msg_type, msg, bridge):
        # An ACK means the transaction is complete
        if self.push_in_progress and msg.type == 0:
            self.push_in_progress = False

    # Every so often, check if we need to retry any operations
    def timed_wp_retry(self, bridge):
        # Retry cases for fetching
        if self.fetch_in_progress and self._check_to():
            if self.fetch_current == -1:
                # Retry getting the count
                bridge.master.waypoint_request_list_send()
            else:
                # Retry the current waypoint index
                bridge.master.waypoint_request_send(self.fetch_current)

        # Retry cases for pushing
        if self.push_in_progress and self._check_to():
            if self.push_current == -1:
                # Retry sending the count
                bridge.master.waypoint_count_send(self._cnt())
            else:
                # Retry sending the last-requested waypoint
                self._push(bridge, self.push_current)

    ''' ROS service callbacks '''

    # Generic handler for a range of waypoints
    # NOTE: default args are what is needed to fetch ALL waypoints
    def _wp_get(self, bridge, low=0, high=-1, last_only=False):
        # Make sure another service request isn't active
        if self.fetch_in_progress or self.push_in_progress:
            return { 'ok' : False, 'wp' : [] }

        # Clear the old list
        self._clr()

        # Initialize the retry timeout, THEN change state, THEN request
        self._reset_to()
        self.fetch_current = -1
        self.fetch_lowest = low
        self.fetch_highest = high
        self.fetch_last_only = last_only
        self.fetch_in_progress = True
        bridge.master.waypoint_request_list_send()

        # Wait up to N seconds for transaction to complete
        stop_time = time.time() + 10.0  # TODO: tune this
        while self.fetch_in_progress and time.time() < stop_time:
            time.sleep(0.1)  # TODO: tune this

        if self.fetch_in_progress:
            self.fetch_in_progress = False
            return { 'ok' : False, 'wp' : [] }
        else:
            wplist = []
            for i in range(self.fetch_highest+1):
                wp = self._get(i)
                if not wp:
                    continue
                (f, co, cu, a, p1, p2, p3, p4, x, y, z) = wp
                wp_msg = apmsg.Waypoint()
                wp_msg.seq = i
                wp_msg.frame = f
                wp_msg.command = co
                wp_msg.current = bool(cu)
                wp_msg.autocontinue = bool(a)
                wp_msg.param1 = p1
                wp_msg.param2 = p2
                wp_msg.param3 = p3
                wp_msg.param4 = p4
                wp_msg.x = x
                wp_msg.y = y
                wp_msg.z = z
                wplist.append(wp_msg)
            return { 'ok' : True, 'wp' : wplist }

    # Handle incoming ROS service requests to get all waypoints
    def srv_wp_getall(self, req, bridge):
        return self._wp_get(bridge)

    # Handle incoming ROS service requests to get L..H waypoints
    def srv_wp_getrange(self, req, bridge):
        if not isinstance(req.low, int) or not isinstance(req.high, int):
            return { 'ok' : False }
        return self._wp_get(bridge, low=req.low, high=req.high)

    # Handle incoming ROS service requests to get last waypoint
    def srv_wp_getlast(self, req, bridge):
        return self._wp_get(bridge, last_only=True)

    # Handle incoming ROS service requests to get all waypoints
    def srv_wp_setall(self, req, bridge):
        # Make sure another service request isn't active
        #  and that there's at least one waypoint to push
        if self.fetch_in_progress or self.push_in_progress \
                                  or len(req.wp) == 0:
            return { 'ok' : False }

        # Load waypoints into list
        self._clr()
        for w in req.wp:
            self._set(w.seq, (w.frame, w.command, w.current, w.autocontinue,
                              w.param1, w.param2, w.param3, w.param4,
                              w.x, w.y, w.z))

        # Initialize the retry timeout, THEN change state, THEN send count
        self._reset_to()
        self.push_current = -1
        self.push_in_progress = True
        bridge.master.waypoint_count_send(self._cnt())

        # Wait up to 5 seconds for transaction to complete
        stop_time = time.time() + 5.0  # TODO: tune this
        while self.push_in_progress and time.time() < stop_time:
            time.sleep(0.1)  # TODO: tune this

        # Declare victory! (or defeat)
        if self.push_in_progress:
            self.push_in_progress = False
            return { 'ok' : False }
        else:
            return { 'ok' : True }


#-----------------------------------------------------------------------
# init()

def init(bridge):
    w_obj = mavbridge_wp()
    bridge.add_mavlink_event("MISSION_COUNT", w_obj.mav_mission_count)
    bridge.add_mavlink_event("MISSION_ITEM", w_obj.mav_mission_item)
    bridge.add_mavlink_event("MISSION_REQUEST", w_obj.mav_mission_request)
    bridge.add_mavlink_event("MISSION_ACK", w_obj.mav_mission_ack)
    bridge.add_timed_event(2, w_obj.timed_wp_retry)
    bridge.add_ros_srv_event("wp_getall", apsrv.WPGetAll, w_obj.srv_wp_getall)
    bridge.add_ros_srv_event("wp_getrange", apsrv.WPGetRange, w_obj.srv_wp_getrange)
    bridge.add_ros_srv_event("wp_getlast", apsrv.WPGetAll, w_obj.srv_wp_getlast)
    bridge.add_ros_srv_event("wp_setall", apsrv.WPSetAll, w_obj.srv_wp_setall)
    return w_obj
