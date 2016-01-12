#!/usr/bin/env python
#
# mavbridge_param.py - Legacy parameter services
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

import rospy
from pymavlink import mavutil
import autopilot_bridge.msg as apmsg
import autopilot_bridge.srv as apsrv
import time
import sys

#-----------------------------------------------------------------------
# Class to access mavlink params from ROS

class mavbridge_param(object):
    def __init__(self, master, use_cache=False):
        self.master = master
        self.params = {}

        # If using the cache instead of always re-fetching params
        # NOTE: In some cases, the autopilot may not broadcast updated params
        #  when another system (like a GCS instance) changes a value; hence
        #  DON'T use caching unless you know it works in your case.
        self.use_cache=use_cache
        self.param_highest = -1
        self.initial_fetch_done = False
        if use_cache:
            # Start a fetch of all params
            self.master.param_fetch_all()

    # If a value is in the cache, make it None (so a 'get' must refetch it)
    def _noneify_value(self, name):
        pn = name.upper()
        if pn in self.params:
            (v,t,i) = self.params[pn]
            self.params[pn] = (None, t, i)

    # Get a value
    def _get(self, name):
        tries = 3  # TODO: Tune this value
        pn = name.upper()

        # If not using cache, make sure param is re-fetched from AP
        if not self.use_cache:
            self._noneify_value(pn)

        while tries:
            if pn in self.params and self.params[pn][0] is not None:
                # If we've already fetched this value
                return float(self.params[pn][0])
            else:
                # Otherwise try asking for it (and let mav_param_value()
                #  handle the response for us)
                tries -= 1
                self.master.param_fetch_one(pn)
                time.sleep(0.2)  # TODO: Tune this value
        return None

    ''' API '''

    # Handle incoming PARAM_VALUE messages
    # NOTE: If caching, then we assume that PARAM_VALUE messages are broadcast
    #  and params[] is kept up to date by this handler
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

        # If not relying on the cache, no need for a retry mechanism
        if not self.use_cache:
            return

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

    # Handle incoming ROS service requests to get a single param value
    def srv_param_get(self, req, bridge):
        val = self._get(str(req.name))
        if val is not None:
            return { 'ok' : True, 'value' : val }
        else:
            return { 'ok' : False, 'value' : float(0.0) }

    # Handle incoming ROS service requests to get a list of param values
    # NOTE: If a requested param cannot be fetched, it *won't* be included
    #  in the response (we can send None over ROS). Hence, lack of a
    #  param in the response is an implicit ok=False.
    def srv_param_getlist(self, req, bridge):
        params = []

        for name in req.name:
            val = self._get(str(name))
            if val is not None:
                pp = apmsg.ParamPair()
                pp.name = str(name)
                pp.value = float(val)
                params.append(pp)
        return { 'param' : params }

    # Handle incoming ROS service requests to set param values
    def srv_param_set(self, req, bridge):
        # Make sure the parameter exists first
        old = self._get(str(req.name))
        if old is None:
            return { 'ok' : False }

        # Try to set the new param value
        self.master.param_set_send(str(req.name), float(req.value))

        # We definitely want to re-fetch the param in this case
        self._noneify_value(str(req.name))

        # Give the autopilot a moment
        time.sleep(0.2)  # TODO: Tune this value

        # If the newly-fetched parameter matches in value, we're good
        new = self._get(str(req.name))
        return new is not None and abs(new - req.value) < 0.00003


#-----------------------------------------------------------------------
# init()

def init(bridge):
    p_obj = mavbridge_param(bridge.get_master())
    bridge.add_mavlink_event("PARAM_VALUE", p_obj.mav_param_value)
    bridge.add_ros_srv_event("param_get", apsrv.ParamGet, p_obj.srv_param_get)
    bridge.add_ros_srv_event("param_getlist", apsrv.ParamGetList,
                             p_obj.srv_param_getlist)
    bridge.add_ros_srv_event("param_set", apsrv.ParamSet, p_obj.srv_param_set)
    return p_obj
