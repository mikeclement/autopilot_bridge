#!/usr/bin/env python
#
# mavbridge_fpr.py - Fence, Rally, and Param services
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
from threading import RLock

#-----------------------------------------------------------------------
# Class to manipulate fences, params, and rally points

class mavbridge_fpr(object):
    def __init__(self, master):
        self._master = master

        # Locks to prevent multiple get/set calls from racing
        self._param_lock = RLock()
        self._fence_lock = RLock()
        self._rally_lock = RLock()

        # Stores for MAVLink handlers to cache received data
        self._params = {}        # str -> float
        self._fence_points = {}  # int -> MAVLink_fence_point_message
        self._rally_points = {}  # int -> MAVLink_rally_point_message

    def _get_item(self, index, store, fetch_cb, tries=10, wait=0.2, force=True):
        '''generic fetch function'''
        if index in store:
            if force: del store[index]
            else: return store[index]
        for i in range(tries):
            fetch_cb()
            time.sleep(wait)
            if index in store: return store[index]
        return None

    def _set_item(self, set_cb, fetch_cb, check_cb=None, tries=10, wait=0.2):
        '''generic set function'''
        for i in range(tries):
            set_cb()
            time.sleep(wait)  # Extra time for set to be processed
            val = fetch_cb()
            if val is None:
                continue
            if callable(check_cb) and not check_cb(val):
                continue
            return True
        return False

    def _get_param(self, name, tries=10, force=True):
        '''fetch a parameter by name'''
        with self._param_lock:
            pn = str(name).upper()
            f = lambda : self._master.param_fetch_one(pn)
            return self._get_item(pn, self._params, f,
                                  tries=tries, force=force)

    def _set_param(self, name, value, tries=10):
        '''set a parameter'''
        with self._param_lock:
            pn = str(name).upper()
            pv = float(value)
            s = lambda : self._master.param_set_send(pn, pv)
            f = lambda : self._get_param(pn, tries=2, force=False)
            c = lambda v: bool(abs(v - pv) < 0.00003)
            if pn in self._params: del self._params[pn]
            return self._set_item(s, f, c, tries=tries)

    def _get_fence_point(self, index, tries=10, force=True):
        '''fetch a fence point by index'''
        with self._fence_lock:
            f = lambda : self._master.mav.fence_fetch_point_send( \
                             self._master.target_system,
                             self._master.target_component,
                             index)
            return self._get_item(index, self._fence_points, f,
                                  tries=tries, force=force)

    def _set_fence_point(self, p, tries=10):
        '''set a fence point'''
        with self._fence_lock:
            s = lambda : self._master.mav.send(p)
            f = lambda : self._get_fence_point(p.idx, tries=2, force=False)
            # Check condition taken from MAVProxy's fence module
            c = lambda v: bool(abs(v.lat - p.lat) < 0.00003 and 
                               abs(v.lng - p.lng) < 0.00003)
            if p.idx in self._fence_points: del self._fence_points[p.idx]
            return self._set_item(s, f, c, tries=tries)

    def _get_rally_point(self, index, tries=10, force=True):
        '''fetch a rally point by index'''
        with self._rally_lock:
            f = lambda : self._master.mav.rally_fetch_point_send( \
                             self._master.target_system,
                             self._master.target_component,
                             index)
            return self._get_item(index, self._rally_points, f,
                                  tries=tries, force=force)

    def _set_rally_point(self, p, tries=10):
        '''set a rally point'''
        with self._rally_lock:
            s = lambda : self._master.mav.send(p)
            f = lambda : self._get_rally_point(p.idx, tries=2, force=False)
            # Check condition taken from MAVProxy's fence module
            c = lambda v: bool(abs(v.lat - p.lat) < 0.00003 and 
                               abs(v.lng - p.lng) < 0.00003 and
                               abs(v.alt - p.alt) < 0.00003 and
                               abs(v.break_alt - p.break_alt) < 0.00003 and
                               abs(v.break_alt - p.break_alt) < 0.00003 and
                               abs(v.land_dir - p.land_dir) < 0.00003 and
                               v.flags == p.flags)
            if p.idx in self._rally_points: del self._rally_points[p.idx]
            return self._set_item(s, f, c, tries=tries)

    def _getall_points(self, count_param, fetch_cb, point_cb):
        '''ROS get-all wrapper for _get_*_point()'''
        try:
            points = []
            count = self._get_param(count_param)
            if count is None:  # Couldn't get param -> FAIL
                return { 'ok' : False, 'points' : [] }
            if count == 0.0:  # No fence points -> OK
                return { 'ok' : True, 'points' : [] }
            for c in range(int(count)):
                p = fetch_cb(c)
                if p is None or int(p.count) != count:
                    return { 'ok' : False, 'points' : [] }
                points.append(point_cb(p))
            return { 'ok' : True, 'points' : points }
        except Exception as ex:
            return { 'ok' : False, 'points' : [] }

    def _setall_points(self, pre_cb, points, point_cb, set_cb, post_cb):
        '''ROS set-all wrapper for _set_*_point()'''
        try:
            if len(points) == 0:
                return { 'ok' : False }
            pre_val = pre_cb()
            if pre_val is None:
                return { 'ok' : False }
            for i in range(len(points)):
                p = point_cb(i, points[i])
                p.count = len(points)
                if set_cb(p) is False:
                    return { 'ok' : False }
            return { 'ok' : post_cb(pre_val) }
        except Exception as ex:
            return { 'ok' : False }

    '''API'''

    def mav_param_value(self, msg_type, msg, bridge):
        '''handle PARAM_VALUE messages (needed by _get_param())'''
        pn = str(msg.param_id).upper()
        pv = float(msg.param_value)
        self._params[pn] = pv

    def mav_fence_point(self, msg_type, msg, bridge):
        '''handle FENCE_POINT messages (needed by _get_fence_point())'''   
        self._fence_points[msg.idx] = msg

    def mav_rally_point(self, msg_type, msg, bridge):
        '''handle RALLY_POINT messages (needed by _get_rally_point())'''   
        self._rally_points[msg.idx] = msg

    def srv_param_get(self, req, bridge):
        '''service to get a parameter by name'''
        val = self._get_param(req.name)
        if val is None:
            return { 'ok' : False, 'value' : 0.0 }
        return { 'ok' : True, 'value' : val }

    def srv_param_getlist(self, req, bridge):
        '''service to get *large* lists of parameters'''
        # Since we're doing several interdependent ops, get the lock up front
        with self._param_lock:
            values = []
            # Take advantage of fetch-all and caching
            # NOTE: If list is small, one-by-one fetching might be better
            self._params = {}
            self._master.param_fetch_all()
            # Wait a moment for some data to come in
            time.sleep(2.0)
            for p in req.name:
                pn = str(p).upper()
                pv = self._get_param(pn, force=False)  # NOTE: Using cache
                if isinstance(pv, float):
                    values.append(apmsg.ParamPair(pn, pv))
            return { 'param' : values }

    def srv_param_set(self, req, bridge):
        '''service to set a parameter'''
        res = self._set_param(req.name, req.value)
        return { 'ok' : res }

    def srv_param_setlist(self, req, bridge):
        '''service to set *large* lists of parameters'''
        # Since we're doing several interdependent ops, get the lock up front
        with self._param_lock:
            # Take advantage of fetch-all and caching
            # NOTE: If list is small, one-by-one fetching might be better
            self._params = {}
            self._master.param_fetch_all()
            # Wait a moment for some data to come in
            time.sleep(2.0)
            for p in req.param:
                pn = str(p.name).upper()
                pv = float(p.value)
                # First check if param is already correctly set
                auto_pv = self._get_param(pn, force=False)  # NOTE: Using cache
                if auto_pv is None:
                    rospy.logwarn("PARAM %s doesn't exist" % pn)
                    return { 'ok' : False }
                if abs(pv - auto_pv) < 0.00003:
                    continue
                # If not, set it correctly
                res = self._set_param(pn, pv)
                if not res:
                    rospy.logwarn("PARAM %s couldn't be set" % pn)
                    return { 'ok' : False }
                rospy.loginfo("PARAM %s --> %0.06f" % (pn, pv))
            return { 'ok' : True }

    def srv_fence_getall(self, req, bridge):
        '''service to get list of fence points'''
        # Since we're doing several interdependent ops, get the lock up front
        with self._fence_lock:
            p = lambda i: apmsg.Fencepoint(i.lat, i.lng)
            return self._getall_points('FENCE_TOTAL', self._get_fence_point, p)

    def srv_fence_setall(self, req, bridge):
        '''service to set list of fence points'''
        # Since we're doing several interdependent ops, get the lock up front
        # NOTE: If this service fails, we don't guarantee the fence state
        with self._fence_lock:
            def pre():
                action = self._get_param('FENCE_ACTION')
                if action is None:
                    return None
                if self._set_param('FENCE_ACTION',
                                   mavutil.mavlink.FENCE_ACTION_NONE) is False:
                    return None
                if self._set_param('FENCE_TOTAL', len(req.points)) is False:
                    return None
                return action
            pf = lambda i,p: mavutil.mavlink.MAVLink_fence_point_message( \
                                 self._master.target_system,
                                 self._master.target_component,
                                 i,
                                 0,
                                 p.lat,
                                 p.lon)
            def post(action):
                return self._set_param('FENCE_ACTION', action)
            return self._setall_points(pre, req.points, pf,
                                       self._set_fence_point, post)

    def srv_rally_getall(self, req, bridge):
        '''service to get list of rally points'''
        # Since we're doing several interdependent ops, get the lock up front
        with self._rally_lock:
            p = lambda i: apmsg.Rallypoint(i.lat, i.lng, i.alt,
                                           i.break_alt, i.land_dir, i.flags)
            return self._getall_points('RALLY_TOTAL', self._get_rally_point, p)

    def srv_rally_setall(self, req, bridge):
        '''service to set list of rally points'''
        # Since we're doing several interdependent ops, get the lock up front
        # NOTE: If this service fails, we don't guarantee the rally state
        with self._rally_lock:
            def pre():
                if self._set_param('RALLY_TOTAL', len(req.points)) is False:
                    return None
                return True
            pf = lambda i,p: mavutil.mavlink.MAVLink_rally_point_message( \
                                 self._master.target_system,
                                 self._master.target_component,
                                 i,
                                 0,
                                 p.lat,
                                 p.lon,
                                 p.alt,
                                 p.break_alt,
                                 p.land_dir,
                                 p.flags)
            post = lambda x: True
            return self._setall_points(pre, req.points, pf,
                                       self._set_rally_point, post)

#-----------------------------------------------------------------------
# init()

def init(bridge):
    obj = mavbridge_fpr(bridge.get_master())
    bridge.add_mavlink_event("PARAM_VALUE", obj.mav_param_value)
    bridge.add_mavlink_event("FENCE_POINT", obj.mav_fence_point)
    bridge.add_mavlink_event("RALLY_POINT", obj.mav_rally_point)
    bridge.add_ros_srv_event("fpr_param_get", apsrv.ParamGet, obj.srv_param_get)
    bridge.add_ros_srv_event("fpr_param_getlist", apsrv.ParamGetList, obj.srv_param_getlist)
    bridge.add_ros_srv_event("fpr_param_set", apsrv.ParamSet, obj.srv_param_set)
    bridge.add_ros_srv_event("fpr_param_setlist", apsrv.ParamSetList, obj.srv_param_setlist)
    bridge.add_ros_srv_event("fpr_fence_getall", apsrv.FenceGetAll, obj.srv_fence_getall)
    bridge.add_ros_srv_event("fpr_fence_setall", apsrv.FenceSetAll, obj.srv_fence_setall)
    bridge.add_ros_srv_event("fpr_rally_getall", apsrv.RallyGetAll, obj.srv_rally_getall)
    bridge.add_ros_srv_event("fpr_rally_setall", apsrv.RallySetAll, obj.srv_rally_setall)
    return obj
