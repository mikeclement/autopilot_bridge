#!/usr/bin/env python
#
# mavbridge_file.py - Module to load autopilot configs from file
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
import autopilot_bridge.msg as apmsg
import autopilot_bridge.srv as apsrv

#-----------------------------------------------------------------------
# Primary class definition

class mavbridge_file(object):

    def __init__(self):
        self.ros_basename = rospy.get_name()

    def _do_parse(self, f, parse_cb, items):
        for line in f:
            line = line.rstrip("\n")

            # Ignore comments and blanks
            if not line or line.startswith('#'): continue

            # NOTE: if file is ill-formatted, fail altogether
            item = parse_cb(*line.split())
            # Allow parsers to ignore some lines
            if item is None: continue
            items.append(item)

    def _do_load(self, file_name, srv_name, srv_type, parse_cb, item_attr):
        try:
            # Set up service proxy
            srv_name = self.ros_basename + '/' + srv_name
            rospy.wait_for_service(srv_name, 3.0)
            srv = rospy.ServiceProxy(srv_name, srv_type)

            # Create a request object (NOTE: hackish, relies on service impl)
            req = srv_type._request_class()
        except Exception as ex:
            rospy.logwarn("Loader: service setup error: " + str(ex))
            return { 'ok' : False }

        try:
            # Gets the list attribute of that object
            items = getattr(req, item_attr)

            # Read in file contents
            with open(file_name, 'r') as f:
                self._do_parse(f, parse_cb, items)
        except Exception as ex:
            rospy.logwarn("Loader: read error: " + str(ex))
            return { 'ok' : False }

        try:
            # Call service
            res = srv(req)
            return { 'ok' : res.ok }
        except Exception as ex:
            rospy.logwarn("Loader: service call error: " + str(ex))
            return { 'ok' : False }

    ''' Type-specific parsers '''

    def _parse_param(self, *args):
        p = apmsg.ParamPair()
        p.name = str(args[0]).upper()
        p.value = float(args[1])
        return p

    def _parse_fence(self, *args):
        p = apmsg.Fencepoint()
        p.lat = float(args[0])
        p.lon = float(args[1])
        return p

    def _parse_rally(self, *args):
        p = apmsg.Rallypoint()
        p.lat = float(args[1]) * 1e7
        p.lon = float(args[2]) * 1e7
        p.alt = float(args[3])
        p.break_alt = float(args[4])
        p.land_dir = float(args[5]) * 1e2
        p.flags = int(args[6])
        return p

    def _parse_wp(self, *args):
        if args[0] == 'QGC':
            if args[2] != '110':
                raise Exception("wrong wp file version")
            return None
        p = apmsg.Waypoint()
        p.seq = int(args[0])
        p.frame = int(args[2])
        p.command = int(args[3])
        p.current = bool(int(args[1]))
        p.autocontinue = bool(int(args[11]))
        p.param1 = float(args[4])
        p.param2 = float(args[5])
        p.param3 = float(args[6])
        p.param4 = float(args[7])
        p.x = float(args[8])
        p.y = float(args[9])
        p.z = float(args[10])
        return p

    ''' Service calls '''

    def srv_load_param(self, req, bridge):
        return self._do_load(
            req.name,
            "fpr_param_setlist",
            apsrv.ParamSetList,
            self._parse_param,
            "param")

    def srv_load_fence(self, req, bridge):
        return self._do_load(
            req.name,
            "fpr_fence_setall",
            apsrv.FenceSetAll,
            self._parse_fence,
            "points")

    def srv_load_rally(self, req, bridge):
        return self._do_load(
            req.name,
            "fpr_rally_setall",
            apsrv.RallySetAll,
            self._parse_rally,
            "points")

    def srv_load_wp(self, req, bridge):
        return self._do_load(
            req.name,
            "wp_setall",
            apsrv.WPSetAll,
            self._parse_wp,
            "points")

#-----------------------------------------------------------------------
# init()

def init(bridge):
    obj = mavbridge_file()
    bridge.add_ros_srv_event("load_param", apsrv.FileLoad, obj.srv_load_param)
    bridge.add_ros_srv_event("load_fence", apsrv.FileLoad, obj.srv_load_fence)
    bridge.add_ros_srv_event("load_rally", apsrv.FileLoad, obj.srv_load_rally)
    bridge.add_ros_srv_event("load_wp", apsrv.FileLoad, obj.srv_load_wp)
    return obj

