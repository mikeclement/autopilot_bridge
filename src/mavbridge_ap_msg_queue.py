#!/usr/bin/env python
#
# mavbridge_ap_msg_queue.py - Autopilot message queue
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

import rospy

import autopilot_bridge.srv as apsrv
import autopilot_bridge.msg as apmsg

from collections import deque
from threading import RLock
import re

class AP_MsgQueue(object):
    def __init__(self):
        self.__msg_q = deque(maxlen=100)
        
        self.__curr_seq_num = 1  #current sequence number

        self._q_lock = RLock()  #lock for the queue (__msg_q)

    def handle_statustext(self, msg_type, msg, bridge):
        #there are some messages that are not needed:
        if self.filter_message(msg.text):
            return

        with self._q_lock:
            self.__msg_q.appendleft((self.__curr_seq_num, msg.text))
            self.__curr_seq_num += 1

    #Returns True if I want to filter the message, false otherwise
    def filter_message(self, msg):
        if re.search("Executing command ID",msg) is not None:
            return True
        
        if re.search("Executing nav command ID",msg) is not None:
            return True

        if re.search("command received:",msg) is not None:
            return True
        
        return False

    #Return the previous N messages in the queue (parameter: req.n).
    #When looping through the message queue, if we arrive at the 
    #sequence number req.since_seq prior to reaching N, then stop publishing.
    def get_last_n_msgs(self, req, bridge):
        n = req.n
        stop_at = req.since_seq
        if n > len(self.__msg_q):
            #can't return more than what is in the queue
            n = len(self.__msg_q)

        msgs = []
        
        with self._q_lock:
            for i in range(0, n):
                (sequence_num, msg_text) = self.__msg_q[i]
            
                if sequence_num <= stop_at:
                    break

                ap_msg = apmsg.AutoPilotMsg()
                ap_msg.seq = sequence_num
                ap_msg.text = msg_text

                msgs.append(ap_msg)
            
        #service requires a response
        return { 'msgs' : msgs }

def init(bridge):
    obj = AP_MsgQueue()
    bridge.add_mavlink_event("STATUSTEXT", obj.handle_statustext)
    bridge.add_ros_srv_event("ap_msg_queue_last_n", apsrv.ReqPrevNMsgs, 
            obj.get_last_n_msgs)
    return obj
