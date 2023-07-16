#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
from dynamic_reconfigure.server import Server
from v2x.cfg import V2XConfig

class RSU: