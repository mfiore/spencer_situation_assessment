#!/usr/bin/env python

import sys
import rospy
from annotated_mapping.srv import *

def test_map_switch(n1, n2, n3):
    rospy.wait_for_service('switch_map')
    try:
        switch_map = rospy.ServiceProxy('switch_map', SwitchMap3)
        resp1 = switch_map("1", "2","3")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    test_map_switch(0,1,2)