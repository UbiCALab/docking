#!/usr/bin/env python
# coding=utf-8

import rospy
from std_srvs.srv import SetBool


if __name__ == '__main__':
    try:
        rospy.init_node('go_back_home', log_level=rospy.DEBUG)
        srv_dock_client = rospy.ServiceProxy('allow_dockin_srv', SetBool)
        srv_nav_client = rospy.ServiceProxy('start_navigation_srv', SetBool)
        srv_dock_client.wait_for_service(10)
        srv_nav_client.wait_for_service(10)
        srv_dock_client(True)
        srv_nav_client(True)
    except rospy.ROSInterruptException:
        pass
