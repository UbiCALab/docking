#!/usr/bin/env python
# coding=utf-8

import rospy
from std_srvs.srv import SetBool
import actionlib
from ant_robot.msg import ReadEpcsAction, ReadEpcsGoal
import datetime


if __name__ == '__main__':
    try:
        rospy.init_node('control', log_level=rospy.DEBUG)
        srv_dock_client = rospy.ServiceProxy('allow_dockin_srv', SetBool)
        srv_nav_client = rospy.ServiceProxy('start_navigation_srv', SetBool)
        sac_reader = actionlib.SimpleActionClient('read_epcs', ReadEpcsAction)  # Action client to the tag reader
        srv_dock_client.wait_for_service(10)
        srv_nav_client.wait_for_service(10)

        goal = ReadEpcsGoal()
        goal.max_duration = 60
        goal.mission_id = "Docking"
        goal.init_time = datetime.now().strftime('%y%m%d-%H%M%S')
        sac_reader.send_goal(goal)

        srv_dock_client(True)
        srv_nav_client(True)
    except rospy.ROSInterruptException:
        pass
