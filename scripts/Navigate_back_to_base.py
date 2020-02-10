#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import numpy as np
from ant_robot.msg import EpcInfo


class navigate_back_to_base:

    def __init__(self):
        self.listener = tf.TransformListener()
        self.yaw = [0, np.pi / 2, np.pi, 3 * np.pi / 2] # Modify the order to match it with the antenna ports
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'base_link'
        self.rate = rospy.Rate(0.25)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_pose = MoveBaseGoal()
        self.sub_RFID = rospy.Subscriber('epcs', EpcInfo, self.RFID_cb)
        self.msg_rfid = EpcInfo()
        # self.tf_prefix = rospy.get_param('/tf_prefix', rospy.get_namespace()[0:-1] + '_tf/')

    def next_move(self, ant_port):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw[ant_port])
        self.pose.pose.orientation.x = quaternion[0]
        self.pose.pose.orientation.y = quaternion[1]
        self.pose.pose.orientation.z = quaternion[2]
        self.pose.pose.orientation.w = quaternion[3]
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(5))
        self.goal_pose.target_pose = self.listener.transformPose('odom', self.pose)
        self.client.send_goal(self.goal_pose)
        self.rate.sleep()

        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 1
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.position.x = 10
        self.listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(5))
        self.goal_pose.target_pose = self.listener.transformPose('odom', self.pose)
        self.client.send_goal(self.goal_pose)
        self.rate.sleep()

    def RFID_cb(self, msg):
        self.msg_rfid = msg


if __name__ == '__main__':
    try:
        rospy.init_node('nav_to_base', log_level=rospy.DEBUG)
        RFID_tag_reach = False
        nbb = navigate_back_to_base()
        while not RFID_tag_reach:
            if nbb.msg_rfid.epc == rospy.get_param('rfid_config/base1'):
                nbb.next_move(nbb.msg_rfid.antenna_port)
                nbb.rate.sleep()
            RFID_tag_reach = (nbb.msg_rfid.RSSI >= 1)

    except rospy.ROSInterruptException:
        pass
