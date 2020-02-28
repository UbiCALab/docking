#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import numpy as np
from ant_robot.msg import EpcInfo
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Odometry


def def_pose(position, quaternion):
    pose = PoseStamped()
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pose.pose.position.y = position[0]
    pose.pose.position.z = position[1]
    pose.pose.position.x = position[2]
    return pose.pose


class Stack:

    def __init__(self):
        self.size = int()
        self.stack = list()


class NavigateBackToBase:

    def __init__(self):
        """

        """
        # self.namespace = rospy.get_namespace().strip('/')
        self.listener = tf.TransformListener()
        self.yaw = [0, np.pi / 2, np.pi, 3 * np.pi / 2]
        self.goal_pose = MoveBaseGoal()
        self.pose = PoseStamped()
        self.tf_prefix = rospy.get_param('/tf_prefix', 'robot1_tf/')
        self.pose.header.frame_id = self.tf_prefix + 'base_link'
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.rfid_node_name = 'rfid_interaction/'
        self.sub_RFID = rospy.Subscriber(self.rfid_node_name + 'epcs', EpcInfo, self.rfid_cb)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.base1_epc = rospy.get_param('nav_to_base/base1')
        self.samples = rospy.get_param('nav_to_base/samples')
        self.dock_srv = rospy.Service('start_docking_srv', Empty, self.cb_start_docking)
        self.RFID_tag_reach = False
        self.msgs_full = False
        self.antenna = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        self.count = 0
        self.time = rospy.Time.to_sec(rospy.Time.now())
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def cb_start_docking(self):
        self.RFID_tag_reach = True
        self.client.cancel_all_goals()
        return EmptyResponse()

    def next_move(self, position, quaternion):
        self.pose.pose = def_pose(position, quaternion)
        self.transform_and_send_goal()

    def transform_and_send_goal(self):
        self.listener.waitForTransform(self.tf_prefix + 'odom', self.tf_prefix + 'base_link', rospy.Time(0),rospy.Duration(5))
        self.goal_pose.target_pose = self.listener.transformPose(self.tf_prefix + 'odom', self.pose)
        self.client.send_goal(self.goal_pose)
        self.client.wait_for_result(rospy.Duration(10))

    def full_movement(self, string):
        rospy.logdebug(string + ":    " + str(self.antenna[2].index(max(self.antenna[2])) + 1))
        self.next_move([0, 0, 0], tf.transformations.quaternion_from_euler(0, 0, self.yaw[self.antenna_decision()]))
        if not self.RFID_tag_reach:
            self.next_move([0, 0, 1], [0, 0, 0, 1])

    def rfid_cb(self, msg):
        if self.base1_epc == msg.epc:  # Update antenna decision matrix
            self.antenna[0][msg.antenna_port - 1] += 1
            self.antenna[1][msg.antenna_port - 1] += 10 ** ((msg.RSSI - 30) / 10)
            if self.count < self.samples:
                self.count += 1
            elif self.count == self.samples:
                self.count += 1
                self.msgs_full = True  # stop averaging
            print("counter: " + str(self.count))
            print(self.antenna)  # [count][RSSI][AV]

    def antenna_decision(self):
        self.antenna[2][0] = self.antenna[1][0] * self.antenna[0][0]
        self.antenna[2][1] = self.antenna[1][1] * self.antenna[0][1]
        self.antenna[2][2] = self.antenna[1][2] * self.antenna[0][2]
        self.antenna[2][3] = self.antenna[1][3] * self.antenna[0][3]
        return self.antenna[2].index(max(self.antenna[2]))


if __name__ == '__main__':
    try:
        rospy.init_node('nav_to_base', log_level=rospy.DEBUG)
        nbb = NavigateBackToBase()
        while not nbb.RFID_tag_reach:
            if nbb.msgs_full:
                nbb.msgs_full = False  # Restart normal sample acquisition
                nbb.full_movement("Port decision")
                nbb.antenna_decision()  # Saving averages for R1
                nbb.antenna[0] = [0, 0, 0, 0]  # Emptying antenna decision matrix
                nbb.antenna[1] = [0, 0, 0, 0]  # but keeping average in antenna[2]
                nbb.count = 0
                nbb.time = rospy.Time.to_sec(rospy.Time.now())
            if rospy.Time.to_sec(rospy.Time.now()) > nbb.time + rospy.Time.to_sec(rospy.Time(60)):
                nbb.full_movement("Recovery!!! Port decision")  # R1: the robot doesn't receive new TAG readings after a move
                                                                # Then, use the measurements taken during the movement
        rospy.logdebug("                                                             ")
        rospy.logdebug("                       Process completed                     ")
        rospy.logdebug("                                                             ")
        rospy.logdebug("                        DOCKING STARTED                      ")
        rospy.logdebug("                                                             ")

    except rospy.ROSInterruptException:
        pass
