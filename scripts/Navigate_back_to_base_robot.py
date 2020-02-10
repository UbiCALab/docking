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


class navigate_back_to_base:

    def __init__(self):
        self.listener = tf.TransformListener()
        self.yaw = [0, np.pi/2, np.pi, 3*np.pi/2]
        self.pose = PoseStamped()
        self.tf_prefix = rospy.get_param('/tf_prefix', 'robot1_tf/')
        self.pose.header.frame_id = self.tf_prefix + 'base_link'
        self.rate = rospy.Rate(0.1)
        self.namespace = rospy.get_namespace().strip('/')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_pose = MoveBaseGoal()
        self.rfid_nodename = 'rfid_interaction/'
        self.sub_RFID = rospy.Subscriber(self.rfid_nodename + 'epcs', EpcInfo, self.RFID_cb)
        self.base1_epc = rospy.get_param('nav_to_base/base1')
        self.msgs_full = False
        self.dock_srv = rospy.Service('start_docking_srv', Empty, self.cb_start_docking)
        self.RFID_tag_reach = False
        self.antenna = [[0,0,0,0],[0,0,0,0],[0,0,0,0]]
        self.count = 0
        self.samples = 5
	
    def cb_start_docking(self, msg):
        self.RFID_tag_reach = True
        self.client.cancel_all_goals()
        return EmptyResponse()

    def move_turn(self, ant_port):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw[ant_port])
        self.pose.pose.orientation.x = quaternion[0]
        self.pose.pose.orientation.y = quaternion[1]
        self.pose.pose.orientation.z = quaternion[2]
        self.pose.pose.orientation.w = quaternion[3]
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.transform_and_send_goal()

    def move_straight(self):
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 1
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.position.x = 1
        self.transform_and_send_goal()

    def transform_and_send_goal(self):
        self.listener.waitForTransform(self.tf_prefix + 'odom', self.tf_prefix + 'base_link', rospy.Time(0), rospy.Duration(5))
        self.goal_pose.target_pose = self.listener.transformPose(self.tf_prefix + 'odom', self.pose)
        self.client.send_goal(self.goal_pose)        
        self.client.wait_for_result(rospy.Duration(10))

    def RFID_cb(self, msg):
        if self.base1_epc == msg.epc: # Update antenna decision matrix
            if self.count < self.samples:
                self.decision_matrix_update(msg)
            elif self.count == self.samples:
                self.decision_matrix_update(msg)
                # average calculation
                self.antenna[2][3]=self.antenna[1][3]*self.antenna[0][3]
                self.antenna[2][2]=self.antenna[1][2]*self.antenna[0][2]
                self.antenna[2][0]=self.antenna[1][0]*self.antenna[0][0]
                self.antenna[2][1]=self.antenna[1][1]*self.antenna[0][1]
                self.msgs_full = True # stop averaging
                self.antenna = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]  # Empty matrix
            else: # keeps reading tags while moving
                self.decision_matrix_update(msg)

    def decision_matrix_update(self, msg):
        self.count += 1
        self.antenna[0][msg.antenna_port - 1] += 1
        self.antenna[1][msg.antenna_port - 1] += 10 ** ((msg.RSSI - 30) / 10)
        print("[count][RSSI][AV]: " + str(self.antenna))
        print("EPC: " + str(msg.epc))
        print("counter: " + str(self.count))


if __name__ == '__main__':
    try:
        rospy.init_node('nav_to_base', log_level=rospy.DEBUG)
        nbb = navigate_back_to_base()
        while not nbb.RFID_tag_reach:
            if nbb.msgs_full:
                nbb.msgs_full = False
                rospy.logdebug("Port decision: " + str(nbb.antenna[2].index(max(nbb.antenna[2]))+1))
                nbb.move_turn(nbb.antenna[2].index(max(nbb.antenna[2])))
                if not nbb.RFID_tag_reach:
                    nbb.move_straight()
        nbb.antenna = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]  # Empty matrix
        nbb.count = 0
        rospy.logdebug("                                                             ")
        rospy.logdebug("                       Process completed                     ")
        rospy.logdebug("                                                             ")
        rospy.logdebug("                        DOCKING STARTED                      ")
        rospy.logdebug("                                                             ")

    except rospy.ROSInterruptException:
        pass
