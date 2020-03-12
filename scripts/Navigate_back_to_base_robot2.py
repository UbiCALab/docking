#!/usr/bin/env python
# coding=utf-8

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import numpy as np
from ant_robot.msg import EpcInfo
from std_srvs.srv import Empty, EmptyResponse, SetBool
from nav_msgs.msg import Path


def quaternion_to_list(quaternion):
    """

    :param quaternion:
    :return:
    """
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def def_pose(position, quaternion, frame_id="robot1_tf/base_link"):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    return pose


class HomeTagReading:

    def __init__(self, antenna_port, rssi, orientation, read_time):
        self.antenna_port = antenna_port
        self.rssi = rssi
        self.orientation = orientation
        self.read_time = read_time


class NavigateBackToBase:

    def __init__(self):
        # self.namespace = rospy.get_namespace().strip('/').time = rospy.Time.to_sec(rospy.Time.now())
        self.reading_list = []
        self.reading_average = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        self.listener = tf.TransformListener()
        self.yaw = [0, np.pi / 2, np.pi, 3 * np.pi / 2]
        self.navigation_mode_on = False
        self.DS_found = False
        self.time = rospy.Time.to_sec(rospy.Time.now())
        self.msgs_full = False
        self.port_rot = 0
        self.goal_pose = MoveBaseGoal()
        self.tf_prefix = rospy.get_param('/tf_prefix', 'robot1_tf/')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.rfid_node_name = 'rfid_interaction/'
        self.sub_RFID = rospy.Subscriber(self.rfid_node_name + 'epcs', EpcInfo, self.rfid_cb)
        self.sub_global = rospy.Subscriber('move_base/DWAPlannerROS/global_plan', Path, self.cb_global_plan)
        self.base1_epc = rospy.get_param('nav_to_base/base1')
        self.samples = rospy.get_param('nav_to_base/samples')
        self.n_antennas = rospy.get_param('nav_to_base/number_of_antennas')
        self.time_between_readings = rospy.get_param('nav_to_base/time_between_readings')
        self.dock_srv = rospy.Service('start_docking_srv', Empty, self.cb_start_docking)
        self.nav_srv = rospy.Service('start_navigation_srv', SetBool, self.cb_start_navigation)
        self.antenna_map = dict()
        self.antenna_map['front'] = rospy.get_param('rfid_interaction/prod/antenna_port/front')
        self.antenna_map['back'] = rospy.get_param('rfid_interaction/prod/antenna_port/back')
        self.antenna_map['left'] = rospy.get_param('rfid_interaction/prod/antenna_port/left')
        self.antenna_map['right'] = rospy.get_param('rfid_interaction/prod/antenna_port/right')
        self.is_active_filtering = rospy.get_param('rfid_interaction/prod/active_filtering/is_active')

    def cb_global_plan(self, msg):
        if np.linalg.norm([msg.poses[-1].pose.position.x - msg.poses[0].pose.position.x,
                           msg.poses[-1].pose.position.x - msg.poses[0].pose.position.x]) > 2:
            print("CANCELLING GOALS: PLANNER TOO LONG")
            self.client.cancel_all_goals()
            self.reading_list = []

    def cb_start_navigation(self, msg):
        self.navigation_mode_on = msg.data
        self.DS_found = False
        return True, ''

    def cb_start_docking(self, msg):
        self.DS_found = True
        self.client.cancel_all_goals()
        return EmptyResponse()

    def base_link_to_odom_transfrom(self, pose):
        self.listener.waitForTransform(self.tf_prefix + 'odom', self.tf_prefix + 'base_link', rospy.Time(0),
                                       rospy.Duration(5))
        return self.listener.transformPose(self.tf_prefix + 'odom', pose)

    def send_goal(self, odom_goal):
        self.goal_pose.target_pose = odom_goal
        print("GOAL POSE:    " + str(self.goal_pose))
        self.client.send_goal(self.goal_pose)
        self.client.wait_for_result()
        rospy.on_shutdown(self.client.cancel_goal)

    def full_movement(self, string):
        print(string)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw[self.antenna_decision()])
        self.reading_list = []
        self.reading_average = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        self.send_goal(self.base_link_to_odom_transfrom(def_pose([0, 0, 0], quaternion)))
        if not self.DS_found:
            self.send_goal(self.base_link_to_odom_transfrom(def_pose([1, 0, 0], [0, 0, 0, 1])))

    def rfid_cb(self, msg):
        if self.base1_epc == msg.epc:  # Update reads decision matrix
            self.time = rospy.Time.to_sec(rospy.Time.now())
            self.reading_list.append(HomeTagReading(msg.antenna_port - 1, 10 ** ((msg.RSSI - 30) / 10),
                                                    self.base_link_to_odom_transfrom(def_pose([0, 0, 0], [0, 0, 0, 1]))
                                                    .pose.orientation, rospy.Time.to_sec(rospy.Time.now())))
            if len(self.reading_list) >= self.samples:
                self.msgs_full = True  # stop averaging
            rospy.logdebug("stack size: " + str(len(self.reading_list)) + "    port: " + str(msg.antenna_port - 1) +
                           "    RSSI: " + str(10 ** ((msg.RSSI - 30) / 10)))

    def antenna_decision(self):
        for reading in self.reading_list:  # for reading in self.reading_list[-self.samples:]:
            rotated_port = self.port_rotation(reading)
            self.reading_average[0][rotated_port] += 1
            self.reading_average[1][rotated_port] += reading.rssi
        return self.weighted_selection()

    # def angle_selection(self):
    #     [ord(x) for x in self.reading_list]
    #     self.reading_list.sort()

    def port_transform(self, rotated_port):
        ports = ['front', 'left', 'back', 'right']  # Port definition for calculations from robot poses
        return self.antenna_map[ports[rotated_port]]

    def weighted_selection(self):
        self.reading_average[2][0] = self.reading_average[1][0] * self.reading_average[0][0]
        self.reading_average[2][1] = self.reading_average[1][1] * self.reading_average[0][1]
        self.reading_average[2][2] = self.reading_average[1][2] * self.reading_average[0][2]
        self.reading_average[2][3] = self.reading_average[1][3] * self.reading_average[0][3]
        rospy.logdebug("reading average: " + str(self.reading_average) + "\n")
        rospy.logdebug("selection: " + str(self.reading_average[2].index(max(self.reading_average[2]))) + "\n")
        return self.reading_average[2].index(max(self.reading_average[2]))

    def port_rotation(self, reading):
        angular_difference = tf.transformations.euler_from_quaternion(quaternion_to_list(
            self.base_link_to_odom_transfrom(def_pose([0, 0, 0], [0, 0, 0, 1])).pose.orientation))[2] - \
                             tf.transformations.euler_from_quaternion(quaternion_to_list(reading.orientation))[2]
        print("resta: " + str(angular_difference) + "   final angle: " + str(tf.transformations.
            euler_from_quaternion(quaternion_to_list(self.base_link_to_odom_transfrom(def_pose([0, 0, 0], [0, 0, 0, 1]))
                .pose.orientation))) + "   measurement angle: " +
                    str(tf.transformations.euler_from_quaternion(quaternion_to_list(reading.orientation))))
        if (-np.pi / 4 <= angular_difference <= np.pi / 4) or (7 * np.pi / 4 <= angular_difference <= -7 * np.pi / 4):
            self.port_rot = 0
        elif (-np.pi / 4 >= angular_difference >= -3 * np.pi / 4) or (
                5 * np.pi / 4 <= angular_difference <= 7 * np.pi / 4):
            self.port_rot = 1
        elif (3 * np.pi / 4 <= angular_difference <= 5 * np.pi / 4) or (
                -3 * np.pi / 4 <= angular_difference <= -5 * np.pi / 4):
            self.port_rot = 2
        elif (np.pi / 4 <= angular_difference <= 3 * np.pi / 4) or (
                -5 * np.pi / 4 >= angular_difference >= -7 * np.pi / 4):
            self.port_rot = 3
        print("\nantenna_port: " + str(reading.antenna_port) + "   port_rotation: " + str(self.port_rot) +
              "   final_antenna_port: " + str((reading.antenna_port + self.port_rot) % self.n_antennas))
        return self.port_transform((reading.antenna_port + self.port_rot) % self.n_antennas)

    def go_back_home(self):
        odom = self.base_link_to_odom_transfrom(def_pose([0, 0, 0], [0, 0, 0, 1]))
        v = [-odom.pose.position.x, -odom.pose.position.y, 0]
        self.send_goal(def_pose([odom.pose.position.x, odom.pose.position.y, odom.pose.position.z] +
                                v / np.linalg.norm(v), [0, 0, 0, 1], "robot1_tf/odom"))


if __name__ == '__main__':
    try:
        rospy.init_node('nav_to_base', log_level=rospy.DEBUG)
        nbb = NavigateBackToBase()
        while True:
            if (not nbb.DS_found) & nbb.navigation_mode_on:
                if nbb.msgs_full:
                    nbb.msgs_full = False
                    nbb.full_movement("NORMAL BEHAVIOR")
                if (rospy.Time.to_sec(rospy.Time.now()) > nbb.time +
                        rospy.Time.to_sec(rospy.Time(nbb.time_between_readings))):
                    if len(nbb.reading_list) == 0:
                        nbb.go_back_home()
                    else:
                        nbb.full_movement("1st RECOVERY!!!")
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass
