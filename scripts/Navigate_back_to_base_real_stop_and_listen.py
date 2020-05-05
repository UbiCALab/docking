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


def def_pose(position, quaternion, frame_id="robot2_tf/base_link"):
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


class BaseTagReading:

    def __init__(self, antenna_port, rssi, orientation, read_time):
        self.antenna_port = antenna_port
        self.rssi = rssi
        self.orientation = orientation
        self.read_time = read_time


class NavigateBackToBase:

    def __init__(self):
        # self.namespace = rospy.get_namespace().strip('/')
        self.reading_list = []
        self.reading_average = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        self.ports = ['front', 'left', 'back', 'right']  # Port definition for calculations from robot poses
        self.listener = tf.TransformListener()
        self.navigation_mode_on = False
        self.DS_found = False
        self.time = rospy.Time.to_sec(rospy.Time.now())
        self.msgs_full = False
        self.robot_moving = False
        self.goal_pose = MoveBaseGoal()
        self.tf_prefix = rospy.get_param('tf_prefix') + '/'  # , 'robot2_tf')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.rfid_node_name = 'rfid_interaction/'
        self.sub_RFID = rospy.Subscriber(self.rfid_node_name + 'epcs', EpcInfo, self.cb_rfid_readings)
        self.sub_global = rospy.Subscriber('move_base/DWAPlannerROS/global_plan', Path, self.cb_global_plan)
        self.base1_epc = rospy.get_param('nav_to_base/base1')
        self.samples = rospy.get_param('nav_to_base/samples')
        self.n_antennas = rospy.get_param('nav_to_base/number_of_antennas')
        self.time_between_readings = rospy.get_param('nav_to_base/time_between_readings')
        self.dock_srv = rospy.Service('start_docking_srv', Empty, self.cb_start_docking)
        self.nav_srv = rospy.Service('start_navigation_srv', SetBool, self.cb_start_navigation)

        self.antenna_map = dict()
        self.antenna_map['front'] = rospy.get_param('rfid_interaction/prod/antenna_port/front') - 1
        self.antenna_map['back'] = rospy.get_param('rfid_interaction/prod/antenna_port/back') - 1
        self.antenna_map['left'] = rospy.get_param('rfid_interaction/prod/antenna_port/left') - 1
        self.antenna_map['right'] = rospy.get_param('rfid_interaction/prod/antenna_port/right') - 1
        self.yaw = [0] * self.n_antennas
        self.yaw[self.antenna_map['front']] = 0
        self.yaw[self.antenna_map['left']] = np.pi / 2
        self.yaw[self.antenna_map['back']] = np.pi
        self.yaw[self.antenna_map['right']] = 3 * np.pi / 2
        self.direction = [[0, 0, 0]] * 4
        self.direction[self.antenna_map['front']] = [1, 0, 0]
        self.direction[self.antenna_map['left']] = [0, 1, 0]
        self.direction[self.antenna_map['back']] = [-1, 0, 0]
        self.direction[self.antenna_map['right']] = [0, -1, 0]

    def cb_global_plan(self, msg):
        if np.linalg.norm([msg.poses[-1].pose.position.x - msg.poses[0].pose.position.x,
                           msg.poses[-1].pose.position.y - msg.poses[0].pose.position.y]) > 2:
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

    def cb_rfid_readings(self, msg):
        if (self.base1_epc == msg.epc) & (not self.robot_moving):
            self.time = rospy.Time.to_sec(rospy.Time.now())
            self.reading_list.append(BaseTagReading(msg.antenna_port - 1, 10 ** ((msg.RSSI - 30) / 10),
                                                    self.base_link_to_odom_transfrom(def_pose([0, 0, 0], [0, 0, 0, 1]))
                                                    .pose.orientation, rospy.Time.to_sec(rospy.Time.now())))
            if len(self.reading_list) >= self.samples:
                self.msgs_full = True  # stop averaging
            rospy.logdebug("stack size: " + str(len(self.reading_list)) + "    port: " + str(msg.antenna_port - 1) +
                           "    RSSI: " + str(10 ** ((msg.RSSI - 30) / 10)))

    def base_link_to_odom_transfrom(self, pose):
        self.listener.waitForTransform(self.tf_prefix + 'odom', self.tf_prefix + 'base_link', rospy.Time(0),
                                       rospy.Duration(5))
        return self.listener.transformPose(self.tf_prefix + 'odom', pose)

    def send_goal(self, odom_goal, string):
        self.goal_pose.target_pose = odom_goal
        print("GOAL POSE: " + string + "\n" + str(self.goal_pose))
        # self.client.send_goal(self.goal_pose)
        # self.client.wait_for_result()
        rospy.sleep(30)
        rospy.on_shutdown(self.client.cancel_goal)

    def full_movement(self, string):
        print(string)
        antenna = self.antenna_decision()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw[antenna])
        pose = self.direction[antenna]
        self.reading_list = []
        self.reading_average = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        self.send_goal(self.base_link_to_odom_transfrom(def_pose(pose, quaternion)), "2en1")
        # self.send_goal(self.base_link_to_odom_transfrom(def_pose([0, 0, 0], quaternion)), "turn")
        # if not self.DS_found:
        #     self.send_goal(self.base_link_to_odom_transfrom(def_pose([1, 0, 0], [0, 0, 0, 1])), "move")

    def antenna_decision(self):
        for reading in self.reading_list:  # for reading in self.reading_list[-self.samples:]:
            rotated_port = self.port_rotation(reading)
            self.reading_average[0][rotated_port] += 1
            self.reading_average[1][rotated_port] += reading.rssi
        return self.weighted_selection()

    # def angle_selection(self):
    #     [ord(x) for x in self.reading_list]
    #     self.reading_list.sort()

    def weighted_selection(self):
        for x in range(self.n_antennas):
            self.reading_average[2][x] = self.reading_average[1][x] * self.reading_average[0][x]
        rospy.logdebug("reading average: " + str(self.reading_average) + "\n")
        rospy.logdebug("antenna selection: " + str(self.reading_average[2].index(max(self.reading_average[2]))) + "\n")
        return self.reading_average[2].index(max(self.reading_average[2]))

    def port_rotation(self, reading):
        angular_difference = tf.transformations.euler_from_quaternion(quaternion_to_list(
            self.base_link_to_odom_transfrom(def_pose([0, 0, 0], [0, 0, 0, 1])).pose.orientation))[2] - \
                             tf.transformations.euler_from_quaternion(quaternion_to_list(reading.orientation))[2]
        # print("\nangular_difference: " + str(angular_difference) + "   final angle: " + str(tf.transformations.
        #     euler_from_quaternion(quaternion_to_list(self.base_link_to_odom_transfrom(def_pose([0, 0, 0], [0, 0, 0, 1]))
        #                        .pose.orientation)))[2] + "   measurement angle: " +
        #       str(tf.transformations.euler_from_quaternion(quaternion_to_list(reading.orientation)))[2])
        if (-np.pi / 4 <= angular_difference <= np.pi / 4) or (7 * np.pi / 4 <= angular_difference <= -7 * np.pi / 4):
            rotation = 0
        elif (-np.pi / 4 >= angular_difference >= -3 * np.pi / 4) or (
                5 * np.pi / 4 <= angular_difference <= 7 * np.pi / 4):
            rotation = 1
        elif (3 * np.pi / 4 <= angular_difference <= 5 * np.pi / 4) or (
                -3 * np.pi / 4 <= angular_difference <= -5 * np.pi / 4):
            rotation = 2
        elif (np.pi / 4 <= angular_difference <= 3 * np.pi / 4) or (
                -5 * np.pi / 4 >= angular_difference >= -7 * np.pi / 4):
            rotation = 3
        else:
            rotation = 0
            rospy.logdebug("Rotation out of limits: CHECK port rotation method!!!!!!!!")
        port_rot = [0] * self.n_antennas
        for i in range(self.n_antennas):
            if self.antenna_map.items()[i][1] == reading.antenna_port:
                port_face = self.antenna_map.items()[i][0]
            port_rot[i] = self.ports[(i + rotation) % self.n_antennas]
        return self.antenna_map[port_rot[self.ports.index(port_face)]]

    def go_back_home(self):
        print("GOING BACK HOME\n")
        odom = self.base_link_to_odom_transfrom(def_pose([0, 0, 0], [0, 0, 0, 1]))
        v = [-odom.pose.position.x, -odom.pose.position.y, 0]
        self.send_goal(def_pose([odom.pose.position.x, odom.pose.position.y, odom.pose.position.z] +
                                v / np.linalg.norm(v), [0, 0, 0, 1], self.tf_prefix + "odom"), "going back home")


if __name__ == '__main__':
    try:
        rospy.init_node('nav_to_base', log_level=rospy.DEBUG)
        nbb = NavigateBackToBase()
        while not rospy.is_shutdown():
            if (not nbb.DS_found) & nbb.navigation_mode_on:
                if nbb.msgs_full:
                    nbb.msgs_full = False
                    nbb.robot_moving = True
                    nbb.full_movement("NORMAL BEHAVIOR\n")
                    nbb.robot_moving = False
                    rospy.sleep(2)
                if (rospy.Time.to_sec(rospy.Time.now()) > nbb.time +
                        rospy.Time.to_sec(rospy.Time(nbb.time_between_readings))):
                    if len(nbb.reading_list) == 0:
                        nbb.go_back_home()
                    else:
                        nbb.full_movement("1st RECOVERY!!!\n")

    except rospy.ROSInterruptException:
        pass
