#!/usr/bin/env python

from nav_msgs.msg import Odometry, OccupancyGrid
import rospy
import tf
from geometry_msgs.msg import Vector3Stamped, PointStamped, PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from turtlebot_actions.msg import TurtlebotMoveAction, TurtlebotMoveGoal
import numpy as np
import math


class SergiMoveBase:

    def __init__(self):
        self.g = PointStamped()
        self.listener = tf.TransformListener()
        rospy.Subscriber('odom', Odometry, self.odometry_cb)
        self.odom_msg = Odometry()
        rospy.Subscriber('move_base/local_costmap/costmap', OccupancyGrid, self.costmap_cb)
        self.costmap_msg = OccupancyGrid()
        self.a1 = Vector3Stamped()
        self.a2 = Vector3Stamped()
        self.a3 = Vector3Stamped()
        self.a4 = Vector3Stamped()
        self.a1.vector.x = 0.18
        self.a1.vector.y = 0
        self.a1.vector.z = 0
        self.a1.header.frame_id = 'base_link'
        self.a2.vector.x = 0
        self.a2.vector.y = 0.18
        self.a2.vector.z = 0
        self.a2.header.frame_id = 'base_link'
        self.a3.vector.x = -0.18
        self.a3.vector.y = 0
        self.a3.vector.z = 0
        self.a3.header.frame_id = 'base_link'
        self.a4.vector.x = 0
        self.a4.vector.y = -0.18
        self.a4.vector.z = 0
        self.a4.header.frame_id = 'base_link'
        self.p = Vector3Stamped()
        self.p.header.frame_id = 'odom'
        self.yaw = [0, np.pi / 2, np.pi, 3 * np.pi / 2]
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'base_link'
        self.rate = rospy.Rate(0.25)
        self.sac_tb = actionlib.SimpleActionClient('turtlebot_move', TurtlebotMoveAction)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_tw = TurtlebotMoveGoal()
        self.goal_pose = MoveBaseGoal()

    def set_goal(self, x, y, z):
        self.g.point.x = x
        self.g.point.y = y
        self.g.point.z = z
        self.g.header.frame_id = 'odom'

    def odometry_cb(self, msg):
        self.odom_msg = msg

    def ant_calc(self):
        self.p.vector.x = self.odom_msg.pose.pose.position.x - self.g.point.x
        self.p.vector.y = self.odom_msg.pose.pose.position.y - self.g.point.y
        self.listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(5))
        a1_odom = self.listener.transformVector3('odom', self.a1)
        a2_odom = self.listener.transformVector3('odom', self.a2)
        a3_odom = self.listener.transformVector3('odom', self.a3)
        a4_odom = self.listener.transformVector3('odom', self.a4)
        a = [a1_odom.vector.x * self.p.vector.x + a1_odom.vector.y * self.p.vector.y,
             a2_odom.vector.x * self.p.vector.x + a2_odom.vector.y * self.p.vector.y,
             a3_odom.vector.x * self.p.vector.x + a3_odom.vector.y * self.p.vector.y,
             a4_odom.vector.x * self.p.vector.x + a4_odom.vector.y * self.p.vector.y]
        i = a.index(min(a))
        rospy.logdebug("Antenna num." + str(a.index(min(a))))
        return i

    def next_move(self, i):
        # self.sac_tb.wait_for_server()
        # self.goal_tw.forward_distance = 0
        # self.goal_tw.turn_distance = self.yaw[i]
        # self.sac_tb.send_goal(self.goal_tw)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw[i])
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
        self.pose.pose.orientation.w = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.position.x = 0.5
        self.listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(5))
        self.goal_pose.target_pose = self.listener.transformPose('odom', self.pose)
        self.client.send_goal(self.goal_pose)
        self.rate.sleep()

    def check_goal(self):
        return math.sqrt(((self.g.point.x-self.odom_msg.pose.pose.position.x)**2) +
                         ((self.g.point.y-self.odom_msg.pose.pose.position.y)**2)) < 0.2
        # return np.absolute(np.subtract(self.g.point, self.odom_msg.pose.pose.position)) < 0.5

    def costmap_cb(self, msg):
        self.costmap_msg = msg



if __name__ == '__main__':
    try:
        rospy.init_node('oodometry', log_level=rospy.DEBUG)
        RFID_tag_reach = False
        smb = SergiMoveBase()
        smb.set_goal(1, 0, 0)
        # while not RFID_tag_reach:
        smb.next_move(smb.ant_calc())
        RFID_tag_reach = smb.check_goal()
        print smb.costmap_msg


    except rospy.ROSInterruptException:
        pass

