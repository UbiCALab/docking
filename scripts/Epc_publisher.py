#!/usr/bin/env python

from nav_msgs.msg import Odometry
import rospy
import tf
from ant_robot.msg import EpcInfo
from geometry_msgs.msg import Vector3Stamped, PointStamped
import math


class EpcsPublisher:

    def __init__(self):
        self.p = Vector3Stamped()
        self.p.header.frame_id = 'odom'
        self.g = PointStamped()
        self.g.point.x = -5
        self.g.point.y = 5
        self.g.point.z = 0
        self.g.header.frame_id = 'odom'
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
        self.listener = tf.TransformListener()
        self.odom_msg = Odometry()
        self.msg_rfid = EpcInfo()
        self.base_EPC = rospy.get_param('RFIDrx/base1')
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odometry_cb)
        self.epc_publisher = rospy.Publisher('epcs', EpcInfo, queue_size=1)

    def odometry_cb(self, msg):
        self.odom_msg = msg

    def publish_ecp(self):

        self.msg_rfid.epc = self.base_EPC
        self.msg_rfid.ts = 0
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
        self.msg_rfid.antenna_port = a.index(min(a))
        self.msg_rfid.RSSI = 1 / math.sqrt(self.p.vector.x**2 + self.p.vector.y**2)
        # rospy.logdebug("Antenna num." + str(a.index(min(a))))
        self.epc_publisher.publish(self.msg_rfid)


if __name__ == '__main__':
    rospy.init_node('RFID_Rx', log_level=rospy.DEBUG)
    epc = EpcsPublisher()
    rate = rospy.Rate(0.25)  # 1Hz
    while not rospy.is_shutdown():
        epc.publish_ecp()
        rate.sleep()
