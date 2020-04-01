#!/usr/bin/env python

import roslib; roslib.load_manifest('kobuki_auto_docking')
import rospy
import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal, DockInfraRed
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty, SetBool


class Docking:

    def __init__(self):
        self.IR_sensor = False
        self.start = False
        self.sub = {'dock_ir': rospy.Subscriber('mobile_base/sensors/dock_ir', DockInfraRed, self.dock_ir_call_back, queue_size=1)}
        self.dock_ir = DockInfraRed()
        self.goal = AutoDockingGoal()
        self.client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
        self.state = ''
        self.srv_client = rospy.ServiceProxy('start_docking_srv', Empty)
        self.dock_srv = rospy.Service('allow_dockin_srv', SetBool, self.cb_allow_docking)

    def cb_allow_docking(self, msg):
        self.start = msg.data
        if not self.start:
            self.client.cancel_all_goals()
        return True, ''

    def dock_ir_call_back(self, msg):
        if not rospy.is_shutdown():
            self.dock_ir = [ord(x) for x in msg.data]
            # print("INFRA RED MESSAGE: " + str(self.dock_ir))
            if self.dock_ir[0] + self.dock_ir[1] + self.dock_ir[2] != 0:
                self.IR_sensor = True

    def doneCb(self, status, result):
        if 0:
            print ''
        elif status == GoalStatus.PENDING:
            self.state = 'PENDING'
        elif status == GoalStatus.ACTIVE:
            self.state = 'ACTIVE'
        elif status == GoalStatus.PREEMPTED:
            self.state = 'PREEMPTED'
        elif status == GoalStatus.SUCCEEDED:
            self.state = 'SUCCEEDED'
        elif status == GoalStatus.ABORTED:
            self.state = 'ABORTED'
        elif status == GoalStatus.REJECTED:
            self.state = 'REJECTED'
        elif status == GoalStatus.PREEMPTING:
            self.state = 'PREEMPTING'
        elif status == GoalStatus.RECALLING:
            self.state = 'RECALLING'
        elif status == GoalStatus.RECALLED:
            self.state = 'RECALLED'
        elif status == GoalStatus.LOST:
            self.state = 'LOST'
        # Print state of action server
        print 'Result - [ActionServer: ' + self.state + ']: ' + result.text
        self.IR_sensor = False

    def activeCb(self):
        if 0: print 'Action server went active.'

    def feedbackCb(self, feedback):
        # Print state of dock_drive module (or node.)
        print 'Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text

    def dock_drive_client(self):
        # add timeout setting
        while not self.client.wait_for_server(rospy.Duration(5.0)):
            if rospy.is_shutdown(): return
            print 'Action server is not connected yet. still waiting...'
        self.client.send_goal(self.goal, self.doneCb, self.activeCb, self.feedbackCb)
        print 'Goal: Sent.'
        rospy.on_shutdown(self.client.cancel_goal)
        self.client.wait_for_result(rospy.Duration(30))
        # print '    - status:', self.client.get_goal_status_text()
        return self.client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('docking', log_level=rospy.DEBUG)
        dc = Docking()
        while True:
            if dc.IR_sensor & dc.start:
                dc.srv_client.wait_for_service(10)
                dc.resp = dc.srv_client()
                dc.result = dc.dock_drive_client()
                print("STATE:       " + str(dc.client.get_state()))

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
