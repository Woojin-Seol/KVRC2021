#!/usr/bin/env python

import time
from math import pow, sqrt
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState, ApplyBodyWrench, DeleteModel
from geometry_msgs.msg import Point, Wrench

import signal
import sys

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class mason_moderator():
    def __init__(self):
        rospy.init_node('mason_moderator', anonymous=True)
        self.gt_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gt_cb)
        self.robot_move = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.pusher = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.remover = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.rate = rospy.Rate(1)
        self.stage=1
        self.step=0
        self.start = rospy.Time.now()
        self.p = False
        self.s = False
        self.s2 = False

    def gt_cb(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == '/':
                idx = i
        if idx is not None:
            gt_p=msg.pose[idx].position
            print(rospy.Time.now()-self.start)
            if self.step==0 and dist(gt_p.x, gt_p.y, gt_p.z, -23.86, 13.9, 0.02) < 0.5:
                self.start = rospy.Time.now()
                self.step = self.step+1
                return
            if self.step==1 and rospy.Time.now()-self.start>rospy.Duration(20.0):
                #next step
                self.step = self.step+1
                m=ModelState()
                m.model_name = '/'
                m.pose.position.x=-21.2
                m.pose.position.y=13.9
                m.pose.position.z=1.5
                m.pose.orientation.w=1.0
                self.robot_move(m)
                self.start = rospy.Time.now()
                return

            if self.step==2 and dist(gt_p.x, gt_p.y, gt_p.z, -21.2, 13.9, 0.02) < 0.5:
                self.start = rospy.Time.now()
                return
            if not self.p and self.step==2 and rospy.Time.now()-self.start>rospy.Duration(3.0):
                #force
                force=Wrench()
                force.force.x=-100
                force.force.y=20
                self.pusher("/::base", "", Point(), force, rospy.Time.now(), rospy.Duration(2.0))
                self.p = True
                return
            if self.step==2 and rospy.Time.now()-self.start>rospy.Duration(25.0):
                self.step = self.step+1
                #next
                m=ModelState()
                m.model_name = '/'
                m.pose.position.x=-18.5
                m.pose.position.y=13.9
                m.pose.position.z=1.5
                m.pose.orientation.w=1.0
                self.robot_move(m)
                self.start = rospy.Time.now()
                return
                
            if self.step==3 and dist(gt_p.x, gt_p.y, gt_p.z, -18.5, 13.9, 0.02) < 0.5:
                self.start = rospy.Time.now()
                return
            if not self.s and self.step==3 and rospy.Time.now()-self.start>rospy.Duration(3.0):
                #sphere
                m=ModelState()
                m.model_name = 'unit_sphere_1'
                m.pose.position.x=-16.5
                m.pose.position.y=13.9
                m.pose.position.z=1.0
                m.pose.orientation.w=1.0
                m.twist.linear.x=-10.0
                self.robot_move(m)
                self.s = True
                return
            if not self.s2 and self.step==3 and rospy.Time.now()-self.start>rospy.Duration(6.0):
                self.remover("unit_sphere_1")
                self.s2 = True
                return
            if self.step==3 and rospy.Time.now()-self.start>rospy.Duration(25.0):
                self.step = self.step+1
                #next
                m=ModelState()
                m.model_name = '/'
                m.pose.position.x=-15.5
                m.pose.position.y=13.9
                m.pose.position.z=1.5
                m.pose.orientation.w=1.0
                self.robot_move(m)
                return


    def next_step(self, stage, step):
        #get stage and step, set_model_state using array[stage][step]
        return

def dist(x, y, z, x2, y2, z2):
    return sqrt(pow(x-x2,2) + pow(y-y2,2) + pow(z-z2,2))

if __name__=='__main__':
    mas = mason_moderator()
    time.sleep(1)
    while True:
        try:
            mas.rate.sleep()	    
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)

