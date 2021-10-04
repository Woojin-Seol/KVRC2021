#!/usr/bin/env python

import time
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32

import signal
import sys

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class mason_spawner():
    def __init__(self):
        rospy.init_node('mason_controller', anonymous=True)
        self.champ_subscriber = rospy.Subscriber('/joint_group_position_controller/command_no_arm', JointTrajectory, self.champ_cb)
        self.finger_subscriber = rospy.Subscriber('/joint_finger_simple/command', Float32, self.finger_cb)
        self.quad_arm_pub = rospy.Publisher('/joint_group_position_controller/command', JointTrajectory, queue_size=10)
        self.fingerpub = rospy.Publisher('/joint_finger_position_controller/command', JointTrajectory, queue_size=10)
        self.rate = rospy.Rate(1)
        self.pub_data=[0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0]

    def champ_cb(self, msg):
        msg.joint_names.append("arm_1_joint")
        msg.joint_names.append("arm_2_joint")
        msg.joint_names.append("arm_3_joint")
        msg.joint_names.append("arm_4_joint")
        msg.joint_names.append("arm_5_joint")
        msg.joint_names.append("arm_6_joint")
        for i in range(12):
            self.pub_data[i]=msg.points[0].positions[i]
        self.pub_data[12]=0.0
        self.pub_data[13]=-1.57
        self.pub_data[14]=-1.57
        self.pub_data[15]=0.0
        self.pub_data[16]=-1.57
        self.pub_data[17]=0.0
        msg.points[0].positions=self.pub_data
        self.quad_arm_pub.publish(msg)

    def finger_cb(self, msg):
        fdata=JointTrajectory()
        fpoints=JointTrajectoryPoint()
        fdata.header.stamp=rospy.Time.now()
        fdata.joint_names=["finger_joint", "left_inner_knuckle_joint", "left_inner_finger_joint", "right_outer_knuckle_joint", "right_inner_knuckle_joint", "right_inner_finger_joint"]

        data=msg.data
        pub_data=[data, data, -data, data, data, -data]
        fpoints.positions=pub_data
        fpoints.time_from_start = rospy.Duration(0.1)

        fdata.points.append(fpoints)
        self.fingerpub.publish(fdata)


if __name__=='__main__':
    mas = mason_spawner()
    time.sleep(1)
    while True:
        try:
            mas.rate.sleep()	    
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
            sys.exit(0)
    sys.exit(0)
