#!/usr/bin/env python

import time

import rospy
import rospkg #for get_path
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

import signal
import sys

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class mason_spawner():
    def __init__(self):
        rospy.init_node('mason_spawner', anonymous=True)
        self.gt_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gt_cb)
        self.msg_subscriber = rospy.Subscriber('/spawning_model', Empty, self.msg_cb)
        self.spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.rate = rospy.Rate(1)

        self.if_cam_spawned=False
        self.if_gz_on=False

    def gt_cb(self, msg):
        if not self.if_gz_on:
            rp = rospkg.RosPack()
            package_path = rp.get_path("khnp_competition")
            f = open(package_path+"/../gazebo_map_for_khnp/common/third_camera/model.sdf",'r')

            self.model_name = rospy.get_param("/third_cam_name", 'third_camera')
            self.model_xml = f.read()
            self.initial_pose = Pose()
            self.initial_pose.orientation.w=1.0
            f.close()
            self.if_gz_on=True

    def msg_cb(self, msg):
        if not self.if_cam_spawned and self.if_gz_on:
            response=self.spawner(self.model_name, self.model_xml, "", self.initial_pose, "world")
            print(response.success)
            print(response.status_message)
            if response.success:
                self.if_cam_spawned=True
                print("Cam spawned!!")


if __name__=='__main__':
    mas = mason_spawner()
    time.sleep(2)
    while True:
        try:
            mas.rate.sleep()	    
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
            sys.exit(0)
        finally:
            if mas.if_cam_spawned:
                break
    sys.exit(0)
