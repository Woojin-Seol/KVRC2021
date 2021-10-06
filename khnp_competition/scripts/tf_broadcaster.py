#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from tf2_msgs.msg import TFMessage
import tf

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class caster():
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)
        self.model_name = rospy.get_param("/robot_name", '/')
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.base_cb)
        self.rate = rospy.Rate(2)

        self.br = tf.TransformBroadcaster()

    def base_cb(self, msg):
        self.timestamp = rospy.Time.now()
        for i in range(len(msg.name)):
            if msg.name[i] == self.model_name:
                current_pose = msg.pose[i]
                self.br.sendTransform((current_pose.position.x, current_pose.position.y, current_pose.position.z),\
(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w),\
self.timestamp,"base_link","map")
        self.br.sendTransform((-0.04, 0.0, 0.05), (0.0,0.0,-0.7071,0.7071), self.timestamp, "d435_camera_link","ARM_6_1")
        self.br.sendTransform((0.28, 0.0, 0.08), (0.5,-0.5,0.5,-0.5), self.timestamp, "l515_camera_link","base_link")
        self.br.sendTransform((-0.28, 0.0, 0.08), (-0.5,-0.5,0.5,0.5), self.timestamp, "l515_camera_link_back","base_link")
        self.br.sendTransform((0.285, 0.032, 0.1), (0.5,-0.5,0.5,-0.5), self.timestamp, "t265_camera_link","base_link")
        self.br.sendTransform((0.285, -0.032, 0.1), (0.5,-0.5,0.5,-0.5), self.timestamp, "t265_camera_link_right","base_link")
        return

if __name__ == '__main__':
    cas = caster()
    while 1:
        try:
            cas.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
