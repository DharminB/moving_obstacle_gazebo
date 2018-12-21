#!/usr/bin/env python  

import roslib
import rospy

from geometry_msgs.msg import PoseStamped
import tf

def robot_position_cb(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    quat = msg.pose.orientation
    angle = (quat.x, quat.y, quat.z, quat.w)

    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, 0),
                     angle,
                     msg.header.stamp,
                     "robot",
                     "world")

if __name__ == '__main__':
    rospy.init_node('pose_broadcaster')
    rospy.Subscriber('robot_position', PoseStamped, robot_position_cb)
    rospy.spin()
