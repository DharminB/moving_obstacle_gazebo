#!/usr/bin/env python  

import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('pose_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    rospy.loginfo("outside while")
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/robot', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rospy.loginfo(trans)
        rospy.loginfo(rot)
        rate.sleep()
