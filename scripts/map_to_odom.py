#!/usr/bin/env python  

import roslib
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import tf

class MapToOdom(object):

    """publish a transform from map to odom"""

    _robot_init_pos_x = -1.0
    _robot_init_pos_y = 0.0
    _robot_init_pos_z = 0.0
    _robot_init_quat = (0,0,0,1)

    def __init__(self):
        """TODO: to be defined1. """
        rospy.init_node('map_to_odom')
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.robot_sub = rospy.Subscriber('robot_position', PoseStamped, self.robot_position_cb)
        self.br = tf.TransformBroadcaster()
        self._robot_init_theta = tf.transformations.euler_from_quaternion(self._robot_init_quat)[2]
        self.robot_x, self.robot_y, self.robot_theta = 0,0,0
        self.old_transform = 0
        
    def robot_position_cb(self, msg):
        """ callback function for robot position messages
        :msg: PoseStamped object
        :returns: None
        """
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        quat = msg.pose.orientation
        angle = (quat.x, quat.y, quat.z, quat.w)
        self.robot_theta = tf.transformations.euler_from_quaternion(angle)[2]
#         rospy.loginfo(str(x) + ", " + str(y) + ", " + str(theta))
        self.robot_x -= self._robot_init_pos_x
        self.robot_y -= self._robot_init_pos_y
        self.robot_theta -= self._robot_init_theta

    def odom_cb(self, msg):
        """callback function for odometry messages

        :msg: Odometry object
        :returns: None
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        angle = (quat.x, quat.y, quat.z, quat.w)
        theta = tf.transformations.euler_from_quaternion(angle)[2]
        transform_x = self.robot_x - x
        transform_y = self.robot_y - y
        transform_theta = self.robot_theta - theta
#         rospy.loginfo("Odometry position")
#         rospy.loginfo(str(x) + ", " + str(y) + ", " + str(theta))
#         rospy.loginfo("difference in position")
#         rospy.loginfo(str(transform_x) + ", " + str(transform_y) + ", " + str(transform_theta))
        transform_quat = tf.transformations.quaternion_from_euler(0, 0, transform_theta)
        """
        robot_position is unreliable. It produces erroneous values in x axis few time a minute
        The below section makes sure that those kind of error do not affect the transform calculation
        """
        if abs(transform_x - self.old_transform) > 0.10 :
            self.old_transform = transform_x/2
        else :
            self.old_transform = transform_x
            self.br.sendTransform((transform_x, transform_y, 0),
                             transform_quat,
                             msg.header.stamp,
                             "odom",
                             "map")


if __name__ == '__main__':
    map_to_odom = MapToOdom()
    rospy.spin()
