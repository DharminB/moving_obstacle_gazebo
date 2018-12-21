#! /usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point

class RelativePosition(object):

    """Get position of robot from /robot_position and position of moving obstacles from /moving_obstacle_position
    and publish relative position of obstacles from robot"""

    def __init__(self):
        self.robot_pose_subscriber = rospy.Subscriber('robot_position', PoseStamped, self.robot_position_cb)
        self.obstacle_pose_subscriber = rospy.Subscriber('moving_obstacle_position', PoseArray, self.obstacle_position_cb)
        self.relative_pose_publisher = rospy.Publisher('relative_obstacle_pose', PolygonStamped, queue_size=5)
        rospy.loginfo("Initiated node")
        self._latest_msg_time = None
        self._latest_robot_pose = None
        self._latest_obstacle_poses = None

    def robot_position_cb(self, msg):
        """callback function for robot position subscriber

        :msg: PoseStamped
        :returns: None
        """
        self._latest_robot_pose = msg
        if self._latest_msg_time == msg.header.stamp :
            self.calculate_relative_position()
        else :
            self._latest_msg_time = msg.header.stamp

    def obstacle_position_cb(self, msg):
        """callback function for obstacle position Subscriber

        :msg: PoseArray
        :returns: None
        """
        self._latest_obstacle_poses = msg
        if self._latest_msg_time == msg.header.stamp :
            self.calculate_relative_position()
        else :
            self._latest_msg_time = msg.header.stamp

    def calculate_relative_position(self):
        """calculate relative position of obstacles from robot position
        :returns: None
        """
#         rospy.loginfo("inside calculate relative position")
        relative_poses = PolygonStamped()
        relative_poses.header.stamp = self._latest_robot_pose.header.stamp
        robot_x = self._latest_robot_pose.pose.position.x
        robot_y = self._latest_robot_pose.pose.position.y

        for obstacle_pose in self._latest_obstacle_poses.poses :
            dist = self.calculate_distance(
                    robot_x,
                    robot_y,
                    obstacle_pose.position.x,
                    obstacle_pose.position.y)
            theta = math.atan2(obstacle_pose.position.y - robot_y, obstacle_pose.position.x - robot_x)
            point = Point()
            point.x = math.cos(theta) * dist
            point.y = math.sin(theta) * dist
            relative_poses.polygon.points.append(point)
            
            
        self.relative_pose_publisher.publish(relative_poses)

    def calculate_distance(self, x1, y1, x2, y2):
        """calculate cartesian distance between 2 points
        :x1: float
        :y1: float
        :x2: float
        :y2: float
        :returns: float
        """
        return ((x1-x2)**2 + (y1-y2)**2)**0.5

# ======== Main ==============
if __name__ == '__main__':
    rospy.init_node('relative_position')
    RelativePosition()
    rospy.spin()
