#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

class RobotPosition(object):

    """get the position of robot from /gazebo/model_states and re publish it with /robot_position"""

    _robot_name = "youbot"

    def __init__(self):
        self.robot_pose_publisher = rospy.Publisher('robot_position', PoseStamped, queue_size=5)
        self.gazebo_pose_subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, self.callback_function)
        rospy.loginfo("Initiated node")

    def callback_function(self, msg): 
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        if self._robot_name in msg.name :
            index = msg.name.index(self._robot_name)
            pose.pose = msg.pose[index]
        self.robot_pose_publisher.publish(pose)

if __name__ == '__main__':
    rospy.init_node('robot_position')
    RobotPosition()
    rospy.spin()
