/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modified by: Dharmin B.
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <iostream>


ros::Publisher pub;
std::set<std::string> names;
/////////////////////////////////////////////////
// Function is called every time a message is received.
void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{

//     ::google::protobuf::int32 sec = posesStamped->time().sec();
//     ::google::protobuf::int32 nsec = posesStamped->time().nsec();
//     std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp.sec = posesStamped->time().sec();
    pose_array.header.stamp.nsec = posesStamped->time().nsec();
    std::vector<geometry_msgs::Pose> poses;
    std::set<std::string> serviced_names;

    for (int i =0; i < posesStamped->pose_size(); ++i)
    {
        const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
        std::string name = pose.name();
        if (names.find(name) != names.end() && !(serviced_names.find(name) != serviced_names.end()))
        {
            serviced_names.insert(name);
//             std::cout << pose.DebugString();
            geometry_msgs::Pose ros_pose;
            ros_pose.position.x = pose.position().x();
            ros_pose.position.y = pose.position().y();
            ros_pose.position.z = pose.position().z();
            poses.push_back(ros_pose);
        }
    }
    pose_array.poses = poses;
//     std::cout << pose_array << std::endl;
    pub.publish(pose_array);
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
    // start ros node
    ros::init(_argc, _argv, "independent_listener_ros");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::PoseArray>("moving_obstacle_position", 1000);
    ros::Rate loop_rate(10);

    ROS_INFO("Initiated node");

    // Load gazebo
    gazebo::client::setup(_argc, _argv);
    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    // Listen to Gazebo pose info topic
    gazebo::transport::SubscriberPtr sub =
    node->Subscribe("~/pose/info", posesStampedCallback);
    
    //assigning names
    for (int i = 0; i < 10; ++i) {
        names.insert("world::cylinder" + std::to_string(i));
    }

    ROS_INFO("Listening...");
    // while ros node is alive
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Terminating");

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
