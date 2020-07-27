#include "airsim_ros_client.h"

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/imu/ImuBase.hpp"

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/console.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <mutex>
#include <chrono>
#include <cstdlib>


std::string host_ip = "192.168.22.29";

ros::Timer ground_truth_pos_timer;
ros::Publisher ground_truth_pos_publisher;
Client ground_truth_client(host_ip);
msr::airlib::Kinematics::State state_ground_truth;
msr::airlib::MultirotorState cheat_drone_state;

Eigen::Vector4d home_orientation(0.707107, 0.0, 0.0, -0.707107);
Eigen::Vector4d cur_ori;
Eigen::Vector4d ref_to_home_ori;

Eigen::Vector4d quatMulti(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
    Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
          p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
          p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
          p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}


void pubGroundTruthPoseCB(const ros::TimerEvent& event) {
    cheat_drone_state  = ground_truth_client.getMultirotorState();
    state_ground_truth  = ground_truth_client.simGetGroundTruthKinematics();

    nav_msgs::Odometry odom_ned_msg;
    
    odom_ned_msg.pose.pose.position.x = state_ground_truth.pose.position.x();
    odom_ned_msg.pose.pose.position.y = state_ground_truth.pose.position.y();
    odom_ned_msg.pose.pose.position.z = state_ground_truth.pose.position.z();
    
    cur_ori << state_ground_truth.pose.orientation.w(), state_ground_truth.pose.orientation.x(), state_ground_truth.pose.orientation.y(), state_ground_truth.pose.orientation.z();
    ref_to_home_ori = quatMulti(home_orientation, cur_ori);
    odom_ned_msg.pose.pose.orientation.w = state_ground_truth.pose.orientation.w();//ref_to_home_ori(0);
    odom_ned_msg.pose.pose.orientation.x = state_ground_truth.pose.orientation.x();//ref_to_home_ori(1);
    odom_ned_msg.pose.pose.orientation.y = state_ground_truth.pose.orientation.y();//ref_to_home_ori(2);
    odom_ned_msg.pose.pose.orientation.z = state_ground_truth.pose.orientation.z();//ref_to_home_ori(3);

    odom_ned_msg.twist.twist.linear.x = state_ground_truth.twist.linear.x();
    odom_ned_msg.twist.twist.linear.y = state_ground_truth.twist.linear.y();
    odom_ned_msg.twist.twist.linear.z = state_ground_truth.twist.linear.z();

    odom_ned_msg.twist.twist.angular.x = state_ground_truth.twist.angular.x();
    odom_ned_msg.twist.twist.angular.y = state_ground_truth.twist.angular.y();
    odom_ned_msg.twist.twist.angular.z = state_ground_truth.twist.angular.z();

    odom_ned_msg.header.stamp = ros::Time::now();
    
    ground_truth_pos_publisher.publish(odom_ned_msg);

    
}

void testTCPCallBack() {
    ;
}

int main(int argc, char ** argv)
{
    
    ros::init(argc, argv, "cheat_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");

    /*
    try
    {
        ground_truth_client.confirmConnection();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    */


    ground_truth_pos_publisher = nh_.advertise<nav_msgs::Odometry>("/airsim/drone_state/local_pose", 1);
    ground_truth_pos_timer = nh_.createTimer(ros::Duration(0.01), &pubGroundTruthPoseCB); // TODO MODIFY TIME
    
    ROS_INFO("start publish");
    ros::spin();
    
    
    return 0;
} 

