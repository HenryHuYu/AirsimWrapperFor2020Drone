#include "common/common_utils/FileSystem.hpp"

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

// TODO more specific
void writeImgtoFile(cv::Mat img, std::string s = " ") {
    static int i = 0;
    std::ostringstream oss;
    std::string file_name="/home/henry/Desktop/img/";
    oss<< file_name << s << "_" << i << ".png";
    ROS_INFO("%s\n", file_name.data());
    i++;
    cv::imwrite(oss.str(), img);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("image callback triggered");
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    writeImgtoFile(cv_ptr->image, "test");
}


void depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("Depthimage callback triggered");
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    writeImgtoFile(cv_ptr->image, "depth");
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "image_listener_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");

    image_transport::ImageTransport it(nh_);
    image_transport::Subscriber scece_sub = it.subscribe("/airsim/sensor/scene_image", 1, imageCallback);
    image_transport::Subscriber depth_sub = it.subscribe("/airsim/sensor/depth_image", 1, depthImageCallback);
  
    ROS_INFO("start spin");
    ros::spin();
    
    return 0;
} 

