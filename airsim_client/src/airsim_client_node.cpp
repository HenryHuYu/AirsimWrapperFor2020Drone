
#include "airsim_ros_client.h"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "airsim_client_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");


    std::string host_ip = "192.168.22.29";
    // nh_private_.getParam("host_ip", host_ip);
    //DroneController dc(nh_, nh_private_, host_ip);
    SensorWrapper sw(nh_, nh_private_, host_ip);
    ROS_INFO("finish init sensor wrapper");
    sw.img_front_async_spinner_.start();
    ros::spin();

    return 0;
} 
