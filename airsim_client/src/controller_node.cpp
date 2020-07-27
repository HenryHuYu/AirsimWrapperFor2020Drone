
#include "airsim_ros_client.h"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");


    std::string host_ip = "192.168.22.29";
    // nh_private_.getParam("host_ip", host_ip);
    
    DroneController dc(nh_, nh_private_, host_ip);
    /*
    Eigen::Vector4d a(0.7070, 0.0,0.0,0.7072);
    Eigen::Matrix3d x = dc.quat2RotMatrix(a);
    std::cout << x << std::endl;
    Eigen::Matrix3d r; 
    r << 0.0, 1.0, 0.0, 
        -1.0, 0.0, 0.0,
        0.0, 0.0, 1;
    Eigen::Vector4d q = dc.rot2Quaternion(r);
    std::cout << dc.quatMultiplication(q, a) << std::endl;

    std::cout << q; 
    ROS_INFO("finish init controller");
    */
    ros::spin();

} 

