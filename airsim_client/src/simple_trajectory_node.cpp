
#include "simple_trajectory.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_trajectory");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    // trajectoryPublisher referencePublisher(nh, nh_private);
    Eigen::Vector3d init_home;
    init_home << 0.0, 0.0, 1.209;
    trajectoryPublisher refPub(nh, nh_private, init_home);
    
    ros::spin();
    return 0;
}
