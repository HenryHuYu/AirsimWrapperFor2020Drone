

#include "simple_trajectory.h"


circletrajectory::circletrajectory():
    dt_(0.1),
    T_(10)
{
 
    radius_ = 1.0;
    omega_ = 2.0;
    axis_ << 0.0, 0.0, -1.0;
    radius_3d_ << radius_, 0.0, 0.0;
    traj_origin_ << 0.0, 0.0, 0.0;
}

circletrajectory::~circletrajectory() {
    
}

Eigen::Vector3d circletrajectory::getPosition(double time) {

    Eigen::Vector3d position;
    double theta;

    theta = omega_* time;
    position = std::cos(theta) * radius_3d_
                 + std::sin(theta) * axis_.cross(radius_3d_)
                 + (1 - std::cos(theta)) * axis_.dot(radius_3d_) * axis_
                 + traj_origin_;
    
    return position;
}

Eigen::Vector3d circletrajectory::getVelocity(double time) {
    Eigen::Vector3d velocity;
    double theta;
    velocity = omega_ * axis_.cross(getPosition(time));
    return velocity;
}

Eigen::Vector3d circletrajectory::getAcceleration(double time){

    Eigen::Vector3d acceleration;
    acceleration = omega_ * axis_.cross(getVelocity(time));
    return acceleration;
}

//--------------------------------------------------------------------------------------//

verticaltrajectory::verticaltrajectory(Eigen::Vector3d home_position):
    dt_(0.1),
    T_(3),
    home_position_(home_position)
{
    c1 = 0.04;
    c2 = -0.12;
    c3 = -0.36;
}

verticaltrajectory::~verticaltrajectory(){

}

Eigen::Vector3d verticaltrajectory::getPosition(double time) {
    Eigen::Vector3d position;
    double z = c1 * std::pow(time, 3) + c2 * std::pow(time, 2) + c3 * time;
    position << 0.0, 0.0, z;
    return position + home_position_;
}

Eigen::Vector3d verticaltrajectory::getVelocity(double time) {
    Eigen::Vector3d vel;
    double z_vel = 3 * c1 * std::pow(time, 2) + 2 * c2 * time + c3;
    vel << 0.0, 0.0, z_vel;
    return vel;
}

Eigen::Vector3d verticaltrajectory::getAcceleration(double time) {
    Eigen::Vector3d acc;
    double z_a = 6 * c1 * time + 2 * c2;
    acc << 0.0, 0.0, z_a;
    return acc;
}


//----------------------------------------------------------------------------------// 

zytrajectory::zytrajectory(Eigen::Vector3d home_position):
    dt_(0.1),
    T_(3),
    home_position_(home_position)
{
    c1 = -0.12;
    c2 = 0.36;
    c3 = 1.08;
}


Eigen::Vector3d zytrajectory::getPosition(double time) {
    Eigen::Vector3d position;
    double yz = c1 * std::pow(time, 3) + c2 * std::pow(time, 2) + c3 * time;
    position << 0.0, yz, 0;
    return position;// + home_position_;
}

Eigen::Vector3d zytrajectory::getVelocity(double time) {
    Eigen::Vector3d vel;
    double yz_vel = 3 * c1 * std::pow(time, 2) + 2 * c2 * time + c3;
    vel << 0.0,  yz_vel, 0;
    return vel;
}

Eigen::Vector3d zytrajectory::getAcceleration(double time) {
    Eigen::Vector3d acc;
    double yz_a = 6 * c1 * time + 2 * c2;
    acc << 0.0, yz_a, 0;
    return acc;
}

//-------------------------------------------------------------------------------------//

trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const Eigen::Vector3d home_bias):
nh_(nh),
nh_private_(nh_private),
vertical_traj(home_bias),
zy_traj(home_bias_)
{
    
    referencePub_ = nh_.advertise<geometry_msgs::TwistStamped>("/airsim/traj_target/posevel", 2);
    refloop_timer_ = nh_.createTimer(ros::Duration(0.02), &trajectoryPublisher::pubTargetCB, this);
}

trajectoryPublisher::~trajectoryPublisher(){}

void trajectoryPublisher::pubTargetCB(const ros::TimerEvent& event) {

    updateTarget(TRAJ_ZY);
    geometry_msgs::TwistStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.twist.angular.x = p_targ(0);
    msg.twist.angular.y = p_targ(1);
    msg.twist.angular.z = p_targ(2);
    msg.twist.linear.x = v_targ(0);
    msg.twist.linear.y = v_targ(1);
    msg.twist.linear.z = v_targ(2);
    referencePub_.publish(msg);
}

void trajectoryPublisher::updateTarget(int type) {
    if (first_pub_)
    {
        first_pub_ = false;
        first_time_ = ros::Time::now();
    }
    curr_time_ = ros::Time::now();
    trigger_time_ = (curr_time_ - first_time_).toSec();
    ROS_INFO_STREAM("trigger time is " << trigger_time_);
    
    if (trigger_time_ >= 3.0)
    {
        trigger_time_ = 3.0;
    }
    
    switch (type)
    {
    case TRAJ_VERTICAL:
    {
        p_targ = vertical_traj.getPosition(trigger_time_);

        v_targ = vertical_traj.getVelocity(trigger_time_);
        break;
    }
       
    
    case TRAJ_CIRCLE:
    {
        p_targ = circle_traj.getPosition(trigger_time_);
        v_targ = circle_traj.getVelocity(trigger_time_);
        break;
    }

    case TRAJ_ZY:
    {
        p_targ = zy_traj.getPosition(trigger_time_);
        v_targ = zy_traj.getVelocity(trigger_time_);
        break;
    }
    default:
        break;
    }
    

    

    
}