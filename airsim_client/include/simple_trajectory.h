
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/SetBool.h>


#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>



#define TRAJ_VERTICAL 0
#define TRAJ_CIRCLE 1
#define TRAJ_ZY 2



class circletrajectory {
    private:
        double dt_;
        double T_;
        double radius_;
        double omega_;
        Eigen::Vector3d axis_;
        Eigen::Vector3d radius_3d_;
        Eigen::Vector3d traj_origin_;
    
    public:
        circletrajectory();
        ~circletrajectory();

        Eigen::Vector3d getPosition(double time);
        Eigen::Vector3d getVelocity(double time);
        Eigen::Vector3d getAcceleration(double time); 
        double getsamplingTime(){return dt_;};
        double getDuration(){ return T_;};

};

class verticaltrajectory {
    private:
        double dt_;
        double T_;
        double c1, c2, c3;
        
        Eigen::Vector3d home_position_;
    
    public:
        verticaltrajectory(Eigen::Vector3d home_position);
        ~verticaltrajectory();

        Eigen::Vector3d getPosition(double time);
        Eigen::Vector3d getVelocity(double time);
        Eigen::Vector3d getAcceleration(double time); 
        double getsamplingTime(){return dt_;};
        double getDuration(){ return T_;};

};

class zytrajectory {
    private:
        double dt_;
        double T_;
        double c1, c2, c3;
        
        Eigen::Vector3d home_position_;
    
    public:
        zytrajectory(Eigen::Vector3d home_position);
        ~zytrajectory(){};

        Eigen::Vector3d getPosition(double time);
        Eigen::Vector3d getVelocity(double time);
        Eigen::Vector3d getAcceleration(double time); 
        double getsamplingTime(){return dt_;};
        double getDuration(){ return T_;};
};



class trajectoryPublisher
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Publisher referencePub_;
        ros::Timer refloop_timer_;

        Eigen::Vector3d home_bias_;
        // traj
        circletrajectory circle_traj;
        verticaltrajectory vertical_traj;
        zytrajectory zy_traj;


        Eigen::Vector3d p_targ, v_targ, a_targ;

        // util
        ros::Time first_time_;
        bool first_pub_ = true;
        double trigger_time_;
        ros::Time curr_time_;

    public:
        trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, Eigen::Vector3d home_bias);
        ~trajectoryPublisher();

        void pubTargetCB(const ros::TimerEvent& event);
        void updateTarget(int type);
};

