#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"

STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/imu/ImuBase.hpp"

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_srvs/SetBool.h>
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
#include <iostream>
#include <chrono>

typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
typedef msr::airlib::MultirotorRpcLibClient Client;

#define PI_CONST 3.14159265

class SensorWrapper
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // client
    Client sensor_client_;
    Client image_client_;
    

    //thread mutex
    std::recursive_mutex drone_control_mutex_;

    // ros topic 
    image_transport::Publisher scene_img_publisher_;
    image_transport::Publisher depth_img_publisher_;
    ros::Publisher imu_publisher_;
    ros::Publisher height_publisher_;

    
    ros::CallbackQueue img_pub_queue_;
 
    image_transport::Subscriber test_; // ToBe Del
    
    ros::Timer img_loop_timer_;
    ros::Timer imu_loop_timer_;
    ros::Timer height_loop_timer_;

    // imu time functional
    ros::Time first_sensor_t_ros;
    int64_t first_sensor_t_unreal = -1;

    float init_baro_altitude;
    bool init_baro_flag = true;

    // functional
    void initSubandPub();
    void initTimer();

    ros::Time unrealTime2RosTime(uint64_t unreal_ts);
    sensor_msgs::ImagePtr getDepthImgFromResponce(const ImageResponse& img_response,
                                                          const ros::Time curr_ros_time,
                                                          const std::string frame_id);

    sensor_msgs::ImagePtr getSceneImgFromResponce(const ImageResponse& img_response,
                                                          const ros::Time curr_ros_time, 
                                                          const std::string frame_id);

    cv::Mat manual_decode_depth(const ImageResponse& img_response) const;
   
    sensor_msgs::Imu getImuMsgFromAirsim(const msr::airlib::ImuBase::Output& imu_data);
    



  public:
    SensorWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const std::string& host_ip);
    ~SensorWrapper(){};


    // ros::AsyncSpinner img_down_async_spinner_;

    void imuRequestTimerCB(const ros::TimerEvent& event);
    void imgRequestTimerCB(const ros::TimerEvent& event);
    void heightRequestTimerCB(const ros::TimerEvent& event);

    void pubHeightFromBaro(const msr::airlib::BarometerBase::Output& baro_data); // intepret as heigh
    void pubImgsFromResponse(const vector<ImageResponse>& response);
    

    // used as test
    void pubTestImage();

    ros::AsyncSpinner img_front_async_spinner_;
   
};


using namespace Eigen;

class DroneController {
  private:
  // ros 
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber reference_sub_;
    ros::Subscriber local_pose_sub_;
    

    ros::Timer cmd_loop_timer, status_loop_timer;
    ros::ServiceServer ctrl_trigger_srv_;
    ros::ServiceServer takeoff_unitheight_srv;

  // airsim
    Client control_client_;


  // control

    enum FlightState {
        WAITFORHOME, TAKEOFF, HOVER, MISSION_EXEC, LANDING, LANDED
    }drone_state;

    Eigen::Vector3d target_pos_, target_vel_, target_acc_, target_jerk_, target_pos_prev_, target_vel_prev_;
    Eigen::Vector3d cur_pos_, cur_vel_, cur_acc_;
    Eigen::Vector3d g_;
    Eigen::Vector4d cur_att_q_, des_att_q_;
    Eigen::Vector4d cmd_body_rate_; // wx, wy, wz, thrust
    
    bool velocity_yaw_ = false;
  
    double mav_yaw_;
    double init_yaw_rad_ = PI_CONST / 2;

    ros::Time last_request_, reference_request_now_, reference_request_last_;
    double reference_request_dt_;

  // para control
    Eigen::Vector3d Kpos_, Kvel_, D_;
    double attctrl_tau_ = 0.1;
    double tau_x, tau_y, tau_z;
    double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
    double max_fb_acc_ = 6.0;  // without direction
    double norm_thrust_const_ = 0.05;
    double norm_thrust_offset_ = 0.1;
  
  // para time
    float cmd_time_ = 0.02;


  // util

    inline Eigen::Vector3d toEigen(const geometry_msgs::Point& p) {
      Eigen::Vector3d ev3(p.x, p.y, p.z);
      return ev3;
    }

    inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3& v3) {
      Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
      return ev3;
    }

  public:

    DroneController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const std::string & host_ip);
    ~DroneController(){};

    // control
    void cmdLoopCallBack(const ros::TimerEvent& event);
    void localPoseCallBack(const nav_msgs::Odometry& msg);
    void targetCallback(const geometry_msgs::TwistStamped& msg);

    // control functional
    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);

    // util 
    Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R);
    Eigen::Matrix3d acc2Rotmat(const Eigen::Vector3d &vector_acc, const double &yaw);
    Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q);
    Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
    Eigen::Vector4d geometricAttController(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);
    Eigen::Vector3d matrixHatInv(const Eigen::Matrix3d &m);
    void executeCmd();
    void takeoffUnitHeight(); // takeoff and hover, waiting for trajectory

    // state related
    template <class T>
    void waitForHome(const T* t, const std::string& msg, double hz = 2.0) {
        ros::Rate pause(hz);
        ROS_INFO_STREAM(msg);
        while (ros::ok() && !(*t))
        {
           ros::spinOnce();
           pause.sleep();
        }  
    };


    geometry_msgs::Pose home_position_;
    bool received_home_position_ = false;
   
};
