
#include "airsim_ros_client.h"


SensorWrapper::SensorWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const std::string & host_ip):
    nh_(nh), 
    nh_private_(nh_private),
    sensor_client_(host_ip),
    image_client_(host_ip),
    img_front_async_spinner_(1, &img_pub_queue_)
    {
        
        //
        try
        {
            sensor_client_.confirmConnection();
            image_client_.confirmConnection();
        }
        catch (rpc::rpc_error&  e) {
            std::string msg = e.get_error().as<std::string>();
            std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
        }
        
       
        initSubandPub();
        initTimer();
       
       
        
    }

void SensorWrapper::initSubandPub() {
    image_transport::ImageTransport image_transporter(nh_);  // nh_private?
    scene_img_publisher_ = image_transporter.advertise("/airsim/sensor/scene_image", 1);
    depth_img_publisher_ = image_transporter.advertise("/airsim/sensor/depth_image", 1);
    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("/airsim/sensor/imu", 1);
    height_publisher_ = nh_.advertise<sensor_msgs::FluidPressure>("/airsim/sensor/baroheight", 1);
    
}


void SensorWrapper::initTimer() {
    
    
    imu_loop_timer_ = nh_private_.createTimer(ros::Duration(0.1), &SensorWrapper::imuRequestTimerCB, this);
    height_loop_timer_ = nh_private_.createTimer(ros::Duration(0.1), &SensorWrapper::heightRequestTimerCB, this);
    ros::TimerOptions img_timer_options(ros::Duration(1), boost::bind(&SensorWrapper::imgRequestTimerCB, this, _1), &img_pub_queue_);
    img_loop_timer_ = nh_private_.createTimer(img_timer_options);
}

void SensorWrapper::pubTestImage() {
    cv::Mat test_img = cv::imread("/home/henry/Desktop/imgpubtest.png");
    sensor_msgs::ImagePtr img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", test_img).toImageMsg();
    scene_img_publisher_.publish(img_ptr);
}


void SensorWrapper::imuRequestTimerCB(const ros::TimerEvent& event) {
    auto imu_data = sensor_client_.getImuData();
    sensor_msgs::Imu imu_msg =  getImuMsgFromAirsim(imu_data);
    imu_publisher_.publish(imu_msg);
}

void SensorWrapper::heightRequestTimerCB(const ros::TimerEvent& event) {
    auto baro_data = sensor_client_.getBarometerData();
    pubHeightFromBaro(baro_data);
}

void SensorWrapper::imgRequestTimerCB(const ros::TimerEvent& event) {
    
    try
    {
        ROS_INFO("try request imgs");
        std::vector<ImageRequest> request = { ImageRequest("0", ImageType::Scene, false, false), ImageRequest("0", ImageType::DepthPlanner, true, false) };
        const vector<ImageResponse>& response =  image_client_.simGetImages(request);
        pubImgsFromResponse(response);
    }
    catch(const std::exception& e)
    {
         ROS_INFO("wrong get img");
    }
    
    ROS_INFO("try request imgs");
   
}

cv::Mat SensorWrapper::manual_decode_depth(const ImageResponse& img_response) const
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;
    std::ostringstream oss;
    oss << "width is " << img_width << " height is " << img_response.height;
    std::cout<< oss.str() << std::endl;
    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

// functional


ros::Time SensorWrapper::unrealTime2RosTime(uint64_t unreal_ts) {
    if (first_sensor_t_unreal < 0) {
        first_sensor_t_unreal = unreal_ts;
        first_sensor_t_ros = ros::Time::now();
    }
   //double t_ = (unreal_ts - first_sensor_t_unreal)/1e9;
    //return  first_sensor_t_ros + ros::Duration(t_);  // TODO CHECK 1e9?
    return ros::Time::now();
}
sensor_msgs::ImagePtr SensorWrapper::getDepthImgFromResponce(const ImageResponse& img_response,
                                                          const ros::Time curr_ros_time,
                                                          const std::string frame_id)
{
    cv::Mat depth_img = manual_decode_depth(img_response);
    sensor_msgs::ImagePtr depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp = curr_ros_time;
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}



sensor_msgs::ImagePtr SensorWrapper::getSceneImgFromResponce(const ImageResponse& img_response,
                                                          const ros::Time curr_ros_time, 
                                                          const std::string frame_id)
{
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; 
    img_msg_ptr->header.stamp = curr_ros_time; // TODO: Changed, check if right 
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

sensor_msgs::Imu SensorWrapper::getImuMsgFromAirsim(const msr::airlib::ImuBase::Output& imu_data) {
    sensor_msgs::Imu imu_msg;
    // imu_msg.header.frame_id = "/airsim/odom_local_ned";// todo multiple drones
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();

    // todo radians per second
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

    // meters/s2^m 
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();
    ROS_INFO_STREAM("ori time is " << imu_data.time_stamp);
    imu_msg.header.stamp = unrealTime2RosTime(imu_data.time_stamp);
    ROS_INFO_STREAM("after time is " << imu_msg.header.stamp);
    // imu_msg.orientation_covariance = ;
    // imu_msg.angular_velocity_covariance = ;
    // imu_msg.linear_acceleration_covariance = ;

    return imu_msg;
}

void SensorWrapper::pubHeightFromBaro(const msr::airlib::BarometerBase::Output& baro_data) {
    
    if (init_baro_flag)
    {
        init_baro_altitude = baro_data.altitude;
        init_baro_flag = false;
    }
    sensor_msgs::FluidPressure fp_msg;
    fp_msg.header.stamp = unrealTime2RosTime(baro_data.time_stamp);
    fp_msg.fluid_pressure = baro_data.altitude - init_baro_altitude;
    // fp_msg.variance = 0;  // TODO variance
    height_publisher_.publish(fp_msg);
}

// responce 0 -> scene  1->depth
void SensorWrapper::pubImgsFromResponse(const vector<ImageResponse>& response) {
    ros::Time curr_ros_time = ros::Time::now(); 
    std::cout <<" image received " << response.size() << std::endl;
    if (response.size() == 2)
    {    
        ImageResponse scene_img = response[0];
        ImageResponse depth_img = response[1];

        scene_img_publisher_.publish(getSceneImgFromResponce(scene_img, curr_ros_time, "scene_image"));
        depth_img_publisher_.publish(getDepthImgFromResponce(depth_img, curr_ros_time, "depth_image"));
    }
      
}

// ----------------------------------------------------------------------------------------//
// denote q as w x y z;

DroneController::DroneController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,  const std::string & host_ip) :
    nh_(nh),
    nh_private_(nh_private),
    control_client_(host_ip)
    {
        
        cmd_loop_timer = nh_.createTimer(ros::Duration(0.02), &DroneController::cmdLoopCallBack, this);

        local_pose_sub_ = nh_.subscribe("/airsim/drone_state/local_pose", 1, &DroneController::localPoseCallBack, this, ros::TransportHints().tcpNoDelay()); // TODO check why no delay
        
        reference_sub_ = nh_.subscribe("/airsim/traj_target/posevel", 1, &DroneController::targetCallback, this, ros::TransportHints().tcpNoDelay());
        // takeoff_unitheight_srv = nh_.advertiseService("takeoff_unitheight", &DroneController::takeoffUnitHeight, this);
        

        drone_state = WAITFORHOME;
        g_ << 0.0 , 0.0, 9.8;
        Kpos_ << -8.0, -8.0, -8.0;
      
        Kvel_ << -1.5, -1.5, -3.3;
        D_ << 0.0, 0.0, 0.0;

        attctrl_tau_ = 0.12;

        max_fb_acc_ = 8.0;
        norm_thrust_const_ = 0.05;
        norm_thrust_offset_ = 0.1;

        target_pos_ << 0.0, 0.0, 0.0;
        target_vel_ << 0.0, 0.0, 0.0;
        target_acc_ << 0.0, 0.0, 0.0;
        
        control_client_.enableApiControl(true);
        control_client_.armDisarm(true);
        
    }


void DroneController::takeoffUnitHeight() {
    control_client_.enableApiControl(true);
    control_client_.armDisarm(true);
    std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
    // control_client_.moveToPositionAsync(home_position_(0),home_position_(1), home_position_(2) - 1, 2)->waitOnLastTask();
    control_client_.hoverAsync();

}


void DroneController::localPoseCallBack(const nav_msgs::Odometry& msg) {
    if (!received_home_position_)
    {
        received_home_position_ = true;
        home_position_ = msg.pose.pose;
    }
    
    cur_pos_(0) = msg.pose.pose.position.x;
    cur_pos_(1) = msg.pose.pose.position.y;
    cur_pos_(2) = msg.pose.pose.position.z;
    cur_att_q_(0) = msg.pose.pose.orientation.w;
    cur_att_q_(1) = msg.pose.pose.orientation.x;
    cur_att_q_(2) = msg.pose.pose.orientation.y;
    cur_att_q_(3) = msg.pose.pose.orientation.z;

    cur_vel_(0) = msg.twist.twist.linear.x;
    cur_vel_(1) = msg.twist.twist.linear.y;
    cur_vel_(2) = msg.twist.twist.linear.z;
   

}

 void DroneController::targetCallback(const geometry_msgs::TwistStamped& msg) {
    reference_request_last_ = reference_request_now_;
    target_pos_prev_ = target_pos_;
    target_vel_prev_ = target_vel_;

    reference_request_now_ = ros::Time::now();
    reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

    target_pos_ = toEigen(msg.twist.angular);  // do not treat as vel or angular vel
    target_vel_ = toEigen(msg.twist.linear);

    if(reference_request_dt_ > 0) target_acc_ = (target_vel_ - target_vel_prev_ ) / reference_request_dt_;
    else target_acc_ = Eigen::Vector3d::Zero();
 }

void DroneController::cmdLoopCallBack(const ros::TimerEvent& event) {
    
    switch (drone_state)
    {
        case WAITFORHOME:
        {
            waitForHome(&received_home_position_, "waiting for home ...");
            drone_state = MISSION_EXEC;
            break;
        }
        case TAKEOFF:
        {
            ROS_INFO("Takeoff");
            break;
        }
        case HOVER:
        {
            ROS_INFO("hover state");
            break;
        }
            
        case MISSION_EXEC:
        {
            computeBodyRateCmd(cmd_body_rate_, target_pos_, target_vel_, target_acc_);
            
            executeCmd();
            break;
        }
        case LANDING:
        {
            ROS_INFO("landing");
            break;
        }

        default:
            break;
        }
        
}

// airsim interact 
void DroneController::executeCmd() {
    float roll_rate, pitch_rate, yaw_rate, throttle;
    roll_rate = cmd_body_rate_(0);
    pitch_rate = cmd_body_rate_(1);
    yaw_rate = cmd_body_rate_(2);
    throttle = cmd_body_rate_(3);
    ROS_INFO_STREAM("body rate is " << cmd_body_rate_(0) << "  " << cmd_body_rate_(1) << "   " << cmd_body_rate_(2));
    ROS_INFO_STREAM("throttle is  " << cmd_body_rate_(3));
    control_client_.moveByAngleRatesThrottleAsync(roll_rate, pitch_rate, yaw_rate, throttle, cmd_time_); // TODO if need to wait until finish
    ROS_INFO_STREAM("-------------------------------------------");
}

// control function 
void DroneController::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc) {
    const Eigen::Vector3d a_ref = target_acc;
    // check what does this work?
    if(velocity_yaw_) {
        mav_yaw_ = std::atan2(-1.0 * cur_vel_(1), cur_vel_(0));
    }
    const Eigen::Matrix3d R_ref = acc2Rotmat(g_ - a_ref, init_yaw_rad_); // to be changed
    ROS_INFO_STREAM("R_ref is ");
    ROS_INFO_STREAM(R_ref);
    ROS_INFO_STREAM("cur vel is "<< cur_vel_(0) << "  " << cur_vel_(1) << "  " << cur_vel_(2));
    ROS_INFO_STREAM("target vel is "<< target_vel(0) << "  " << target_vel(1) << "  " << target_vel(2));
    const Eigen::Vector3d pos_err = cur_pos_ - target_pos;
    const Eigen::Vector3d vel_err = cur_vel_ - target_vel;
    ROS_INFO_STREAM("pos err is "<< pos_err(0) << "  " << pos_err(1) << "  " << pos_err(2));
    ROS_INFO_STREAM("vel err is "<< vel_err(0) << "  " << vel_err(1) << "  " << vel_err(2));

    Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_err + Kvel_.asDiagonal() * vel_err;
    ROS_INFO_STREAM("a_fb is "<< a_fb(0) << "  " << a_fb(1) << "  " << a_fb(2));
    if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // why clip here? 

    const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel; // rotor drag  TODO CHECK on this
    // const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;
    const Eigen::Vector3d a_des = -(a_fb + a_ref - a_rd - g_);
    
    
    des_att_q_ = rot2Quaternion(acc2Rotmat(a_des, init_yaw_rad_));
    bodyrate_cmd = geometricAttController(des_att_q_, a_des, cur_att_q_);
    

}

// acc world NED   g_ 0 0 9.8
Eigen::Matrix3d DroneController::acc2Rotmat(const Eigen::Vector3d &vector_acc, const double &yaw) {
    Eigen::Vector4d des_quat;
    Eigen::Vector3d yaw_plane;
    yaw_plane(0) = std::cos(yaw);
    yaw_plane(1) = std::sin(yaw);
    yaw_plane(2) = 0.0;
    
    Eigen::Vector3d z_b_des = vector_acc / vector_acc.norm(); // checked, still local NED
    Eigen::Vector3d y_b_des = z_b_des.cross(yaw_plane) / (z_b_des.cross(yaw_plane)).norm();
    //Eigen::Vector3d x_b_des = yaw_plane.cross(vector_acc) / (yaw_plane.cross(vector_acc)).norm();
    //Eigen::Vector3d y_b_des = vector_acc.cross(x_b_des) / (vector_acc.cross(x_b_des)).norm();
    Eigen::Vector3d x_b_des = y_b_des.cross(z_b_des) / (y_b_des.cross(z_b_des)).norm();
    Eigen::Matrix3d rotmat;
    rotmat <<   x_b_des(0), y_b_des(0), z_b_des(0),
                x_b_des(1), y_b_des(1), z_b_des(1),
                x_b_des(2), y_b_des(2), z_b_des(2);
    
    // Eigen::Vector4d quat = rot2Quaternion(rotmat);
    return rotmat;
}


Eigen::Vector4d DroneController::rot2Quaternion(const Eigen::Matrix3d &R){
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Matrix3d DroneController::quat2RotMatrix(const Eigen::Vector4d &q){
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
    2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2),
    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2),
    2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}


Eigen::Vector4d DroneController::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
   Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
          p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
          p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
          p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}


Eigen::Vector4d DroneController::geometricAttController(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att){ // ref_acc -> des acc
  // Geometric attitude controller
  // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.  
  // The original paper inputs moment commands, but for offboard control angular rate commands are sent

  Eigen::Vector4d ratecmd;
  Eigen::Matrix3d rotmat; //Rotation matrix of current atttitude
  Eigen::Matrix3d rotmat_d; //Rotation matrix of desired attitude
  Eigen::Vector3d zb;
  Eigen::Vector3d error_att;

  rotmat = quat2RotMatrix(curr_att);
  rotmat_d = quat2RotMatrix(ref_att);

  error_att = 0.5 * matrixHatInv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat);
  ratecmd.head(3) = (2.0 / attctrl_tau_) * error_att;
  rotmat = quat2RotMatrix(cur_att_q_);
  zb = rotmat.col(2); // cur zb
  ROS_INFO_STREAM("geometric controll zb is ");
  ROS_INFO_STREAM(zb);
  ROS_INFO_STREAM(ref_acc);
  ratecmd(3) = std::max(0.0, std::min(1.0, (norm_thrust_const_ * ref_acc.dot(zb)) + norm_thrust_offset_)); //Calculate thrust // actually need to be - - 

  return ratecmd;
}

Eigen::Vector3d DroneController::matrixHatInv(const Eigen::Matrix3d &m) {
  Eigen::Vector3d v;
  //TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}
