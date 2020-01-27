//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------


#include <ual_backend_dji_ros/ual_backend_dji_ros.h>
#include <Eigen/Eigen>
#include <ros/ros.h>


geometry_msgs::Pose::_orientation_type q;
// geometry_msgs::QuaternionStamped current_attitude;
double roll;
double pitch;
double yaw;

double altitude_offset;
double alt_1;
int alt_counter = 0;
float target_z;
bool going_up;


namespace grvc { namespace ual {

BackendDjiRos::BackendDjiRos()
    : Backend()
{
    // Parse arguments
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");
    float position_th_param, orientation_th_param;
    pnh.param<float>("position_th", position_th_param, 0.33);
    pnh.param<float>("orientation_th", orientation_th_param, 0.6);
    position_th_ = position_th_param*position_th_param;
    orientation_th_ = 0.5*(1 - cos(orientation_th_param));

    float vel_factor_param;
    pnh.param<float>("vel_factor", vel_factor_param, 0.7);
    vel_factor_max = vel_factor_param;
    
    pnh.param<bool>("laser_altimeter", laser_altimeter, false);
    pnh.param<bool>("self_arming", self_arming, false);

    pnh.param<float>("xy_vel_max", mpc_xy_vel_max, 2.3);
    // pnh.param<float>("z_vel_max", mpc_z_vel_max, 1.0);
    pnh.param<float>("z_vel_max_up", mpc_z_vel_max_up, 2.0);
    pnh.param<float>("z_vel_max_dn", mpc_z_vel_max_dn, 2.0);
    pnh.param<float>("yawrate_max", mc_yawrate_max, 0.8);

    ROS_INFO("BackendDjiRos constructor with id %d",robot_id_);
    // ROS_INFO("BackendDjiRos: thresholds = %f %f", position_th_, orientation_th_);

    // // Init ros communications
    ros::NodeHandle nh;
    std::string dji_ns = "dji_sdk";
        
    //ROS services
    std::string activation_srv = dji_ns + "/activation";
    std::string arming_srv = dji_ns + "/drone_arm_control";
    std::string set_local_pos_ref_srv = dji_ns + "/set_local_pos_ref";
    std::string sdk_control_authority_srv = dji_ns + "/sdk_control_authority";
    std::string drone_task_control_srv = dji_ns + "/drone_task_control";
    std::string mission_waypoint_upload_srv = dji_ns + "/mission_waypoint_upload";
    std::string mission_waypoint_setSpeed_srv = dji_ns + "/mission_waypoint_setSpeed";
    std::string mission_waypoint_action_srv = dji_ns + "/mission_waypoint_action";

    // ROS subscribed topics
    std::string get_position_topic = dji_ns + "/local_position";
    std::string get_linear_velocity_topic = dji_ns + "/velocity";
    std::string get_angular_velocity_topic = dji_ns + "/angular_velocity_fused";
    std::string get_position_global_topic = dji_ns + "/gps_position";
    std::string get_attitude_topic = dji_ns + "/attitude";
    std::string get_status_topic = dji_ns + "/flight_status";
    std::string get_mode_topic = dji_ns + "/display_mode";

    std::string get_laser_altitude_topic = "/laser_altitude";

    // ROS published topics
    std::string flight_control_topic = dji_ns + "/flight_control_setpoint_generic";

    // ROS services' Clients
    activation_client_ = nh.serviceClient<dji_sdk::Activation>(activation_srv.c_str());
    arming_client_ = nh.serviceClient<dji_sdk::DroneArmControl>(arming_srv.c_str());
    set_local_pos_ref_client_ = nh.serviceClient<dji_sdk::SetLocalPosRef>(set_local_pos_ref_srv.c_str());
    sdk_control_authority_client_ = nh.serviceClient<dji_sdk::SDKControlAuthority>(sdk_control_authority_srv.c_str());
    drone_task_control_client_ = nh.serviceClient<dji_sdk::DroneTaskControl>(drone_task_control_srv.c_str());
    mission_waypoint_upload_client = nh.serviceClient<dji_sdk::MissionWpUpload>(mission_waypoint_upload_srv.c_str());
    mission_waypoint_setSpeed_client = nh.serviceClient<dji_sdk::MissionWpSetSpeed>(mission_waypoint_setSpeed_srv.c_str());
    mission_waypoint_action_client = nh.serviceClient<dji_sdk::MissionWpAction>(mission_waypoint_action_srv.c_str());

    flight_control_pub_ = nh.advertise<sensor_msgs::Joy>(flight_control_topic.c_str(), 1);
    // mavros_ref_pose_global_pub_ = nh.advertise<mavros_msgs::GlobalPositionTarget>(set_pose_global_topic.c_str(), 1);
    

    //test publishers
    lookahead_pub = nh.advertise<std_msgs::Float64>("lookahead", 1);
    offset_y_pub = nh.advertise<std_msgs::Float64>("offset_y", 1);
    ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ref_pose_y", 1);
    ///

    flight_status_sub_ = nh.subscribe<std_msgs::UInt8>(get_status_topic.c_str(), 1, \
        [this](const std_msgs::UInt8::ConstPtr& _msg) {
            this->flight_status_ = *_msg;
    });

    display_mode_sub_ = nh.subscribe<std_msgs::UInt8>(get_mode_topic.c_str(), 1, \
        [this](const std_msgs::UInt8::ConstPtr _msg) {
            this->display_mode_ = *_msg;
    });

    position_sub_ = nh.subscribe<geometry_msgs::PointStamped>(get_position_topic.c_str(), 1, \
        [this](const geometry_msgs::PointStamped::ConstPtr& _msg) {
            this->current_position_ = *_msg;
            this->cur_pose_.pose.position = this->current_position_.point;
    });

    position_global_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(get_position_global_topic.c_str(), 1, \
        [this](const sensor_msgs::NavSatFix::ConstPtr& _msg) {
            this->current_position_global_ = *_msg;
    });
    
    attitude_sub_ = nh.subscribe<geometry_msgs::QuaternionStamped>(get_attitude_topic.c_str(), 1, \
        [this](const geometry_msgs::QuaternionStamped::ConstPtr& _msg) {
            this->current_attitude_ = *_msg;
            this->cur_pose_.pose.orientation = this->current_attitude_.quaternion;
    });

    laser_altitude_sub_ = nh.subscribe<std_msgs::Float64>(get_laser_altitude_topic.c_str(), 1, \
        [this](const std_msgs::Float64::ConstPtr& _msg) {
            this->current_laser_altitude_ = *_msg;
    });

    linear_velocity_sub_ = nh.subscribe<geometry_msgs::Vector3Stamped>(get_linear_velocity_topic.c_str(), 1, \
        [this](const geometry_msgs::Vector3Stamped::ConstPtr& _msg) {
            this->current_linear_velocity_ = *_msg;
            this->cur_vel_.twist.linear = this->current_linear_velocity_.vector;
    });

    angular_velocity_sub_ = nh.subscribe<geometry_msgs::Vector3Stamped>(get_angular_velocity_topic.c_str(), 1, \
        [this](const geometry_msgs::Vector3Stamped::ConstPtr& _msg) {
            this->current_angular_velocity_ = *_msg;
            this->cur_vel_.twist.angular = this->current_angular_velocity_.vector;
    });

    // // TODO: Check this and solve frames issue
    // // Wait until we have pose
    // while (!mavros_has_pose_ && ros::ok()) {
    //     // ROS_INFO("BackendDjiRos: Waiting for pose");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // }
    // initHomeFrame();

    control_thread_ = std::thread(&BackendDjiRos::controlThread, this);
        
    // Wait for dji_sdk_node is running
    ros::service::waitForService("dji_sdk/activation");
    dji_sdk::Activation activation;
    activated_ = activation_client_.call(activation);

    ROS_INFO("BackendDjiRos %d running!", robot_id_);
}

void BackendDjiRos::controlThread() {
    ros::param::param<double>("~dji_offboard_rate", control_thread_frequency_, 30.0);
    double hold_pose_time = 3.0;  // [s]  TODO param?
    int buffer_size = std::ceil(hold_pose_time * control_thread_frequency_);
    position_error_.set_size(buffer_size);
    orientation_error_.set_size(buffer_size);
    ros::Rate rate(control_thread_frequency_);

    //test
    std_msgs::Float64 offset_y_;

    while (ros::ok()) {
        sensor_msgs::Joy reference_joy;
        float control_flag;

        switch(control_mode_) {
        case eControlMode::IDLE:
            break;
        case eControlMode::LOCAL_VEL:

            control_flag = (DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::VERTICAL_VELOCITY       |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND |
                DJISDK::STABLE_ENABLE);

            reference_joy.axes.push_back(reference_vel_.twist.linear.x);
            reference_joy.axes.push_back(reference_vel_.twist.linear.y);
            reference_joy.axes.push_back(reference_vel_.twist.linear.z);
            reference_joy.axes.push_back(reference_vel_.twist.angular.z);
            reference_joy.axes.push_back(control_flag);
            flight_control_pub_.publish(reference_joy);

            // reference_pose_.pose.position = current_position_.point;
            if ( ros::Time::now().toSec() - last_command_time_.toSec() >=0.5 ) {
                control_mode_ = eControlMode::IDLE;
            }
            // mavros_ref_vel_pub_.publish(ref_vel_);
            // ref_pose_ = cur_pose_;
            break;

        case eControlMode::LOCAL_POSE: 
                        
            BackendDjiRos::Quaternion2EulerAngle(q, roll, pitch, yaw);
             
            control_flag = (DJISDK::HORIZONTAL_POSITION |
                DJISDK::VERTICAL_POSITION       |
                DJISDK::YAW_ANGLE            |
                DJISDK::HORIZONTAL_GROUND |
                DJISDK::STABLE_ENABLE);
            //flag = (0x80 | 0x10 | 0x00 | 0x02 | 0x01);

            offset_x = reference_pose_.pose.position.x - current_position_.point.x;
            offset_y = reference_pose_.pose.position.y - current_position_.point.y;
            offset_xy = sqrt(offset_x*offset_x + offset_y*offset_y);
            offset_x1 = offset_x / offset_xy * std::min(10.0, offset_xy);
            offset_y1 = offset_y / offset_xy * std::min(10.0, offset_xy);
            reference_joy.axes.push_back(vel_factor * offset_x1);
            reference_joy.axes.push_back(vel_factor * offset_y1);
            
            //test
            offset_y_.data = offset_y;
            offset_y_pub.publish(offset_y_);
            ref_pose_pub.publish(reference_pose_);
            ///

            // if (laser_altimeter == true) {
            //     // ROS_INFO("laser alt on: %s", laser_altimeter ? "true":"false");
            //     if ( current_laser_altitude_.data == 0.0 || altimeter_fail() ) {
            //         reference_joy.axes.push_back(reference_pose_.pose.position.z);
            //     }   
            //     else {
            //         altitude_offset = reference_pose_.pose.position.z - current_laser_altitude_.data;
            //         reference_joy.axes.push_back(current_position_.point.z + 2*altitude_offset);
            //     }
            // } else {
            //     reference_joy.axes.push_back(reference_pose_.pose.position.z);
            // }

            // if (laser_altimeter == true && current_laser_altitude_.data != 0.0 && !altimeter_fail()) {
            //     altitude_offset = reference_pose_.pose.position.z - current_laser_altitude_.data;
            //     reference_joy.axes.push_back(current_position_.point.z + 2*altitude_offset);
            // } else {
            //     reference_joy.axes.push_back(reference_pose_.pose.position.z);
            // }

            
            if (going_up) {
                if (fabs(cur_vel_.twist.linear.z) - mpc_z_vel_max_up < 0 && target_z < reference_pose_.pose.position.z) {
                    target_z += 0.04*mpc_z_vel_max_up;
                } else if (fabs(cur_vel_.twist.linear.z) - mpc_z_vel_max_up > 0.1 && target_z < reference_pose_.pose.position.z) {
                    // target_z -= 0.02*mpc_z_vel_max_up;
                }
            } else {
                if (fabs(cur_vel_.twist.linear.z) - mpc_z_vel_max_dn < 0 && target_z > reference_pose_.pose.position.z) {
                    target_z -= 0.04*mpc_z_vel_max_dn;
                } else if (fabs(cur_vel_.twist.linear.z) - mpc_z_vel_max_dn > 0.1 && target_z > reference_pose_.pose.position.z) {
                    // target_z += 0.02*mpc_z_vel_max_dn;
                }
            }
            // std::cout << "UAL reference_joy yaw:  " << yaw << std::endl;

            // reference_joy.axes.push_back(reference_pose_.pose.position.z);
            reference_joy.axes.push_back(target_z);
            reference_joy.axes.push_back(yaw);
            reference_joy.axes.push_back(control_flag);

            flight_control_pub_.publish(reference_joy);

            // std::cout << current_laser_altitude_ << std::endl;

            break; 
        case eControlMode::GLOBAL_POSE:

            control_flag = (DJISDK::HORIZONTAL_POSITION |
                DJISDK::VERTICAL_POSITION       |
                DJISDK::YAW_ANGLE            |
                DJISDK::HORIZONTAL_GROUND |
                DJISDK::STABLE_ENABLE);

            reference_joy.axes.push_back(100000*(reference_pose_global_.longitude - current_position_global_.longitude));
            reference_joy.axes.push_back(100000*(reference_pose_global_.latitude - current_position_global_.latitude));
            reference_joy.axes.push_back(reference_pose_global_.altitude);
            reference_joy.axes.push_back(yaw);
            reference_joy.axes.push_back(control_flag);

            flight_control_pub_.publish(reference_joy);

            break;
        }
        // // Error history update
        // double dx = ref_pose_.pose.position.x - cur_pose_.pose.position.x;
        // double dy = ref_pose_.pose.position.y - cur_pose_.pose.position.y;
        // double dz = ref_pose_.pose.position.z - cur_pose_.pose.position.z;
        // double positionD = dx*dx + dy*dy + dz*dz; // Equals distance^2

        // double quatInnerProduct = ref_pose_.pose.orientation.x*cur_pose_.pose.orientation.x + \
        // ref_pose_.pose.orientation.y*cur_pose_.pose.orientation.y + \
        // ref_pose_.pose.orientation.z*cur_pose_.pose.orientation.z + \
        // ref_pose_.pose.orientation.w*cur_pose_.pose.orientation.w;
        // double orientationD = 1.0 - quatInnerProduct*quatInnerProduct;  // Equals (1-cos(rotation))/2

        // position_error_.update(positionD);
        // orientation_error_.update(orientationD);

        // State update
        this->state_ = guessState();

        rate.sleep();
    }
}

grvc::ual::State BackendDjiRos::guessState() {
    // Sequentially checks allow state deduction
    if (!this->isReady()) { return uav_abstraction_layer::State::UNINITIALIZED; }
    if (this->flight_status_.data == DJISDK::FlightStatus::STATUS_STOPPED) { 
        if (self_arming) {
            return uav_abstraction_layer::State::LANDED_ARMED;
        } else {
            return uav_abstraction_layer::State::LANDED_DISARMED; 
        }
    }
    if (this->flight_status_.data == DJISDK::FlightStatus::STATUS_ON_GROUND) { return uav_abstraction_layer::State::LANDED_ARMED; }
    if (this->calling_takeoff && this->flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR ) 
        { return uav_abstraction_layer::State::TAKING_OFF; }
    if (this->calling_land && this->flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR ) 
        { return uav_abstraction_layer::State::LANDING; }
    if (!this->calling_takeoff && !this->calling_land 
        && this->flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR 
        // && this->display_mode_.data == 6) 
        && this->display_mode_.data == DJISDK::DisplayMode::MODE_NAVI_SDK_CTRL) 
        // && this->display_mode_.data == DJISDK::DisplayMode::MODE_P_GPS) 
        { return uav_abstraction_layer::State::FLYING_AUTO; }

    return uav_abstraction_layer::State::FLYING_MANUAL;
}

bool BackendDjiRos::altimeter_fail() {
    double alt_2 = current_laser_altitude_.data;
    if(alt_1 == alt_2 && alt_1 != 0.0){
        alt_counter ++;
    }
    else {
        alt_counter = 0;
    }
    // std::cout << "alt_counter  "<< alt_counter << std::endl;
    alt_1 = alt_2;
    if (alt_counter > 100) return true;
    else return false;
}

void BackendDjiRos::Quaternion2EulerAngle(const geometry_msgs::Pose::_orientation_type& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	yaw = atan2(siny, cosy);
}
void BackendDjiRos::setArmed(bool _value) {
    int arm;
    if(_value) {arm=1;}
    else if(!_value) {arm=0;}
    dji_sdk::DroneArmControl arming_service;
    arming_service.request.arm = _value;
    arming_client_.call(arming_service);

//     mavros_msgs::CommandBool arming_service;
//     arming_service.request.value = _value;
//     // Arm: unabortable?
//     while (ros::ok()) {
//         if (!arming_client_.call(arming_service)) {
//             ROS_ERROR("Error in arming service calling!");
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(300));
//         ROS_INFO("Arming service response.success = %s", arming_service.response.success ? "true" : "false");
//         ROS_INFO("Trying to set armed to %s... mavros_state_.armed = %s", _value ? "true" : "false", mavros_state_.armed ? "true" : "false");
//         bool armed = mavros_state_.armed;  // WATCHOUT: bug-prone ros-bool/bool comparison 
//         if (armed == _value) { break; }  // Out-of-while condition
//     }
}

// void BackendDjiRos::setFlightMode(const std::string& _flight_mode) {
//     mavros_msgs::SetMode flight_mode_service;
//     flight_mode_service.request.base_mode = 0;
//     flight_mode_service.request.custom_mode = _flight_mode;
//     // Set mode: unabortable?
//     while (mavros_state_.mode != _flight_mode && ros::ok()) {
//         if (!flight_mode_client_.call(flight_mode_service)) {
//             ROS_ERROR("Error in set flight mode [%s] service calling!", _flight_mode.c_str());
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(300));
// #ifdef MAVROS_VERSION_BELOW_0_20_0
//         ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
//             flight_mode_service.response.success ? "true" : "false");
// #else
//         ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
//             flight_mode_service.response.mode_sent ? "true" : "false");
// #endif
//         ROS_INFO("Trying to set [%s] mode; mavros_state_.mode = [%s]", _flight_mode.c_str(), mavros_state_.mode.c_str());
//     }
// }

void BackendDjiRos::recoverFromManual() {
    if (display_mode_.data != DJISDK::DisplayMode::MODE_P_GPS) {
        ROS_ERROR("Unable to recover from manual. Not in P_GPS MODE");
        ROS_INFO("Please switch rc to P_GPS MODE");
        return;
    }
    dji_sdk::SDKControlAuthority sdk_control_authority;
    sdk_control_authority.request.control_enable = dji_sdk::SDKControlAuthority::Request::REQUEST_CONTROL;
    sdk_control_authority_client_.call(sdk_control_authority);

    reference_vel_.twist.linear.x = 0;
    reference_vel_.twist.linear.y = 0;
    reference_vel_.twist.linear.z = 0;
    reference_vel_.twist.angular.z = 0;

    control_mode_ = eControlMode::LOCAL_VEL;
    ros::Duration(0.5).sleep();

    control_mode_ = eControlMode::IDLE;  
    
    if(sdk_control_authority.response.result){
        ROS_INFO("Recovered from manual mode!");
    } else {
        ROS_WARN("Unable to recover from manual mode (not in manual!)");
    }
    this->state_ = guessState();
}

void BackendDjiRos::setHome(bool set_z) {

    dji_sdk::SetLocalPosRef set_local_pos_ref;
    set_local_pos_ref_client_.call(set_local_pos_ref);
    home_set_ = true;
    // local_start_pos_ = -Eigen::Vector3d(cur_pose_.pose.position.x, \
    //     cur_pose_.pose.position.y, cur_pose_.pose.position.z);
}

void BackendDjiRos::takeOff(double _height) {
    if (_height < 0.0) {
        ROS_ERROR("Takeoff height must be positive!");
        return;
    }

    if(!home_set_){
        dji_sdk::SetLocalPosRef set_local_pos_ref;
        set_local_pos_ref_client_.call(set_local_pos_ref);
        home_set_ = true;
    }
 
    dji_sdk::SDKControlAuthority sdk_control_authority;
    sdk_control_authority.request.control_enable = dji_sdk::SDKControlAuthority::Request::REQUEST_CONTROL;
    sdk_control_authority_client_.call(sdk_control_authority);

    dji_sdk::DroneTaskControl drone_task_control;
    drone_task_control.request.task = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF;
    drone_task_control_client_.call(drone_task_control);
    ROS_INFO("Taking Off...");


    calling_takeoff = true;

    reference_pose_.pose.position.x = current_position_.point.x;
    reference_pose_.pose.position.y = current_position_.point.y;
    reference_pose_.pose.position.z = _height;
    q.x = current_attitude_.quaternion.x;
    q.y = current_attitude_.quaternion.y;
    q.z = current_attitude_.quaternion.z;
    q.w = current_attitude_.quaternion.w;

    while (current_position_.point.z < 1.0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    target_z = 2.0;
    // going_up = true;
    if (_height > 1.2) {going_up = true;}
    else {going_up = false;}
    control_mode_ = eControlMode::LOCAL_POSE;    // Control in position


    while ( !(fabs(_height - current_position_.point.z) < 1.5*position_th_) ) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // if (flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR) {
    // // if(flight_status_ == DJISDK::FlightStatus::STATUS_IN_AIR) {
       
    //     ROS_INFO("Flying!");
    // }
    
    ROS_INFO("Flying!");
    calling_takeoff = false;

    // Update state right now!
    this->state_ = guessState();

    control_mode_ = eControlMode::IDLE;    //Disable control in position

}

void BackendDjiRos::land() {
    calling_land = true;

    dji_sdk::DroneTaskControl drone_task_control;
    drone_task_control.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;
    drone_task_control_client_.call(drone_task_control);
   
    if(!drone_task_control.response.result) {
        ROS_ERROR("Land fail");
    }
    else if(drone_task_control.response.result) {
    ROS_INFO("Landing...");
    }

    while (flight_status_.data != DJISDK::FlightStatus::STATUS_STOPPED) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO("Landed!");
    


    control_mode_ = eControlMode::IDLE;  

    calling_land = false;

    // Update state right now!
    this->state_ = guessState();

    // control_mode_ = eControlMode::LOCAL_POSE;  // Back to control in position (just in case)
    // // Set land mode
    // setFlightMode("AUTO.LAND");
    // ROS_INFO("Landing...");
    // ref_pose_ = cur_pose_;
    // ref_pose_.pose.position.z = 0;
    // // Landing is unabortable!
    // while (ros::ok()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     if (mavros_extended_state_.landed_state == 
    //         mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) { break; }  // Out-of-while condition
    // }
    // setArmed(false);  // Now disarm!
    // ROS_INFO("Landed!");
}

// ros::ServiceServer go_home_service =
//                 nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
//                 go_home_srv,
//                 [this](std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
//                 return this->goHome();

void BackendDjiRos::goHome() {
    dji_sdk::DroneTaskControl drone_task_control;
    drone_task_control.request.task = dji_sdk::DroneTaskControl::Request::TASK_GOHOME;
    drone_task_control_client_.call(drone_task_control);

    control_mode_ = eControlMode::IDLE; 
}
void BackendDjiRos::setVelocity(const Velocity& _vel) {
    control_mode_ = eControlMode::LOCAL_VEL;  // Velocity control!
    reference_vel_ = _vel;

    last_command_time_ = ros::Time::now();
}

bool BackendDjiRos::isReady() const {    
        return activated_;
   }

void BackendDjiRos::setPose(const geometry_msgs::PoseStamped& _world) {
    
    control_mode_ = eControlMode::LOCAL_POSE;    // Control in position

    vel_factor = vel_factor_max;
    reference_pose_ = _world;
    q.x = reference_pose_.pose.orientation.x;
    q.y = reference_pose_.pose.orientation.y;
    q.z = reference_pose_.pose.orientation.z;
    q.w = reference_pose_.pose.orientation.w;
}

// TODO: Move from here?
struct PurePursuitOutput {
    geometry_msgs::Point next;
    float t_lookahead;
};

// TODO: Move from here?
PurePursuitOutput DjiPurePursuit(geometry_msgs::Point _current, geometry_msgs::Point _initial, geometry_msgs::Point _final, float _lookahead) {

    PurePursuitOutput out;
    out.next = _current;
    out.t_lookahead = 0;
    if (_lookahead <= 0) {
        ROS_ERROR("Lookahead must be non-zero positive!");
        return out;
    }

    Eigen::Vector3f x0 = Eigen::Vector3f(_current.x, _current.y, _current.z);
    Eigen::Vector3f x1 = Eigen::Vector3f(_initial.x, _initial.y, _initial.z);
    Eigen::Vector3f x2 = Eigen::Vector3f(_final.x, _final.y, _final.z);
    Eigen::Vector3f p = x0;

    Eigen::Vector3f x_21 = x2 - x1;
    float d_21 = x_21.norm();
    float t_min = - x_21.dot(x1-x0) / (d_21*d_21);

    Eigen::Vector3f closest_point = x1 + t_min*(x2-x1);
    float distance = (closest_point - x0).norm();

    float t_lookahead = t_min;
    if (_lookahead > distance) {
        float a = sqrt(_lookahead*_lookahead - distance*distance);
        t_lookahead = t_min + a/d_21;
    }

    if (t_lookahead <= 0.0) {
        p = x1;
        t_lookahead = 0.0;
        // ROS_INFO("p = x1");
    } else if (t_lookahead >= 1.0) {
        p = x2;
        t_lookahead = 1.0;
        // ROS_INFO("p = x2");
    } else {
        p = x1 + t_lookahead*(x2-x1);
        // ROS_INFO("L = %f; norm(x0-p) = %f", _lookahead, (x0-p).norm());
    }

    out.next.x = p(0);
    out.next.y = p(1);
    out.next.z = p(2);
    out.t_lookahead = t_lookahead;
    return out;
}

void BackendDjiRos::goToWaypoint(const Waypoint& _world) {
    control_mode_ = eControlMode::LOCAL_POSE;    // Control in position

    float current_xy_vel = sqrt(cur_vel_.twist.linear.x*cur_vel_.twist.linear.x + cur_vel_.twist.linear.y*cur_vel_.twist.linear.y);
    float current_z_vel = fabs(cur_vel_.twist.linear.z);
    // vel_factor = 0.001;
    vel_factor = std::min(current_xy_vel/6, vel_factor_max);

    reference_pose_ = _world;
    q.x = reference_pose_.pose.orientation.x;
    q.y = reference_pose_.pose.orientation.y;
    q.z = reference_pose_.pose.orientation.z;
    q.w = reference_pose_.pose.orientation.w;

    target_z = cur_pose_.pose.position.z;
    if (reference_pose_.pose.position.z > cur_pose_.pose.position.z) {going_up = true;}
    else {going_up = false;}

    geometry_msgs::PoseStamped homogen_world_pos;

    // No transform is needed
    homogen_world_pos = _world;

    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // std::string waypoint_frame_id = tf2::getFrameId(_world);

    // if ( waypoint_frame_id == "" || waypoint_frame_id == uav_home_frame_id_ ) {
    //     // No transform is needed
    //     homogen_world_pos = _world;
    // }
    // else {
    //     // We need to transform
    //     geometry_msgs::TransformStamped transformToHomeFrame;

    //     if ( cached_transforms_.find(waypoint_frame_id) == cached_transforms_.end() ) {
    //         // waypoint_frame_id not found in cached_transforms_
    //         transformToHomeFrame = tfBuffer.lookupTransform(uav_home_frame_id_, waypoint_frame_id, ros::Time(0), ros::Duration(1.0));
    //         cached_transforms_[waypoint_frame_id] = transformToHomeFrame; // Save transform in cache
    //     } else {
    //         // found in cache
    //         transformToHomeFrame = cached_transforms_[waypoint_frame_id];
    //     }
        
    //     tf2::doTransform(_world, homogen_world_pos, transformToHomeFrame);
        
    // }

//    std::cout << "Going to waypoint: " << homogen_world_pos.pose.position << std::endl;

    // // Do we still need local_start_pos_?
    // homogen_world_pos.pose.position.x -= local_start_pos_[0];
    // homogen_world_pos.pose.position.y -= local_start_pos_[1];
    // homogen_world_pos.pose.position.z -= local_start_pos_[2];

    // Smooth pose reference passing!
    geometry_msgs::Point final_position = homogen_world_pos.pose.position;
    geometry_msgs::Point initial_position = cur_pose_.pose.position;
    double ab_x = final_position.x - initial_position.x;
    double ab_y = final_position.y - initial_position.y;
    double ab_z = final_position.z - initial_position.z;

    Eigen::Quaterniond final_orientation = Eigen::Quaterniond(homogen_world_pos.pose.orientation.w, 
        homogen_world_pos.pose.orientation.x, homogen_world_pos.pose.orientation.y, homogen_world_pos.pose.orientation.z);
    Eigen::Quaterniond initial_orientation = Eigen::Quaterniond(cur_pose_.pose.orientation.w, 
        cur_pose_.pose.orientation.x, cur_pose_.pose.orientation.y, cur_pose_.pose.orientation.z);

    float linear_distance  = sqrt(ab_x*ab_x + ab_y*ab_y + ab_z*ab_z);
    float linear_threshold = sqrt(position_th_);
    if (linear_distance > linear_threshold) {
        // float mpc_xy_vel_max   = updateParam("MPC_XY_VEL_MAX");
        // float mpc_z_vel_max_up = updateParam("MPC_Z_VEL_MAX_UP");
        // float mpc_z_vel_max_dn = updateParam("MPC_Z_VEL_MAX_DN");
        // float mc_yawrate_max   = updateParam("MC_YAWRATE_MAX");

        float mpc_z_vel_max = (ab_z > 0)? mpc_z_vel_max_up : mpc_z_vel_max_dn;
        float xy_distance = sqrt(ab_x*ab_x + ab_y*ab_y);
        float z_distance = fabs(ab_z);
        bool z_vel_is_limit = (mpc_z_vel_max*xy_distance < mpc_xy_vel_max*z_distance);

        ros::Rate rate(30);  // [Hz]
        float next_to_final_distance = linear_distance;
        float lookahead = 0.05;
        float error_xy_vel;
        
        //test
        std_msgs::Float64 msg_lookahead;
        ///

        
        while (next_to_final_distance > linear_threshold && !abort_ && ros::ok()) {
            current_xy_vel = sqrt(cur_vel_.twist.linear.x*cur_vel_.twist.linear.x + cur_vel_.twist.linear.y*cur_vel_.twist.linear.y);
            current_z_vel = fabs(cur_vel_.twist.linear.z);
            // if (z_vel_is_limit) {
            //     if (current_z_vel > 1.0*mpc_z_vel_max) { lookahead -= 0.05; }  // TODO: Other thesholds, other update politics?
            //     if (current_z_vel < 0.95*mpc_z_vel_max) { lookahead += 0.05; }  // TODO: Other thesholds, other update politics?
            //     // ROS_INFO("current_z_vel = %f", current_z_vel);
            // } else {
            //     // if (current_xy_vel > 1.0*mpc_xy_vel_max) { lookahead -= 0.05; }  // TODO: Other thesholds, other update politics?
            //     // if (current_xy_vel < 0.95*mpc_xy_vel_max) { lookahead += 0.05; }  // TODO: Other thesholds, other update politics?
            //     error_xy_vel = mpc_xy_vel_max - current_xy_vel;
            //     lookahead += 0.05*error_xy_vel;
            //     // ROS_INFO("current_xy_vel = %f", current_xy_vel);
            // }

            error_xy_vel = mpc_xy_vel_max - current_xy_vel;
            if (error_xy_vel > 0 && vel_factor < vel_factor_max) {
                vel_factor += 0.003;
                // vel_factor += 0.0025*error_xy_vel;
            } else if (error_xy_vel < 0.2 && vel_factor > 0.2) {
                // vel_factor -= 0.001;
            }
            // std::cout << "vel_factor: " << vel_factor << std::endl;


            // PurePursuitOutput pp = DjiPurePursuit(cur_pose_.pose.position, initial_position, final_position, lookahead);
            // Waypoint wp_i;
            // wp_i.pose.position.x = pp.next.x;
            // wp_i.pose.position.y = pp.next.y;
            // wp_i.pose.position.z = pp.next.z;
            // Eigen::Quaterniond q_i = initial_orientation.slerp(pp.t_lookahead, final_orientation);
            // wp_i.pose.orientation.w = q_i.w();
            // wp_i.pose.orientation.x = q_i.x();
            // wp_i.pose.orientation.y = q_i.y();
            // wp_i.pose.orientation.z = q_i.z();
            // reference_pose_.pose = wp_i.pose;
            // next_to_final_distance = (1.0 - pp.t_lookahead) * linear_distance;
            // // ROS_INFO("next_to_final_distance = %f", next_to_final_distance);

            // //test
            // msg_lookahead.data = lookahead;
            // lookahead_pub.publish(msg_lookahead);
            // ///

            rate.sleep();
        }
    }
    // ROS_INFO("All points sent!");

    // Finally set pose
    reference_pose_.pose = homogen_world_pos.pose;
    // position_error_.reset();
    // orientation_error_.reset();

    // Wait until we arrive: abortable
    while(!referencePoseReached() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Freeze in case it's been aborted
    if (abort_ && freeze_) {
        reference_pose_ = cur_pose_;
    }
    


}

void	BackendDjiRos::goToWaypointGeo(const WaypointGeo& _wp){
    
    dji_sdk::MissionWpAction mission_waypoint_action;
    dji_sdk::MissionWpUpload mission_waypoint_upload;
    
    dji_sdk::MissionWaypointTask mission_waypoint_task;
    dji_sdk::MissionWaypoint current_waypoint;
    dji_sdk::MissionWaypoint mission_waypoint;
    dji_sdk::MissionWaypointAction waypoint_action;

    // waypoint_action.action_repeat = 0;

    std::cout << "test2" << std::endl;

    // current_waypoint.latitude = current_position_global_.latitude;
    // current_waypoint.longitude = current_position_global_.longitude;
    // current_waypoint.altitude = current_position_global_.altitude;

    mission_waypoint.latitude = _wp.latitude;
    mission_waypoint.longitude = _wp.longitude;
    mission_waypoint.altitude = _wp.altitude;
    mission_waypoint.damping_distance = 0;
    mission_waypoint.target_yaw = 0;
    mission_waypoint.target_gimbal_pitch = 0;
    mission_waypoint.turn_mode = 0;
    mission_waypoint.has_action = 0;
    // mission_waypoint.action_time_limit = 100;
    // mission_waypoint.waypoint_action = waypoint_action;

    mission_waypoint_task.mission_waypoint.push_back(mission_waypoint);

    mission_waypoint_task.velocity_range = 2.0;
    mission_waypoint_task.idle_velocity = 5;
    mission_waypoint_task.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
    mission_waypoint_task.mission_exec_times = 1;
    mission_waypoint_task.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
    mission_waypoint_task.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
    mission_waypoint_task.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_AUTO;
    mission_waypoint_task.gimbal_pitch_mode =  dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
    std::cout << "test4" << std::endl;

    // mission_waypoint_task.mission_waypoint[0] = mission_waypoint[0];
    std::cout << "test5" << std::endl;

    mission_waypoint_upload.request.waypoint_task = mission_waypoint_task;
    std::cout << "test6" << std::endl;

    mission_waypoint_upload_client.call(mission_waypoint_upload);
   
    mission_waypoint_action.request.action = 0;
    mission_waypoint_action_client.call(mission_waypoint_action);
    
    
    // control_mode_ = eControlMode::GLOBAL_POSE; // Control in position
    
    // reference_pose_global_.latitude = _wp.latitude;
    // reference_pose_global_.longitude = _wp.longitude;
    // reference_pose_global_.altitude = _wp.altitude;
    

    // // Wait until we arrive: abortable
    // while(!referencePoseReached() && !abort_ && ros::ok()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    // // Freeze in case it's been aborted
    // if (abort_ && freeze_) {
    //     ref_pose_ = cur_pose_;
    // }
}

/*void BackendDjiRos::trackPath(const WaypointList &_path) {
    // TODO: basic imlementation, ideally different from a stack of gotos
}*/

Pose BackendDjiRos::pose() {
        Pose out;

        out.pose.position.x = current_position_.point.x;
        out.pose.position.y = current_position_.point.y;
        if (laser_altimeter == true && current_laser_altitude_.data != 0.0 && !altimeter_fail() ) {
            out.pose.position.z = current_laser_altitude_.data;
        } else {
            out.pose.position.z = current_position_.point.z;
        }
        out.pose.orientation.x = current_attitude_.quaternion.x;
        out.pose.orientation.y = current_attitude_.quaternion.y;
        out.pose.orientation.z = current_attitude_.quaternion.z;
        out.pose.orientation.w = current_attitude_.quaternion.w;

        // if (pose_frame_id_ == "") {
        //     // Default: local pose
        //     out.header.frame_id = uav_home_frame_id_;
        // }
        // else {
        //     // Publish pose in different frame
        //     Pose aux = out;
        //     geometry_msgs::TransformStamped transformToPoseFrame;
        //     std::string pose_frame_id_map = "inv_" + pose_frame_id_;

        //     if ( cached_transforms_.find(pose_frame_id_map) == cached_transforms_.end() ) {
        //         // inv_pose_frame_id_ not found in cached_transforms_
        //         tf2_ros::Buffer tfBuffer;
        //         tf2_ros::TransformListener tfListener(tfBuffer);
        //         transformToPoseFrame = tfBuffer.lookupTransform(pose_frame_id_,uav_home_frame_id_, ros::Time(0), ros::Duration(1.0));
        //         cached_transforms_[pose_frame_id_map] = transformToPoseFrame; // Save transform in cache
        //     } else {
        //         // found in cache
        //         transformToPoseFrame = cached_transforms_[pose_frame_id_map];
        //     }

        //     tf2::doTransform(aux, out, transformToPoseFrame);
        //     out.header.frame_id = pose_frame_id_;
        // }

        return out;
}

Pose BackendDjiRos::referencePose() {
    return reference_pose_;
}

Velocity BackendDjiRos::velocity() const {
    Velocity out;

    out.twist.linear = current_linear_velocity_.vector;
    out.twist.angular = current_angular_velocity_.vector;

    return out;
}

Odometry BackendDjiRos::odometry() const {}

Transform BackendDjiRos::transform() const {
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "map";
    out.child_frame_id = "uav_" + std::to_string(robot_id_);

    out.transform.translation.x = current_position_.point.x;
    out.transform.translation.y = current_position_.point.y;
    out.transform.translation.z = current_position_.point.z;
    out.transform.rotation = current_attitude_.quaternion;        

    return out;
}

bool BackendDjiRos::referencePoseReached() {

    double position_min, position_mean, position_max;
    double orientation_min, orientation_mean, orientation_max;
    if (!position_error_.get_stats(position_min, position_mean, position_max)) { return false; }
    if (!orientation_error_.get_stats(orientation_min, orientation_mean, orientation_max)) { return false; }
    
    double position_diff = position_max - position_min;
    double orientation_diff = orientation_max - orientation_min;
    bool position_holds = (position_diff < position_th_) && (fabs(position_mean) < 0.5*position_th_);
    bool orientation_holds = (orientation_diff < orientation_th_) && (fabs(orientation_mean) < 0.5*orientation_th_);

    // if (position_holds && orientation_holds) {  // DEBUG
    //     ROS_INFO("position: %f < %f) && (%f < %f)", position_diff, position_th_, fabs(position_mean), 0.5*position_th_);
    //     ROS_INFO("orientation: %f < %f) && (%f < %f)", orientation_diff, orientation_th_, fabs(orientation_mean), 0.5*orientation_th_);
    //     ROS_INFO("Arrived!");
    // }

    return position_holds && orientation_holds;
}

// void BackendDjiRos::initHomeFrame() {

//     uav_home_frame_id_ = "uav_" + std::to_string(robot_id_) + "_home";
//     local_start_pos_ << 0.0, 0.0, 0.0;

//     // Get frame from rosparam
//     std::string parent_frame;
//     std::vector<double> home_pose(3, 0.0);

//     ros::param::get("~home_pose",home_pose);
//     ros::param::param<std::string>("~home_pose_parent_frame", parent_frame, "map");

//     geometry_msgs::TransformStamped static_transformStamped;

//     static_transformStamped.header.stamp = ros::Time::now();
//     static_transformStamped.header.frame_id = parent_frame;
//     static_transformStamped.child_frame_id = uav_home_frame_id_;
//     static_transformStamped.transform.translation.x = home_pose[0];
//     static_transformStamped.transform.translation.y = home_pose[1];
//     static_transformStamped.transform.translation.z = home_pose[2];

//     if(parent_frame == "map" || parent_frame == "") {
//         static_transformStamped.transform.rotation.x = 0;
//         static_transformStamped.transform.rotation.y = 0;
//         static_transformStamped.transform.rotation.z = 0;
//         static_transformStamped.transform.rotation.w = 1;
//     }
//     else {
//         tf2_ros::Buffer tfBuffer;
//         tf2_ros::TransformListener tfListener(tfBuffer);
//         geometry_msgs::TransformStamped transform_to_map;
//         transform_to_map = tfBuffer.lookupTransform(parent_frame, "map", ros::Time(0), ros::Duration(2.0));
//         static_transformStamped.transform.rotation = transform_to_map.transform.rotation;
//     }

//     static_tf_broadcaster_ = new tf2_ros::StaticTransformBroadcaster();
//     static_tf_broadcaster_->sendTransform(static_transformStamped);
// }

}}	// namespace grvc::ual
