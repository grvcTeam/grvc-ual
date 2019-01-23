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

#include <string>
#include <chrono>
#include <uav_abstraction_layer/backend_mavros.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <uav_abstraction_layer/geographic_to_cartesian.h>
#include <mavros_msgs/ParamGet.h>

namespace grvc { namespace ual {

BackendMavros::BackendMavros()
    : Backend()
{
    // Parse arguments
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");
    float position_th_param, orientation_th_param;
    pnh.param<float>("position_th", position_th_param, 0.33);
    pnh.param<float>("orientation_th", orientation_th_param, 0.65);
    position_th_ = position_th_param*position_th_param;
    orientation_th_ = 0.5*(1 - cos(orientation_th_param));

    ROS_INFO("BackendMavros constructor with id %d",robot_id_);
    // ROS_INFO("BackendMavros: thresholds = %f %f", position_th_, orientation_th_);

    // Init ros communications
    ros::NodeHandle nh;
    std::string mavros_ns = "mavros";
    std::string set_mode_srv = mavros_ns + "/set_mode";
    std::string arming_srv = mavros_ns + "/cmd/arming";
    std::string get_param_srv = mavros_ns + "/param/get";
    std::string set_pose_topic = mavros_ns + "/setpoint_position/local";
    std::string set_pose_global_topic = mavros_ns + "/setpoint_raw/global";
    std::string set_vel_topic = mavros_ns + "/setpoint_velocity/cmd_vel";
    std::string pose_topic = mavros_ns + "/local_position/pose";
    std::string geo_pose_topic = mavros_ns + "/global_position/global";
    std::string vel_topic = mavros_ns + "/local_position/velocity";
    std::string state_topic = mavros_ns + "/state";
    std::string extended_state_topic = mavros_ns + "/extended_state";

    flight_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(set_mode_srv.c_str());
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(arming_srv.c_str());

    mavros_ref_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(set_pose_topic.c_str(), 1);
    mavros_ref_pose_global_pub_ = nh.advertise<mavros_msgs::GlobalPositionTarget>(set_pose_global_topic.c_str(), 1);
    mavros_ref_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>(set_vel_topic.c_str(), 1);

    mavros_cur_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic.c_str(), 1, \
        [this](const geometry_msgs::PoseStamped::ConstPtr& _msg) {
            this->cur_pose_ = *_msg;
            this->mavros_has_pose_ = true;
    });
    mavros_cur_vel_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(vel_topic.c_str(), 1, \
        [this](const geometry_msgs::TwistStamped::ConstPtr& _msg) {
            this->cur_vel_ = *_msg;
            this->cur_vel_.header.frame_id = this->uav_home_frame_id_;
    });
    mavros_cur_geo_pose_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(geo_pose_topic.c_str(), 1, \
        [this](const sensor_msgs::NavSatFix::ConstPtr& _msg) {
            this->cur_geo_pose_ = *_msg;
            if (!this->mavros_has_geo_pose_) {
                if (_msg->position_covariance[0] < 1.2 && _msg->position_covariance[0] > 0 && _msg->header.seq > 100) {
                    this->mavros_has_geo_pose_ = true;
                    // ROS_INFO("Has Geo Pose! %f",_msg->position_covariance[0]);
                }
            }
    });
    mavros_cur_state_sub_ = nh.subscribe<mavros_msgs::State>(state_topic.c_str(), 1, \
        [this](const mavros_msgs::State::ConstPtr& _msg) {
            this->mavros_state_ = *_msg;
    });
    mavros_cur_extended_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>(extended_state_topic.c_str(), 1, \
        [this](const mavros_msgs::ExtendedState::ConstPtr& _msg) {
            this->mavros_extended_state_ = *_msg;
    });

    // Wait until mavros is connected
    while (!mavros_state_.connected && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // TODO: Check this and solve frames issue
    initHomeFrame();

    // Thread publishing target pose at 10Hz for offboard mode
    offboard_thread_ = std::thread(&BackendMavros::offboardThreadLoop, this);

    // Client to get parameters from mavros and required default values
    get_param_client_ = nh.serviceClient<mavros_msgs::ParamGet>(get_param_srv.c_str());
    mavros_params_["MPC_XY_VEL_MAX"]   =   2.0;  // [m/s]   Default value
    mavros_params_["MPC_Z_VEL_MAX_UP"] =   3.0;  // [m/s]   Default value
    mavros_params_["MPC_Z_VEL_MAX_DN"] =   1.0;  // [m/s]   Default value
    mavros_params_["MC_YAWRATE_MAX"]   = 200.0;  // [deg/s] Default value
    mavros_params_["MPC_TKO_SPEED"]    =   1.5;  // [m/s]   Default value
    // Updating here is non-sense as service seems to be slow in waking up

    ROS_INFO("BackendMavros %d running!",robot_id_);
}

BackendMavros::~BackendMavros() {
    if (offboard_thread_.joinable()) { offboard_thread_.join(); }
}

void BackendMavros::offboardThreadLoop(){
    ros::param::param<double>("~mavros_offboard_rate", offboard_thread_frequency_, 30.0);
    double hold_pose_time = 3.0;  // [s]  TODO param?
    int buffer_size = std::ceil(hold_pose_time * offboard_thread_frequency_);
    position_error_.set_size(buffer_size);
    orientation_error_.set_size(buffer_size);
    ros::Rate rate(offboard_thread_frequency_);
    while (ros::ok()) {
        switch(control_mode_){
        case eControlMode::LOCAL_VEL:
            mavros_ref_vel_pub_.publish(ref_vel_);
            ref_pose_ = cur_pose_;
            if ( ros::Time::now().toSec() - last_command_time_.toSec() >=0.5 ) {
                control_mode_ = eControlMode::LOCAL_POSE;
            }
            break;
        case eControlMode::LOCAL_POSE:
            ref_pose_.header.stamp = ros::Time::now();
            mavros_ref_pose_pub_.publish(ref_pose_);
            ref_vel_.twist.linear.x = 0;
            ref_vel_.twist.linear.y = 0;
            ref_vel_.twist.linear.z = 0;
            ref_vel_.twist.angular.z = 0;
            break;
        case eControlMode::GLOBAL_POSE:
            ref_vel_.twist.linear.x = 0;
            ref_vel_.twist.linear.y = 0;
            ref_vel_.twist.linear.z = 0;
            ref_vel_.twist.angular.z = 0;
            ref_pose_ = cur_pose_;

            mavros_msgs::GlobalPositionTarget msg;    
            msg.latitude = ref_pose_global_.latitude;
            msg.longitude = ref_pose_global_.longitude;
            msg.altitude = ref_pose_global_.altitude;
            msg.header.stamp = ros::Time::now();
            msg.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
            msg.type_mask = 4088; //((4095^1)^2)^4;

            mavros_ref_pose_global_pub_.publish(msg);
            break;
        }
        // Error history update
        double dx = ref_pose_.pose.position.x - cur_pose_.pose.position.x;
        double dy = ref_pose_.pose.position.y - cur_pose_.pose.position.y;
        double dz = ref_pose_.pose.position.z - cur_pose_.pose.position.z;
        double positionD = dx*dx + dy*dy + dz*dz; // Equals distance^2

        double quatInnerProduct = ref_pose_.pose.orientation.x*cur_pose_.pose.orientation.x + \
        ref_pose_.pose.orientation.y*cur_pose_.pose.orientation.y + \
        ref_pose_.pose.orientation.z*cur_pose_.pose.orientation.z + \
        ref_pose_.pose.orientation.w*cur_pose_.pose.orientation.w;
        double orientationD = 1.0 - quatInnerProduct*quatInnerProduct;  // Equals (1-cos(rotation))/2

        position_error_.update(positionD);
        orientation_error_.update(orientationD);

        // State update
        this->state_ = guessState();

        rate.sleep();
    }
}

Backend::State BackendMavros::guessState() {
    // Sequentially checks allow state deduction
    if (!this->isReady()) { return UNINITIALIZED; }
    if (!this->mavros_state_.armed) { return LANDED_DISARMED; }
    if (this->mavros_extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) { return LANDED_ARMED; }  // TODO(franreal): Use LANDED_STATE_IN_AIR instead?
    if (this->calling_takeoff) { return TAKING_OFF; }
    if (this->calling_land) { return LANDING; }
    if (this->mavros_state_.mode == "OFFBOARD") { return FLYING_AUTO; }
    return FLYING_MANUAL;
}

void BackendMavros::setFlightMode(const std::string& _flight_mode) {
    mavros_msgs::SetMode flight_mode_service;
    flight_mode_service.request.base_mode = 0;
    flight_mode_service.request.custom_mode = _flight_mode;
    // Set mode: unabortable?
    while (mavros_state_.mode != _flight_mode && ros::ok()) {
        if (!flight_mode_client_.call(flight_mode_service)) {
            ROS_ERROR("Error in set flight mode [%s] service calling!", _flight_mode.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
            flight_mode_service.response.success ? "true" : "false");
#else
        ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
            flight_mode_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("Trying to set [%s] mode; mavros_state_.mode = [%s]", _flight_mode.c_str(), mavros_state_.mode.c_str());
    }
}

void BackendMavros::recoverFromManual() {
    if (!mavros_state_.armed || mavros_extended_state_.landed_state != 
        mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR) {
        ROS_WARN("Unable to recover from manual mode (not flying!)");
        return;
    }

    if (mavros_state_.mode != "POSCTL" &&
        mavros_state_.mode != "ALTCTL" &&
        mavros_state_.mode != "STABILIZED") {
        ROS_WARN("Unable to recover from manual mode (not in manual!)");
        return;
    }

    // Set mode to OFFBOARD and state to FLYING
    ref_pose_ = cur_pose_;
    control_mode_ = eControlMode::LOCAL_POSE;
    setFlightMode("OFFBOARD");
    ROS_INFO("Recovered from manual mode!");
}

void BackendMavros::setHome(bool set_z) {
    double z_offset = set_z ? cur_pose_.pose.position.z : 0.0;
    local_start_pos_ = -Eigen::Vector3d(cur_pose_.pose.position.x, \
        cur_pose_.pose.position.y, z_offset);
}

void BackendMavros::takeOff(double _height) {
    if (_height < 0.0) {
        ROS_ERROR("Takeoff height must be positive!");
        return;
    }
    calling_takeoff = true;

    control_mode_ = eControlMode::LOCAL_POSE;  // Take off control is performed in position (not velocity)

    float acc_max = 1.0;  // TODO: From param?
    float vel_max = updateParam("MPC_TKO_SPEED");

    float a = sqrt(_height * acc_max);
    if (a < vel_max) {
        vel_max = a;
    }
    float t1 = vel_max / acc_max;
    float h1 = 0.5 * acc_max * t1 * t1;
    float t2 = t1 + (_height - 2.0 * h1) / vel_max;
    // float h2 = _height - h1;
    float t3 = t2 + t1;

    float t = 0.0;
    float delta_t = 0.1;  // [s]
    ros::Rate rate(1.0 / delta_t);

    ref_pose_ = cur_pose_;
    float base_z  = cur_pose_.pose.position.z;
    float delta_z = 0;

    setFlightMode("OFFBOARD");
    while ((t < t3) && ros::ok()) {  // Unabortable!
        if (t < t1) {
            delta_z = 0.5 * acc_max * t * t;
        } else if (t < t2) {
            delta_z = h1 + vel_max * (t - t1);
        } else {
            delta_z = _height - 0.5 * acc_max * (t3 - t) * (t3 - t);
        }

        if (delta_z > _height) {
            ROS_WARN("Unexpected delta_z value [%f]", delta_z);
        } else {
            ref_pose_.pose.position.z = base_z + delta_z;
        }

        rate.sleep();
        t += delta_t;
    }
    ref_pose_.pose.position.z = base_z + _height;

    // Now wait (unabortable!)
    while (!referencePoseReached() && (this->mavros_state_.mode == "OFFBOARD") && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO("Flying!");
    calling_takeoff = false;

    // Update state right now!
    this->state_ = guessState();
}

void BackendMavros::land() {
    calling_land = true;

    control_mode_ = eControlMode::LOCAL_POSE;  // Back to control in position (just in case)
    // Set land mode
    setFlightMode("AUTO.LAND");
    ROS_INFO("Landing...");
    ref_pose_ = cur_pose_;
    ref_pose_.pose.position.z = 0;
    // Landing is unabortable!
    while ((this->mavros_state_.mode == "AUTO.LAND") && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (mavros_extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
            // setArmed(false);  // Disabled for safety and symmetry
            ROS_INFO("Landed!");
            break;  // Out-of-while condition
        }  
    }
    calling_land = false;

    // Update state right now!
    this->state_ = guessState();
}

void BackendMavros::setVelocity(const Velocity& _vel) {
    control_mode_ = eControlMode::LOCAL_VEL;  // Velocity control!

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::Vector3Stamped vel_in, vel_out;
    vel_in.header = _vel.header;
    vel_in.vector = _vel.twist.linear;
    std::string vel_frame_id = tf2::getFrameId(vel_in);

    if (vel_frame_id == "map" || vel_frame_id == "" || vel_frame_id == uav_home_frame_id_) {
        // No transform is needed
        ref_vel_ = _vel;
    }
    else {
        // We need to transform
        geometry_msgs::TransformStamped transform;
        bool tf_exists = true;
        try {
            transform = tfBuffer.lookupTransform(uav_home_frame_id_, vel_frame_id, ros::Time(0), ros::Duration(0.3));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            tf_exists = false;
            ref_vel_ = _vel;
        }
        
        if(tf_exists) {
            tf2::doTransform(vel_in, vel_out, transform);
            ref_vel_.header = vel_out.header;
            ref_vel_.twist.linear = vel_out.vector;
            ref_vel_.twist.angular = _vel.twist.angular;
        }
    }
    last_command_time_ = ros::Time::now();
}

bool BackendMavros::isReady() const {
    if (ros::param::has("~map_origin_geo")) {
        return mavros_has_geo_pose_;
    } else {
        return mavros_has_pose_ && (fabs(this->cur_pose_.pose.position.y) > 1e-8);  // Means the filter has converged!
    }
}

void BackendMavros::setPose(const geometry_msgs::PoseStamped& _world) {
    control_mode_ = eControlMode::LOCAL_POSE;    // Control in position

    geometry_msgs::PoseStamped homogen_world_pos;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::string waypoint_frame_id = tf2::getFrameId(_world);

    if ( waypoint_frame_id == "" || waypoint_frame_id == uav_home_frame_id_ ) {
        // No transform is needed
        homogen_world_pos = _world;
    }
    else {
        // We need to transform
        geometry_msgs::TransformStamped transformToHomeFrame;

        if ( cached_transforms_.find(waypoint_frame_id) == cached_transforms_.end() ) {
            // waypoint_frame_id not found in cached_transforms_
            transformToHomeFrame = tfBuffer.lookupTransform(uav_home_frame_id_, waypoint_frame_id, ros::Time(0), ros::Duration(1.0));
            cached_transforms_[waypoint_frame_id] = transformToHomeFrame; // Save transform in cache
        } else {
            // found in cache
            transformToHomeFrame = cached_transforms_[waypoint_frame_id];
        }
        
        tf2::doTransform(_world, homogen_world_pos, transformToHomeFrame);
        
    }

//    std::cout << "Going to waypoint: " << homogen_world_pos.pose.position << std::endl;

    // Do we still need local_start_pos_?
    homogen_world_pos.pose.position.x -= local_start_pos_[0];
    homogen_world_pos.pose.position.y -= local_start_pos_[1];
    homogen_world_pos.pose.position.z -= local_start_pos_[2];

    ref_pose_.pose = homogen_world_pos.pose;
}

// TODO: Move from here?
struct PurePursuitOutput {
    geometry_msgs::Point next;
    float t_lookahead;
};

// TODO: Move from here?
PurePursuitOutput PurePursuit(geometry_msgs::Point _current, geometry_msgs::Point _initial, geometry_msgs::Point _final, float _lookahead) {

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

void BackendMavros::goToWaypoint(const Waypoint& _world) {
    control_mode_ = eControlMode::LOCAL_POSE;    // Control in position

    geometry_msgs::PoseStamped homogen_world_pos;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::string waypoint_frame_id = tf2::getFrameId(_world);

    if ( waypoint_frame_id == "" || waypoint_frame_id == uav_home_frame_id_ ) {
        // No transform is needed
        homogen_world_pos = _world;
    }
    else {
        // We need to transform
        geometry_msgs::TransformStamped transformToHomeFrame;

        if ( cached_transforms_.find(waypoint_frame_id) == cached_transforms_.end() ) {
            // waypoint_frame_id not found in cached_transforms_
            transformToHomeFrame = tfBuffer.lookupTransform(uav_home_frame_id_, waypoint_frame_id, ros::Time(0), ros::Duration(1.0));
            cached_transforms_[waypoint_frame_id] = transformToHomeFrame; // Save transform in cache
        } else {
            // found in cache
            transformToHomeFrame = cached_transforms_[waypoint_frame_id];
        }
        
        tf2::doTransform(_world, homogen_world_pos, transformToHomeFrame);
        
    }

//    std::cout << "Going to waypoint: " << homogen_world_pos.pose.position << std::endl;

    // Do we still need local_start_pos_?
    homogen_world_pos.pose.position.x -= local_start_pos_[0];
    homogen_world_pos.pose.position.y -= local_start_pos_[1];
    homogen_world_pos.pose.position.z -= local_start_pos_[2];

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
        float mpc_xy_vel_max   = updateParam("MPC_XY_VEL_MAX");
        float mpc_z_vel_max_up = updateParam("MPC_Z_VEL_MAX_UP");
        float mpc_z_vel_max_dn = updateParam("MPC_Z_VEL_MAX_DN");
        float mc_yawrate_max   = updateParam("MC_YAWRATE_MAX");

        float mpc_z_vel_max = (ab_z > 0)? mpc_z_vel_max_up : mpc_z_vel_max_dn;
        float xy_distance = sqrt(ab_x*ab_x + ab_y*ab_y);
        float z_distance = fabs(ab_z);
        bool z_vel_is_limit = (mpc_z_vel_max*xy_distance < mpc_xy_vel_max*z_distance);

        ros::Rate rate(10);  // [Hz]
        float next_to_final_distance = linear_distance;
        float lookahead = 0.05;
        while (next_to_final_distance > linear_threshold && !abort_ && ros::ok()) {
            float current_xy_vel = sqrt(cur_vel_.twist.linear.x*cur_vel_.twist.linear.x + cur_vel_.twist.linear.y*cur_vel_.twist.linear.y);
            float current_z_vel = fabs(cur_vel_.twist.linear.z);
            if (z_vel_is_limit) {
                if (current_z_vel > 0.8*mpc_z_vel_max) { lookahead -= 0.05; }  // TODO: Other thesholds, other update politics?
                if (current_z_vel < 0.5*mpc_z_vel_max) { lookahead += 0.05; }  // TODO: Other thesholds, other update politics?
                // ROS_INFO("current_z_vel = %f", current_z_vel);
            } else {
                if (current_xy_vel > 0.8*mpc_xy_vel_max) { lookahead -= 0.05; }  // TODO: Other thesholds, other update politics?
                if (current_xy_vel < 0.5*mpc_xy_vel_max) { lookahead += 0.05; }  // TODO: Other thesholds, other update politics?
                // ROS_INFO("current_xy_vel = %f", current_xy_vel);
            }
            PurePursuitOutput pp = PurePursuit(cur_pose_.pose.position, initial_position, final_position, lookahead);
            Waypoint wp_i;
            wp_i.pose.position.x = pp.next.x;
            wp_i.pose.position.y = pp.next.y;
            wp_i.pose.position.z = pp.next.z;
            Eigen::Quaterniond q_i = initial_orientation.slerp(pp.t_lookahead, final_orientation);
            wp_i.pose.orientation.w = q_i.w();
            wp_i.pose.orientation.x = q_i.x();
            wp_i.pose.orientation.y = q_i.y();
            wp_i.pose.orientation.z = q_i.z();
            ref_pose_.pose = wp_i.pose;
            next_to_final_distance = (1.0 - pp.t_lookahead) * linear_distance;
            // ROS_INFO("next_to_final_distance = %f", next_to_final_distance);
            rate.sleep();
        }
    }
    // ROS_INFO("All points sent!");

    // Finally set pose
    ref_pose_.pose = homogen_world_pos.pose;
    position_error_.reset();
    orientation_error_.reset();

    // Wait until we arrive: abortable
    while(!referencePoseReached() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Freeze in case it's been aborted
    if (abort_ && freeze_) {
        ref_pose_ = cur_pose_;
    }
}

void BackendMavros::goToWaypointGeo(const WaypointGeo& _wp) {
    control_mode_ = eControlMode::GLOBAL_POSE; // Control in position
    
    ref_pose_global_.latitude = _wp.latitude;
    ref_pose_global_.longitude = _wp.longitude;
    ref_pose_global_.altitude = _wp.altitude;

    // Wait until we arrive: abortable
    while(!referencePoseReached() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Freeze in case it's been aborted
    if (abort_ && freeze_) {
        ref_pose_ = cur_pose_;
    }
}

/*void BackendMavros::trackPath(const WaypointList &_path) {
    // TODO: basic imlementation, ideally different from a stack of gotos
}*/

Pose BackendMavros::pose() {
        Pose out;

        out.pose.position.x = cur_pose_.pose.position.x + local_start_pos_[0];
        out.pose.position.y = cur_pose_.pose.position.y + local_start_pos_[1];
        out.pose.position.z = cur_pose_.pose.position.z + local_start_pos_[2];
        out.pose.orientation = cur_pose_.pose.orientation;

        if (pose_frame_id_ == "") {
            // Default: local pose
            out.header.frame_id = uav_home_frame_id_;
        }
        else {
            // Publish pose in different frame
            Pose aux = out;
            geometry_msgs::TransformStamped transformToPoseFrame;
            std::string pose_frame_id_map = "inv_" + pose_frame_id_;

            if ( cached_transforms_.find(pose_frame_id_map) == cached_transforms_.end() ) {
                // inv_pose_frame_id_ not found in cached_transforms_
                tf2_ros::Buffer tfBuffer;
                tf2_ros::TransformListener tfListener(tfBuffer);
                transformToPoseFrame = tfBuffer.lookupTransform(pose_frame_id_,uav_home_frame_id_, ros::Time(0), ros::Duration(1.0));
                cached_transforms_[pose_frame_id_map] = transformToPoseFrame; // Save transform in cache
            } else {
                // found in cache
                transformToPoseFrame = cached_transforms_[pose_frame_id_map];
            }

            tf2::doTransform(aux, out, transformToPoseFrame);
            out.header.frame_id = pose_frame_id_;
        }

        out.header.stamp = cur_pose_.header.stamp;
        return out;
}

Velocity BackendMavros::velocity() const {
    return cur_vel_;
}

Odometry BackendMavros::odometry() const {
    Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = uav_home_frame_id_;
    odom.child_frame_id = uav_frame_id_;
    odom.pose.pose.position.x = cur_pose_.pose.position.x + local_start_pos_[0];
    odom.pose.pose.position.y = cur_pose_.pose.position.y + local_start_pos_[1];
    odom.pose.pose.position.z = cur_pose_.pose.position.z + local_start_pos_[2];
    odom.pose.pose.orientation = cur_pose_.pose.orientation;
    odom.twist.twist = cur_vel_.twist;

    return odom;
}

Transform BackendMavros::transform() const {
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = uav_home_frame_id_;
    out.child_frame_id = uav_frame_id_;
    out.transform.translation.x = cur_pose_.pose.position.x + local_start_pos_[0];
    out.transform.translation.y = cur_pose_.pose.position.y + local_start_pos_[1];
    out.transform.translation.z = cur_pose_.pose.position.z + local_start_pos_[2];
    out.transform.rotation = cur_pose_.pose.orientation;
    return out;
}

bool BackendMavros::referencePoseReached() {

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

void BackendMavros::initHomeFrame() {

    local_start_pos_ << 0.0, 0.0, 0.0;

    // Get frames from rosparam
    ros::param::param<std::string>("~uav_frame",uav_frame_id_,"uav_" + std::to_string(robot_id_));
    ros::param::param<std::string>("~uav_home_frame",uav_home_frame_id_, "uav_" + std::to_string(robot_id_) + "_home");
    std::string parent_frame;
    ros::param::param<std::string>("~home_pose_parent_frame", parent_frame, "map");
    
    std::vector<double> home_pose(3, 0.0);
    if (ros::param::has("~home_pose")) {
        ros::param::get("~home_pose",home_pose);
    }
    else if (ros::param::has("~map_origin_geo")) {
        ROS_WARN("Be careful, you should only use this mode with RTK GPS!");
        while (!this->mavros_has_geo_pose_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::vector<double> map_origin_geo(3, 0.0);
        ros::param::get("~map_origin_geo",map_origin_geo);
        geographic_msgs::GeoPoint origin_geo, actual_coordinate_geo;
        origin_geo.latitude = map_origin_geo[0];
        origin_geo.longitude = map_origin_geo[1];
        origin_geo.altitude = 0; //map_origin_geo[2];
        actual_coordinate_geo.latitude = cur_geo_pose_.latitude;
        actual_coordinate_geo.longitude = cur_geo_pose_.longitude;
        actual_coordinate_geo.altitude = 0; //cur_geo_pose_.altitude;
        if(map_origin_geo[0]==0 && map_origin_geo[1]==0) {
            ROS_WARN("Map origin is set to 0. Define map_origin_geo param by a vector in format [lat,lon,alt].");
        }
        geometry_msgs::Point32 map_origin_cartesian = geographic_to_cartesian (actual_coordinate_geo, origin_geo);

        home_pose[0] = map_origin_cartesian.x;
        home_pose[1] = map_origin_cartesian.y;
        home_pose[2] = map_origin_cartesian.z;
    }
    else {
        ROS_WARN("No home pose or map origin was defined. Home frame will be equal to map.");
    }

    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = parent_frame;
    static_transformStamped.child_frame_id = uav_home_frame_id_;
    static_transformStamped.transform.translation.x = home_pose[0];
    static_transformStamped.transform.translation.y = home_pose[1];
    static_transformStamped.transform.translation.z = home_pose[2];

    if(parent_frame == "map" || parent_frame == "") {
        static_transformStamped.transform.rotation.x = 0;
        static_transformStamped.transform.rotation.y = 0;
        static_transformStamped.transform.rotation.z = 0;
        static_transformStamped.transform.rotation.w = 1;
    }
    else {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transform_to_map;
        transform_to_map = tfBuffer.lookupTransform(parent_frame, "map", ros::Time(0), ros::Duration(2.0));
        static_transformStamped.transform.rotation = transform_to_map.transform.rotation;
    }

    static_tf_broadcaster_ = new tf2_ros::StaticTransformBroadcaster();
    static_tf_broadcaster_->sendTransform(static_transformStamped);
}

double BackendMavros::updateParam(const std::string& _param_id) {
    mavros_msgs::ParamGet get_param_service;
    get_param_service.request.param_id = _param_id;
    if (get_param_client_.call(get_param_service) && get_param_service.response.success) {
        mavros_params_[_param_id] = get_param_service.response.value.integer? 
            get_param_service.response.value.integer : get_param_service.response.value.real;
        ROS_INFO("Parameter [%s] value is [%f]", get_param_service.request.param_id.c_str(), mavros_params_[_param_id]);
    } else if (mavros_params_.count(_param_id)) {
        ROS_ERROR("Error in get param [%s] service calling, leaving current value [%f]", 
            get_param_service.request.param_id.c_str(), mavros_params_[_param_id]);
    } else {
        mavros_params_[_param_id] = 0.0;
        ROS_ERROR("Error in get param [%s] service calling, initializing it to zero", 
            get_param_service.request.param_id.c_str());
    }
    return mavros_params_[_param_id];
}

}}	// namespace grvc::ual
