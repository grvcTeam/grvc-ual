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

namespace grvc { namespace ual {

BackendMavros::BackendMavros()
    : Backend()
{
    // Parse arguments
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");
    std::string ns_prefix;
    pnh.param<std::string>("ns_prefix", ns_prefix, "uav_");
    float position_th_param, orientation_th_param;
    pnh.param<float>("position_th", position_th_param, 0.33);
    pnh.param<float>("orientation_th", orientation_th_param, 0.65);
    position_th_ = position_th_param*position_th_param;
    orientation_th_ = 0.5*(1 - cos(orientation_th_param));

    ROS_INFO("BackendMavros constructor with id %d",robot_id_);
    // ROS_INFO("BackendMavros: thresholds = %f %f", position_th_, orientation_th_);

    // Init ros communications
    ros::NodeHandle nh;
    std::string mavros_ns = ns_prefix + std::to_string(this->robot_id_) + "/mavros";
    std::string set_mode_srv = mavros_ns + "/set_mode";
    std::string arming_srv = mavros_ns + "/cmd/arming";
    std::string set_pose_topic = mavros_ns + "/setpoint_position/local";
    std::string set_pose_global_topic = mavros_ns + "/setpoint_raw/global";
    std::string set_vel_topic = mavros_ns + "/setpoint_velocity/cmd_vel";
    std::string pose_topic = mavros_ns + "/local_position/pose";
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
    mavros_cur_state_sub_ = nh.subscribe<mavros_msgs::State>(state_topic.c_str(), 1, \
        [this](const mavros_msgs::State::ConstPtr& _msg) {
            this->mavros_state_ = *_msg;
    });
    mavros_cur_extended_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>(extended_state_topic.c_str(), 1, \
        [this](const mavros_msgs::ExtendedState::ConstPtr& _msg) {
            this->mavros_extended_state_ = *_msg;
    });

    // TODO: Check this and solve frames issue
    // Wait until we have pose
    while (!mavros_has_pose_ && ros::ok()) {
        // ROS_INFO("BackendMavros: Waiting for pose");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    initHomeFrame();

    // Thread publishing target pose at 10Hz for offboard mode
    offboard_thread_ = std::thread(&BackendMavros::offboardThreadLoop, this);

    ROS_INFO("BackendMavros %d running!",robot_id_);
}

void BackendMavros::offboardThreadLoop(){
    // TODO: Check this frequency
    offboard_thread_frequency_ = 10;  // [Hz]
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
            break;
        case eControlMode::LOCAL_POSE:
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

        rate.sleep();
    }
}

void BackendMavros::setArmed(bool _value) {
    mavros_msgs::CommandBool arming_service;
    arming_service.request.value = _value;
    // Arm: unabortable?
    while (ros::ok()) {
        if (!arming_client_.call(arming_service)) {
            ROS_ERROR("Error in arming service calling!");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        ROS_INFO("Arming service response.success = %s", arming_service.response.success ? "true" : "false");
        ROS_INFO("Trying to set armed to %s... mavros_state_.armed = %s", _value ? "true" : "false", mavros_state_.armed ? "true" : "false");
        bool armed = mavros_state_.armed;  // WATCHOUT: bug-prone ros-bool/bool comparison 
        if (armed == _value) { break; }  // Out-of-while condition
    }
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
    if (mavros_state_.mode == "POSCTL" ||
        mavros_state_.mode == "ALTCTL" ||
        mavros_state_.mode == "STABILIZED") {
        control_mode_ = eControlMode::LOCAL_POSE;
        ref_pose_ = cur_pose_;
        setFlightMode("OFFBOARD");
        ROS_INFO("Recovered from manual mode!");
    } else {
        ROS_WARN("Unable to recover from manual mode (not in manual!)");
    }
}

void BackendMavros::setHome() {
    local_start_pos_ = -Eigen::Vector3d(cur_pose_.pose.position.x, \
        cur_pose_.pose.position.y, cur_pose_.pose.position.z);
}

void BackendMavros::takeOff(double _height) {
    control_mode_ = eControlMode::LOCAL_POSE;  // Take off control is performed in position (not velocity)

    setArmed(true);
    ref_pose_ = cur_pose_;
    ref_pose_.pose.position.z += _height;
    setFlightMode("OFFBOARD");

    // Wait until take off: unabortable!
    while (!referencePoseReached() && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO("Flying!");
}

void BackendMavros::land() {
    control_mode_ = eControlMode::LOCAL_POSE;  // Back to control in position (just in case)
    // Set land mode
    setFlightMode("AUTO.LAND");
    ROS_INFO("Landing...");
    ref_pose_ = cur_pose_;
    ref_pose_.pose.position.z = 0;
    // Landing is unabortable!
    while (ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (mavros_extended_state_.landed_state == 
            mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) { break; }  // Out-of-while condition
    }
    setArmed(false);  // Now disarm!
    ROS_INFO("Landed!");
}

void BackendMavros::setVelocity(const Velocity& _vel) {
    control_mode_ = eControlMode::LOCAL_VEL;  // Velocity control!
    // TODO: _vel world <-> body tf...
    ref_vel_ = _vel;
}

bool BackendMavros::isReady() const {
    return mavros_has_pose_;  // TODO: Other condition?
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

void	BackendMavros::goToWaypointGeo(const WaypointGeo& _wp){
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

        return out;
}

Velocity BackendMavros::velocity() const {
    return cur_vel_;
}

Transform BackendMavros::transform() const {
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = uav_home_frame_id_;
    out.child_frame_id = "uav_" + std::to_string(robot_id_);
    out.transform.translation.x = cur_pose_.pose.position.x + local_start_pos_[0];
    out.transform.translation.y = cur_pose_.pose.position.y + local_start_pos_[1];
    out.transform.translation.z = cur_pose_.pose.position.z + local_start_pos_[2];
    out.transform.rotation = cur_pose_.pose.orientation;
    return out;
}

bool BackendMavros::referencePoseReached() {

    double position_min, position_mean, position_max;
    double orientation_min, orientation_mean, orientation_max;
    if (!position_error_.metrics(position_min, position_mean, position_max)) { return false; }
    if (!orientation_error_.metrics(orientation_min, orientation_mean, orientation_max)) { return false; }
    
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

    uav_home_frame_id_ = "uav_" + std::to_string(robot_id_) + "_home";
    local_start_pos_ << 0.0, 0.0, 0.0;

    // Get frame from rosparam
    std::string frame_id;
    std::string parent_frame;
    std::string units;
    std::vector<double> translation;
    std::string uav_home_text;

    uav_home_text = uav_home_frame_id_;

    if ( ros::param::has(uav_home_text) ) {
        ros::param::get(uav_home_text + "/home_frame_id", frame_id);
        ros::param::get(uav_home_text + "/parent_frame", parent_frame);
        ros::param::get(uav_home_text + "/units", units);
        ros::param::get(uav_home_text + "/translation",translation);

        geometry_msgs::TransformStamped static_transformStamped;

        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = parent_frame;
        static_transformStamped.child_frame_id = frame_id;
        static_transformStamped.transform.translation.x = translation[0];
        static_transformStamped.transform.translation.y = translation[1];
        static_transformStamped.transform.translation.z = translation[2];

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
    else {
        // No param with local frame -> Global control
        // TODO: Initialization of home frame based on GPS estimation
        ROS_ERROR("No uav_%d_home_frame found in rosparam. Please define starting position with relate to a common map frame.",robot_id_);
    }
}

}}	// namespace grvc::ual
