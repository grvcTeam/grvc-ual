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
#include <ual_backend_gazebo_light/ual_backend_gazebo_light.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <random>

// Frames/s for publishing in gazebo topics
#define FPS 25.0

namespace grvc { namespace ual {

BackendGazeboLight::BackendGazeboLight()
    : Backend(), generator_(std::chrono::system_clock::now().time_since_epoch().count()), tf_listener_(tf_buffer_)
{
    ROS_INFO("BackendGazeboLight constructor");

    // Init ros communications
    ros::NodeHandle nh;

    // Parse arguments
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");
    pnh.param<float>("max_horizontal_velocity", max_horizontal_velocity_, 1.6);  // [m/s]
    pnh.param<float>("max_vertical_velocity",   max_vertical_velocity_,   1.2);  // [m/s]
    pnh.param<float>("max_yaw_rate", max_yaw_rate_, 1.0);  // [rad/s]
    pnh.param<float>("max_position_error",    max_position_error_,    0.1);   // [m]
    pnh.param<float>("max_orientation_error", max_orientation_error_, 0.01);  // [rad]
    float noise_var;
    pnh.param<float>("noise_var", noise_var, 0.0);
    float position_th_param, orientation_th_param, hold_pose_time_param;
    pnh.param<float>("position_th", position_th_param, 0.33);
    pnh.param<float>("orientation_th", orientation_th_param, 0.65);
    // pnh.param<float>("hold_pose_time", hold_pose_time_param, 3.0);
    position_th_ = position_th_param*position_th_param;
    orientation_th_ = 0.5*(1 - cos(orientation_th_param));
    // hold_pose_time_ = std::max(hold_pose_time_param, 0.001f);  // Force min value

    distribution_ = new std::normal_distribution<double>(0.0, noise_var);

    // This backend will animate a model named model_name inside Gazebo
    pnh.param<std::string>("model_name", model_name_, std::string("mbzirc_") + std::to_string(robot_id_));

    std::string model_state_pub_topic = "/gazebo/set_model_state";
    std::string model_state_sub_topic = "/gazebo/model_states";
    model_state_publisher_ = nh.advertise<gazebo_msgs::ModelState>(model_state_pub_topic, 1);
    model_state_subscriber_ = nh.subscribe<gazebo_msgs::ModelStates>(model_state_sub_topic, 1, \
        [this](const gazebo_msgs::ModelStatesConstPtr& _msg) {
            for (int i = 0; i < _msg->name.size(); i++) {
                if (_msg->name[i] == this->model_name_) {
                    this->model_pose_ = _msg->pose[i];
                    this->has_pose_ = true;
                    break;
                }
            }
    });

    // Wait until model has pose
    while (!has_pose_) {
        ROS_INFO("Waiting for Gazebo model named: %s", model_name_.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    initHomeFrame();

    // Thread publishing target pose at FPS [Hz]
    offboard_thread_ = std::thread([this]() {
        ros::Rate rate(FPS);
        gazebo_msgs::ModelState current;
        current.model_name = model_name_;
        while (ros::ok()) {
            if (control_in_vel_) {
                ref_pose_ = cur_pose_;
                bool timeout = ((ros::Time::now().toSec() - last_command_time_.toSec()) >= 0.5);
                if (timeout) { control_in_vel_ = false; }

            } else {
                ref_vel_ = calculateRefVel(ref_pose_);
            }
            move();
            
            current.pose = gazebo_pose_.pose;
            current.reference_frame = "map";
            model_state_publisher_.publish(current);
            
            ros::spinOnce();
            rate.sleep();
        }
    });

    int rotors_count = 6;  // TODO: Start big and decrease if response.status_message == "ApplyBodyWrench: body does not exist"?
    std::string wrench_uri = "/gazebo/apply_body_wrench";
    wrench_client_ = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>(wrench_uri);
    rotors_timer_ = nh.createTimer(ros::Duration(1.0), [this, rotors_count, wrench_uri](const ros::TimerEvent&) {
        gazebo_msgs::ApplyBodyWrench wrench_srv;
        wrench_srv.request.reference_frame = "map";
        wrench_srv.request.duration = ros::Duration(2.0);

        float torque = this->flying_? 10.0: 2.0;
        int rotors_sign[] = {-1, +1, -1, +1, +1, -1};  // TODO: depends on rotors_count
        for (int i = 0; i < rotors_count; i++) {
            wrench_srv.request.body_name = model_name_ + "::rotor_" + std::to_string(i);
            wrench_srv.request.wrench.torque.z = rotors_sign[i] * torque;  // TODO: Use cur_vel_?
            if (this->wrench_client_.call(wrench_srv)) {
                // ROS_INFO("success: %s, status_message: %s", wrench_srv.response.success? "true": "false", wrench_srv.response.status_message.c_str());
                if (!wrench_srv.response.success) {
                    ROS_WARN("[%s] status_message: %s", wrench_srv.request.body_name.c_str(), wrench_srv.response.status_message.c_str());
                }
            } else {
                ROS_ERROR("Failed to call service %s", wrench_uri.c_str());
            }
        }
    });

    ROS_INFO("BackendGazeboLight %d running!", robot_id_);
    this->state_ = uav_abstraction_layer::State::LANDED_ARMED;
}

BackendGazeboLight::~BackendGazeboLight() {
    if (offboard_thread_.joinable()) { offboard_thread_.join(); }
}

bool BackendGazeboLight::isReady() const {
    return has_pose_;
}

void BackendGazeboLight::move() {
    double dt = 1 / FPS;

    cur_vel_.header.frame_id = uav_home_frame_id_;
    cur_vel_.twist.linear.x = (0.2 * ref_vel_.twist.linear.x + 0.8 * cur_vel_.twist.linear.x);
    cur_vel_.twist.linear.y = (0.2 * ref_vel_.twist.linear.y + 0.8 * cur_vel_.twist.linear.y);
    cur_vel_.twist.linear.z = (0.2 * ref_vel_.twist.linear.z + 0.8 * cur_vel_.twist.linear.z);
    cur_vel_.twist.angular.z = (0.5 * ref_vel_.twist.angular.z + 0.5 * cur_vel_.twist.angular.z);

    cur_pose_.pose.position.x += dt * cur_vel_.twist.linear.x;
    cur_pose_.pose.position.y += dt * cur_vel_.twist.linear.y;
    cur_pose_.pose.position.z += dt * cur_vel_.twist.linear.z;

    cur_pose_noisy_.pose.position.x = cur_pose_.pose.position.x + distribution_->operator()(generator_);
    cur_pose_noisy_.pose.position.y = cur_pose_.pose.position.y + distribution_->operator()(generator_);
    cur_pose_noisy_.pose.position.z = cur_pose_.pose.position.z + distribution_->operator()(generator_);

    double cur_yaw = 2.0 * atan2(cur_pose_.pose.orientation.z, cur_pose_.pose.orientation.w);
    cur_yaw += dt * cur_vel_.twist.angular.z;
    cur_pose_.pose.orientation.x = 0;
    cur_pose_.pose.orientation.y = 0;
    cur_pose_.pose.orientation.z = sin(0.5*cur_yaw);
    cur_pose_.pose.orientation.w = cos(0.5*cur_yaw);
    cur_pose_noisy_.pose.orientation = cur_pose_.pose.orientation;

    // Transform to map
    geometry_msgs::TransformStamped transformToGazeboFrame;

    if ( cached_transforms_.find("inv_map") == cached_transforms_.end() ) {
        // inv_map not found in cached_transforms_
        try {  // TODO: This try-catch is repeated several times, make a function?
            transformToGazeboFrame = tf_buffer_.lookupTransform("map", uav_home_frame_id_, ros::Time(0), ros::Duration(0.2));
            cached_transforms_["inv_map"] = transformToGazeboFrame; // Save transform in cache
        } catch (tf2::TransformException &ex) {
            ROS_WARN("At line [%d]: %s", __LINE__, ex.what());
            return;
        }
    } else {
        // found in cache
        transformToGazeboFrame = cached_transforms_["inv_map"];
    }
    tf2::doTransform(cur_pose_, gazebo_pose_, transformToGazeboFrame);

    // Resolve model shakiness
    double dx = gazebo_pose_.pose.position.x - model_pose_.position.x;
    double dy = gazebo_pose_.pose.position.y - model_pose_.position.y;
    double dz = gazebo_pose_.pose.position.z - model_pose_.position.z;
    double shakiness_metric = dx*dx + dy*dy + dz*dz;  // TODO: Other metric? orientation?
    double shakiness_threshold = 0.06;  // TODO: Tune?
    if (shakiness_metric > shakiness_threshold) {
        ROS_WARN("Resolving pose conflict: [%lf, %lf, %lf, %lf, %lf, %lf, %lf] -> [%lf, %lf, %lf, %lf, %lf, %lf, %lf]", 
            gazebo_pose_.pose.position.x, gazebo_pose_.pose.position.y, gazebo_pose_.pose.position.z, 
            gazebo_pose_.pose.orientation.x, gazebo_pose_.pose.orientation.y, gazebo_pose_.pose.orientation.z, gazebo_pose_.pose.orientation.w, 
            model_pose_.position.x, model_pose_.position.y, model_pose_.position.z, 
            model_pose_.orientation.x, model_pose_.orientation.y, model_pose_.orientation.z, model_pose_.orientation.w);
        gazebo_pose_.pose = model_pose_;  // TODO: Or something in between?

        // Update also internal pose
        geometry_msgs::TransformStamped transformToHomeFrame;
        if ( cached_transforms_.find("map") == cached_transforms_.end() ) {
            try {
                transformToHomeFrame = tf_buffer_.lookupTransform(uav_home_frame_id_, "map", ros::Time(0), ros::Duration(0.2));
                cached_transforms_["map"] = transformToHomeFrame; // Save transform in cache
            } catch (tf2::TransformException &ex) {
                ROS_WARN("At line [%d]: %s", __LINE__, ex.what());
                return;
            }
        } else {
            transformToHomeFrame = cached_transforms_["map"];
        }
        tf2::doTransform(gazebo_pose_, cur_pose_, transformToHomeFrame);

        // Do not add noise
        cur_pose_noisy_ = cur_pose_;
    }
}

Velocity BackendGazeboLight::calculateRefVel(Pose _target_pose) {
    Velocity vel;

    if(flying_) {
        double dx = _target_pose.pose.position.x - cur_pose_noisy_.pose.position.x;
        double dy = _target_pose.pose.position.y - cur_pose_noisy_.pose.position.y;
        double dz = _target_pose.pose.position.z - cur_pose_noisy_.pose.position.z;
        double dYaw = 2*atan2(_target_pose.pose.orientation.z,_target_pose.pose.orientation.w) - 2*atan2(cur_pose_noisy_.pose.orientation.z,cur_pose_noisy_.pose.orientation.w);
        while (dYaw < -M_PI) dYaw += 2*M_PI;
        while (dYaw >  M_PI) dYaw -= 2*M_PI;

        double Th = sqrt( dx*dx + dy*dy ) / max_horizontal_velocity_;
        double Tz = std::abs( dz / max_vertical_velocity_ );
        double TYaw = std::abs( dYaw / max_yaw_rate_);
        double T = std::max(Th, Tz);
        T = std::max(T, TYaw);

        if ( T < 1/FPS ) {
            T = 1/FPS;
        }

        vel.twist.linear.x = dx / T;
        vel.twist.linear.y = dy / T;
        vel.twist.linear.z = dz / T;
        vel.twist.angular.z = dYaw / T;

        if ( std::abs( dx ) < max_position_error_ ) { 
            vel.twist.linear.x = 0.0;
            cur_pose_.pose.position.x = 0.8*cur_pose_.pose.position.x + 0.2*_target_pose.pose.position.x;
        }
        if ( std::abs( dy ) < max_position_error_ ) {
            vel.twist.linear.y = 0.0;
            cur_pose_.pose.position.y = 0.8*cur_pose_.pose.position.y + 0.2*_target_pose.pose.position.y;
        }
        if ( std::abs( dz ) < max_position_error_ ) {
            vel.twist.linear.z = 0.0;
            cur_pose_.pose.position.z = 0.8*cur_pose_.pose.position.z + 0.2*_target_pose.pose.position.z;
        }
        if ( std::abs( dYaw ) < max_orientation_error_ ) {
            vel.twist.angular.z = 0.0;
            cur_pose_.pose.orientation.x = 0.5*cur_pose_.pose.orientation.x + 0.5*_target_pose.pose.orientation.x;
            cur_pose_.pose.orientation.y = 0.5*cur_pose_.pose.orientation.y + 0.5*_target_pose.pose.orientation.y;
            cur_pose_.pose.orientation.z = 0.5*cur_pose_.pose.orientation.z + 0.5*_target_pose.pose.orientation.z;
            cur_pose_.pose.orientation.w = 0.5*cur_pose_.pose.orientation.w + 0.5*_target_pose.pose.orientation.w;
        }
    }
    else {
        vel.twist.linear.x = 0;
        vel.twist.linear.y = 0;
        vel.twist.linear.z = 0;
    }

    return vel;
}

void BackendGazeboLight::takeOff(double _height) {
    this->state_ = uav_abstraction_layer::State::TAKING_OFF;
    control_in_vel_ = false;  // Take off control is performed in position (not velocity)

    // Set offboard mode after saving home pose
    home_pose_ = cur_pose_;
    ref_pose_ = home_pose_;
    ref_pose_.pose.position.z += _height;

    // Set flying flag true
    flying_ = true;

    // Wait until take off: unabortable!
    while (!referencePoseReached() && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO("Flying!");
    this->state_ = uav_abstraction_layer::State::FLYING_AUTO;
}

void BackendGazeboLight::land() {
    this->state_ = uav_abstraction_layer::State::LANDING;
    control_in_vel_ = false;  // Back to control in position (just in case)

    ROS_INFO("Landing...");
    ref_pose_ = cur_pose_;
    ref_pose_.pose.position.z = 0.0; // TODO: Check minimum altitud in Gazebo
    // Landing is unabortable!
    while (!referencePoseReached() && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO("Landed!");
    flying_ = false;
    this->state_ = uav_abstraction_layer::State::LANDED_ARMED;
}

void BackendGazeboLight::setVelocity(const Velocity& _vel) {
    control_in_vel_ = true;  // Velocity control!
    
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
            transform = tf_buffer_.lookupTransform(uav_home_frame_id_, vel_frame_id, ros::Time(0), ros::Duration(0.3));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("At line [%d]: %s", __LINE__, ex.what());
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

void BackendGazeboLight::setPose(const geometry_msgs::PoseStamped& _world) {
    control_in_vel_ = false;  // Control in position

    geometry_msgs::PoseStamped homogen_world_pos;
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
            try {
                transformToHomeFrame = tf_buffer_.lookupTransform(uav_home_frame_id_, waypoint_frame_id, ros::Time(0), ros::Duration(0.2));
                cached_transforms_[waypoint_frame_id] = transformToHomeFrame; // Save transform in cache
            } catch (tf2::TransformException &ex) {
                ROS_WARN("At line [%d]: %s", __LINE__, ex.what());
                return;
            }
        } else {
            // found in cache
            transformToHomeFrame = cached_transforms_[waypoint_frame_id];
        }
        
        tf2::doTransform(_world, homogen_world_pos, transformToHomeFrame);
        
    }

//    std::cout << "Going to waypoint: " << homogen_world_pos.pose.position << std::endl;

    ref_pose_.pose = homogen_world_pos.pose;
}

void BackendGazeboLight::goToWaypoint(const Waypoint& _world) {
    // Set pose!
    setPose(_world);

    // Wait until we arrive: abortable
    while(!referencePoseReached() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Freeze in case it's been aborted
    if (abort_) {
        ref_pose_ = cur_pose_;
    }
}

void BackendGazeboLight::goToWaypointGeo(const WaypointGeo& _world) {
    assert(false); // 666 NOT IMPLEMENTED YET
}

Pose BackendGazeboLight::pose() {
        Pose out;

        out.pose.position.x = cur_pose_noisy_.pose.position.x;
        out.pose.position.y = cur_pose_noisy_.pose.position.y;
        out.pose.position.z = cur_pose_noisy_.pose.position.z;
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
                try {
                    transformToPoseFrame = tf_buffer_.lookupTransform(pose_frame_id_,uav_home_frame_id_, ros::Time(0), ros::Duration(0.2));
                    cached_transforms_[pose_frame_id_map] = transformToPoseFrame; // Save transform in cache
                } catch (tf2::TransformException &ex) {
                    ROS_WARN("At line [%d]: %s", __LINE__, ex.what());
                    return Pose();
                }
            } else {
                // found in cache
                transformToPoseFrame = cached_transforms_[pose_frame_id_map];
            }

            tf2::doTransform(aux, out, transformToPoseFrame);
            out.header.frame_id = pose_frame_id_;
        }
        out.header.stamp = ros::Time::now();

        return out;
}

Pose BackendGazeboLight::referencePose() {
    Pose out;

    out.pose.position.x = ref_pose_.pose.position.x;
    out.pose.position.y = ref_pose_.pose.position.y;
    out.pose.position.z = ref_pose_.pose.position.z;
    out.pose.orientation = ref_pose_.pose.orientation;

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
            try {
                transformToPoseFrame = tf_buffer_.lookupTransform(pose_frame_id_,uav_home_frame_id_, ros::Time(0), ros::Duration(1.0));
                cached_transforms_[pose_frame_id_map] = transformToPoseFrame; // Save transform in cache
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("In referencePose: %s. Returning non transformed pose.", ex.what());
                return out;
            }
        } else {
            // found in cache
            transformToPoseFrame = cached_transforms_[pose_frame_id_map];
        }

        tf2::doTransform(aux, out, transformToPoseFrame);
        out.header.frame_id = pose_frame_id_;
    }

    out.header.stamp = ref_pose_.header.stamp;
    return out;
}

Velocity BackendGazeboLight::velocity() const {
    return cur_vel_;
}

Odometry BackendGazeboLight::odometry() const {
    Odometry odom;
    
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = uav_home_frame_id_;
    odom.child_frame_id = uav_frame_id_;
    odom.pose.pose = cur_pose_.pose;
    odom.twist.twist = cur_vel_.twist;

    return odom;
}

Transform BackendGazeboLight::transform() const {
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = uav_home_frame_id_;
    out.child_frame_id = uav_frame_id_;
    out.transform.translation.x = cur_pose_.pose.position.x;
    out.transform.translation.y = cur_pose_.pose.position.y;
    out.transform.translation.z = cur_pose_.pose.position.z;
    out.transform.rotation = cur_pose_.pose.orientation;
    return out;
}

bool BackendGazeboLight::referencePoseReached() {
    double dx = ref_pose_.pose.position.x - cur_pose_.pose.position.x;
    double dy = ref_pose_.pose.position.y - cur_pose_.pose.position.y;
    double dz = ref_pose_.pose.position.z - cur_pose_.pose.position.z;
    double positionD = dx*dx + dy*dy + dz*dz; // Equals distance^2

    double quatInnerProduct = ref_pose_.pose.orientation.x*cur_pose_.pose.orientation.x + \
    ref_pose_.pose.orientation.y*cur_pose_.pose.orientation.y + \
    ref_pose_.pose.orientation.z*cur_pose_.pose.orientation.z + \
    ref_pose_.pose.orientation.w*cur_pose_.pose.orientation.w;
    double orientationD = 1 - quatInnerProduct*quatInnerProduct;  // Equals (1-cos(rotation))/2

    /*ROS_INFO("ref_position = [%f, %f, %f] cur_position = [%f, %f, %f]", \
    ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z, \
    cur_pose_.pose.position.x, cur_pose_.pose.position.y, cur_pose_.pose.position.z);*/
    //ROS_INFO("pD = %f,\t oD = %f", positionD, orientationD);

    if ((positionD > position_th_) || (orientationD > orientation_th_)) {
        return false;
    } else {
        // cur_pose_ = ref_pose_;
        return true;
    }
}

void BackendGazeboLight::initHomeFrame() {

    // Get frame prefix from namespace
    std::string ns = ros::this_node::getNamespace();
    uav_frame_id_ = ns + "/base_link";
    uav_home_frame_id_ = ns + "/odom";
    while (uav_frame_id_[0]=='/') {
        uav_frame_id_.erase(0,1);
    }
    while (uav_home_frame_id_[0]=='/') {
        uav_home_frame_id_.erase(0,1);
    }

    std::string parent_frame;
    ros::param::param<std::string>("~home_pose_parent_frame", parent_frame, "map");

    std::vector<double> home_pose(3, 0.0);
    if (ros::param::has("~home_pose")) {
        ros::param::get("~home_pose", home_pose);
    } else {
        ROS_WARN("No home pose or map origin was defined. Home frame will be equal to map.");
    }

    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = parent_frame;
    static_transformStamped.child_frame_id = uav_home_frame_id_;
    static_transformStamped.transform.translation.x = home_pose[0];
    static_transformStamped.transform.translation.y = home_pose[1];
    static_transformStamped.transform.translation.z = home_pose[2];

    if (parent_frame == "map" || parent_frame == "") {
        static_transformStamped.transform.rotation.x = 0;
        static_transformStamped.transform.rotation.y = 0;
        static_transformStamped.transform.rotation.z = 0;
        static_transformStamped.transform.rotation.w = 1;
    } else {
        try {
            geometry_msgs::TransformStamped transform_to_map;
            transform_to_map = tf_buffer_.lookupTransform(parent_frame, "map", ros::Time(0), ros::Duration(2.0));
            static_transformStamped.transform.rotation = transform_to_map.transform.rotation;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("At line [%d]: %s", __LINE__, ex.what());
            return;
        }
    }

    static_tf_broadcaster_ = new tf2_ros::StaticTransformBroadcaster();
    static_tf_broadcaster_->sendTransform(static_transformStamped);

    // Set initial pose in local frame
    cur_pose_.pose.position.x = 0.0;
    cur_pose_.pose.position.y = 0.0;
    cur_pose_.pose.position.z = 0.0;
    cur_pose_.pose.orientation = model_pose_.orientation;
    ref_pose_ = cur_pose_;
}

}}	// namespace grvc::ual
