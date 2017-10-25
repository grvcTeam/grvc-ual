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
#include <uav_abstraction_layer/backend_light.h>
#include <argument_parser/argument_parser.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <gazebo_msgs/LinkState.h>
#include <random>

// Frames/s for publishing in gazebo topics
#define FPS 25.0

namespace grvc { namespace ual {

BackendLight::BackendLight(grvc::utils::ArgumentParser& _args)
    : Backend(_args), generator_(std::chrono::system_clock::now().time_since_epoch().count()), distribution_(0.0,_args.getArgument("noise_var", 0.0))
{
    ROS_INFO("BackendLight constructor");

    // Parse arguments
    robot_id_ = _args.getArgument("uav_id", 1);
    pose_frame_id_ = _args.getArgument<std::string>("pose_frame_id", "");
    max_h_vel_ = _args.getArgument("max_h_vel", 1.6); // m/s
    max_v_vel_ = _args.getArgument("max_v_vel", 1.2); // m/s
    max_yaw_vel_ = _args.getArgument("max_yaw_vel", 1.0); // rad/s
    max_pose_error_ = _args.getArgument("max_pose_error", 0.1); // m
    max_orient_error_ = _args.getArgument("max_orient_error", 0.01); // rad

    // Init ros communications
    ros::NodeHandle nh;

    initHomeFrame();

    // Create GazeboAnimatedLink object. TODO: Get model name by args
    link_name_ = _args.getArgument("link_name", std::string("mbzirc_") + std::to_string(robot_id_) + std::string("::base_link"));
    ROS_INFO("Gazebo link name: %s",link_name_.c_str());

    std::string link_state_pub_topic = "/gazebo/set_link_state";
    link_state_publisher_ = nh.advertise<gazebo_msgs::LinkState>(link_state_pub_topic, 1);

    // Thread publishing target pose at 25Hz
    offboard_thread_ = std::thread([this]() {
        ros::Rate rate(FPS);
        gazebo_msgs::LinkState current;
        current.link_name = link_name_;
        while (ros::ok()) {
            if (control_in_vel_) {
                ref_pose_ = cur_pose_;
            } else {
                ref_vel_ = calcVel(ref_pose_);
            }
            move();
            
            current.pose = gazebo_pose_.pose;
            current.reference_frame = "map";
            link_state_publisher_.publish(current);
            
            ros::spinOnce();
            rate.sleep();
        }
    });
}

bool BackendLight::isReady() const {
    return true;
}

void BackendLight::move() {
    double t = 1 / FPS;

    cur_vel_.twist.linear.x = (0.2 * ref_vel_.twist.linear.x + 0.8 * cur_vel_.twist.linear.x);
    cur_vel_.twist.linear.y = (0.2 * ref_vel_.twist.linear.y + 0.8 * cur_vel_.twist.linear.y);
    cur_vel_.twist.linear.z = (0.2 * ref_vel_.twist.linear.z + 0.8 * cur_vel_.twist.linear.z);
    cur_vel_.twist.angular.z = (0.5 * ref_vel_.twist.angular.z + 0.5 * cur_vel_.twist.angular.z);

    cur_pose_.pose.position.x += t * cur_vel_.twist.linear.x;
    cur_pose_.pose.position.y += t * cur_vel_.twist.linear.y;
    cur_pose_.pose.position.z += t * cur_vel_.twist.linear.z;

    cur_pose_noisy_.pose.position.x = cur_pose_.pose.position.x + distribution_(generator_);
    cur_pose_noisy_.pose.position.y = cur_pose_.pose.position.y + distribution_(generator_);
    cur_pose_noisy_.pose.position.z = cur_pose_.pose.position.z + distribution_(generator_);

    tf2::Quaternion quat1(cur_pose_.pose.orientation.x,cur_pose_.pose.orientation.y,cur_pose_.pose.orientation.z,cur_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(quat1);
    double cur_yaw, cur_pitch, cur_roll;
    m.getRPY(cur_roll, cur_pitch, cur_yaw);
    cur_roll += t * cur_vel_.twist.angular.x;
    cur_pitch += t * cur_vel_.twist.angular.y;
    cur_yaw += t * cur_vel_.twist.angular.z;
    tf2::Quaternion quat2;
    quat2.setRPY(cur_roll, cur_pitch, cur_yaw);
    cur_pose_.pose.orientation.x = quat2.x();
    cur_pose_.pose.orientation.y = quat2.y();
    cur_pose_.pose.orientation.z = quat2.z();
    cur_pose_.pose.orientation.w = quat2.w();
    cur_pose_noisy_.pose.orientation = cur_pose_.pose.orientation;

    // Transform to map
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformToGazeboFrame;

    if ( cached_transforms_.find("inv_map") == cached_transforms_.end() ) {
        // inv_map not found in cached_transforms_
        transformToGazeboFrame = tfBuffer.lookupTransform("map", uav_home_frame_id_, ros::Time(0), ros::Duration(0.2));
        cached_transforms_["inv_map"] = transformToGazeboFrame; // Save transform in cache
    } else {
        // found in cache
        transformToGazeboFrame = cached_transforms_["inv_map"];
    }
    tf2::doTransform(cur_pose_, gazebo_pose_, transformToGazeboFrame);
}

Velocity BackendLight::calcVel(Pose _target_pose) {
    Velocity vel;

    if(flying_) {
        double dx = _target_pose.pose.position.x - cur_pose_noisy_.pose.position.x;
        double dy = _target_pose.pose.position.y - cur_pose_noisy_.pose.position.y;
        double dz = _target_pose.pose.position.z - cur_pose_noisy_.pose.position.z;
        double dYaw = 2*atan2(_target_pose.pose.orientation.z,_target_pose.pose.orientation.w) - 2*atan2(cur_pose_noisy_.pose.orientation.z,cur_pose_noisy_.pose.orientation.w);
        while (dYaw < -M_PI) dYaw += 2*M_PI;
        while (dYaw >  M_PI) dYaw -= 2*M_PI;

        double Th = sqrt( dx*dx + dy*dy ) / max_h_vel_;
        double Tz = std::abs( dz / max_v_vel_ );
        double TYaw = std::abs( dYaw / max_yaw_vel_);
        double T = std::max(Th, Tz);
        T = std::max(T, TYaw);

        if ( T < 1/FPS ) {
            T = 1/FPS;
        }

        vel.twist.linear.x = dx / T;
        vel.twist.linear.y = dy / T;
        vel.twist.linear.z = dz / T;
        vel.twist.angular.z = dYaw / T;

        if ( std::abs( dx ) < max_pose_error_ ) { vel.twist.linear.x = 0.0; }
        if ( std::abs( dy ) < max_pose_error_ ) { vel.twist.linear.y = 0.0; }
        if ( std::abs( dz ) < max_pose_error_ ) { vel.twist.linear.z = 0.0; }
        if ( std::abs( dYaw ) < max_orient_error_ ) { vel.twist.angular.z = 0.0; }
    }
    else {
        vel.twist.linear.x = 0;
        vel.twist.linear.y = 0;
        vel.twist.linear.z = 0;
    }

    return vel;
}

void BackendLight::takeOff(double _height) {
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
}

void BackendLight::land() {
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
}

void BackendLight::setVelocity(const Velocity& _vel) {
    control_in_vel_ = true;  // Velocity control!
    // TODO: _vel world <-> body tf...
    ref_vel_ = _vel;
}

void BackendLight::setPositionError(const PositionError& _pos_error) {
    Eigen::Vector3d vector_error = {_pos_error.vector.x, _pos_error.vector.y, _pos_error.vector.z};
    double dt = 0.03;  // TODO: use time in headers?
    integral_control_vel_ += vector_error * dt;
    Velocity vel;
    // TODO: create pid util?
    vel.twist.linear.x = p_gain_xy_ * vector_error[0] + \
        k_i_xy_ * integral_control_vel_[0] + \
        k_d_xy_ * (vector_error[0] - previous_error_control_vel_[0]) / dt;
    vel.twist.linear.y = p_gain_xy_ * vector_error[1] + \
        k_i_xy_ * integral_control_vel_[1] + \
        k_d_xy_ * (vector_error[1] - previous_error_control_vel_[1]) / dt;
    vel.twist.linear.z = p_gain_z_ * vector_error[2] + \
        k_i_z_ * integral_control_vel_[2] + \
        k_d_z_ * (vector_error[2] - previous_error_control_vel_[2]) / dt;
    vel.twist.angular.z  = 0.0;
    setVelocity(vel);
}

void BackendLight::goToWaypoint(const Waypoint& _world) {
    control_in_vel_ = false;  // Control in position

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
            transformToHomeFrame = tfBuffer.lookupTransform(uav_home_frame_id_, waypoint_frame_id, ros::Time(0), ros::Duration(0.2));
            cached_transforms_[waypoint_frame_id] = transformToHomeFrame; // Save transform in cache
        } else {
            // found in cache
            transformToHomeFrame = cached_transforms_[waypoint_frame_id];
        }
        
        tf2::doTransform(_world, homogen_world_pos, transformToHomeFrame);
        
    }

//    std::cout << "Going to waypoint: " << homogen_world_pos.pose.position << std::endl;

    ref_pose_.pose = homogen_world_pos.pose;

    // Wait until we arrive: abortable
    while(!referencePoseReached() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Freeze in case it's been aborted
    if (abort_) {
        ref_pose_ = cur_pose_;
    }
}

void BackendLight::goToWaypointGeo(const WaypointGeo& _world) {
    assert(false); // 666 NOT IMPLEMENTED YET
}

Pose BackendLight::pose() {
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
                tf2_ros::Buffer tfBuffer;
                tf2_ros::TransformListener tfListener(tfBuffer);
                transformToPoseFrame = tfBuffer.lookupTransform(pose_frame_id_,uav_home_frame_id_, ros::Time(0), ros::Duration(0.2));
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

Velocity BackendLight::velocity() const {
    return cur_vel_;
}

Transform BackendLight::transform() const {
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = uav_home_frame_id_;
    out.child_frame_id = "uav_" + std::to_string(robot_id_);
    out.transform.translation.x = cur_pose_.pose.position.x;
    out.transform.translation.y = cur_pose_.pose.position.y;
    out.transform.translation.z = cur_pose_.pose.position.z;
    out.transform.rotation = cur_pose_.pose.orientation;
    return out;
}

bool BackendLight::referencePoseReached() const {
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

    if ((positionD > 0.1) || (orientationD > 0.1))  // TODO: define thresholds
        return false;
    else
        return true;
}

void BackendLight::initHomeFrame() {

    uav_home_frame_id_ = "uav_" + std::to_string(robot_id_) + "_home";
    local_start_pos_ << 0.0, 0.0, 0.0;

    // Get frame from rosparam
    std::string frame_id;
    std::string parent_frame;
    std::string units;
    std::vector<double> translation;
    double gz_yaw;
    std::string uav_home_text;

    uav_home_text = uav_home_frame_id_;

    if ( ros::param::has(uav_home_text) ) {
        ros::param::get(uav_home_text + "/home_frame_id", frame_id);
        ros::param::get(uav_home_text + "/parent_frame", parent_frame);
        ros::param::get(uav_home_text + "/units", units);
        ros::param::get(uav_home_text + "/translation",translation);
        ros::param::get(uav_home_text + "/gz_initial_yaw",gz_yaw);

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

        // Set initial pose in local frame
        cur_pose_.pose.position.x = 0.0;
        cur_pose_.pose.position.y = 0.0;
        cur_pose_.pose.position.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0,0.0,gz_yaw);
        cur_pose_.pose.orientation.x = quat.x();
        cur_pose_.pose.orientation.y = quat.y();
        cur_pose_.pose.orientation.z = quat.z();
        cur_pose_.pose.orientation.w = quat.w();
        ref_pose_ = cur_pose_;
    }
    else {
        // No param with local frame -> Global control
        // TODO: Initialization of home frame based on GPS estimation
        ROS_ERROR("No uav_%d_home_frame found in rosparam. Please define starting position with relate to a common map frame.",robot_id_);
    }
}

}}	// namespace grvc::ual
