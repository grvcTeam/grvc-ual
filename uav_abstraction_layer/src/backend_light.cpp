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

BackendLight::BackendLight()
    : Backend(), generator_(std::chrono::system_clock::now().time_since_epoch().count())
{
    ROS_INFO("BackendLight constructor");

    // Init ros communications
    ros::NodeHandle nh;

    // Parse arguments
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");
    pnh.param<float>("max_h_vel", max_h_vel_, 1.6); // m/s
    pnh.param<float>("max_v_vel", max_v_vel_, 1.2); // m/s
    pnh.param<float>("max_yaw_vel", max_yaw_vel_, 1.0); // rad/s
    pnh.param<float>("max_pose_error", max_pose_error_, 0.1); // m
    pnh.param<float>("max_orient_error", max_orient_error_, 0.01); // rad
    float noise_var;
    pnh.param<float>("noise_var", noise_var, 0.0);

    distribution_ = new std::normal_distribution<double>(0.0,noise_var);

    initHomeFrame();

    // Create GazeboAnimatedLink object.
    pnh.param<std::string>( "link_name", link_name_, std::string("mbzirc_") + std::to_string(robot_id_) + std::string("::base_link") );
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
                if ( ros::Time::now().toSec() - last_command_time_.toSec() >=0.5 ) {
                    control_in_vel_ = false;
                }
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

    ROS_INFO("BackendLight %d running!", robot_id_);
}

bool BackendLight::isReady() const {
    return true;
}

void BackendLight::move() {
    double t = 1 / FPS;

    cur_vel_.header.frame_id = uav_home_frame_id_;
    cur_vel_.twist.linear.x = (0.2 * ref_vel_.twist.linear.x + 0.8 * cur_vel_.twist.linear.x);
    cur_vel_.twist.linear.y = (0.2 * ref_vel_.twist.linear.y + 0.8 * cur_vel_.twist.linear.y);
    cur_vel_.twist.linear.z = (0.2 * ref_vel_.twist.linear.z + 0.8 * cur_vel_.twist.linear.z);
    cur_vel_.twist.angular.z = (0.5 * ref_vel_.twist.angular.z + 0.5 * cur_vel_.twist.angular.z);

    cur_pose_.pose.position.x += t * cur_vel_.twist.linear.x;
    cur_pose_.pose.position.y += t * cur_vel_.twist.linear.y;
    cur_pose_.pose.position.z += t * cur_vel_.twist.linear.z;

    cur_pose_noisy_.pose.position.x = cur_pose_.pose.position.x + distribution_->operator()(generator_);
    cur_pose_noisy_.pose.position.y = cur_pose_.pose.position.y + distribution_->operator()(generator_);
    cur_pose_noisy_.pose.position.z = cur_pose_.pose.position.z + distribution_->operator()(generator_);

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

void BackendLight::setPose(const geometry_msgs::PoseStamped& _world) {
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
}

void BackendLight::goToWaypoint(const Waypoint& _world) {
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

Odometry BackendLight::odometry() const {
    Odometry odom;
    
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = uav_home_frame_id_;
    odom.child_frame_id = uav_frame_id_;
    odom.pose.pose = cur_pose_.pose;
    odom.twist.twist = cur_vel_.twist;

    return odom;
}

Transform BackendLight::transform() const {
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

    // Get frame from rosparam
    ros::param::param<std::string>("~uav_frame",uav_frame_id_,"uav_" + std::to_string(robot_id_));
    ros::param::param<std::string>("~uav_home_frame",uav_home_frame_id_, "uav_" + std::to_string(robot_id_) + "_home");
    std::string parent_frame;
    std::vector<double> home_pose(4, 0.0);
    ros::param::get("~home_pose",home_pose);
    ros::param::param<std::string>("~home_pose_parent_frame", parent_frame, "map");

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

    // Set initial pose in local frame
    cur_pose_.pose.position.x = 0.0;
    cur_pose_.pose.position.y = 0.0;
    cur_pose_.pose.position.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0,0.0,home_pose[3]);
    cur_pose_.pose.orientation.x = quat.x();
    cur_pose_.pose.orientation.y = quat.y();
    cur_pose_.pose.orientation.z = quat.z();
    cur_pose_.pose.orientation.w = quat.w();
    ref_pose_ = cur_pose_;
}

}}	// namespace grvc::ual
