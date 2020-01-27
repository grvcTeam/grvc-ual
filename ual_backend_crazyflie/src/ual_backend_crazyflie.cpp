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

#include <mavros_msgs/ParamGet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <ual_backend_crazyflie/ual_backend_crazyflie.h>
#include <uav_abstraction_layer/geographic_to_cartesian.h>
#include <chrono>
#include <string>

namespace grvc {
namespace ual {

BackendCrazyflie::BackendCrazyflie()
    : Backend() {
    // Parse arguments
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");
    float position_th_param, orientation_th_param;
    pnh.param<float>("position_th", position_th_param, 0.33);
    pnh.param<float>("orientation_th", orientation_th_param, 0.65);
    position_th_ = position_th_param * position_th_param;
    orientation_th_ = 0.5 * (1 - cos(orientation_th_param));

    ROS_INFO("BackendCrazyflie constructor with id %d", robot_id_);
    // ROS_INFO("BackendCrazyflie: thresholds = %f %f", position_th_, orientation_th_);

    // Init ros communications
    ros::NodeHandle nh;
    std::string crazyflie_ns = "/cf" + std::to_string(robot_id_);
   
    // std::string set_mode_srv = crazyflie_ns + "/set_mode";
    // std::string arming_srv = crazyflie_ns + "/cmd/arming";
    // std::string get_param_srv = crazyflie_ns + "/param/get";
    std::string set_pose_topic = crazyflie_ns + "/goal";
    // std::string set_pose_global_topic = crazyflie_ns + "/setpoint_raw/global";
    std::string set_vel_topic = crazyflie_ns + "/cmd_vel";
    std::string pose_topic = crazyflie_ns + "/pose";
    // std::string geo_pose_topic = crazyflie_ns + "/global_position/global";
    // std::string vel_topic = crazyflie_ns + "/local_position/velocity";
    std::string state_topic = crazyflie_ns + "/state";
    // std::string extended_state_topic = crazyflie_ns + "/extended_state";

    takeoff_client_ = nh.serviceClient<crazyflie_driver::Takeoff>(crazyflie_ns + "/takeoff");
    land_client_ = nh.serviceClient<crazyflie_driver::Land>(crazyflie_ns + "/land");
    go_to_client_ = nh.serviceClient<crazyflie_driver::GoTo>(crazyflie_ns + "/goto");

    crazyflie_cur_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic.c_str(), 1,
                                                                       [this](const geometry_msgs::PoseStamped::ConstPtr& _msg) {
                                                                           this->cur_pose_ = *_msg;
                                                                           this->cf_has_pose_ = true;
                                                                       });

    crazyflie_cur_state_sub_ = nh.subscribe<std_msgs::Int8>(state_topic.c_str(), 1,
                                                            [this](const std_msgs::Int8::ConstPtr& _msg) {
                                                                this->crazyflie_state_ = *_msg;
                                                            });

    ROS_INFO("wait_for_service /takeoff");
    takeoff_client_.waitForExistence();
    ROS_INFO("found /takeoff");

    // TODO: Check this and solve frames issue
    initHomeFrame();

    // Thread publishing target pose at 10Hz for offboard mode
    offboard_thread_ = std::thread(&BackendCrazyflie::offboardThreadLoop, this);

    ROS_INFO("BackendCrazyflie %d running!", robot_id_);
}

BackendCrazyflie::~BackendCrazyflie() {
    if (offboard_thread_.joinable()) {
        offboard_thread_.join();
    }
}

void BackendCrazyflie::offboardThreadLoop() {
    offboard_thread_frequency_ = 30.0;
    ros::Rate rate(offboard_thread_frequency_);
    while (ros::ok() /* && crazyflie_state_.data == 1 */) {
        switch (control_mode_) {
            // case eControlMode::LOCAL_VEL:
            //     crazyflie_ref_vel_pub_.publish(ref_vel_);
            //     ref_pose_ = cur_pose_;
            //     if (ros::Time::now().toSec() - last_command_time_.toSec() >= 0.5) {
            //         control_mode_ = eControlMode::LOCAL_POSE;
            //     }
            //     break;
            case eControlMode::LOCAL_POSE:
                ref_pose_.header.stamp = ros::Time::now();
                crazyflie_ref_pose_pub_.publish(ref_pose_);
                ref_vel_.twist.linear.x = 0;
                ref_vel_.twist.linear.y = 0;
                ref_vel_.twist.linear.z = 0;
                ref_vel_.twist.angular.z = 0;
                break;
        }

        // State update
        this->state_ = guessState();

        rate.sleep();
    }
}

grvc::ual::State BackendCrazyflie::guessState() {
    // Sequentially checks allow state deduction
    if (!this->isReady()) {
        return uav_abstraction_layer::State::UNINITIALIZED;
    }
    // if (!this->crazyflie_state_.armed) {
    //     return LANDED_DISARMED;
    // }
    if (this->isReady() && crazyflie_state_.data == 0) {
        return uav_abstraction_layer::State::LANDED_ARMED;
    }
    if (this->calling_takeoff) {
        return uav_abstraction_layer::State::TAKING_OFF;
    }
    if (this->calling_land) {
        return uav_abstraction_layer::State::LANDING;
    }

    return uav_abstraction_layer::State::FLYING_AUTO;
}

void BackendCrazyflie::setFlightMode(const std::string& _flight_mode) {
    // TODO!
}

void BackendCrazyflie::recoverFromManual() {
    // TODO!
}

void BackendCrazyflie::setHome(bool set_z) {
    // double z_offset = set_z ? cur_pose_.pose.position.z : 0.0;
    // local_start_pos_ = -Eigen::Vector3d(cur_pose_.pose.position.x,
    //                                     cur_pose_.pose.position.y, z_offset);
}

void BackendCrazyflie::takeOff(double _height) {
    if (_height < 0.0) {
        ROS_ERROR("Takeoff height must be positive!");
        return;
    }
    calling_takeoff = true;

    control_mode_ = eControlMode::LOCAL_POSE;  // Take off control is performed in position (not velocity)

    double takeoff_vel = 1.0;
    crazyflie_driver::Takeoff takeoff_service;
    takeoff_service.request.groupMask = 0;
    takeoff_service.request.height = _height;
    takeoff_service.request.duration = ros::Duration( _height / takeoff_vel );

    takeoff_client_.call(takeoff_service);
    ROS_INFO("Taking off!");

    // while (crazyflie_state_.data == 2 && ros::ok()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    ROS_INFO("Flying!");
    calling_takeoff = false;

    // Update state right now!
    this->state_ = guessState();
}

void BackendCrazyflie::land() {
    calling_land = true;

    control_mode_ = eControlMode::LOCAL_POSE;  // Back to control in position (just in case)

    crazyflie_driver::Land land_service;

    takeoff_client_.call(land_service);
    ROS_INFO("Landing!");

    // while (crazyflie_state_.data == 3 && ros::ok()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    ROS_INFO("Landed!");
    calling_land = false;

    // Update state right now!
    this->state_ = guessState();
}

void BackendCrazyflie::setVelocity(const Velocity& _vel) {
    // TODO: WARNING
}

bool BackendCrazyflie::isReady() const {
    return true;
}

void BackendCrazyflie::setPose(const geometry_msgs::PoseStamped& _world) {
    control_mode_ = eControlMode::LOCAL_POSE;  // Control in position
    ref_pose_ = _world;
}

void BackendCrazyflie::goToWaypoint(const Waypoint& _world) {
    // TODO: WARNING
    double go_to_vel = 1.0;
    crazyflie_driver::GoTo go_to_service;
    geometry_msgs::Point goal_point;
    goal_point.x = _world.pose.position.x - init_pose_.pose.position.x;
    goal_point.y = _world.pose.position.y - init_pose_.pose.position.y;
    goal_point.z = _world.pose.position.z - init_pose_.pose.position.z;

    geometry_msgs::Pose pose_to_goal;
    pose_to_goal.position.x = goal_point.x - cur_pose_.pose.position.x;
    pose_to_goal.position.y = goal_point.y - cur_pose_.pose.position.y;
    pose_to_goal.position.z = goal_point.z - cur_pose_.pose.position.z;

    float dist_to_goal = sqrt(pose_to_goal.position.x*pose_to_goal.position.x + pose_to_goal.position.y*pose_to_goal.position.y + pose_to_goal.position.z*pose_to_goal.position.z);

    go_to_service.request.groupMask = 0;
    go_to_service.request.relative = false;
    go_to_service.request.goal = goal_point;
    go_to_service.request.yaw = 0.0;
    go_to_service.request.duration = ros::Duration( dist_to_goal / go_to_vel );

    takeoff_client_.call(go_to_service);
    ROS_INFO("Going to waypoint!");

    // while (crazyflie_state_.data == 3 && ros::ok()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    ROS_INFO("Arrived to waypoint!");
}

void BackendCrazyflie::goToWaypointGeo(const WaypointGeo& _wp) {
    // TODO: WARNING
}

Pose BackendCrazyflie::pose() {
    Pose out;

    out.pose.position.x = cur_pose_.pose.position.x /* + local_start_pos_[0] */;
    out.pose.position.y = cur_pose_.pose.position.y /* + local_start_pos_[1] */;
    out.pose.position.z = cur_pose_.pose.position.z /* + local_start_pos_[2] */;
    out.pose.orientation = cur_pose_.pose.orientation;

    // if (pose_frame_id_ == "") {
    //     // Default: local pose
    //     out.header.frame_id = uav_home_frame_id_;
    // } else {
    //     // Publish pose in different frame
    //     Pose aux = out;
    //     geometry_msgs::TransformStamped transformToPoseFrame;
    //     std::string pose_frame_id_map = "inv_" + pose_frame_id_;

    //     if (cached_transforms_.find(pose_frame_id_map) == cached_transforms_.end()) {
    //         // inv_pose_frame_id_ not found in cached_transforms_
    //         tf2_ros::Buffer tfBuffer;
    //         tf2_ros::TransformListener tfListener(tfBuffer);
    //         transformToPoseFrame = tfBuffer.lookupTransform(pose_frame_id_, uav_home_frame_id_, ros::Time(0), ros::Duration(1.0));
    //         cached_transforms_[pose_frame_id_map] = transformToPoseFrame;  // Save transform in cache
    //     } else {
    //         // found in cache
    //         transformToPoseFrame = cached_transforms_[pose_frame_id_map];
    //     }

    //     tf2::doTransform(aux, out, transformToPoseFrame);
    //     out.header.frame_id = pose_frame_id_;
    // }
    out.header.frame_id = "map";
    out.header.stamp = cur_pose_.header.stamp;
    return out;
}

Pose BackendCrazyflie::referencePose() {
    return ref_pose_;
}

Velocity BackendCrazyflie::velocity() const {
    // TODO: WARNING
    return cur_vel_;
}

Odometry BackendCrazyflie::odometry() const {
    // TODO: WARNING
    Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = uav_home_frame_id_;
    odom.child_frame_id = uav_frame_id_;
    odom.pose.pose.position.x = cur_pose_.pose.position.x /* + local_start_pos_[0] */;
    odom.pose.pose.position.y = cur_pose_.pose.position.y /* + local_start_pos_[1] */;
    odom.pose.pose.position.z = cur_pose_.pose.position.z /* + local_start_pos_[2] */;
    odom.pose.pose.orientation = cur_pose_.pose.orientation;
    odom.twist.twist = cur_vel_.twist;

    return odom;
}

Transform BackendCrazyflie::transform() const {
    // TODO: WARNING
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = uav_home_frame_id_;
    out.child_frame_id = uav_frame_id_;
    out.transform.translation.x = cur_pose_.pose.position.x /* + local_start_pos_[0] */;
    out.transform.translation.y = cur_pose_.pose.position.y /* + local_start_pos_[1] */;
    out.transform.translation.z = cur_pose_.pose.position.z /* + local_start_pos_[2] */;
    if (cur_pose_.pose.orientation.w == 0) {
        // out.transform.rotation = cur_pose_.pose.orientation;
        out.transform.rotation.w = cur_pose_.pose.orientation.w + 1;
    } else {
        out.transform.rotation = cur_pose_.pose.orientation;
    }
    return out;
}

bool BackendCrazyflie::referencePoseReached() {
    double position_min, position_mean, position_max;
    double orientation_min, orientation_mean, orientation_max;
    // if (!position_error_.get_stats(position_min, position_mean, position_max)) { return false; }
    // if (!orientation_error_.get_stats(orientation_min, orientation_mean, orientation_max)) { return false; }

    double position_diff = position_max - position_min;
    double orientation_diff = orientation_max - orientation_min;
    bool position_holds = (position_diff < position_th_) && (fabs(position_mean) < 0.5 * position_th_);
    bool orientation_holds = (orientation_diff < orientation_th_) && (fabs(orientation_mean) < 0.5 * orientation_th_);

    // if (position_holds && orientation_holds) {  // DEBUG
    //     ROS_INFO("position: %f < %f) && (%f < %f)", position_diff, position_th_, fabs(position_mean), 0.5*position_th_);
    //     ROS_INFO("orientation: %f < %f) && (%f < %f)", orientation_diff, orientation_th_, fabs(orientation_mean), 0.5*orientation_th_);
    //     ROS_INFO("Arrived!");
    // }

    return position_holds && orientation_holds;
}

void BackendCrazyflie::initHomeFrame() {
    // local_start_pos_ << 0.0, 0.0, 0.0;

    // // Get frames from rosparam
    ros::param::param<std::string>("~uav_frame", uav_frame_id_, "uav_" + std::to_string(robot_id_));
    ros::param::param<std::string>("~uav_home_frame", uav_home_frame_id_, "uav_" + std::to_string(robot_id_) + "_home");
    std::string parent_frame;
    ros::param::param<std::string>("~home_pose_parent_frame", parent_frame, "map");

    std::vector<double> home_pose(3, 0.0);
    if (ros::param::has("~home_pose")) {
        ros::param::get("~home_pose", home_pose);
    } else if (ros::param::has("~map_origin_geo")) {
        ROS_WARN("Be careful, you should only use this mode with RTK GPS!");
        while (!this->mavros_has_geo_pose_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::vector<double> map_origin_geo(3, 0.0);
        ros::param::get("~map_origin_geo", map_origin_geo);
        geographic_msgs::GeoPoint origin_geo, actual_coordinate_geo;
        origin_geo.latitude = map_origin_geo[0];
        origin_geo.longitude = map_origin_geo[1];
        origin_geo.altitude = 0;  //map_origin_geo[2];
        actual_coordinate_geo.latitude = cur_geo_pose_.latitude;
        actual_coordinate_geo.longitude = cur_geo_pose_.longitude;
        actual_coordinate_geo.altitude = 0;  //cur_geo_pose_.altitude;
        if (map_origin_geo[0] == 0 && map_origin_geo[1] == 0) {
            ROS_WARN("Map origin is set to 0. Define map_origin_geo param by a vector in format [lat,lon,alt].");
        }
        geometry_msgs::Point32 map_origin_cartesian = geographic_to_cartesian(actual_coordinate_geo, origin_geo);

        home_pose[0] = map_origin_cartesian.x;
        home_pose[1] = map_origin_cartesian.y;
        home_pose[2] = map_origin_cartesian.z;
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
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transform_to_map;
        transform_to_map = tfBuffer.lookupTransform(parent_frame, "map", ros::Time(0), ros::Duration(2.0));
        static_transformStamped.transform.rotation = transform_to_map.transform.rotation;
    }

    static_tf_broadcaster_ = new tf2_ros::StaticTransformBroadcaster();
    static_tf_broadcaster_->sendTransform(static_transformStamped);

    init_pose_.pose.position.x = home_pose[0];
    init_pose_.pose.position.y = home_pose[1];
    init_pose_.pose.position.z = home_pose[2];
}

double BackendCrazyflie::updateParam(const std::string& _param_id) {
    // TODO?
}

}  // namespace ual
}  // namespace grvc
