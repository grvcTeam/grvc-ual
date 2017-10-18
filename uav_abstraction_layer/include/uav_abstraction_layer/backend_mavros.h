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
#ifndef UAV_ABSTRACTION_LAYER_BACKEND_MAVROS_H
#define UAV_ABSTRACTION_LAYER_BACKEND_MAVROS_H

#include <thread>
#include <Eigen/Core>

#include <uav_abstraction_layer/backend.h>
#include <argument_parser/argument_parser.h>
#include <handy_tools/pid_controller.h>
#include <ros/ros.h>

//Mavros services
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

//Mavros messages
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace grvc { namespace ual {

class BackendMavros : public Backend {

public:
    BackendMavros(grvc::utils::ArgumentParser& _args);

    /// Backend is initialized and ready to run tasks?
    bool	         isReady() const override;
    /// Latest pose estimation of the robot
    virtual Pose	 pose() override;
    /// Latest velocity estimation of the robot
    virtual Velocity velocity() const override;
    /// Latest transform estimation of the robot
    virtual Transform transform() const override;

    /// Go to the specified waypoint, following a straight line
    /// \param _wp goal waypoint
    void	goToWaypoint(const Waypoint& _wp) override;

    /// Go to the specified waypoint in geographic coordinates, following a straight line
    /// \param _wp goal waypoint in geographic coordinates
    void	goToWaypointGeo(const WaypointGeo& _wp);

    /// Follow a list of waypoints, one after another
    // void trackPath(const Path& _path) override;
    /// Perform a take off maneuver
    /// \param _height target height that must be reached to consider the take off complete
    void    takeOff(double _height) override;
    /// Land on the current position.
    void	land() override;
    /// Set velocities
    /// \param _vel target velocity in world coordinates
    void    setVelocity(const Velocity& _vel) override;
    /// Set position error control
    /// \param _pos_error position error in world coordinates
    void	setPositionError(const PositionError& _pos_error) override;
    /// Recover from manual flight mode
    /// Use it when FLYING uav is switched to manual mode and want to go BACK to auto.
    void    recoverFromManual() override;

private:
    void offboardThreadLoop();
    void arm();
    void initHomeFrame();
    bool referencePoseReached() const;
    void setFlightMode(const std::string& _flight_mode);

    //WaypointList path_;
    geometry_msgs::PoseStamped home_pose_;
    geometry_msgs::PoseStamped ref_pose_;
    sensor_msgs::NavSatFix     ref_pose_global_;
    geometry_msgs::PoseStamped cur_pose_;
    geometry_msgs::TwistStamped ref_vel_;
    geometry_msgs::TwistStamped cur_vel_;
    mavros_msgs::State mavros_state_;

    //Control
    enum class eControlMode {LOCAL_VEL, LOCAL_POSE, GLOBAL_POSE};
    bool mavros_has_pose_ = false;
    eControlMode control_mode_ = eControlMode::LOCAL_POSE;
    grvc::utils::PidController* pid_x_;
    grvc::utils::PidController* pid_y_;
    grvc::utils::PidController* pid_z_;

    /// Ros Communication
    ros::ServiceClient flight_mode_client_;
    ros::ServiceClient arming_client_;
    ros::Publisher mavros_ref_pose_pub_;
    ros::Publisher mavros_ref_pose_global_pub_;
    ros::Publisher mavros_ref_vel_pub_;
    ros::Subscriber mavros_cur_pose_sub_;
    ros::Subscriber mavros_cur_vel_sub_;
    ros::Subscriber mavros_cur_state_sub_;

    unsigned int robot_id_;
    std::string pose_frame_id_;
    std::string uav_home_frame_id_;
    tf2_ros::StaticTransformBroadcaster * static_tf_broadcaster_;
    std::map <std::string, geometry_msgs::TransformStamped> cached_transforms_;
    Eigen::Vector3d local_start_pos_;

    std::thread offboard_thread_;
};

}}	// namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_BACKEND_MAVROS_H
