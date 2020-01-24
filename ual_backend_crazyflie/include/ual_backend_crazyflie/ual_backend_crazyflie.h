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
#ifndef UAV_ABSTRACTION_LAYER_BACKEND_CRAZYFLIE_H
#define UAV_ABSTRACTION_LAYER_BACKEND_CRAZYFLIE_H

#include <thread>
#include <vector>

#include <uav_abstraction_layer/backend.h>
#include <ros/ros.h>

#include <crazyflie_driver/Takeoff.h>
#include <crazyflie_driver/Land.h>
#include <crazyflie_driver/GoTo.h>
#include <crazyflie_driver/AddCrazyflie.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>

namespace grvc { namespace ual {

class BackendCrazyflie : public Backend {

public:
    BackendCrazyflie();
    ~BackendCrazyflie();

    /// Backend is initialized and ready to run tasks?
    bool	         isReady() const override;
    /// Latest pose estimation of the robot
    virtual Pose	 pose() override;
    /// Latest velocity estimation of the robot
    virtual Velocity velocity() const override;
    /// Latest odometry estimation of the robot
    virtual Odometry odometry() const override;
    /// Latest transform estimation of the robot
    virtual Transform transform() const override;
    /// Current reference pose
    virtual Pose referencePose() override;

    /// Set pose
    /// \param _pose target pose
    void    setPose(const geometry_msgs::PoseStamped& _pose) override;

    /// Go to the specified waypoint, following a straight line
    /// \param _wp goal waypoint
    void	goToWaypoint(const Waypoint& _wp) override;

    /// Go to the specified waypoint in geographic coordinates, following a straight line
    /// \param _wp goal waypoint in geographic coordinates
    void	goToWaypointGeo(const WaypointGeo& _wp);

    /// Perform a take off maneuver
    /// \param _height target height that must be reached to consider the take off complete
    void    takeOff(double _height) override;
    /// Land on the current position.
    void	land() override;
    /// Set velocities
    /// \param _vel target velocity in world coordinates
    void    setVelocity(const Velocity& _vel) override;
    /// Recover from manual flight mode
    /// Use it when FLYING uav is switched to manual mode and want to go BACK to auto.
    void    recoverFromManual() override;
    /// Set home position
    void    setHome(bool set_z) override;

private:
    void offboardThreadLoop();
    void initHomeFrame();
    bool referencePoseReached();
    void setFlightMode(const std::string& _flight_mode);
    double updateParam(const std::string& _param_id);
    State guessState();

    geometry_msgs::PoseStamped  ref_pose_;
    sensor_msgs::NavSatFix      ref_pose_global_;
    geometry_msgs::PoseStamped  cur_pose_;
    sensor_msgs::NavSatFix      cur_geo_pose_;
    geometry_msgs::PoseStamped  init_pose_;
    geometry_msgs::TwistStamped ref_vel_;
    geometry_msgs::TwistStamped cur_vel_;
    std_msgs::Int8              crazyflie_state_;

    //Control
    enum class eControlMode {LOCAL_VEL, LOCAL_POSE, GLOBAL_POSE};
    eControlMode control_mode_ = eControlMode::LOCAL_POSE;
    bool cf_has_pose_ = false;
    bool mavros_has_geo_pose_ = false;
    float position_th_;
    float orientation_th_;

    /// Ros Communication
    ros::ServiceClient flight_mode_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient get_param_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient go_to_client_;
    ros::Publisher crazyflie_ref_pose_pub_;
    ros::Publisher crazyflie_ref_pose_global_pub_;
    ros::Publisher crazyflie_ref_vel_pub_;
    ros::Subscriber crazyflie_cur_pose_sub_;
    ros::Subscriber crazyflie_cur_geo_pose_sub_;
    ros::Subscriber crazyflie_cur_vel_sub_;
    ros::Subscriber crazyflie_cur_state_sub_;
    ros::Subscriber crazyflie_cur_extended_state_sub_;

    int robot_id_;
    std::string pose_frame_id_;
    std::string uav_home_frame_id_;
    std::string uav_frame_id_;
    tf2_ros::StaticTransformBroadcaster * static_tf_broadcaster_;
    std::map <std::string, geometry_msgs::TransformStamped> cached_transforms_;
    std::map<std::string, double> mavros_params_;
    ros::Time last_command_time_;

    std::thread offboard_thread_;
    double offboard_thread_frequency_;

    bool calling_takeoff = false;
    bool calling_land = false;
};

}}	// namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_BACKEND_CRAZYFLIE_H
