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
#ifndef UAV_ABSTRACTION_LAYER_BACKEND_LIGHT_H
#define UAV_ABSTRACTION_LAYER_BACKEND_LIGHT_H

#include <thread>
#include <random>

#include <uav_abstraction_layer/backend.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace grvc { namespace ual {

class BackendGazeboLight : public Backend {

public:
    BackendGazeboLight();
    ~BackendGazeboLight();

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
    void	goToWaypointGeo(const WaypointGeo& _wp) override;
    
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
    void    recoverFromManual() override {}
    /// Set home position
    void    setHome(bool set_z) override {}

private:
    void initHomeFrame();
    bool referencePoseReached();
    void move();
    Velocity calculateRefVel(Pose _target_pose);

    geometry_msgs::PoseStamped home_pose_;
    geometry_msgs::PoseStamped ref_pose_;
    geometry_msgs::PoseStamped cur_pose_;
    geometry_msgs::PoseStamped cur_pose_noisy_;
    geometry_msgs::PoseStamped gazebo_pose_;
    geometry_msgs::TwistStamped ref_vel_;
    geometry_msgs::TwistStamped cur_vel_;

    //Gazebo animated model
    std::string model_name_;
    bool has_pose_ = false;
    geometry_msgs::Pose model_pose_;
    ros::Publisher model_state_publisher_;
    ros::Subscriber model_state_subscriber_;
    ros::ServiceClient wrench_client_;
    ros::Timer rotors_timer_;

    //Noise
    std::default_random_engine generator_;
    std::normal_distribution<double>* distribution_;

    //Control
    bool flying_ = false;
    bool control_in_vel_ = false;
    float max_horizontal_velocity_;
    float max_vertical_velocity_;
    float max_yaw_rate_;
    float max_position_error_;
    float max_orientation_error_;
    float position_th_;
    float orientation_th_;
    // float hold_pose_time_;  // TODO: add?

    int robot_id_;
    std::string pose_frame_id_;
    std::string uav_home_frame_id_;
    std::string uav_frame_id_;
    tf2_ros::StaticTransformBroadcaster * static_tf_broadcaster_;
    std::map <std::string, geometry_msgs::TransformStamped> cached_transforms_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Time last_command_time_;

    std::thread offboard_thread_;
};

}}	// namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_BACKEND_LIGHT_H
