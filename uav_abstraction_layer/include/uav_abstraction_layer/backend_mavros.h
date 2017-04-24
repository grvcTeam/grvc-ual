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

#include <uav_abstraction_layer/backend.h>
#include <argument_parser/argument_parser.h>
//#include <grvc_quadrotor_hal/types.h>  // TODO: types!
#include <functional>
#include <thread>
#include <ros/ros.h>

//Mavros services
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
//#include <grvc_quadrotor_hal/float32.h>

//Mavros messages
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


namespace grvc { namespace ual {

/// Common interface for back end implementations of hal
class BackendMavros : public Backend{
public:
    typedef std::function<void(TaskState)>	StateCallBack;
public:
    BackendMavros(const char* _node_name, int _argc, char** _argv, StateCallBack _scb);

    bool		isReady() const override;
    /// Go to the specified waypoint, following a straight line.
    /// \param _wp goal waypoint.
    void		goToWP(const Waypoint& _wp) override;
    /// Follow a list of waypoints, one after another
    void		trackPath(const WaypointList& _path) override;
    /// Perform a take off maneuver
    /// \param _height targer height that must be reached to consider the take off complete.
    void		takeOff(double _height) override;
    /// Land on the current position.
    void		land() override;
    /// Set velocities
    void        setVelocity(const Velocity& _vel) override;
    /// Set position error
    void		setPositionError(const Vec3& _pos_error) override;

    /// Cancel execution of the current task
    void		abort() override;
    /// Latest pose estimation of the robot
    Pose	    pose() const override;

private:
    void parseArguments(int _argc, char** _argv);
    void startRosCommunications();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr&);
    void stateCallback(const mavros_msgs::State::ConstPtr&);
    void update();
    bool reachedGoal() const;
    void initLocalCoordMatrix();
    void arm();
    // void setLandMode();
    // void setOffBoardMode();
    void setFlightMode(const std::string& _flight_mode, bool _stubborn = false);

    // bool controlVelPgainServiceCallback(grvc_quadrotor_hal::float32::Request  &req, grvc_quadrotor_hal::float32::Response &res);
    // bool controlVelKiServiceCallback(grvc_quadrotor_hal::float32::Request  &req, grvc_quadrotor_hal::float32::Response &res);
    // bool controlVelKdServiceCallback(grvc_quadrotor_hal::float32::Request  &req, grvc_quadrotor_hal::float32::Response &res);
private:
    utils::ArgumentParser args_;
    WaypointList path_;
    geometry_msgs::PoseStamped home_pose_;
    geometry_msgs::PoseStamped ref_pose_;
    geometry_msgs::PoseStamped cur_pose_;
    geometry_msgs::TwistStamped ref_vel_;
    mavros_msgs::State mavros_state_;

    ros::NodeHandle nh_;

    //Control
    bool mavros_has_pose_ = false;
    bool control_in_vel_ = false;
    Vec3 integral_control_vel_ = {0,0,0};
    Vec3 previous_error_control_vel_ = {0,0,0};
    float p_gain_xy_ = 0.4;  // TODO: PID? Tune!
    float k_i_xy_ = 0.07;
    float k_d_xy_ = 0.0;
    float p_gain_z_ = 0.4;  // TODO: PID? Tune!
    float k_i_z_ = 0.05;
    float k_d_z_ = 0.0;


    /// Ros Communication
    ros::ServiceClient flight_mode_client_;
    ros::ServiceClient arming_client_;
    ros::Publisher mavros_ref_pose_pub_;
    ros::Publisher mavros_ref_vel_pub_;
    ros::Subscriber mavros_cur_pose_sub_;
    ros::Subscriber mavros_cur_state_sub_;
    // ros::ServiceServer control_vel_p_gain_;
    // ros::ServiceServer control_vel_k_i_;
    // ros::ServiceServer control_vel_k_d_;
    // ros::Publisher control_vel_params_pub_;

    unsigned int robot_id_;
    std::string mavros_ns_;
    // std::string set_mode_srv_;
    // std::string arming_srv_;
    // std::string set_pose_topic_;
    // std::string set_vel_topic_;
    // std::string pose_topic_;
    // std::string state_topic_;
    // std::string control_vel_p_gain_srv_;
    // std::string control_vel_k_i_srv_;
    // std::string control_vel_k_d_srv_;
    // std::string control_vel_params_pub_topic_;

    // Eigen::Matrix4d localTransform;
    // utils::Cartesian3d localStartPos;

    std::thread offboardThread_;
    // std::thread control_vel_params_pub_Thread_;
    bool mAborting = false;
		};

	}
}	// namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_BACKEND_MAVROS_H
