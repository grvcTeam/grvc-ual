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
#ifndef UAV_ABSTRACTION_LAYER_BACKEND_MAVROS_FW_H
#define UAV_ABSTRACTION_LAYER_BACKEND_MAVROS_FW_H

#include <thread>
#include <vector>
#include <Eigen/Core>
#include <ros/ros.h>

#include <uav_abstraction_layer/backend.h>
#include <uav_abstraction_layer/posePID.h>

//Mavros services
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

//Mavros messages
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPoseStamped.h>

//UAL messages (TRASLADAR A ESTE BACKEND?)
#include <uav_abstraction_layer/MissionElement.h>
#include <uav_abstraction_layer/ParamFloat.h>

namespace grvc { namespace ual {

class BackendMavrosFW : public Backend {

public:
    BackendMavrosFW();
    ~BackendMavrosFW();

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
    /// Execute mission of a sequence of waypoints
    void	setMission(const std::vector<uav_abstraction_layer::MissionElement>& _waypoint_element_list) override;
    /// Set velocities
    /// \param _vel target velocity in world coordinates
    void    setVelocity(const Velocity& _vel) override;
    /// Recover from manual flight mode
    /// Use it when FLYING uav is switched to manual mode and want to go BACK to auto.
    void    recoverFromManual() override;
    /// Set home position
    void    setHome(bool set_z) override;

private:
    void missionThreadLoop();
    void getAutopilotInformation();
    void initHomeFrame();
    // void initPosePID();
    bool referencePoseReached();
    void setFlightMode(const std::string& _flight_mode);
    double updateParam(const std::string& _param_id);
    State guessState();
    // bool takeOffPX4(double _height);
    // bool takeOffAPM(double _height);
    // void goToWaypointPX4(const Waypoint& _wp);
    // void goToWaypointAPM(const Waypoint& _wp);

    // FW specifics
    void arm(const bool& _arm);
    void setParam(const std::string& _param_id,const int& _param_value);
    bool pushMission(const mavros_msgs::WaypointList& _wp_list);
    void clearMission();
    void addTakeOffWp(mavros_msgs::WaypointList& _wp_list, const uav_abstraction_layer::MissionElement& _waypoint_element, const int& wp_set_index);
    void addPassWpList(mavros_msgs::WaypointList& _wp_list, const uav_abstraction_layer::MissionElement& _waypoint_element, const int& wp_set_index);
    void addLoiterWpList(mavros_msgs::WaypointList& _wp_list, const uav_abstraction_layer::MissionElement& _waypoint_element, const int& wp_set_index);
    void addLandWpList(mavros_msgs::WaypointList& _wp_list, const uav_abstraction_layer::MissionElement& _waypoint_element, const int& wp_set_index);
    void addSpeedWpList(mavros_msgs::WaypointList& _wp_list, const uav_abstraction_layer::MissionElement& _waypoint_element, const int& wp_set_index);
    std::vector<geographic_msgs::GeoPoseStamped> uniformizeSpatialField( const uav_abstraction_layer::MissionElement& _waypoint_element);
    geographic_msgs::GeoPoseStamped poseStampedtoGeoPoseStamped(const geometry_msgs::PoseStamped& _posestamped );
    geometry_msgs::PoseStamped geoPoseStampedtoPoseStamped(const geographic_msgs::GeoPoseStamped _geoposestamped );
    mavros_msgs::Waypoint geoPoseStampedtoGlobalWaypoint(const geographic_msgs::GeoPoseStamped& _geoposestamped );
    float getMissionYaw(const geometry_msgs::Quaternion& quat);
    void checkMissionParams(const std::map<std::string, float>& existing_params_map, const std::vector<std::string>& required_params, const int& wp_set_index);
    void initMission();
    
    //WaypointList path_;
    geometry_msgs::PoseStamped  ref_pose_;
    sensor_msgs::NavSatFix      ref_pose_global_;
    geometry_msgs::PoseStamped  cur_pose_;
    sensor_msgs::NavSatFix      cur_geo_pose_;
    geometry_msgs::TwistStamped ref_vel_;
    geometry_msgs::TwistStamped cur_vel_;
    geometry_msgs::TwistStamped cur_vel_body_;
    mavros_msgs::State          mavros_state_;
    mavros_msgs::ExtendedState  mavros_extended_state_;

    //Mission
    int  mavros_reached_wp_;
    mavros_msgs::WaypointList  mavros_cur_mission_;
    geographic_msgs::GeoPoint origin_geo_;
    std::vector<int> takeoff_wps_on_mission_;
    std::vector<int> land_wps_on_mission_;
    float mission_aux_height_;
    float mission_aux_distance_;
    float mission_takeoff_minimum_pitch_;
    float mission_loit_heading_;
    float mission_loit_radius_;
    float mission_loit_forward_moving_;
    float mission_land_precision_mode_;
    float mission_land_abort_alt_;
    float mission_pass_orbit_distance_;
    float mission_pass_acceptance_radius_;
    
    /// Possible mission waypoint types
    enum MissionElementType {
        TAKEOFF_POSE,
        TAKEOFF_AUX,
        PASS,
        LOITER_UNLIMITED,
        LOITER_TURNS,
        LOITER_TIME,
        LOITER_HEIGHT,
        LAND_POSE,
        LAND_AUX,
    };
    
    //Control
    enum class eControlMode {LOCAL_VEL, LOCAL_POSE, GLOBAL_POSE, NONE};
    eControlMode control_mode_ = eControlMode::NONE;
    bool mavros_has_pose_ = false;
    bool mavros_has_geo_pose_ = false;
    float position_th_;
    float orientation_th_;
    float hold_pose_time_;
    PosePID *pose_pid_;
    bool is_pose_pid_enabled_ = false;

    /// Ros Communication
    ros::ServiceClient flight_mode_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient get_param_client_;
    ros::ServiceClient set_param_client_;
    ros::ServiceClient push_mission_client_;
    ros::ServiceClient clear_mission_client_;
    ros::Publisher mavros_ref_pose_pub_;
    ros::Publisher mavros_ref_pose_global_pub_;
    ros::Publisher mavros_ref_vel_pub_;
    ros::Subscriber mavros_cur_pose_sub_;
    ros::Subscriber mavros_cur_geo_pose_sub_;
    ros::Subscriber mavros_cur_vel_sub_;
    ros::Subscriber mavros_cur_vel_body_sub_;
    ros::Subscriber mavros_cur_state_sub_;
    ros::Subscriber mavros_cur_extended_state_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber mavros_reached_wp_sub_;
    ros::Subscriber mavros_cur_mission_sub_;

    int robot_id_;
    enum struct AutopilotType {PX4, APM, UNKNOWN};
    AutopilotType autopilot_type_ = AutopilotType::UNKNOWN;
    std::string pose_frame_id_;
    std::string uav_home_frame_id_;
    std::string uav_frame_id_;
    tf2_ros::StaticTransformBroadcaster * static_tf_broadcaster_;
    std::map <std::string, geometry_msgs::TransformStamped> cached_transforms_;
    std::map<std::string, double> mavros_params_;
    Eigen::Vector3d local_start_pos_;
    ros::Time last_command_time_;

    std::thread mission_thread_;
    double mission_thread_frequency_;

    bool calling_takeoff_ = false;
    bool calling_land_ = false;
};

}}	// namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_BACKEND_MAVROS_FW_H
