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
#include <ual_backend_mavros_fw/ual_backend_mavros_fw.h>
#include <Eigen/Eigen>
#include <ros/package.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <uav_abstraction_layer/geographic_to_cartesian.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/VehicleInfoGet.h>
#include <mavros/utils.h>

#include <uav_abstraction_layer/ParamFloat.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>

#define FIRMWARE_VERSION_TYPE_DEV 0 /* development release | */
#define FIRMWARE_VERSION_TYPE_ALPHA 64 /* alpha release | */
#define FIRMWARE_VERSION_TYPE_BETA 128 /* beta release | */
#define FIRMWARE_VERSION_TYPE_RC 192 /* release candidate | */
#define FIRMWARE_VERSION_TYPE_OFFICIAL 255 /* official stable release | */
#define FIRMWARE_VERSION_TYPE_ENUM_END 256 /*  | */

namespace grvc { namespace ual {

BackendMavrosFW::BackendMavrosFW()
    : Backend(), tf_listener_(tf_buffer_)
{
    // Parse arguments
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");

    pnh.param<float>("mission/aux_distance", mission_aux_distance_, 100.0);
    pnh.param<float>("mission/aux_height", mission_aux_height_, 10.0);
    pnh.param<float>("mission/takeoff_minimum_pitch", mission_takeoff_minimum_pitch_, 5.0);
    pnh.param<float>("mission/loit_heading", mission_loit_heading_, 0.0);
    pnh.param<float>("mission/loit_radius", mission_loit_radius_, 50.0);
    pnh.param<float>("mission/loit_forward_moving", mission_loit_forward_moving_, 1.0);
    pnh.param<float>("mission/land_precision_mode", mission_land_precision_mode_, 0.0);
    pnh.param<float>("mission/land_abort_alt", mission_land_abort_alt_, 0.0);
    pnh.param<float>("mission/pass_orbit_distance", mission_pass_orbit_distance_, 10.0);
    pnh.param<float>("mission/pass_acceptance_radius", mission_pass_acceptance_radius_, 50.0);


    // Init variables
    cur_pose_.pose.orientation.x = 0;
    cur_pose_.pose.orientation.y = 0;
    cur_pose_.pose.orientation.z = 0;
    cur_pose_.pose.orientation.w = 1;

    ROS_INFO("BackendMavrosFW constructor with id [%d]", robot_id_);
    // ROS_INFO("BackendMavrosFW: thresholds = %f %f", position_th_, orientation_th_);

    // Init ros communications
    ros::NodeHandle nh;
    std::string mavros_ns = "mavros";
    std::string set_mode_srv = mavros_ns + "/set_mode";
    std::string arming_srv = mavros_ns + "/cmd/arming";
    std::string get_param_srv = mavros_ns + "/param/get";
    std::string set_param_srv = mavros_ns + "/param/set";
    std::string push_mission_srv = mavros_ns + "/mission/push";
    std::string clear_mission_srv = mavros_ns + "/mission/clear";
    std::string set_pose_topic = mavros_ns + "/setpoint_position/local";
    std::string set_pose_global_topic = mavros_ns + "/setpoint_raw/global";
    std::string set_vel_topic = mavros_ns + "/setpoint_velocity/cmd_vel";
    std::string pose_topic = mavros_ns + "/local_position/pose";
    std::string geo_pose_topic = mavros_ns + "/global_position/global";
#ifdef MAVROS_VERSION_BELOW_0_29_0
    std::string vel_topic_local = mavros_ns + "/local_position/velocity";
#else
    std::string vel_topic_local = mavros_ns + "/local_position/velocity_local";
#endif
    std::string vel_topic_body = mavros_ns + "/local_position/velocity_body";
    std::string state_topic = mavros_ns + "/state";
    std::string extended_state_topic = mavros_ns + "/extended_state";
    std::string reached_mission_topic = mavros_ns + "/mission/reached";
    std::string waypoints_mission_topic = mavros_ns + "/mission/waypoints";

    flight_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(set_mode_srv.c_str());
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(arming_srv.c_str());
    // Client to push missions to mavros
    push_mission_client_ = nh.serviceClient<mavros_msgs::WaypointPush>(push_mission_srv.c_str());
    // Client to clear the mission on mavros
    clear_mission_client_ = nh.serviceClient<mavros_msgs::WaypointClear>(clear_mission_srv.c_str());
    // Client to set parameters from mavros
    set_param_client_ = nh.serviceClient<mavros_msgs::ParamSet>(set_param_srv.c_str());

    mavros_ref_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(set_pose_topic.c_str(), 1);
    mavros_ref_pose_global_pub_ = nh.advertise<mavros_msgs::GlobalPositionTarget>(set_pose_global_topic.c_str(), 1);
    mavros_ref_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>(set_vel_topic.c_str(), 1);

    mavros_cur_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic.c_str(), 1, \
        [this](const geometry_msgs::PoseStamped::ConstPtr& _msg) {
            this->cur_pose_ = *_msg;
            this->mavros_has_pose_ = true;
            if (is_pose_pid_enabled_) {
                ref_vel_ = pose_pid_->update(cur_pose_);
            }
    });
    mavros_cur_vel_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(vel_topic_local.c_str(), 1, \
        [this](const geometry_msgs::TwistStamped::ConstPtr& _msg) {
            this->cur_vel_ = *_msg;
            this->cur_vel_.header.frame_id = this->uav_home_frame_id_;
#ifdef MAVROS_VERSION_BELOW_0_29_0
            this->cur_vel_body_ = *_msg;
            this->cur_vel_body_.header.frame_id = this->uav_frame_id_;
#endif
    });
    mavros_cur_vel_body_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(vel_topic_body.c_str(), 1, \
        [this](const geometry_msgs::TwistStamped::ConstPtr& _msg) {
            this->cur_vel_body_ = *_msg;
            this->cur_vel_body_.header.frame_id = this->uav_frame_id_;
    });
    mavros_cur_geo_pose_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(geo_pose_topic.c_str(), 1, \
        [this](const sensor_msgs::NavSatFix::ConstPtr& _msg) {
            this->cur_geo_pose_ = *_msg;
            if (!this->mavros_has_geo_pose_) {
                if (_msg->position_covariance[0] < 1.2 && _msg->position_covariance[0] > 0 && _msg->header.seq > 100) {
                    this->mavros_has_geo_pose_ = true;
                    // ROS_INFO("Has Geo Pose! %f",_msg->position_covariance[0]);
                }
            }
    });
    mavros_cur_state_sub_ = nh.subscribe<mavros_msgs::State>(state_topic.c_str(), 1, \
        [this](const mavros_msgs::State::ConstPtr& _msg) {
            this->mavros_state_ = *_msg;
    });
    mavros_cur_extended_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>(extended_state_topic.c_str(), 1, \
        [this](const mavros_msgs::ExtendedState::ConstPtr& _msg) {
            this->mavros_extended_state_ = *_msg;
    });

    mavros_reached_wp_sub_ = nh.subscribe<mavros_msgs::WaypointReached>(reached_mission_topic.c_str(), 1, \
        [this](const mavros_msgs::WaypointReached::ConstPtr& _msg) {
            this->mavros_reached_wp_ = _msg->wp_seq;
    });

    mavros_cur_mission_sub_ = nh.subscribe<mavros_msgs::WaypointList>(waypoints_mission_topic.c_str(), 1, \
        [this](const mavros_msgs::WaypointList::ConstPtr& _msg) {
            this->mavros_cur_mission_ = *_msg;
            mission_state_ = this->mavros_cur_mission_.current_seq;
    });

    // Wait until mavros is connected
    while (!mavros_state_.connected && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    getAutopilotInformation();

    // TODO: Check this and solve frames issue
    initHomeFrame();


    // Thread  for mission mode
    mission_thread_ = std::thread(&BackendMavrosFW::missionThreadLoop, this);

    // Client to get parameters from mavros and required default values
    get_param_client_ = nh.serviceClient<mavros_msgs::ParamGet>(get_param_srv.c_str());
    mavros_params_["NAV_DLL_ACT"]   =   1;  // [?]   Default value
    mavros_params_["MIS_DIST_1PS"]   =   900;  // [m]   Default value
    mavros_params_["MIS_DIST_WPS"]   =   900;  // [m]   Default value
    // Updating here is non-sense as service seems to be slow in waking up

    initMission();

    ROS_INFO("BackendMavrosFW [%d] running!", robot_id_);
}

BackendMavrosFW::~BackendMavrosFW() {
    if (mission_thread_.joinable()) { mission_thread_.join(); }
}

void BackendMavrosFW::missionThreadLoop(){

    ros::param::param<double>("~mavros_mission_rate", mission_thread_frequency_, 30.0);
    ros::Rate rate(mission_thread_frequency_);

    while (ros::ok()) {

        if(this->mavros_state_.mode == "AUTO.MISSION"){ 
            if (std::find(takeoff_wps_on_mission_.begin(), takeoff_wps_on_mission_.end(), mavros_cur_mission_.current_seq) != takeoff_wps_on_mission_.end()){
                calling_takeoff_ = true;
            }
            else if (std::find(land_wps_on_mission_.begin(), land_wps_on_mission_.end(), mavros_cur_mission_.current_seq) != land_wps_on_mission_.end()){
                calling_land_ = true;
            }
            else {
                calling_takeoff_ = false;
                calling_land_ = false;
            }
        }
        
        // State update
        this->state_ = guessState();

        rate.sleep();
    }
}

grvc::ual::State BackendMavrosFW::guessState() {
    // Sequentially checks allow state deduction
    if (!this->isReady()) { return uav_abstraction_layer::State::UNINITIALIZED; }
    if (!this->mavros_state_.armed) { return uav_abstraction_layer::State::LANDED_DISARMED; }
    if (this->mavros_extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND ||
        this->mavros_state_.system_status == 3) { return uav_abstraction_layer::State::LANDED_ARMED; }  // TODO(franreal): Use LANDED_STATE_IN_AIR instead?
    if (this->calling_takeoff_) { return uav_abstraction_layer::State::TAKING_OFF; }
    if (this->calling_land_) { return uav_abstraction_layer::State::LANDING; }
    if (this->mavros_state_.mode == "OFFBOARD" || this->mavros_state_.mode == "GUIDED" || this->mavros_state_.mode == "GUIDED_NOGPS" || this->mavros_state_.mode == "AUTO.MISSION") { return uav_abstraction_layer::State::FLYING_AUTO; }
    return uav_abstraction_layer::State::FLYING_MANUAL;
}

void BackendMavrosFW::initMission() {

    setFlightMode("AUTO.LAND");
    arm(false); 
    arm(true);      //TODO(Jose Andres): Should not arm in this node, but otherwise, swithc to AUTO.MISSION won't be allowed
    setParam("NAV_DLL_ACT",0); // To switch mode
    setParam("MIS_DIST_1WP",0);
    setParam("MIS_DIST_WPS",0); // Minimum distance between wps. default 900m
}

void BackendMavrosFW::setFlightMode(const std::string& _flight_mode) {
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

void BackendMavrosFW::arm(const bool& _arm) {
    mavros_msgs::CommandBool arm_service;
    arm_service.request.value = _arm;

    // Set mode: unabortable?
    while (mavros_state_.armed != _arm && ros::ok()) {
        if (!arming_client_.call(arm_service)) {
            ROS_ERROR("Error in [%s] service calling!", _arm ? "arming" : "disarming");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set [%s] response.success = %s", _arm ? "armed" : "disarmed", \
            arm_service.response.success ? "true" : "false");
#else
        // ROS_INFO("Set [%s] response.success = %s", _arm ? "armed" : "disarmed", \
        //     arm_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("  Trying to [%s]; mavros_state_.armed = [%s]", _arm ? "arm" : "disarm", mavros_state_.armed ? "true" : "false");
    }
}

void BackendMavrosFW::recoverFromManual() {
    if (!mavros_state_.armed || mavros_state_.system_status != 4 ||
        ( (autopilot_type_==AutopilotType::PX4) ? (mavros_extended_state_.landed_state !=
        mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR) : false) ) {
        ROS_WARN("Unable to recover from manual mode (not flying!)");
        return;
    }

    if (mavros_state_.mode != "POSCTL" &&
        mavros_state_.mode != "ALTCTL" &&
        mavros_state_.mode != "STABILIZED" &&
        mavros_state_.mode != "STABILIZE" &&
        mavros_state_.mode != "POSITION" &&
        mavros_state_.mode != "LOITER" &&
        mavros_state_.mode != "ALT_HOLD") {
        ROS_WARN("Unable to recover from manual mode (not in manual!)");
        return;
    }

    // Set mode to OFFBOARD and state to FLYING
    ref_pose_ = cur_pose_;
    control_mode_ = eControlMode::LOCAL_POSE;
    switch (autopilot_type_) {
        case AutopilotType::PX4:
            setFlightMode("OFFBOARD");
            break;
        case AutopilotType::APM:
            setFlightMode("GUIDED");
            break;
        default:
            ROS_ERROR("BackendMavrosFW [%d]: Wrong autopilot type", robot_id_);
            return;
    }

    ROS_INFO("Recovered from manual mode!");
}

void BackendMavrosFW::setHome(bool set_z) {
    double z_offset = set_z ? cur_pose_.pose.position.z : 0.0;
    local_start_pos_ = -Eigen::Vector3d(cur_pose_.pose.position.x, \
        cur_pose_.pose.position.y, z_offset);
}

void BackendMavrosFW::takeOff(double _height) {

    if (_height < 0.0) {
        ROS_ERROR("Takeoff height must be positive!");
        return;
    }
    calling_takeoff_ = true;
    bool takeoff_result = false;

    double yaw = tf2::getYaw(cur_pose_.pose.orientation);

    std::vector<MissionElement> new_mission;

    MissionElement takeoff_waypoint_element;

    takeoff_waypoint_element.type = MissionElement::TAKEOFF_AUX;
    uav_abstraction_layer::ParamFloat param; param.name = "aux_distance"; param.value = mission_aux_distance_;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "aux_angle"; param.value = yaw;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "aux_height"; param.value = _height;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "minimum_pitch"; param.value = mission_takeoff_minimum_pitch_;
    takeoff_waypoint_element.params.push_back( param );
    new_mission.push_back(takeoff_waypoint_element);

    MissionElement loiter_waypoint_element;
    loiter_waypoint_element.type = MissionElement::LOITER_UNLIMITED;
    geometry_msgs::PoseStamped loiter_posestamped;
    loiter_posestamped.pose.position.x = pose().pose.position.x;
    loiter_posestamped.pose.position.y = pose().pose.position.y;
    loiter_posestamped.pose.position.z = _height;
    loiter_waypoint_element.waypoints.push_back(loiter_posestamped);
    param.name = "radius"; param.value = 20;
    loiter_waypoint_element.params.push_back( param );
    param.name = "yaw_angle"; param.value = yaw;
    loiter_waypoint_element.params.push_back( param );
    new_mission.push_back(loiter_waypoint_element);

    setMission(new_mission);

    takeoff_result = false; //TODO(JoseAndres): For further control, I do not eliminate this variable

    if (takeoff_result) {
        ROS_INFO("[%d]: Flying!", robot_id_);
    }
    calling_takeoff_ = false;

    // Update state right now!
    this->state_ = guessState();
}

void BackendMavrosFW::land() {

    calling_land_ = true;
    is_pose_pid_enabled_ = false;

    std::vector<MissionElement> new_mission;

    MissionElement takeoff_waypoint_element;

    double yaw = tf2::getYaw(cur_pose_.pose.orientation);

    takeoff_waypoint_element.type = MissionElement::LAND_AUX;
    geometry_msgs::PoseStamped land_posestamped;
    land_posestamped.pose.position.x = cur_pose_.pose.position.x;
    land_posestamped.pose.position.y = cur_pose_.pose.position.y;
    land_posestamped.pose.position.z = 0.0;
    takeoff_waypoint_element.waypoints.push_back(land_posestamped);
    uav_abstraction_layer::ParamFloat param; param.name = "loit_heading"; param.value = mission_loit_heading_;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "loit_radius"; param.value = mission_loit_radius_;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "loit_forward_moving"; param.value = mission_loit_forward_moving_;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "precision_mode"; param.value = mission_land_precision_mode_;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "abort_alt"; param.value = mission_land_abort_alt_;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "yaw_angle"; param.value = yaw;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "aux_distance"; param.value = mission_aux_distance_;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "aux_angle"; param.value = yaw;
    takeoff_waypoint_element.params.push_back( param );
    param.name = "aux_height"; param.value = mission_aux_height_;
    takeoff_waypoint_element.params.push_back( param );
    new_mission.push_back(takeoff_waypoint_element);

    ROS_INFO("Landing...");
    setMission(new_mission);

    calling_land_ = false;

    // Update state right now!
    this->state_ = guessState();
}

void BackendMavrosFW::setVelocity(const Velocity& _vel) {
    ROS_WARN("setVelocity command is not supported by the Mavros Fixed Wing backend!");
}

bool BackendMavrosFW::isReady() const {
    if (ros::param::has("~map_origin_geo")) {
        return mavros_has_geo_pose_;
    } else {
        return mavros_has_pose_ && (fabs(this->cur_pose_.pose.position.y) > 1e-8);  // Means the filter has converged!
    }
}

void BackendMavrosFW::setPose(const geometry_msgs::PoseStamped& _world) {
    ROS_WARN("setVelocity command is not supported by the Mavros Fixed Wing backend!");

    return;
}

void BackendMavrosFW::goToWaypoint(const Waypoint& _world) {

    std::vector<MissionElement> new_mission;

    MissionElement pass_waypoint_element;
    pass_waypoint_element.type = MissionElement::PASS;
    geometry_msgs::PoseStamped pass_posestamped;

    pass_posestamped.pose.position.x = _world.pose.position.x;
    pass_posestamped.pose.position.y = _world.pose.position.y;
    pass_posestamped.pose.position.z = _world.pose.position.z;
    pass_waypoint_element.waypoints.push_back(pass_posestamped);
    
    uav_abstraction_layer::ParamFloat param;
    param.name = "acceptance_radius"; param.value = mission_pass_acceptance_radius_;
    pass_waypoint_element.params.push_back( param );
    param.name = "orbit_distance"; param.value = mission_pass_orbit_distance_;
    pass_waypoint_element.params.push_back( param );
    new_mission.push_back(pass_waypoint_element);

    MissionElement loiter_waypoint_element;
    loiter_waypoint_element.type = MissionElement::LOITER_UNLIMITED;
    geometry_msgs::PoseStamped loiter_posestamped;
    loiter_posestamped.pose.position.x = _world.pose.position.x;
    loiter_posestamped.pose.position.y = _world.pose.position.y;
    loiter_posestamped.pose.position.z = _world.pose.position.z;
    loiter_waypoint_element.waypoints.push_back(loiter_posestamped);
    // uav_abstraction_layer::ParamFloat param;
    param.name = "radius"; param.value = mission_loit_radius_;
    loiter_waypoint_element.params.push_back( param );
    new_mission.push_back(loiter_waypoint_element);

    setMission(new_mission);
    return;
}


void BackendMavrosFW::goToWaypointGeo(const WaypointGeo& _wp) {
    
    std::vector<MissionElement> new_mission;

    MissionElement pass_waypoint_element;
    pass_waypoint_element.type = MissionElement::PASS;
    geometry_msgs::PoseStamped pass_posestamped;
    pass_posestamped.pose.position.x = _wp.latitude;
    pass_posestamped.pose.position.y = _wp.longitude;
    pass_posestamped.pose.position.z = _wp.altitude;
    pass_posestamped.header.frame_id = _wp.header.frame_id;
    pass_waypoint_element.waypoints.push_back(pass_posestamped);
    uav_abstraction_layer::ParamFloat param;
    param.name = "acceptance_radius"; param.value = mission_pass_acceptance_radius_;
    pass_waypoint_element.params.push_back( param );
    param.name = "orbit_distance"; param.value = mission_pass_orbit_distance_;
    pass_waypoint_element.params.push_back( param );
    new_mission.push_back(pass_waypoint_element);

    MissionElement loiter_waypoint_element;
    loiter_waypoint_element.type = MissionElement::LOITER_UNLIMITED;
    geometry_msgs::PoseStamped loiter_posestamped;
    loiter_posestamped.pose.position.x = _wp.latitude;
    loiter_posestamped.pose.position.y = _wp.longitude;
    loiter_posestamped.pose.position.z = _wp.altitude;
    pass_posestamped.header.frame_id = _wp.header.frame_id;
    loiter_waypoint_element.waypoints.push_back(loiter_posestamped);
    // uav_abstraction_layer::ParamFloat param;
    param.name = "radius"; param.value = mission_loit_radius_;
    loiter_waypoint_element.params.push_back( param );
    new_mission.push_back(loiter_waypoint_element);

    setMission(new_mission);
    return;
}

void BackendMavrosFW::setMission(const std::vector<MissionElement>& _mission_element_list) {

    mavros_msgs::WaypointList new_mission;
    takeoff_wps_on_mission_ = std::vector<int>();
    land_wps_on_mission_ = std::vector<int>();
    bool mission_def_error = false;
    ROS_INFO("Checking mission definition...");

    for (std::vector<int>::size_type i = 0; i != _mission_element_list.size(); i++) {
        MissionElement mission_element = _mission_element_list[i];
        
        // switch 
        switch (mission_element.type) {
            case MissionElement::TAKEOFF_POSE:
            case MissionElement::TAKEOFF_AUX:
                addTakeOffWp(new_mission,mission_element,i);
                break;
            case MissionElement::PASS:
                addPassWpList(new_mission,mission_element,i);
                break;
            case MissionElement::LOITER_UNLIMITED:
            case MissionElement::LOITER_TURNS:
            case MissionElement::LOITER_TIME:
            case MissionElement::LOITER_HEIGHT:
                addLoiterWpList(new_mission,mission_element,i);
                break;
            case MissionElement::LAND_POSE:
            case MissionElement::LAND_AUX:
                addLandWpList(new_mission,mission_element,i);
                break;
            default:
                ROS_ERROR("Error in [%d]-th waypoint set, field type is not correct.", static_cast<int>(i));
                mission_def_error = true;
        }
    }

    if (mission_def_error) {
        ROS_ERROR("Mission aborted!");
    }
    else {
        ROS_INFO("Mission definition es correct.");
        clearMission();
        bool push_success = pushMission(new_mission);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (push_success){
            arm(true);
            ROS_INFO("Starting mission!");
            setFlightMode("AUTO.MISSION");
        }
        else {
            ROS_WARN("Mavros rejected the mission.");
        }

    }

}

/*void BackendMavrosFW::trackPath(const WaypointList &_path) {
    // TODO: basic imlementation, ideally different from a stack of gotos
}*/

Pose BackendMavrosFW::pose() {
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
                try {
                    transformToPoseFrame = tf_buffer_.lookupTransform(pose_frame_id_,uav_home_frame_id_, ros::Time(0), ros::Duration(1.0));
                    cached_transforms_[pose_frame_id_map] = transformToPoseFrame; // Save transform in cache
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("In pose: %s. Returning non transformed pose.", ex.what());
                    return out;
                }
            } else {
                // found in cache
                transformToPoseFrame = cached_transforms_[pose_frame_id_map];
            }

            tf2::doTransform(aux, out, transformToPoseFrame);
            out.header.frame_id = pose_frame_id_;
        }

        out.header.stamp = cur_pose_.header.stamp;
        return out;
}

Pose BackendMavrosFW::referencePose() {
    Pose out;

    out.pose.position.x = ref_pose_.pose.position.x + local_start_pos_[0];
    out.pose.position.y = ref_pose_.pose.position.y + local_start_pos_[1];
    out.pose.position.z = ref_pose_.pose.position.z + local_start_pos_[2];
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

Velocity BackendMavrosFW::velocity() const {
    return cur_vel_;
}

Odometry BackendMavrosFW::odometry() const {
    Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = uav_home_frame_id_;
    odom.child_frame_id = uav_frame_id_;
    odom.pose.pose.position.x = cur_pose_.pose.position.x + local_start_pos_[0];
    odom.pose.pose.position.y = cur_pose_.pose.position.y + local_start_pos_[1];
    odom.pose.pose.position.z = cur_pose_.pose.position.z + local_start_pos_[2];
    odom.pose.pose.orientation = cur_pose_.pose.orientation;
    odom.twist.twist = cur_vel_body_.twist;

    return odom;
}

Transform BackendMavrosFW::transform() const {
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = uav_home_frame_id_;
    out.child_frame_id = uav_frame_id_;
    out.transform.translation.x = cur_pose_.pose.position.x + local_start_pos_[0];
    out.transform.translation.y = cur_pose_.pose.position.y + local_start_pos_[1];
    out.transform.translation.z = cur_pose_.pose.position.z + local_start_pos_[2];
    out.transform.rotation = cur_pose_.pose.orientation;
    return out;
}

void BackendMavrosFW::initHomeFrame() {

    local_start_pos_ << 0.0, 0.0, 0.0;

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
        ros::param::get("~home_pose",home_pose);
    }
    else if (ros::param::has("~map_origin_geo")) {
        ROS_WARN("Be careful, you should only use this mode with RTK GPS!");
        while (!this->mavros_has_geo_pose_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::vector<double> map_origin_geo(3, 0.0);
        ros::param::get("~map_origin_geo",map_origin_geo);
        geographic_msgs::GeoPoint origin_geo, actual_coordinate_geo;
        origin_geo.latitude = map_origin_geo[0];
        origin_geo.longitude = map_origin_geo[1];
        origin_geo.altitude = 0; //map_origin_geo[2];
        actual_coordinate_geo.latitude = cur_geo_pose_.latitude;
        actual_coordinate_geo.longitude = cur_geo_pose_.longitude;
        actual_coordinate_geo.altitude = 0; //cur_geo_pose_.altitude;
        if(map_origin_geo[0]==0 && map_origin_geo[1]==0) {
            ROS_WARN("Map origin is set to 0. Define map_origin_geo param by a vector in format [lat,lon,alt].");
        }
        geometry_msgs::Point32 map_origin_cartesian = geographic_to_cartesian (actual_coordinate_geo, origin_geo);

        home_pose[0] = map_origin_cartesian.x;
        home_pose[1] = map_origin_cartesian.y;
        home_pose[2] = map_origin_cartesian.z;
    }
    else {
        ROS_WARN("No home pose or map origin was defined. Home frame will be equal to map.");
    }

    if (ros::param::has("~map_origin_geo")) {

        std::vector<double> map_origin_geo(0.0, 0.0);
        ros::param::get("~map_origin_geo",map_origin_geo);
        origin_geo_.latitude = map_origin_geo[0];
        origin_geo_.longitude = map_origin_geo[1];
        origin_geo_.altitude = 0; //map_origin_geo[2];
        
    }

    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = parent_frame;
    static_transformStamped.child_frame_id = uav_home_frame_id_;
    static_transformStamped.transform.translation.x = home_pose[0];
    static_transformStamped.transform.translation.y = home_pose[1];
    static_transformStamped.transform.translation.z = home_pose[2];
    static_transformStamped.transform.rotation.x = 0;
    static_transformStamped.transform.rotation.y = 0;
    static_transformStamped.transform.rotation.z = 0;
    static_transformStamped.transform.rotation.w = 1;

    if(parent_frame != "map" && parent_frame != "") {
        geometry_msgs::TransformStamped transform_to_map;
        try {
            transform_to_map = tf_buffer_.lookupTransform(parent_frame, "map", ros::Time(0), ros::Duration(2.0));
            static_transformStamped.transform.rotation = transform_to_map.transform.rotation;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("In initHomeFrame: %s. Publishing static TF in ENU.", ex.what());
        }
    }

    static_tf_broadcaster_ = new tf2_ros::StaticTransformBroadcaster();
    static_tf_broadcaster_->sendTransform(static_transformStamped);
}

double BackendMavrosFW::updateParam(const std::string& _param_id) {
    mavros_msgs::ParamGet get_param_service;
    get_param_service.request.param_id = _param_id;
    if (get_param_client_.call(get_param_service) && get_param_service.response.success) {
        mavros_params_[_param_id] = get_param_service.response.value.integer? 
            get_param_service.response.value.integer : get_param_service.response.value.real;
        ROS_DEBUG("Parameter [%s] value is [%f]", get_param_service.request.param_id.c_str(), mavros_params_[_param_id]);
    } else if (mavros_params_.count(_param_id)) {
        ROS_WARN("Error in get param [%s] service calling, leaving current value [%f]", 
            get_param_service.request.param_id.c_str(), mavros_params_[_param_id]);
    } else {
        mavros_params_[_param_id] = 0.0;
        ROS_ERROR("Error in get param [%s] service calling, initializing it to zero", 
            get_param_service.request.param_id.c_str());
    }
    return mavros_params_[_param_id];
}

void BackendMavrosFW::setParam(const std::string& _param_id, const int& _param_value) {
    mavros_msgs::ParamSet set_param_service;
    set_param_service.request.param_id = _param_id;
    set_param_service.request.value.integer = _param_value;     // FIX FOR FLOAT
    set_param_service.request.value.real = 0;

    while (updateParam(_param_id) != _param_value && ros::ok()) {
        if (!set_param_client_.call(set_param_service)) {
            ROS_ERROR("Error in set param [%s] service calling!", _param_id.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
            set_param_service.response.success ? "true" : "false");
#else
        // ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
        //     set_param_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("Trying to set [%s] param to [%10d]", _param_id.c_str(), _param_value);
    }
}

void BackendMavrosFW::getAutopilotInformation() {
    // Call vehicle information service
    ros::NodeHandle nh;
    ros::ServiceClient vehicle_information_cl = nh.serviceClient<mavros_msgs::VehicleInfoGet>("mavros/vehicle_info_get");
    ros::service::waitForService("mavros/vehicle_info_get");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    mavros_msgs::VehicleInfoGet vehicle_info_srv;
    if (!vehicle_information_cl.call(vehicle_info_srv)) {
        ROS_ERROR("Failed to get vehicle information: service call failed");
        exit(0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!vehicle_info_srv.response.success) {
        ROS_ERROR("Failed to get vehicle information");
        exit(0);
    }
    // Autopilot type
    switch (vehicle_info_srv.response.vehicles[0].autopilot) {
        case 3:
            autopilot_type_ = AutopilotType::APM;
            break;
        case 12:
            autopilot_type_ = AutopilotType::PX4;
            break;
        default:
            ROS_ERROR("BackendMavrosFW [%d]: Wrong autopilot type: %s", robot_id_, mavros::utils::to_string((mavlink::common::MAV_AUTOPILOT) vehicle_info_srv.response.vehicles[0].autopilot).c_str());
            exit(0);
    }

    // Autopilot version
    int major_version = ( ((int) vehicle_info_srv.response.vehicles[0].flight_sw_version) >> (8*3)) & 0xFF;
    int minor_version = (vehicle_info_srv.response.vehicles[0].flight_sw_version >> (8*2)) & 0xFF;
    int patch_version = (vehicle_info_srv.response.vehicles[0].flight_sw_version >> (8*1)) & 0xFF;
    int version_type_int = (vehicle_info_srv.response.vehicles[0].flight_sw_version >> (8*0)) & 0xFF;
    std::string version_type;
    switch (version_type_int) {
        case FIRMWARE_VERSION_TYPE_DEV:
            version_type = "dev";
            break;
        case FIRMWARE_VERSION_TYPE_ALPHA:
            version_type = "alpha";
            break;
        case FIRMWARE_VERSION_TYPE_BETA:
            version_type = "beta";
            break;
        case FIRMWARE_VERSION_TYPE_RC:
            version_type = "rc";
            break;
        case FIRMWARE_VERSION_TYPE_OFFICIAL:
        default:
            version_type = "";
    }
    std::string autopilot_version = std::to_string(major_version) + "." + std::to_string(minor_version) + "." + std::to_string(patch_version) + version_type;

    // Autopilot string
    ROS_INFO("BackendMavrosFW [%d]: Connected to %s version %s. Type: %s.", robot_id_,
    mavros::utils::to_string((mavlink::common::MAV_AUTOPILOT) vehicle_info_srv.response.vehicles[0].autopilot).c_str(),
    autopilot_version.c_str(), mavros::utils::to_string((mavlink::common::MAV_TYPE) vehicle_info_srv.response.vehicles[0].type).c_str());
}

bool BackendMavrosFW::pushMission(const mavros_msgs::WaypointList& _wp_list) {
    mavros_msgs::WaypointPush push_waypoint_service;
    push_waypoint_service.request.start_index = 0;
    push_waypoint_service.request.waypoints = _wp_list.waypoints;
    ROS_INFO("Trying to push mission");

    if (!push_mission_client_.call(push_waypoint_service)) {
        ROS_ERROR("Error in push mission service calling!");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
    ROS_INFO("Push mission response.success = %s", set_param_service.response.success ? "true" : "false");
#else
    // ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
    //     set_param_service.response.mode_sent ? "true" : "false");
#endif
    
    return push_waypoint_service.response.success;
}

void BackendMavrosFW::clearMission() {
    mavros_msgs::WaypointClear clear_mission_service;

    if (!clear_mission_client_.call(clear_mission_service)) {
        ROS_ERROR("Error in clear mission service calling!");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
    ROS_INFO("Clear mission response.success = %s", clear_mission_service.response.success ? "true" : "false");
#else
    // ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
    //     set_param_service.response.mode_sent ? "true" : "false");
#endif
    ROS_INFO("Trying to clear mission");
}


void BackendMavrosFW::addTakeOffWp(mavros_msgs::WaypointList& _wp_list, const MissionElement& _waypoint_element, const int& wp_set_index) {

    std::map<std::string, float> params_map;
    for(std::vector<int>::size_type i = 0; i != _waypoint_element.params.size(); i++) {
        params_map.insert( std::pair<std::string,float>(_waypoint_element.params[i].name,_waypoint_element.params[i].value) );    }

    mavros_msgs::Waypoint wp;

    std::vector<geographic_msgs::GeoPoseStamped> usf;   // Stands for Uniformized Spatial Field
    usf = uniformizeSpatialField(_waypoint_element);

    if (_waypoint_element.type == MissionElement::TAKEOFF_POSE) {
        
        if (usf.size() != 1){ ROS_ERROR("Error in [%d]-th waypoint set, posestamped list lenght is not 1!", wp_set_index); } //TODO(JoseAndres): Update errors
        
        wp = geoPoseStampedtoGlobalWaypoint(usf[0]);

        params_map["yaw_angle"] = getMissionYaw(usf[0].pose.orientation);
        
    }

    else if (_waypoint_element.type == MissionElement::TAKEOFF_AUX) {
        std::vector<std::string> required_aux_params { "aux_distance","aux_angle","aux_height" };
        checkMissionParams(params_map, required_aux_params, wp_set_index);

        geometry_msgs::PoseStamped aux_pose = pose();
        aux_pose.pose.position.x += params_map["aux_distance"] * cos(params_map["aux_angle"]);
        aux_pose.pose.position.y += params_map["aux_distance"] * sin(params_map["aux_angle"]);
        aux_pose.pose.position.z += params_map["aux_height"];

        params_map["yaw_angle"] = params_map["aux_angle"];

        wp = geoPoseStampedtoGlobalWaypoint(poseStampedtoGeoPoseStamped(aux_pose));
    }

    wp.frame = 3;      // FRAME_GLOBAL_REL_ALT
    wp.command = 22;        // MAV_CMD_NAV_TAKEOFF
    wp.is_current = true;
    wp.autocontinue = true;

    std::vector<std::string> required_params { "minimum_pitch" };
    checkMissionParams(params_map, required_params, wp_set_index);

    wp.param1 = params_map["minimum_pitch"];    // (if airspeed sensor present), desired pitch without sensor
    wp.param4 = params_map["yaw_angle"];        // (if magnetometer present), ignored without magnetometer. NaN for unchanged.

    _wp_list.waypoints.push_back(wp);
    takeoff_wps_on_mission_.push_back(_wp_list.waypoints.size()-1);

}

void BackendMavrosFW::addPassWpList(mavros_msgs::WaypointList& _wp_list, const MissionElement& _waypoint_element, const int& wp_set_index) {

    std::vector<geographic_msgs::GeoPoseStamped> usf;
    usf = uniformizeSpatialField(_waypoint_element);

    if (usf.size() == 0){ ROS_ERROR("Error in [%d]-th waypoint set, posestamped list is empty!", wp_set_index); }

    std::map<std::string, float> params_map;
    for(std::vector<int>::size_type i = 0; i != _waypoint_element.params.size(); i++) {
        params_map.insert( std::pair<std::string,float>(_waypoint_element.params[i].name,_waypoint_element.params[i].value) );    }

    if ( params_map.count( "speed" ) == 1) {
        addSpeedWpList(_wp_list,_waypoint_element,wp_set_index);
    }

    std::vector<std::string> required_params { "acceptance_radius","orbit_distance" };
    checkMissionParams(params_map, required_params, wp_set_index);

    for ( auto & geoposestamped : usf ) {

        mavros_msgs::Waypoint wp;

        wp = geoPoseStampedtoGlobalWaypoint( geoposestamped);

        params_map["yaw_angle"] = getMissionYaw(geoposestamped.pose.orientation);

        wp.frame = 3;      // FRAME_GLOBAL_REL_ALT
        wp.command = 16;        // MAV_CMD_NAV_WAYPOINT
        wp.is_current = false;
        wp.autocontinue = true;

        wp.param2 = params_map["acceptance_radius"];        // (if the sphere with this radius is hit, the waypoint counts as reached)
        wp.param3 = params_map["orbit_distance"];           // 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit,
                                                            // negative value for counter-clockwise orbit. Allows trajectory control.
        wp.param4 = params_map["yaw_angle"];                // Desired yaw angle at waypoint (rotary wing). NaN for unchanged.

        _wp_list.waypoints.push_back(wp);

    }
}

void BackendMavrosFW::addLoiterWpList(mavros_msgs::WaypointList& _wp_list, const MissionElement& _waypoint_element, const int& wp_set_index) {

    std::vector<geographic_msgs::GeoPoseStamped> usf;
    usf = uniformizeSpatialField(_waypoint_element);

    if (usf.size() == 0){ ROS_ERROR("Error in [%d]-th waypoint set, posestamped list is empty!", wp_set_index); }

    std::map<std::string, float> params_map;
    for(std::vector<int>::size_type i = 0; i != _waypoint_element.params.size(); i++) {
        params_map.insert( std::pair<std::string,float>(_waypoint_element.params[i].name,_waypoint_element.params[i].value) );    }

    if ( params_map.count( "speed" ) == 1) {
        addSpeedWpList(_wp_list,_waypoint_element,wp_set_index);
    }

    for ( auto & geoposestamped : usf ) {

        mavros_msgs::Waypoint wp;
        wp = geoPoseStampedtoGlobalWaypoint( geoposestamped);
        wp.frame = 3;      // FRAME_GLOBAL_REL_ALT
        wp.is_current = false;
        wp.autocontinue = true;

        if (_waypoint_element.type == MissionElement::LOITER_UNLIMITED) {

            std::vector<std::string> required_params { "radius" };
            checkMissionParams(params_map, required_params, wp_set_index);

            params_map["yaw_angle"] = getMissionYaw(geoposestamped.pose.orientation);

            wp.command = 17;        // MAV_CMD_NAV_LOITER_UNLIM
            wp.param3 = params_map["radius"];               // Radius around waypoint. If positive loiter clockwise, else counter-clockwise
            wp.param4 = params_map["yaw_angle"];            // NaN for unchanged.

        }
        else if (_waypoint_element.type == MissionElement::LOITER_TURNS) {

            std::vector<std::string> required_params { "turns","radius","forward_moving" };
            checkMissionParams(params_map, required_params, wp_set_index);

            wp.command = 18;        // MAV_CMD_NAV_LOITER_TURNS
            wp.param1 = params_map["turns"];                // Number of turns.
            wp.param3 = params_map["radius"];               // Radius around waypoint. If positive loiter clockwise, else counter-clockwise
            wp.param4 = params_map["forward_moving"];       // this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location.
                                                            // Else, this is desired yaw angle. NaN for unchanged.

        }
        else if (_waypoint_element.type == MissionElement::LOITER_TIME) {

            std::vector<std::string> required_params { "time","radius","forward_moving" };
            checkMissionParams(params_map, required_params, wp_set_index);

            wp.command = 19;        // MAV_CMD_NAV_LOITER_TIME
            wp.param1 = params_map["time"];                 // 	Loiter time.
            wp.param3 = params_map["radius"];               // 	Radius around waypoint. If positive loiter clockwise, else counter-clockwise.
            wp.param4 = params_map["forward_moving"];       // this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location.
                                                            // Else, this is desired yaw angle. NaN for unchanged.

        }
        else if (_waypoint_element.type == MissionElement::LOITER_HEIGHT) {

            wp.command = 31;        // MAV_CMD_NAV_LOITER_TO_ALT

            std::vector<std::string> required_params { "heading","radius","forward_moving" };
            checkMissionParams(params_map, required_params, wp_set_index);

            wp.param1 = params_map["heading"];              // Heading Required (0 = False)
            wp.param2 = params_map["radius"];               // If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
            wp.param4 = params_map["forward_moving"];       // Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location

        }

        _wp_list.waypoints.push_back(wp);
    }

}

void BackendMavrosFW::addLandWpList(mavros_msgs::WaypointList& _wp_list, const MissionElement& _waypoint_element, const int& wp_set_index) {

    std::vector<geographic_msgs::GeoPoseStamped> usf;
    usf = uniformizeSpatialField(_waypoint_element);

    std::map<std::string, float> params_map;
    for(std::vector<int>::size_type i = 0; i != _waypoint_element.params.size(); i++) {
        params_map.insert( std::pair<std::string,float>(_waypoint_element.params[i].name,_waypoint_element.params[i].value) );    }

    mavros_msgs::Waypoint wp1;
    wp1.frame = 2;      // FRAME_MISSION
    wp1.command = 189;      // MAV_CMD_DO_LAND_START
    wp1.is_current = false;
    wp1.autocontinue = true;

    _wp_list.waypoints.push_back(wp1);
    land_wps_on_mission_.push_back(_wp_list.waypoints.size() -1 );

    mavros_msgs::Waypoint wp2;

    if (_waypoint_element.type == MissionElement::LAND_POSE) {

        if (usf.size() != 2){ ROS_ERROR("Error in [%d]-th waypoint, posestamped list length is not 2!", wp_set_index); }

        wp2 = geoPoseStampedtoGlobalWaypoint(usf[0]);
        
    }

    else if (_waypoint_element.type == MissionElement::LAND_AUX) {

        if (usf.size() != 1){ ROS_ERROR("Error in [%d]-th waypoint, posestamped list length is not 1!", wp_set_index); }

        std::vector<std::string> required_aux_params { "aux_distance","aux_angle", "aux_height" };
        checkMissionParams(params_map, required_aux_params, wp_set_index);

        geometry_msgs::PoseStamped aux_pose = geoPoseStampedtoPoseStamped(usf[0]);
        aux_pose.pose.position.x += params_map["aux_distance"] * cos(params_map["aux_angle"]) + local_start_pos_[0];
        aux_pose.pose.position.y += params_map["aux_distance"] * sin(params_map["aux_angle"]) + local_start_pos_[1];
        aux_pose.pose.position.z += params_map["aux_height"];

        wp2 = geoPoseStampedtoGlobalWaypoint(poseStampedtoGeoPoseStamped(aux_pose));

    }

    wp2.frame = 3;      // FRAME_GLOBAL_REL_ALT
    wp2.command = 31;       // MAV_CMD_NAV_LOITER_TO_ALT
    wp2.is_current = false;
    wp2.autocontinue = true;

    std::vector<std::string> required_params_2 { "loit_heading","loit_radius","loit_forward_moving" };
    checkMissionParams(params_map, required_params_2, wp_set_index);

    wp2.param1 = params_map["loit_heading"];            	// Heading Required (0 = False)
    wp2.param2 = params_map["loit_radius"];                 // If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
    wp2.param4 = params_map["loit_forward_moving"];         // Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location

    _wp_list.waypoints.push_back(wp2);
    land_wps_on_mission_.push_back(_wp_list.waypoints.size() -1 );

    mavros_msgs::Waypoint wp3;
    wp3 = geoPoseStampedtoGlobalWaypoint(usf.back());
    wp3.frame = 3;      // FRAME_GLOBAL_REL_ALT
    wp3.command = 21;       // MAV_CMD_NAV_LAND
    wp3.is_current = false;
    wp3.autocontinue = true;

    params_map["yaw_angle"] = getMissionYaw(usf.back().pose.orientation);

    std::vector<std::string> required_params_3 { "abort_alt","precision_mode" };
    checkMissionParams(params_map, required_params_3, wp_set_index);

    wp3.param1 = params_map["abort_alt"];                   // Minimum target altitude if landing is aborted (0 = undefined/use system default).
    wp3.param2 = params_map["precision_mode"];              // Precision land mode.
    wp3.param4 = params_map["yaw_angle"];                   // Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).

    _wp_list.waypoints.push_back(wp3);
    land_wps_on_mission_.push_back(_wp_list.waypoints.size() -1 );

}

void BackendMavrosFW::addSpeedWpList(mavros_msgs::WaypointList& _wp_list, const MissionElement& _waypoint_element, const int& wp_set_index) {

    std::map<std::string, float> params_map;
    for(std::vector<int>::size_type i = 0; i != _waypoint_element.params.size(); i++) {
        params_map.insert( std::pair<std::string,float>(_waypoint_element.params[i].name,_waypoint_element.params[i].value) ); }

    std::vector<std::string> required_params { "speed" };
    checkMissionParams(params_map, required_params, wp_set_index);

    mavros_msgs::Waypoint wp;
    wp.frame = 2;       // FRAME_MISSION
    wp.command = 178;       // MAV_CMD_DO_CHANGE_SPEED
    wp.is_current = false;
    wp.autocontinue = true;

    wp.param1 = 1.0;                                            // Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
    wp.param2 = params_map["speed"];                            // Speed (-1 indicates no change)
    wp.param3 = -1.0;                                           // Throttle (-1 indicates no change)
    wp.param4 = 0.0;                                            // Relative (0: absolute, 1: relative)
    _wp_list.waypoints.push_back(wp);

}

std::vector<geographic_msgs::GeoPoseStamped> BackendMavrosFW::uniformizeSpatialField( const MissionElement& _waypoint_element){

    std::vector<geographic_msgs::GeoPoseStamped> uniformized;

    for ( auto & posestamped : _waypoint_element.waypoints ) {
        geographic_msgs::GeoPoseStamped homogen_world_pos;
        std::string waypoint_frame_id = tf2::getFrameId(posestamped);
        bool success = true;
        
        if ( waypoint_frame_id == "geo" ) {
            // No transform is needed and already in global coordinates
            homogen_world_pos.header = posestamped.header;
            homogen_world_pos.pose.orientation = posestamped.pose.orientation;
            homogen_world_pos.pose.position.latitude = posestamped.pose.position.x;
            homogen_world_pos.pose.position.longitude = posestamped.pose.position.y;
            homogen_world_pos.pose.position.altitude = posestamped.pose.position.z;

        }
        if ( waypoint_frame_id == "" || waypoint_frame_id == uav_home_frame_id_ ) {
            // No transform is needed. Passed to global
            homogen_world_pos = poseStampedtoGeoPoseStamped(posestamped);
        }
        else {
            // We need to transform
            geometry_msgs::TransformStamped transformToHomeFrame;

            if ( cached_transforms_.find(waypoint_frame_id) == cached_transforms_.end() ) {
                // waypoint_frame_id not found in cached_transforms_
                try {
                    transformToHomeFrame = tf_buffer_.lookupTransform(uav_home_frame_id_, waypoint_frame_id, ros::Time(0), ros::Duration(1.0));
                    cached_transforms_[waypoint_frame_id] = transformToHomeFrame; // Save transform in cache
                }
                catch (tf2::TransformException &ex) {
                    ROS_ERROR("Transformation not found");
                    success = false;
                }
            } else {
                // found in cache
                transformToHomeFrame = cached_transforms_[waypoint_frame_id];
            }
            geometry_msgs::PoseStamped aux = posestamped;
            tf2::doTransform(posestamped, aux, transformToHomeFrame);
            homogen_world_pos = poseStampedtoGeoPoseStamped(aux);
        }

        uniformized.push_back( homogen_world_pos);

    }

    return uniformized;

}


geographic_msgs::GeoPoseStamped BackendMavrosFW::poseStampedtoGeoPoseStamped(const geometry_msgs::PoseStamped& _posestamped ) {

    geometry_msgs::Point32 geo_point;
    geo_point.x = _posestamped.pose.position.x;
    geo_point.y = _posestamped.pose.position.y;
    geo_point.z = _posestamped.pose.position.z;

    geographic_msgs::GeoPoint actual_geo = cartesian_to_geographic(geo_point, origin_geo_);

    geographic_msgs::GeoPoseStamped geopose;
    geopose.pose.position.latitude = actual_geo.latitude;
    geopose.pose.position.longitude = actual_geo.longitude;
    geopose.pose.position.altitude = actual_geo.altitude;
    geopose.header = _posestamped.header;
    geopose.pose.orientation = _posestamped.pose.orientation;

    return geopose;

}


geometry_msgs::PoseStamped BackendMavrosFW::geoPoseStampedtoPoseStamped(const geographic_msgs::GeoPoseStamped _geoposestamped ) {

    geographic_msgs::GeoPoint aux;
    aux.latitude = _geoposestamped.pose.position.latitude;
    aux.longitude = _geoposestamped.pose.position.longitude;
    aux.altitude = _geoposestamped.pose.position.altitude;

    geometry_msgs::Point32 actual_geo = geographic_to_cartesian(aux, origin_geo_);

    geometry_msgs::PoseStamped posestamped;
    posestamped.pose.position.x = actual_geo.x;
    posestamped.pose.position.y = actual_geo.y;
    posestamped.pose.position.z = actual_geo.z;
    posestamped.header = _geoposestamped.header;
    posestamped.pose.orientation = _geoposestamped.pose.orientation;

    return posestamped;

}


mavros_msgs::Waypoint BackendMavrosFW::geoPoseStampedtoGlobalWaypoint(const geographic_msgs::GeoPoseStamped& _geoposestamped ) {

    mavros_msgs::Waypoint waypoint;
    waypoint.x_lat = _geoposestamped.pose.position.latitude;
    waypoint.y_long = _geoposestamped.pose.position.longitude;
    waypoint.z_alt = _geoposestamped.pose.position.altitude;

    return waypoint;

}

float BackendMavrosFW::getMissionYaw(const geometry_msgs::Quaternion& quat) {

    float yaw;

    double norm = quat.x*quat.x  + quat.y*quat.y  + quat.z*quat.z  + quat.w*quat.w;

    if ( norm == 0) {
        yaw = std::nanf("0.0");
    }

    else {
        yaw = tf::getYaw(quat);
    }

    return yaw;
}


void BackendMavrosFW::checkMissionParams(const std::map<std::string, float>& existing_params_map, const std::vector<std::string>& required_params, const int& wp_set_index){
    for ( auto &_param :  required_params){
        if (existing_params_map.count(_param) == 0){
            ROS_ERROR("Warn in [%d]-th waypoint set, [%s] param not provided!", wp_set_index, _param.c_str());
        }
    }
}

}}	// namespace grvc::ual
