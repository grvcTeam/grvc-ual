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


#include <uav_abstraction_layer/back_end_mavros.h>
#include <argument_parser/argument_parser.h>
#include <string>
#include <chrono>
//#include <std_msgs/Float32MultiArray.h>

//#include <Eigen/LU>
#include <grvc_quadrotor_hal/back_end/tinyxml2.h>  // TODO: where?
#include <Eigen/Eigen>
#include <ros/ros.h>
//#include <ros/package.h>

using namespace std;
using namespace grvc::utils;
using namespace tinyxml2;

namespace grvc { namespace ual {

// TODO: node_name? anonymous node?
// TODO: state callback?
BackendMavros::BackendMavros(const char* _node_name, int _argc, char** _argv, StateCallBack _scb)
    : Backend(_scb), args_(_argc, _argv)
{
    ROS_INFO("Backend constructor");
    // Init ros
    //com::RosSingleton::init(_node_name, _argc, _argv);
    ros::init(_argc, _argv, _node_name);
    // nh_ = com::RosSingleton::get()->handle();
    // if(nh_ == NULL)
    //     ROS_INFO("Failed");

    // Init myself
    parseArguments();
    startRosCommunications();

    // Wait until we have pose
    // while(!mavros_has_pose_) {
    //     //ROS_INFO("Waiting for pose");
    //     usleep(1000);
    // }
    //initLocalCoordMatrix();

    // Thread publishing target pose at 10Hz for offboard mode
    offboardThread_ = std::thread([this]() {
        while (ros::ok()) {
            if (control_in_vel_) {
                mavros_ref_vel_pub_.publish(ref_vel_);
                ref_pose_ = cur_pose_;
            } else {
                mavros_ref_pose_pub_.publish(ref_pose_);
                ref_vel_.twist.linear.x = 0;
                ref_vel_.twist.linear.y = 0;
                ref_vel_.twist.linear.z = 0;
                ref_vel_.twist.angular.z = 0;
            }
            // TODO: Check this frequency and use ros::Rate
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}


void BackendMavros::arm() {
    // Arming
    ROS_INFO("Arming");
    mavros_msgs::CommandBool arming_service;
    arming_service.request.value = true;
    arming_service.response.success = false;  // Init value to force first while loop
    // TODO: stubborn?
    while (!arming_service.response.success) {
        if (!arming_client_.call(arming_service)) {
            // TODO: state callback?
            state_cb_(TaskState::aborted);
            return;
        }
        std::cout << "Arming service response = " << (int)arming_service.response.success << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// void BackendMavros::setOffBoardMode() {
//     // Set mode offboard
//     mavros_msgs::SetMode flight_mode_service;
//     flight_mode_service.request.base_mode = 0;
//     flight_mode_service.request.custom_mode = "OFFBOARD";
//     if(!flight_mode_client_.call(flight_mode_service))
//     {
//         state_cb_(TaskState::aborted);
//         return;
//     }
//     ROS_INFO("OFFBOARD!");
//     // TODO: Check we're OFFBOARD
//     std::cout << "Finished call to set offboard mode service with response = "
//                 << (int)flight_mode_service.response.success << std::endl;
// }

// void BackendMavros::setLandMode() {
//     // Set mode auto.land
//     mavros_msgs::SetMode flight_mode_service;
//     flight_mode_service.request.base_mode = 0;
//     flight_mode_service.request.custom_mode = "AUTO.LAND";
//     flight_mode_service.response.success = false;
//     while(!flight_mode_service.response.success) {
//         if(!flight_mode_client_.call(flight_mode_service))
//         {
//             state_cb_(TaskState::aborted);
//             return;
//         }
//         std::cout << "Finished call to set land mode service with response = "
//                     << (int)flight_mode_service.response.success << std::endl;
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
// }

void BackendMavros::setFlightMode(const std::string& _flight_mode, bool _stubborn) {
    mavros_msgs::SetMode flight_mode_service;
    flight_mode_service.request.base_mode = 0;
    flight_mode_service.request.custom_mode = _flight_mode;
    while (_stubborn) {
        if (!flight_mode_client_.call(flight_mode_service)) {
            // TODO: sate callback?
            state_cb_(TaskState::aborted);
            return;
        }
        _stubborn = !flight_mode_service.response.success;
        std::cout << "Set flight mode [" << _flight_mode << "] service response = "
            << (int)flight_mode_service.response.success << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


void BackendMavros::takeOff(double _height) {
    
    // TODO: change state? -> unnecessary control_in_vel_?
    control_in_vel_ = false;

    // TODO: This is blocking and stubborn by default!
    while(!mavros_state_.armed) {
        arm();
        std::cout << "Trying to arm: " << mavros_state_.armed << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // TODO: This is blocking and stubborn by default!
    while(mavros_state_.mode!="OFFBOARD") {
        home_pose_ = cur_pose_;
        // Add home_pose_ to localTransform
        //Eigen::Matrix4d auxTransform = localTransform.inverse().eval();
        //auxTransform.block(0,3,3,1) += Vec3(home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z);
        //localTransform = auxTransform.inverse().eval(); 

        // TODO: wtf!!
        localStartPos.raw -= Vec3(home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z);
        ref_pose_ = home_pose_;
        ref_pose_.pose.position.z += _height;
        setFlightMode("OFFBOARD");
        std::cout << "Trying to set mode: " << mavros_state_.mode << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
    }

    // TODO: Blocking!
    // Wait until we arrive
    while(!reachedGoal()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO("Flying!");
    state_cb_(TaskState::finished);
    return;
}


void BackendMavros::land() {
    control_in_vel_ = false;
    // Set land mode TODO: stubborn?
    setFlightMode("AUTO.LAND", true);
    ROS_INFO("Landing...");
    // TODO: wtf!
    ref_pose_ = cur_pose_;
    ref_pose_.pose.position.z = 0;
    // TODO: better condition checking
    // TODO: Blocking!
    while(!reachedGoal()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO("Landed!");
    state_cb_(TaskState::finished);
    return;
}


void BackendMavros::setVelocity(const Velocity& _vel) {
    control_in_vel_ = true;
    // TODO: _vel world <-> body tf...
    ref_vel_.twist.linear.x = _vel.linear[0];
    ref_vel_.twist.linear.y = _vel.linear[1];
    ref_vel_.twist.linear.z = _vel.linear[2];
    ref_vel_.twist.angular.z = _vel.yaw_rate;
    state_cb_(TaskState::finished);
}

void BackendMavros::setPositionError(const Vec3& _pos_error) {
    double dt = 0.03;  // TODO: use time in headers?
    integral_control_vel_ += _pos_error * dt; 
    Velocity vel;
    // TODO: create pid util?
    vel.linear[0] = p_gain_xy_ * _pos_error[0] + k_i_xy_ * integral_control_vel_[0] + k_d_xy_ * (_pos_error[0] - previous_error_control_vel_[0]) / dt;
    vel.linear[1] = p_gain_xy_ * _pos_error[1] + k_i_xy_ * integral_control_vel_[1] + k_d_xy_ * (_pos_error[1] - previous_error_control_vel_[1]) / dt;
    vel.linear[2] = p_gain_z_ * _pos_error[2] + k_i_z_ * integral_control_vel_[2] + k_d_z_ * (_pos_error[2] - previous_error_control_vel_[2]) / dt;
    vel.yaw_rate  = 0.0;
    setVelocity(vel);
}


bool BackendMavros::isReady() const{
    return mavros_has_pose_;  // TODO: How to use it?
}

void BackendMavros::goToWP(const Waypoint& _world) {
    control_in_vel_ = false;

    // TODO: check waypoint reference system!
    // TODO: Solve frames issue!
    auto homogenWorldPos = Eigen::Vector3d(_world.pos.raw.x(), _world.pos.raw.y(), _world.pos.raw.z());
    //auto homogenRefPos = localTransform.inverse().eval() * homogenWorldPos;
    auto homogenRefPos = homogenWorldPos - localStartPos.raw;
    ref_pose_.pose.position.x = homogenRefPos(0);
    ref_pose_.pose.position.y = homogenRefPos(1);
    ref_pose_.pose.position.z = homogenRefPos(2);
    //double local_yaw = _world.yaw;  // TODO: yaw transform!
    //ref_pose_.pose.orientation.x = 0;
    //ref_pose_.pose.orientation.y = 0;
    //ref_pose_.pose.orientation.z = sin(0.5*local_yaw);
    //ref_pose_.pose.orientation.w = cos(0.5*local_yaw);
    // TODO: yaw in waypoint as an opition
    ref_pose_.pose.orientation = cur_pose_.pose.orientation;        

    // Wait until we arrive TODO: Blocking!
    while(!reachedGoal() && !mAborting ) {
        // TODO: use internal state (MOVING/HOVER/ABORTING) instead of mAborting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    mAborting=false;
    state_cb_(TaskState::finished);
    return;
}


void BackendMavros::trackPath(const WaypointList &_path){
    // TODO: basic imlementation, ideally different from a stack of gotos
}


void BackendMavros::abort(){
    // TODO: use internal state instead
    mAborting=true;
}


Pose BackendMavros::pose() const {
        Pose pose;
        pose.id = std::to_string(robot_id_);
        // TODO: different frames? tf!
        pose.frame = "map";
        // TODO: trnsform to world coordinates
        /*
        Vec4 localPose = Vec4(cur_pose_.pose.position.x,cur_pose_.pose.position.y,cur_pose_.pose.position.z,1.0);
        Vec4 gamePose = localTransform*localPose;
        pose.position = Vec3(gamePose[0],gamePose[1],gamePose[2]);
        */
        pose.position = Vec3(cur_pose_.pose.position.x + localStartPos.raw[0], cur_pose_.pose.position.y + localStartPos.raw[1], cur_pose_.pose.position.z + localStartPos.raw[2]);
        pose.orientation = Vec4(cur_pose_.pose.orientation.x, cur_pose_.pose.orientation.y, cur_pose_.pose.orientation.z, cur_pose_.pose.orientation.w);
        return pose;
}


void BackendMavros::startRosCommunications() {
    // TODO: remove parseArguments()?
    //robot_id_ = args_.getArgument("uav_id", 1);
    // TODO: Check use of mavros_ns_
    //mavros_ns_ = args_.getArgument("mavros_ns", std::string("mavros_1"));

    std::string set_mode_srv = mavros_ns_ + "/set_mode";
    std::string arming_srv = mavros_ns_ + "/cmd/arming";
    std::string set_pose_topic = mavros_ns_ + "/setpoint_position/local";
    std::string set_vel_topic = mavros_ns_ + "/setpoint_velocity/cmd_vel";
    std::string pose_topic = mavros_ns_ + "/local_position/pose";
    std::string state_topic = mavros_ns_ + "/state";

    flight_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_srv.c_str());
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_srv.c_str());
    mavros_ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(set_pose_topic.c_str(), 10);
    mavros_ref_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(set_vel_topic.c_str(), 10);
    mavros_cur_pose_sub_  = nh_.subscribe(pose_topic.c_str(), 10, &BackendMavros::poseCallback,  this);
    mavros_cur_state_sub_ = nh_.subscribe(state_topic.c_str(), 10, &BackendMavros::stateCallback, this);
}

// TODO: as lambdas?
void BackendMavros::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& _pose){
    cur_pose_ = *_pose;
    // TODO: internal state?
    mavros_has_pose_ = true;
}
void BackendMavros::stateCallback(const mavros_msgs::State::ConstPtr& _state) {
    mavros_state_ = *_state;
}

// TODO: useful?
void BackendMavros::update(){
    if(reachedGoal()) {
        if(path_.empty()) {
            state_cb_(TaskState::finished);
        } else {
            /// 666 TODO: send new position in path
            //pos_ref_ = cur_path_.back();
            path_.pop_back();
        }
    }
}


bool BackendMavros::reachedGoal() const{
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

    if((positionD > 0.1) || (orientationD > 0.1))  // TODO: define thresholds
        return false;
    else
        return true;
}


void BackendMavros::initLocalCoordMatrix() {
    
    XMLDocument doc;
    std::string xml_file;
    ros::param::get("frames_file", xml_file);
    std::string path = ros::package::getPath("mbzirc_launchers") + xml_file;
    
    doc.LoadFile(path.c_str());
    
    XMLNode* root = doc.RootElement();
    if(!root) {
        ROS_ERROR("Error loading xml file %s\n", xml_file.c_str());
        return;
    }

    /*
    // Get origin utm coordinates
    XMLElement* origin_element = root->FirstChildElement("origin");
    XMLElement* utm_x_element = origin_element->FirstChildElement("utmx");
    double utm_x;
    utm_x_element->QueryDoubleText(&utm_x);
    XMLElement* utm_y_element = origin_element->FirstChildElement("utmy");
    double utm_y;
    utm_y_element->QueryDoubleText(&utm_y);
    Cartesian3d origin_utm = Cartesian3d({utm_x,utm_y,0.0});

    cout << "Origin: " << origin_utm.raw << endl;
    */

    // Get Game Transform
    XMLElement* game_element = root->FirstChildElement("game");
    XMLElement* rxx_element = game_element->FirstChildElement("rxx");
    double rxx;
    rxx_element->QueryDoubleText(&rxx);
    XMLElement* rxy_element = game_element->FirstChildElement("rxy");
    double rxy;
    rxy_element->QueryDoubleText(&rxy);
    XMLElement* ryx_element = game_element->FirstChildElement("ryx");
    double ryx;
    ryx_element->QueryDoubleText(&ryx);
    XMLElement* ryy_element = game_element->FirstChildElement("ryy");
    double ryy;
    ryy_element->QueryDoubleText(&ryy);
    Eigen::MatrixXd Rgame(2,2);
    Rgame << rxx, rxy, ryx, ryy;

    cout << "Game: " << Rgame << endl;

    // Get robot position
    cout << "Parsing uavs\n";

    // Get list of robots
    XMLElement* robot = root->FirstChildElement("robothome");
    bool found = false;
    double x, y;
    while(robot && !found) {
        //cout << "Found one robot\n";
        unsigned id = robot->IntAttribute("id");
        if(id==robot_id_) {
            found = true;

            XMLElement* x_element = robot->FirstChildElement("x");
            x_element->QueryDoubleText(&x);
            XMLElement* y_element = robot->FirstChildElement("y");
            y_element->QueryDoubleText(&y);
        }

        robot = robot->NextSiblingElement("robothome");
    }

    cout << "Robot: "<< x << " " << y << endl;

    localStartPos = Cartesian3d({x,y,0.0});
    localTransform = Eigen::Matrix4d::Identity();
    localTransform.block(0,0,2,2) << Rgame;
    localTransform.block(0,3,3,1) = localStartPos.raw;
    //localTransform = localTransform.inverse();

    Eigen::Vector2d gameStartPos;
    gameStartPos << x,y;
    Eigen::Vector2d mapStartPos = Rgame * gameStartPos;
    localStartPos = Cartesian3d({mapStartPos(0),mapStartPos(1),0.0});
}


void BackendMavros::parseArguments() {
    robot_id_ = args_.getArgument("uav_id", 1);
    // TODO: Check use of mavros_ns_
    mavros_ns_ = args_.getArgument("mavros_ns", std::string("mavros_1"));
}

// bool BackendMavros::controlVelPgainServiceCallback(grvc_quadrotor_hal::float32::Request  &req, grvc_quadrotor_hal::float32::Response &res){
//     p_gain_xy_ = req.data;    //666 TODO CheckParsedValue
//     res.success = true;
//     return res.success;
// }

// bool BackendMavros::controlVelKiServiceCallback(grvc_quadrotor_hal::float32::Request  &req, grvc_quadrotor_hal::float32::Response &res){
//     k_i_xy_ = req.data;       //666 TODO CheckParsedValue
//     res.success = true;
//     return res.success;
// }
// bool BackendMavros::controlVelKdServiceCallback(grvc_quadrotor_hal::float32::Request  &req, grvc_quadrotor_hal::float32::Response &res){
//     k_d_xy_ = req.data;       //666 TODO CheckParsedValue
//     res.success = true;
//     return res.success;
// }

}}	// namespace grvc::ual
