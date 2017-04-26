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
#include <uav_abstraction_layer/backend_mavros.h>
#include <uav_abstraction_layer/tinyxml2.h>  // TODO: move?
#include <argument_parser/argument_parser.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>

using namespace tinyxml2;

namespace grvc { namespace ual {

BackendMavros::BackendMavros(int _argc, char** _argv)
    : args_(_argc, _argv)
{
    ROS_INFO("BackendMavros constructor");

    // Parse arguments
    robot_id_ = args_.getArgument("uav_id", 1);

    // Init ros communications
    std::string node_name = "backend_" + std::to_string(robot_id_);
    ros::init(_argc, _argv, node_name);
    nh_ = new ros::NodeHandle();

    std::string mavros_ns = "mavros_" + std::to_string(robot_id_);
    std::string set_mode_srv = mavros_ns + "/set_mode";
    std::string arming_srv = mavros_ns + "/cmd/arming";
    std::string set_pose_topic = mavros_ns + "/setpoint_position/local";
    std::string set_vel_topic = mavros_ns + "/setpoint_velocity/cmd_vel";
    std::string pose_topic = mavros_ns + "/local_position/pose";
    std::string state_topic = mavros_ns + "/state";

    flight_mode_client_ = nh_->serviceClient<mavros_msgs::SetMode>(set_mode_srv.c_str());
    arming_client_ = nh_->serviceClient<mavros_msgs::CommandBool>(arming_srv.c_str());

    mavros_ref_pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(set_pose_topic.c_str(), 10);
    mavros_ref_vel_pub_ = nh_->advertise<geometry_msgs::TwistStamped>(set_vel_topic.c_str(), 10);

    mavros_cur_pose_sub_  = nh_->subscribe<geometry_msgs::PoseStamped>(pose_topic.c_str(), 10, \
        [this](const geometry_msgs::PoseStamped::ConstPtr& _msg) {
            this->cur_pose_ = *_msg;
            this->mavros_has_pose_ = true;
    });
    mavros_cur_state_sub_ = nh_->subscribe<mavros_msgs::State>(state_topic.c_str(), 10, \
        [this](const mavros_msgs::State::ConstPtr& _msg) {
            this->mavros_state_ = *_msg;
    });

    // Make communications spin!
    spin_thread_ = std::thread([this](){
        ros::spin();
    });

    // TODO: Check this and solve frames issue
    // Wait until we have pose
    while (!mavros_has_pose_ && ros::ok()) {
        //ROS_INFO("Waiting for pose");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    initLocalCoordMatrix();

    // Thread publishing target pose at 10Hz for offboard mode
    offboard_thread_ = std::thread([this]() {
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
    mavros_msgs::CommandBool arming_service;
    arming_service.request.value = true;
    // Arm: abortable
    while (!mavros_state_.armed && !abort_ && ros::ok()) {
        if (!arming_client_.call(arming_service)) {
            ROS_ERROR("Error in arming service calling!");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        ROS_INFO("Arming service response.success = %s", arming_service.response.success ? "true" : "false");
        ROS_INFO("Trying to arm... mavros_state_.armed = %s", mavros_state_.armed ? "true" : "false");
    }
}

void BackendMavros::setFlightMode(const std::string& _flight_mode) {
    mavros_msgs::SetMode flight_mode_service;
    flight_mode_service.request.base_mode = 0;
    flight_mode_service.request.custom_mode = _flight_mode;
    // Set mode: abortable
    while (mavros_state_.mode != _flight_mode && !abort_ && ros::ok()) {
        if (!flight_mode_client_.call(flight_mode_service)) {
            ROS_ERROR("Error in set flight mode [%s] service calling!", _flight_mode.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
            flight_mode_service.response.success ? "true" : "false");
        ROS_INFO("Trying to set offboard mode; mavros_state_.mode = %s", mavros_state_.mode.c_str());
    }
}

void BackendMavros::takeOff(double _height) {
    control_in_vel_ = false;  // Take off control is performed in position (not velocity)

    arm();
    // Set offboard mode after saving home pose
    home_pose_ = cur_pose_;
    // TODO: solve frames issue!
    local_start_pos_ -= Eigen::Vector3d(home_pose_.pose.position.x, \
        home_pose_.pose.position.y, home_pose_.pose.position.z);
    ref_pose_ = home_pose_;
    ref_pose_.pose.position.z += _height;
    setFlightMode("OFFBOARD");

    // Wait until take off: unabortable!
    while (!referencePoseReached() && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO("Flying!");
}

void BackendMavros::land() {
    control_in_vel_ = false;  // Back to control in position (just in case)
    // Set land mode
    setFlightMode("AUTO.LAND");
    ROS_INFO("Landing...");
    ref_pose_ = cur_pose_;
    ref_pose_.pose.position.z = 0;
    // Landing is unabortable!
    while (mavros_state_.armed && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO("Landed!");
}

void BackendMavros::setVelocity(const Velocity& _vel) {
    control_in_vel_ = true;  // Velocity control!
    // TODO: _vel world <-> body tf...
    ref_vel_ = _vel;
}

void BackendMavros::setPositionError(const PositionError& _pos_error) {
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

bool BackendMavros::isReady() const {
    return mavros_has_pose_;  // TODO: Other condition?
}

void BackendMavros::goToWaypoint(const Waypoint& _world) {
    control_in_vel_ = false;  // Control in position

    // TODO: check waypoint reference system!
    // TODO: Solve frames issue!
    auto homogen_world_pos = Eigen::Vector3d(_world.pose.position.x, \
        _world.pose.position.y, _world.pose.position.z);
    //auto homogenRefPos = localTransform.inverse().eval() * homogenWorldPos;
    auto homogen_ref_pos = homogen_world_pos - local_start_pos_;
    ref_pose_.pose.position.x = homogen_ref_pos(0);
    ref_pose_.pose.position.y = homogen_ref_pos(1);
    ref_pose_.pose.position.z = homogen_ref_pos(2);
    //double local_yaw = _world.yaw;  // TODO: yaw transform!
    //ref_pose_.pose.orientation.x = 0;
    //ref_pose_.pose.orientation.y = 0;
    //ref_pose_.pose.orientation.z = sin(0.5*local_yaw);
    //ref_pose_.pose.orientation.w = cos(0.5*local_yaw);
    // TODO: yaw in waypoint as an opition (yaw_lock = true)
    ref_pose_.pose.orientation = cur_pose_.pose.orientation;        

    // Wait until we arrive: abortable
    while(!referencePoseReached() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/*void BackendMavros::trackPath(const WaypointList &_path) {
    // TODO: basic imlementation, ideally different from a stack of gotos
}*/

Pose BackendMavros::pose() const {
        Pose out;
        ///pose.id = std::to_string(robot_id_);
        // TODO: different frames? tf!
        ///pose.frame = "map";
        // TODO: transform to world coordinates?
        /*
        Vec4 localPose = Vec4(cur_pose_.pose.position.x,cur_pose_.pose.position.y,cur_pose_.pose.position.z,1.0);
        Vec4 gamePose = localTransform*localPose;
        pose.position = Vec3(gamePose[0],gamePose[1],gamePose[2]);
        */
        out.pose.position.x = cur_pose_.pose.position.x + local_start_pos_[0];
        out.pose.position.y = cur_pose_.pose.position.y + local_start_pos_[1];
        out.pose.position.z = cur_pose_.pose.position.z + local_start_pos_[2];
        out.pose.orientation = cur_pose_.pose.orientation;
        return out;
}

bool BackendMavros::referencePoseReached() const {
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

void BackendMavros::initLocalCoordMatrix() {
    XMLDocument doc;
    std::string xml_file;
    ros::param::get("frames_file", xml_file);
    std::string path = ros::package::getPath("px4_bringup") + xml_file;
    
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

    std::cout << "Game: " << Rgame << std::endl;

    // Get robot position
    std::cout << "Parsing uavs\n";

    // Get list of robots
    XMLElement* robot = root->FirstChildElement("robothome");
    bool found = false;
    double x, y;
    while (robot && !found) {
        //cout << "Found one robot\n";
        unsigned id = robot->IntAttribute("id");
        if (id == robot_id_) {
            found = true;
            XMLElement* x_element = robot->FirstChildElement("x");
            x_element->QueryDoubleText(&x);
            XMLElement* y_element = robot->FirstChildElement("y");
            y_element->QueryDoubleText(&y);
        }
        robot = robot->NextSiblingElement("robothome");
    }

    std::cout << "Robot: "<< x << " " << y << std::endl;

    local_start_pos_ << x, y, 0.0;
    local_transform_ = Eigen::Matrix4d::Identity();
    local_transform_.block(0,0,2,2) << Rgame;
    local_transform_.block(0,3,3,1) = local_start_pos_;
    //localTransform = localTransform.inverse();

    Eigen::Vector2d game_start_pos;
    game_start_pos << x,y;
    Eigen::Vector2d map_start_pos = Rgame * game_start_pos;
    local_start_pos_ << map_start_pos(0), map_start_pos(1), 0.0;
}

}}	// namespace grvc::ual
