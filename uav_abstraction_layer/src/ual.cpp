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
#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/GoToWaypointGeo.h>
#include <uav_abstraction_layer/SetHome.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/Empty.h>

using namespace uav_abstraction_layer;

namespace grvc { namespace ual {

UAL::UAL(Backend* _backend) {
    // Error if ROS is not initialized
    if (!ros::isInitialized()) {
        // Init ros node
        ROS_ERROR("UAL needs ROS to be initialized. Initialize ROS before creating an UAL object.");
        exit(EXIT_FAILURE);
    }

    // Get backend first of all
    backend_ = _backend;
    // Get params
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);

    // Assure id uniqueness
    id_is_unique_ = true;
    std::vector<int> ual_ids;
    if (ros::param::has("/ual_ids")) {
        ros::param::get("/ual_ids", ual_ids);
        for (auto id: ual_ids) {
            if (id == robot_id_) {
                id_is_unique_ = false;
            }
        }
        if (!id_is_unique_) {
            ROS_ERROR("Another ual with id [%d] is already running!", robot_id_);
            throw std::runtime_error("Id is not unique, already found in /ual_ids");
        }
    }
    if (id_is_unique_) {
        ual_ids.push_back(robot_id_);
    }
    ros::param::set("/ual_ids", ual_ids);

    // TODO(franreal): check if it's possible to assure id uniqueness with topic names
    // ros::master::V_TopicInfo master_topics;
    // ros::master::getTopics(master_topics);
    // for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    //     const ros::master::TopicInfo& info = *it;
    //     std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
    // }

    // Start server if explicitly asked
    std::string server_mode;
    pnh.param<std::string>("ual_server", server_mode, "on");

    if (server_mode == "on") {
        server_thread_ = std::thread([this]() {
            std::string ual_ns = "ual";
            std::string take_off_srv = ual_ns + "/take_off";
            std::string land_srv = ual_ns + "/land";
            std::string go_to_waypoint_srv = ual_ns + "/go_to_waypoint";
            std::string go_to_waypoint_geo_srv = ual_ns + "/go_to_waypoint_geo";
            std::string set_pose_topic = ual_ns + "/set_pose";
            std::string set_velocity_topic = ual_ns + "/set_velocity";
            std::string recover_from_manual_srv = ual_ns + "/recover_from_manual";
            std::string set_home_srv = ual_ns + "/set_home";
            std::string pose_topic = ual_ns + "/pose";
            std::string velocity_topic = ual_ns + "/velocity";
            std::string odometry_topic = ual_ns + "/odom";
            std::string state_topic = ual_ns + "/state";
            std::string ref_pose_topic = ual_ns + "/ref_pose";

            ros::NodeHandle nh;
            ros::ServiceServer take_off_service =
                nh.advertiseService<TakeOff::Request, TakeOff::Response>(
                take_off_srv,
                [this](TakeOff::Request &req, TakeOff::Response &res) {
                return this->takeOff(req.height, req.blocking);
            });
            ros::ServiceServer land_service =
                nh.advertiseService<Land::Request, Land::Response>(
                land_srv,
                [this](Land::Request &req, Land::Response &res) {
                return this->land(req.blocking);
            });
            ros::ServiceServer go_to_waypoint_service =
                nh.advertiseService<GoToWaypoint::Request, GoToWaypoint::Response>(
                go_to_waypoint_srv,
                [this](GoToWaypoint::Request &req, GoToWaypoint::Response &res) {
                return this->goToWaypoint(req.waypoint, req.blocking);
            });
            ros::ServiceServer go_to_waypoint_geo_service =
                nh.advertiseService<GoToWaypointGeo::Request, GoToWaypointGeo::Response>(
                go_to_waypoint_geo_srv,
                [this](GoToWaypointGeo::Request &req, GoToWaypointGeo::Response &res) {
                return this->goToWaypointGeo(req.waypoint, req.blocking);
            });
            ros::Subscriber set_pose_sub =
                nh.subscribe<geometry_msgs::PoseStamped>(
                set_pose_topic, 1,
                [this](const geometry_msgs::PoseStamped::ConstPtr& _msg) {
                this->setPose(*_msg);
            });
            ros::Subscriber set_velocity_sub =
                nh.subscribe<geometry_msgs::TwistStamped>(
                set_velocity_topic, 1,
                [this](const geometry_msgs::TwistStamped::ConstPtr& _msg) {
                this->setVelocity(*_msg);
            });
            ros::ServiceServer recover_from_manual_service =
                nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
                recover_from_manual_srv,
                [this](std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
                return this->recoverFromManual();
            });
            ros::ServiceServer set_home_service =
                nh.advertiseService<SetHome::Request, SetHome::Response>(
                set_home_srv,
                [this](SetHome::Request &req, SetHome::Response &res) {
                return this->setHome(req.set_z);
            });
            ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
            ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic, 10);
            ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>(odometry_topic, 10);
            ros::Publisher state_pub = nh.advertise<uav_abstraction_layer::State>(state_topic, 10);
            ros::Publisher ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(ref_pose_topic, 10);
            static tf2_ros::TransformBroadcaster tf_pub;

            // Publish @ 30Hz default
            double ual_pub_rate;
            ros::param::param<double>("~pub_rate", ual_pub_rate, 30.0);
            ros::Rate loop_rate(ual_pub_rate);
            while (ros::ok()) {
                pose_pub.publish(this->pose());
                velocity_pub.publish(this->velocity());
                odometry_pub.publish(this->odometry());
                state_pub.publish(this->state());
                tf_pub.sendTransform(this->transform());
                ref_pose_pub.publish(this->referencePose()); //!TODO: publish only during position control?
                loop_rate.sleep();
            }
        });
    }
}

UAL::~UAL() {
    if (!backend_->isIdle()) { backend_->abort(); }
    if (running_thread_.joinable()) { running_thread_.join(); }
    if (server_thread_.joinable()) { server_thread_.join(); }

    if (id_is_unique_) {
        // Remove id from /ual_ids
        std::vector<int> ual_ids;
        ros::param::get("/ual_ids", ual_ids);
        std::vector<int> new_ual_ids;
        for (auto id: ual_ids) {
            if (id != robot_id_) {
                new_ual_ids.push_back(id);
            }
        }
        ros::param::set("/ual_ids", new_ual_ids);
    }
    delete(backend_);
}

bool UAL::setPose(const geometry_msgs::PoseStamped& _pose) {
    // Check required state
    if (backend_->state() != uav_abstraction_layer::State::FLYING_AUTO) {
        ROS_ERROR("Unable to setPose: not FLYING_AUTO!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(false); }

    // Check consistency of pose data (isnan?)
    if ( std::isnan(_pose.pose.position.x) || std::isnan(_pose.pose.position.y) || std::isnan(_pose.pose.position.z) ||
         std::isnan(_pose.pose.orientation.x) || std::isnan(_pose.pose.orientation.y) || std::isnan(_pose.pose.orientation.z) ||
         std::isnan(_pose.pose.orientation.w) ) {
        ROS_ERROR("Unable to setPose: NaN received");
        return false;
    }

    geometry_msgs::PoseStamped ref_pose = _pose;
    validateOrientation(ref_pose.pose.orientation);
    backend_->threadSafeCall(&Backend::setPose, ref_pose);
    return true;
}
bool UAL::goToWaypoint(const Waypoint& _wp, bool _blocking) {
    // Check required state
    if (backend_->state() != uav_abstraction_layer::State::FLYING_AUTO) {
        ROS_ERROR("Unable to goToWaypoint: not FLYING_AUTO!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(false); }

    // Check consistency of pose data (isnan?)
    if ( std::isnan(_wp.pose.position.x) || std::isnan(_wp.pose.position.y) || std::isnan(_wp.pose.position.z) ||
         std::isnan(_wp.pose.orientation.x) || std::isnan(_wp.pose.orientation.y) || std::isnan(_wp.pose.orientation.z) ||
         std::isnan(_wp.pose.orientation.w) ) {
        ROS_ERROR("Unable to goToWaypoint: NaN received");
        return false;
    }

    geometry_msgs::PoseStamped ref_wp = _wp;
    validateOrientation(ref_wp.pose.orientation);
    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::goToWaypoint, ref_wp)) {
            ROS_INFO("Blocking goToWaypoint rejected!");
            return false;
        }
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this, ref_wp]() {
            if (!this->backend_->threadSafeCall(&Backend::goToWaypoint, ref_wp)) {
                ROS_INFO("Non-blocking goToWaypoint rejected!");
            }
        });
    }
    return true;
}
bool UAL::goToWaypointGeo(const WaypointGeo& _wp, bool _blocking) {
    // Check required state
    if (backend_->state() != uav_abstraction_layer::State::FLYING_AUTO) {
        ROS_ERROR("Unable to goToWaypointGeo: not FLYING_AUTO!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(false); }

    // Check consistency of geo pose data (isnan?)
    if ( std::isnan(_wp.latitude) || std::isnan(_wp.longitude) || std::isnan(_wp.altitude) ) {
        ROS_ERROR("Unable to goToWaypointGeo: NaN received");
        return false;
    }

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::goToWaypointGeo, _wp)) {
            ROS_INFO("Blocking goToWaypoint rejected!");
            return false;
        }
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this, _wp]() {
            if (!this->backend_->threadSafeCall(&Backend::goToWaypointGeo, _wp)) {
                ROS_INFO("Non-blocking goToWaypoint rejected!");
            }
        });
    }
    return true;
}

bool UAL::takeOff(double _height, bool _blocking) {
    // Check required state
    if (backend_->state() != uav_abstraction_layer::State::LANDED_ARMED) {
        ROS_ERROR("Unable to takeOff: not LANDED_ARMED!");
        std::cout << backend_->state() << std::endl;
        return false;
    }
    // Check input
    if ( _height < 0.0 || std::isnan(_height) ) {
        ROS_ERROR("Unable to takeOff: height must be positive!");
        return false;
    }
    if (!backend_->isIdle()) { backend_->abort(false); }
    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::takeOff, _height)) {
            ROS_INFO("Blocking takeOff rejected!");
            return false;
        }
    } else {
        if (running_thread_.joinable()) { running_thread_.join(); }
        // Call function on a thread:
        running_thread_ = std::thread ([this, _height]() {
            if (!this->backend_->threadSafeCall(&Backend::takeOff, _height)) {
                ROS_INFO("Non-blocking takeOff rejected!");
            }
        });
    }
    return true;
}

bool UAL::land(bool _blocking) {
    // Check required state
    if (backend_->state() != uav_abstraction_layer::State::FLYING_AUTO) {
        ROS_ERROR("Unable to land: not FLYING_AUTO!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::land)) {
            ROS_INFO("Blocking land rejected!");
            return false;
        }
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this]() {
            if (!this->backend_->threadSafeCall(&Backend::land)) {
                ROS_INFO("Non-blocking land rejected!");
            }
        });
    }
    return true;
}

bool UAL::setVelocity(const Velocity& _vel) {
    // Check required state
    if (backend_->state() != uav_abstraction_layer::State::FLYING_AUTO) {
        ROS_ERROR("Unable to setVelocity: not FLYING_AUTO!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    // Check consistency of velocity data (isnan?)
    if ( std::isnan(_vel.twist.linear.x) || std::isnan(_vel.twist.linear.y) || std::isnan(_vel.twist.linear.z) ||
         std::isnan(_vel.twist.angular.x) || std::isnan(_vel.twist.angular.y) || std::isnan(_vel.twist.angular.z) ) {
        ROS_ERROR("Unable to setVelocity: NaN received");
        return false;
    }

    backend_->threadSafeCall(&Backend::setVelocity, _vel);
    return true;
}

bool UAL::recoverFromManual() {
    // Check required state
    if (backend_->state() != uav_abstraction_layer::State::FLYING_MANUAL) {
        ROS_ERROR("Unable to recoverFromManual: not FLYING_MANUAL!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    backend_->threadSafeCall(&Backend::recoverFromManual);

    return true;
}

bool UAL::setHome(bool set_z) {
    // Check required state
    if ((backend_->state() != uav_abstraction_layer::State::LANDED_DISARMED) && (backend_->state() != uav_abstraction_layer::State::LANDED_ARMED)) {
        ROS_ERROR("Unable to setHome: not LANDED_*!");
        return false;
    }
    backend_->setHome(set_z);

    return true;
}

inline void UAL::validateOrientation(geometry_msgs::Quaternion& _q) {
    double norm2 = _q.x*_q.x  + _q.y*_q.y  + _q.z*_q.z  + _q.w*_q.w;
    if (fabs(norm2 - 1) > 0.01) {  // Threshold for norm2
        if (norm2 == 0) {  // Exactly 0, set current orientation
            ROS_INFO("Orientation quaternion norm is zero, holding current orientation");
            _q = this->pose().pose.orientation;
        } else {
            double norm = sqrt(norm2);
            ROS_WARN("Orientation quaternion norm is %lf, nomalizing it", norm);
            _q.x /= norm;
            _q.y /= norm;
            _q.z /= norm;
            _q.w /= norm;
        }
    }
    // TODO: Other checks? E.g. yaw-only orientation shoud have x = y = 0
    // ROS_INFO("q = [%lf, %lf, %lf, %lf]", _q.x, _q.y, _q.z, _q.w);  // Debug!
}

}}	// namespace grvc::ual
