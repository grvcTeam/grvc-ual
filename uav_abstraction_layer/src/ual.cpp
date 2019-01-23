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

UAL::UAL(int _argc, char** _argv) {
    // Start ROS if not initialized
    if (!ros::isInitialized()) {
        // Init ros node
        ros::init(_argc, _argv, "ual");
    }
    this->init();
}

UAL::UAL() {
    // Error if ROS is not initialized
    if (!ros::isInitialized()) {
        // Init ros node
        ROS_ERROR("UAL needs ROS to be initialized. Initialize ROS before creating UAL object or use UAL(int _argc, char** _argv) constructor.");
        exit(0);
    }
    this->init();
}

void UAL::init() {
    // Create backend first of all, inits ros node
    backend_ = Backend::createBackend();
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
    // TODO: Consider other modes?
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
    if (backend_->state() != Backend::State::FLYING_AUTO) {
        ROS_ERROR("Unable to setPose: not FLYING_AUTO!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(false); }

    // Function is non-blocking in backend TODO: non-thread-safe-call?
    backend_->threadSafeCall(&Backend::setPose, _pose);
    return true;
}
bool UAL::goToWaypoint(const Waypoint& _wp, bool _blocking) {
    // Check required state
    if (backend_->state() != Backend::State::FLYING_AUTO) {
        ROS_ERROR("Unable to goToWaypoint: not FLYING_AUTO!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(false); }

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::goToWaypoint, _wp)) {
            ROS_INFO("Blocking goToWaypoint rejected!");
            return false;
        }
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this, _wp]() {
            if (!this->backend_->threadSafeCall(&Backend::goToWaypoint, _wp)) {
                ROS_INFO("Non-blocking goToWaypoint rejected!");
            }
        });
    }
    return true;
}
bool UAL::goToWaypointGeo(const WaypointGeo& _wp, bool _blocking) {
    // Check required state
    if (backend_->state() != Backend::State::FLYING_AUTO) {
        ROS_ERROR("Unable to goToWaypointGeo: not FLYING_AUTO!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(false); }

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
    if (backend_->state() != Backend::State::LANDED_ARMED) {
        ROS_ERROR("Unable to takeOff: not LANDED_ARMED!");
        return false;
    }
    // Check input
    if (_height < 0.0) {
        ROS_ERROR("Unable to takeOff: height must be positive!");
        return false;
    }

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::takeOff, _height)) {
            ROS_INFO("Blocking takeOff rejected!");
            return false;
        }
    } else {
        if (running_thread_.joinable()) running_thread_.join();
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
    if (backend_->state() != Backend::State::FLYING_AUTO) {
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
    if (backend_->state() != Backend::State::FLYING_AUTO) {
        ROS_ERROR("Unable to setVelocity: not FLYING_AUTO!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    // Function is non-blocking in backend TODO: non-thread-safe-call?
    backend_->threadSafeCall(&Backend::setVelocity, _vel);
    return true;
}

bool UAL::recoverFromManual() {
    // Check required state
    if (backend_->state() != Backend::State::FLYING_MANUAL) {
        ROS_ERROR("Unable to recoverFromManual: not FLYING_MANUAL!");
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    // Direct call! TODO: threadSafeCall?
    backend_->recoverFromManual();

    return true;
}

// TODO: Collapse ual and backend state?
uav_abstraction_layer::State UAL::state() {
    uav_abstraction_layer::State output;
    switch (backend_->state()) {
        case Backend::State::UNINITIALIZED:
            output.state = uav_abstraction_layer::State::UNINITIALIZED;
            break;
        case Backend::State::LANDED_DISARMED:
            output.state = uav_abstraction_layer::State::LANDED_DISARMED;
            break;
        case Backend::State::LANDED_ARMED:
            output.state = uav_abstraction_layer::State::LANDED_ARMED;
            break;
        case Backend::State::TAKING_OFF:
            output.state = uav_abstraction_layer::State::TAKING_OFF;
            break;
        case Backend::State::FLYING_AUTO:
            output.state = uav_abstraction_layer::State::FLYING_AUTO;
            break;
        case Backend::State::FLYING_MANUAL:
            output.state = uav_abstraction_layer::State::FLYING_MANUAL;
            break;
        case Backend::State::LANDING:
            output.state = uav_abstraction_layer::State::LANDING;
            break;
        default:
            ROS_ERROR("Unexpected Backend::State!");
    }
    return output;
}

bool UAL::setHome(bool set_z) {
    // Check required state
    if (backend_->state() != Backend::State::LANDED_DISARMED) {
        ROS_ERROR("Unable to setHome: not LANDED_DISARMED!");
        return false;
    }
    backend_->setHome(set_z);

    return true;
}

}}	// namespace grvc::ual
