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
#ifndef UAV_ABSTRACTION_LAYER_BACKEND_DUMMY_H
#define UAV_ABSTRACTION_LAYER_BACKEND_DUMMY_H

#include <uav_abstraction_layer/backend.h>
#include <ros/ros.h>


namespace grvc { namespace ual {

class BackendDummy : public Backend {

public:
    BackendDummy() : Backend() {
        ROS_WARN("BackendDummy is only for testing porposes");
    }

    /// Backend is initialized and ready to run tasks?
    bool	         isReady() const override { return true; }
    /// Latest pose estimation of the robot
    virtual Pose	 pose() override { return Pose(); }  // TODO: Never moving?
    /// Latest velocity estimation of the robot
    virtual Velocity velocity() const override { return Velocity(); }  // TODO: Never moving?
    /// Latest odometry estimation of the robot
    virtual Odometry odometry() const override { return Odometry(); }  // TODO: Never moving?
    /// Latest transform estimation of the robot
    virtual Transform transform() const override {  // TODO: Never moving?
        Transform out;
        out.header.stamp = ros::Time::now();
        out.header.frame_id = "uav_1_home";  // TODO: uav_id?
        out.child_frame_id = "uav_1";        // TODO: uav_id?
        out.transform.translation.x = 0;
        out.transform.translation.y = 0;
        out.transform.translation.z = 0;
        out.transform.rotation.x = 0;
        out.transform.rotation.y = 0;
        out.transform.rotation.z = 0;
        out.transform.rotation.w = 1;
        return out;
    }

    /// Set pose
    /// \param _pose target pose
    void    setPose(const geometry_msgs::PoseStamped& _pose) override {
        ROS_INFO("BackendDummy::setPose: p.x = %f, p.y = %f, p.z = %f, q.x = %f, q.y = %f, q.z = %f, q.w = %f", _pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z, 
            _pose.pose.orientation.x, _pose.pose.orientation.y, _pose.pose.orientation.z, _pose.pose.orientation.w);
    }

    /// Go to the specified waypoint, following a straight line
    /// \param _wp goal waypoint
    void	goToWaypoint(const Waypoint& _wp) override {
        double yaw = 2 * atan2(_wp.pose.orientation.z, _wp.pose.orientation.w);
        ROS_INFO("BackendDummy::goToWaypoint: x = %f, y = %f, z = %f, yaw = %f", _wp.pose.position.x, _wp.pose.position.y, _wp.pose.position.z, yaw);
    }

    /// Go to the specified waypoint in geographic coordinates, following a straight line
    /// \param _wp goal waypoint in geographic coordinates
    void	goToWaypointGeo(const WaypointGeo& _wp) override {
        ROS_INFO("BackendDummy::goToWaypointGeo: latitude = %f, longitude = %f, altitude = %f", _wp.latitude, _wp.longitude, _wp.altitude);
    }

    /// Follow a list of waypoints, one after another
    // void trackPath(const Path& _path) override;
    /// Perform a take off maneuver
    /// \param _height target height that must be reached to consider the take off complete
    void    takeOff(double _height) override {
        ROS_INFO("BackendDummy::takeOff: height = %f", _height);
    }
    /// Land on the current position.
    void	land() override {
        ROS_INFO("BackendDummy::land");
    }
    /// Set velocities
    /// \param _vel target velocity in world coordinates
    void    setVelocity(const Velocity& _vel) override {
        ROS_INFO("BackendDummy::setVelocity: vx = %f, vy = %f, vz = %f, wz = %f", _vel.twist.linear.x, _vel.twist.linear.y, _vel.twist.linear.z, _vel.twist.angular.z);
    }
    /// Recover from manual flight mode
    /// Use it when FLYING uav is switched to manual mode and want to go BACK to auto.
    void    recoverFromManual() override {
        ROS_INFO("BackendDummy::recoverFromManual");
    }
    /// Set home position
    void    setHome(bool set_z) override {
        ROS_INFO("BackendDummy::setHome: set_z = %s", set_z ? "true" : "false");
    }

    grvc::ual::Pose referencePose() override {}  // TODO: Move this info to State.msg?
};

}}	// namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_BACKEND_DUMMY_H
