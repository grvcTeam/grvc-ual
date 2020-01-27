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
#ifndef UAV_ABSTRACTION_LAYER_UAL_H
#define UAV_ABSTRACTION_LAYER_UAL_H

#include <uav_abstraction_layer/backend.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/State.h>
#include <thread>

namespace grvc { namespace ual {

/// UAL replicates Backend interface, with some extras
class UAL {
public:

    UAL(Backend* _backend);
    ~UAL();

    /// Initialized and ready to run tasks?
    bool	 isReady() const { return backend_->isReady(); }

    /// Idle?
    bool     isIdle() const { return backend_->isIdle(); }

    /// Latest pose estimation of the robot
    Pose	 pose() const { return backend_->pose(); }

    /// Latest velocity estimation of the robot
    Velocity velocity() const { return backend_->velocity(); }

    /// Latest odometry estimation of the robot
    Odometry odometry() const { return backend_->odometry(); }

    /// Latest transform estimation of the robot
    Transform transform() const { return backend_->transform(); }

    /// Current reference pose
    Pose    referencePose() const { return backend_->referencePose(); }

    /// Current robot state
    uav_abstraction_layer::State state() {
        uav_abstraction_layer::State output;
        output.state = backend_->state();
        return output;
    }

    /// Set pose
    /// \param _pose target pose
    bool    setPose(const geometry_msgs::PoseStamped& _pose);

    /// Go to the specified waypoint, following a straight line
    /// \param _wp goal waypoint
    /// \param _blocking indicates if function call is blocking (default = true)
    bool	goToWaypoint(const Waypoint& _wp, bool _blocking = true);

    /// Go to the specified waypoint in geographic coordinates, following a straight line
    /// \param _wp goal waypoint in geographic coordinates
    /// \param _blocking indicates if function call is blocking (default = true)
    bool	goToWaypointGeo(const WaypointGeo& _wp, bool _blocking = true);

    /// Perform a take off maneuver
    /// \param _height target height that must be reached to consider the take off complete
    /// \param _blocking indicates if function call is blocking (default = true)
    bool    takeOff(double _height, bool _blocking = true);

    /// Land on the current position
    /// \param _blocking indicates if function call is blocking (default = true)
    bool	land(bool _blocking = true);

    /// Set velocities
    /// \param _vel target velocity in world coordinates
    bool    setVelocity(const Velocity& _vel);

    /// Recover from manual flight mode
    /// Use it when FLYING uav is switched to manual mode and want to go BACK to auto.
    /// Call is blocking by definition.
    bool    recoverFromManual();

    /// Set home position (Needed to fix px4 local pose drift)
    bool    setHome(bool set_z = false);

protected:
    Backend* backend_;
    std::thread running_thread_;
    std::thread server_thread_;

    int robot_id_;
    bool id_is_unique_;

    void validateOrientation(geometry_msgs::Quaternion& _q);
};

}}	// namespace grvc::ual

#endif  // UAV_ABSTRACTION_LAYER_UAL_H
