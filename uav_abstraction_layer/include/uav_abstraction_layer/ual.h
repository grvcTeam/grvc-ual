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
#include <thread>

namespace grvc { namespace ual {

class UAL {
public:
    UAL(int _argc, char** _argv) {
        backend_ = Backend::createBackend(_argc, _argv);
    }

    // UAL replicates Backend interface, with some extras:

    /// Initialized and ready to run tasks?
    bool	isReady() const { return backend_->isReady(); }
    /// Latest pose estimation of the robot
    Pose	pose() const { return backend_->pose(); }

    /// Go to the specified waypoint, following a straight line
    /// \param _wp goal waypoint
    /// \param _blocking indicates if function call is blocking (default = true)
    void	goToWaypoint(const Waypoint& _wp, bool _blocking = true) {
        if (_blocking) {
            backend_->threadSafeCall(&Backend::goToWaypoint, _wp);
        } else {
            // Call function on a thread!
            std::thread ([&]() {
                goToWaypoint(_wp, true);  // Kind of recursive!
            }).detach();
        }
    }

    /// Perform a take off maneuver
    /// \param _height target height that must be reached to consider the take off complete
    /// \param _blocking indicates if function call is blocking (default = true)
    void    takeOff(double _height, bool _blocking = true) {
        if (_blocking) {
            backend_->threadSafeCall(&Backend::takeOff, _height);
        } else {
            // Call function on a thread!
            std::thread ([&]() {
                takeOff(_height, true);  // Kind of recursive!
            }).detach();
        }
    }

    /// Land on the current position.
    // void	land();
    /// Set velocities
    /// \param _vel target velocity in world coordinates
    // void    setVelocity(const Velocity& _vel);
    /// Set position error control
    /// \param _pos_error position error in world coordinates
    // void	setPositionError(const PositionError& _pos_error);

protected:
    Backend* backend_;
};

}}	// namespace grvc::ual

#endif  // UAV_ABSTRACTION_LAYER_UAL_H
